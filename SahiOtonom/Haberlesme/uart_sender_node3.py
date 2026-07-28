#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
import serial
import time

class UartSenderNode(Node):
    def __init__(self):
        super().__init__('uart_sender_node')

        # Tek port (MIX PORT) - hız, direksiyon ve fren aynı porttan gönderilir
        # Arduino (CH340 çipli) sabit kimlik yolu - takma sırasından etkilenmez.
        # NOT: brltty servisi CH340'ı çaldığı için mask'landı (2026-07-12).
        self.declare_parameter('mix_port',
                               '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud_rate', 38400)

        self.MIX_PORT = self.get_parameter('mix_port').value
        self.BAUD_RATE = self.get_parameter('baud_rate').value

        self.get_logger().info(f'Mix Portu: {self.MIX_PORT}')

        # PID parametreleri (ROS parametresi - kod değişmeden pistte ayarlanabilir)
        # Sapma -1..+1 normalize geldiği için P ağırlıklı kontrol yeterli.
        self.declare_parameter('kp', 1.0)
        # ki: kalıcı küçük sapmayı zamanla sıfırlar (araç ortalıyor ama tam
        # merkeze oturmuyorsa bunu artır; yavaş salınım başlarsa azalt)
        self.declare_parameter('ki', 0.2)
        self.declare_parameter('kd', 0.2)
        # Araç şeride doğru değil de ŞERİTTEN DIŞARI kırıyorsa bunun işaretini çevir
        # (2026-07-12: araç sürekli sağa kaçtığı için +1.0 -> -1.0 yapıldı)
        self.declare_parameter('steering_direction', -1.0)
        # TRİM: Araç düz gitmesi gerekirken yamuk gidiyorsa burayı ayarla.
        # Birim: derece (d komutuyla aynı). Araç SOLA çekiyorsa artır (+5, +10...),
        # SAĞA çekiyorsa azalt (-5, -10...). Merkez = 180 + trim olur.
        self.declare_parameter('steering_trim', 0)
        # DÜZ BAŞLANGIÇ: Sistem açıldıktan sonra bu süre boyunca (saniye)
        # direksiyon merkezde tutulur, şerit takibi devreye girmez.
        # Kamera ve tespit otursun diye - başlangıçtaki sağa/sola çekmeyi önler.
        self.declare_parameter('straight_start_sec', 3.0)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.steering_direction = self.get_parameter('steering_direction').value
        self.steering_trim = self.get_parameter('steering_trim').value
        self.straight_start_sec = self.get_parameter('straight_start_sec').value
        self.start_time = time.time()
        self.straight_phase_done = False

        self.prev_error = 0.0
        self.integral = 0.0
        self.max_steering_angle = 0.5

        # Lateral deviation ve manuel ackermann komutları
        self.current_lateral_deviation = 0.0
        self.manual_speed = 0.0

        # Mix portunu aç
        try:
            self.mix_serial = serial.Serial(self.MIX_PORT, self.BAUD_RATE, timeout=0.1)
            time.sleep(3)
            self.get_logger().info(f'✅ Mix portu (Hız + Direksiyon + Fren) açıldı.')
        except Exception as e:
            self.get_logger().error(f'❌ Mix portu açılamadı: {e}')
            self.mix_serial = None

        # Lateral deviation subscriber'ı ekle
        self.lateral_sub = self.create_subscription(
            Float32,
            '/lane/lateral_deviation',
            self.lateral_deviation_callback,
            10
        )

        # Manuel ackermann komutları için subscription
        self.ackermann_sub = self.create_subscription(
            AckermannDrive,
            '/ackermann_cmd',
            self.ackermann_callback,
            10)

        # Decision making node'undan speed verisi almak için subscriber ekle
        self.speed_sub = self.create_subscription(
            Float32,
            '/speed',
            self.speed_callback,
            10
        )

        self.get_logger().info('🦾 UART Gönderici (Tek Mix Port + PID Lateral Control) başlatıldı. Komut bekleniyor...')

    def send_command(self, prefix, value):
        """Mix porta 'harf,değer' formatında komut gönderir (örn: h,1  f,0  d,127).
        DİKKAT: Sonlandırıcı yok - komutlar porta ayraçsız art arda yazılır."""
        if not (self.mix_serial and self.mix_serial.is_open):
            return
        command = f'{prefix},{value}'
        self.mix_serial.write(command.encode('utf-8'))

    def speed_callback(self, msg: Float32):
        """Decision making node'undan gelen speed verisini işle"""
        try:
            speed = msg.data

            # Hız komutunu işle ve GÖNDER
            speed_signal = self.speed_to_digital_signal(speed)

            # Fren sinyali (hız 1 ise fren yok, 0 ise fren)
            brake_signal = 0 if speed_signal == 1 else 1

            self.send_command('h', speed_signal)   # h -> hız
            self.send_command('f', brake_signal)   # f -> fren

            self.get_logger().info(f'📡 DECISION MAKING HIZ | Hız: {speed:.2f} m/s | Sinyal: h,{speed_signal} f,{brake_signal}')

        except Exception as e:
            self.get_logger().error(f'Speed callback hatası: {e}')

    def lateral_deviation_to_steering_angle(self, lateral_deviation):
        """
        PID kontrolör kullanarak lateral deviation'ı direksiyon açısına dönüştürür
        """
        error = lateral_deviation

        # PID hesaplama
        p_term = self.kp * error

        self.integral += error
        self.integral = max(-0.3, min(self.integral, 0.3))
        i_term = self.ki * self.integral

        d_term = self.kd * (error - self.prev_error)
        self.prev_error = error

        pid_output = self.steering_direction * -(p_term + i_term + d_term)

        steering_angle = max(-self.max_steering_angle,
                           min(pid_output, self.max_steering_angle))

        return steering_angle

    def lateral_deviation_callback(self, msg: Float32):
        """Lateral deviation mesajını alır ve direksiyon kontrolü yapar.
        DİREKSİYONUN TEK SAHİBİ BU FONKSİYON - başka hiçbir yer d komutu göndermez."""
        self.current_lateral_deviation = msg.data

        # DÜZ BAŞLANGIÇ FAZI: ilk straight_start_sec saniye direksiyon merkezde
        # tutulur, PID devreye girmez (kamera/tespit otursun, araç düz kalksın).
        if time.time() - self.start_time < self.straight_start_sec:
            self.integral = 0.0
            self.prev_error = 0.0
            self.send_command('d', self.angle_to_byte(0.0))
            return
        if not self.straight_phase_done:
            self.straight_phase_done = True
            self.get_logger().info('🛣️ Düz başlangıç fazı bitti - şerit takibi devrede.')

        steering_angle = self.lateral_deviation_to_steering_angle(self.current_lateral_deviation)

        angle_byte = self.angle_to_byte(steering_angle)
        self.send_command('d', angle_byte)   # d -> direksiyon
        self.get_logger().info(f'🎯 LATERAL KONTROL | Dev: {self.current_lateral_deviation:.3f} | '
                             f'Steering: {steering_angle:.3f} rad | Byte: d,{angle_byte} | '
                             f'Durum: {"SAĞ tarafta→SOLA" if self.current_lateral_deviation < 0 else "SOL tarafta→SAĞA" if self.current_lateral_deviation > 0 else "MERKEZ"}')

    def speed_to_digital_signal(self, speed_ms):
        """Gelen hız değerine göre 1 (İleri Git) veya 0 (Dur/Fren) döndürür."""
        if speed_ms > 0.1:
            return 1
        else:
            return 0

    def angle_to_byte(self, angle_rad):
        """Direksiyon açısını 0-360 aralığına çevirir (merkez = 180 + trim).
        -0.5 rad (tam sol) -> 0 | 0 rad (merkez) -> 180 | +0.5 rad (tam sağ) -> 360
        steering_trim, mekanik merkez kaymasını düzeltmek için değere eklenir."""
        angle_rad = max(-0.5, min(angle_rad, 0.5))
        value = int(round((angle_rad + 0.5) * 360)) + self.steering_trim
        return max(0, min(value, 360))

    def ackermann_callback(self, msg: AckermannDrive):
        """Ackermann komutunu alır - sadece hız kontrolü için kullanılır"""
        try:
            # Hız komutunu işle ve GÖNDER
            speed_signal = self.speed_to_digital_signal(msg.speed)
            brake_signal = 0 if speed_signal == 1 else 1

            self.send_command('h', speed_signal)   # h -> hız
            self.send_command('f', brake_signal)   # f -> fren
            self.get_logger().info(f'🎮 MANUEL KONTROL | Hız: {msg.speed:.2f} m/s | Sinyal: h,{speed_signal} f,{brake_signal}')

            self.manual_speed = msg.speed

            # NOT: Buradaki steering_angle BİLEREK yok sayılıyor.
            # Direksiyonun tek sahibi lateral_deviation_callback'teki PID'dir;
            # iki kontrolcünün aynı direksiyonu çekiştirmesi şerit takibini bozuyordu.

        except Exception as e:
            self.get_logger().error(f'UART gönderme hatası: {e}')

    def __del__(self):
        """Node kapanırken motorları güvenli duruma getir."""
        if hasattr(self, 'mix_serial') and self.mix_serial and self.mix_serial.is_open:
            self.send_command('h', 0)      # hızı kes
            self.send_command('f', 1)      # freni uygula
            self.send_command('d', self.angle_to_byte(0.0))  # direksiyonu merkeze al (trim dahil)
            self.mix_serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = UartSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
