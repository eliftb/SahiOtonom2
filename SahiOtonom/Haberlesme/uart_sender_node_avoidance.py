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

        # Portları ve ayarları tanımla
        self.declare_parameter('speed_port', '/dev/ttyACM1')      # FREN/İTKİ PORTU
        self.declare_parameter('steering_port', '/dev/ttyACM0')   # DİREKSİYON PORTU
        self.declare_parameter('stop_port', '/dev/ttyACM2')       # STOP PORTU
        self.declare_parameter('baud_rate', 38400)
        
        self.SPEED_PORT = self.get_parameter('speed_port').value
        self.STEERING_PORT = self.get_parameter('steering_port').value
        self.STOP_PORT = self.get_parameter('stop_port').value
        self.BAUD_RATE = self.get_parameter('baud_rate').value
        
        self.get_logger().info(f'Hız Portu: {self.SPEED_PORT}')
        self.get_logger().info(f'Stop Portu: {self.STOP_PORT}')
        self.get_logger().info(f'Direksiyon Portu: {self.STEERING_PORT}')
        
        # PID Kontrolör parametreleri - lateral deviation için
        self.kp = 0.05
        self.ki = 0.5
        self.kd = 1.0
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.max_steering_angle = 0.5
        
        # Lateral deviation ve manuel ackermann komutları
        self.current_lateral_deviation = 0.0
        self.manual_speed = 0.0
        self.use_lateral_control = True
        
        # Hız portunu aç
        try:
            self.speed_serial = serial.Serial(self.SPEED_PORT, self.BAUD_RATE, timeout=0.1)
            time.sleep(1)
            self.get_logger().info(f'✅ Hız portu (FREN/İTKİ) açıldı.')
        except Exception as e:
            self.get_logger().error(f'❌ Hız portu açılamadı: {e}')
            self.speed_serial = None

        # Stop portunu aç - değişken adı düzeltildi
        try:
            self.stop_serial = serial.Serial(self.STOP_PORT, self.BAUD_RATE, timeout=0.1)
            time.sleep(1)
            self.get_logger().info(f'✅ Stop portu (FREN) açıldı.')
        except Exception as e:
            self.get_logger().error(f'❌ Stop portu açılamadı: {e}')
            self.stop_serial = None
            
        # Direksiyon portunu aç
        try:
            self.steering_serial = serial.Serial(self.STEERING_PORT, self.BAUD_RATE, timeout=0.1)
            time.sleep(1)
            self.get_logger().info(f'✅ Direksiyon portu açıldı.')
        except Exception as e:
            self.get_logger().error(f'❌ Direksiyon portu açılamadı: {e}')
            self.steering_serial = None
        
        # Lateral deviation subscriber'ı ekle
        self.lateral_sub = self.create_subscription(
            Float32,
            '/lane/lateral_new_deviation',
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
        
        self.get_logger().info('🦾 UART Gönderici (Decision Making Uyumlu + PID Lateral Control) başlatıldı. Komut bekleniyor...')

    def speed_callback(self, msg: Float32):
        """Decision making node'undan gelen speed verisini işle"""
        try:
            speed = msg.data
            
            # Hız komutunu işle ve GÖNDER
            if self.speed_serial and self.speed_serial.is_open:
                speed_signal = self.speed_to_digital_signal(speed)
                
                # Stop sinyali de gönder
                if self.stop_serial and self.stop_serial.is_open:
                    stop_signal = 0 if speed_signal == 1 else 1
                    self.stop_serial.write(stop_signal.to_bytes(1, byteorder='little'))
                
                self.speed_serial.write(speed_signal.to_bytes(1, byteorder='little'))
                self.get_logger().info(f'📡 DECISION MAKING HIZ | Hız: {speed:.2f} m/s | Sinyal: {speed_signal}')
                
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
        
        pid_output = -(p_term + i_term + d_term)
        
        steering_angle = max(-self.max_steering_angle, 
                           min(pid_output, self.max_steering_angle))
        
        return steering_angle

    def lateral_deviation_callback(self, msg: Float32):
        """Lateral deviation mesajını alır ve direksiyon kontrolü yapar"""
        self.current_lateral_deviation = msg.data
        
        if self.use_lateral_control:
            steering_angle = self.lateral_deviation_to_steering_angle(self.current_lateral_deviation)
            
            if self.steering_serial and self.steering_serial.is_open:
                angle_byte = self.angle_to_byte(steering_angle)
                self.steering_serial.write(angle_byte.to_bytes(1, byteorder='little'))
                self.get_logger().info(f'🎯 LATERAL KONTROL | Dev: {self.current_lateral_deviation:.3f} | '
                                     f'Steering: {steering_angle:.3f} rad | Byte: {angle_byte} | '
                                     f'Durum: {"SAĞ tarafta→SOLA" if self.current_lateral_deviation < 0 else "SOL tarafta→SAĞA" if self.current_lateral_deviation > 0 else "MERKEZ"}')

    def speed_to_digital_signal(self, speed_ms):
        """Gelen hız değerine göre 1 (İleri Git) veya 0 (Dur/Fren) döndürür."""
        if speed_ms > 0.1:
            return 1
        else:
            return 0

    def angle_to_byte(self, angle_rad):
        """Direksiyon açısını oransal olarak (0-255) çevirir."""
        angle_rad = max(-0.5, min(angle_rad, 0.5))
        byte_value = int((angle_rad + 0.5) * 255)
        return max(0, min(byte_value, 255))

    def ackermann_callback(self, msg: AckermannDrive):
        """Ackermann komutunu alır - sadece hız kontrolü için kullanılır"""
        try:
            # Hız komutunu işle ve GÖNDER
            if self.speed_serial and self.speed_serial.is_open:
                speed_signal = self.speed_to_digital_signal(msg.speed)
                if self.stop_serial and self.stop_serial.is_open:
                    stop_signal = 0 if speed_signal == 1 else 1
                    self.stop_serial.write(stop_signal.to_bytes(1, byteorder='little'))
                self.speed_serial.write(speed_signal.to_bytes(1, byteorder='little'))
                self.get_logger().info(f'🎮 MANUEL KONTROL | Hız: {msg.speed:.2f} m/s | Sinyal: {speed_signal}')
                
            self.manual_speed = msg.speed
            
            # Eğer manuel direksiyon komutu gelirse
            if abs(msg.steering_angle) > 0.02:
                self.use_lateral_control = False
                if self.steering_serial and self.steering_serial.is_open:
                    angle_byte = self.angle_to_byte(msg.steering_angle)
                    self.steering_serial.write(angle_byte.to_bytes(1, byteorder='little'))
                    self.get_logger().info(f'🎮 MANUEL DİREKSİYON | Açı: {msg.steering_angle:.3f} rad | Byte: {angle_byte}')
            else:
                if not self.use_lateral_control:
                    self.use_lateral_control = True
                    self.get_logger().info('🎯 Lateral kontrol yeniden etkinleştirildi')
                
        except Exception as e:
            self.get_logger().error(f'UART gönderme hatası: {e}')

    def __del__(self):
        """Node kapanırken motorları güvenli duruma getir."""
        if hasattr(self, 'speed_serial') and self.speed_serial and self.speed_serial.is_open:
            self.speed_serial.write((0).to_bytes(1,'little'))
            self.speed_serial.close()
        if hasattr(self, 'steering_serial') and self.steering_serial and self.steering_serial.is_open:
            self.steering_serial.write((127).to_bytes(1,'little'))
            self.steering_serial.close()
        if hasattr(self, 'stop_serial') and self.stop_serial and self.stop_serial.is_open:  # Typo düzeltildi
            self.stop_serial.write((127).to_bytes(1,'little'))
            self.stop_serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = UartSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()