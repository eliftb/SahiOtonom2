#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32, Bool, Int32
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
import math
import serial
import os
import sys
import time

# Kalıcı kalibrasyon değerleri (bkz. kalibrasyon.yaml)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kalibrasyon import kalibrasyon

KAL = kalibrasyon('uart_sender_node')

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
        #
        # DİKKAT - ki ve kd'nin BİRİMİ DEĞİŞTİ (2026-08-17). Eskiden PID dt
        # kullanmıyordu: integral her mesajda +error, türev ise iki mesaj
        # arasındaki ham fark. Bu, kontrolcüyü örnekleme hızına bağımlı yapıyordu
        # ve şerit tespiti hızı sahneye göre değiştiği için (3-10 FPS arası)
        # kazançların etkisi sürekli kayıyordu. Ölçüm: türev teriminin genliği
        # 3 Hz'de 0.0164, 30 Hz'de 0.0025 - yani hız arttıkça damping kayboluyor
        # ve araç şerit merkezi etrafında salınıyordu.
        # Artık ki [1/s], kd [s] birimli ve çıkış örnekleme hızından bağımsız.
        # Buradaki varsayılanlar eski davranışın ~10 Hz'deki karşılığı:
        #   ki 0.2/örnek -> 2.0 /s      kd 0.2/örnek -> 0.02 s
        self.declare_parameter('kp', 1.0)
        # ki: kalıcı küçük sapmayı zamanla sıfırlar (araç ortalıyor ama tam
        # merkeze oturmuyorsa bunu artır; yavaş salınım başlarsa azalt)
        self.declare_parameter('ki', 2.0)
        self.declare_parameter('kd', 0.02)
        # İntegral teriminin direksiyona katkı SINIRI (radyan). Doğrudan radyan
        # olmasının sebebi: eskiden integralin kendisi sınırlanıyordu, o yüzden
        # sınırın direksiyona ne kadar etki ettiği ki'ye bağlıydı ve okunmuyordu.
        self.declare_parameter('i_limit', 0.06)
        # Türev filtresi (0-1). 30 Hz'de ham türev ölçüm gürültüsünü büyütüyor;
        # bu EMA katsayısı ne kadar küçükse türev o kadar yumuşak.
        self.declare_parameter('d_filter', 0.3)
        # Araç şeride doğru değil de ŞERİTTEN DIŞARI kırıyorsa bunun işaretini çevir
        # (2026-07-12: araç sürekli sağa kaçtığı için +1.0 -> -1.0 yapıldı)
        self.declare_parameter('steering_direction', -1.0)
        # TRİM: Araç düz gitmesi gerekirken yamuk gidiyorsa burayı ayarla.
        # Birim: derece (d komutuyla aynı). Araç SOLA çekiyorsa artır (+5, +10...),
        # SAĞA çekiyorsa azalt (-5, -10...). Merkez = 180 + trim olur.
        self.declare_parameter('steering_trim', KAL('steering_trim', 0))
        # --- KAVŞAK: ŞERİT KAYBOLUNCA YÖN TUTMA ------------------------------
        # Kavşakta şerit çizgileri biter. Şerit takibi girdisiz kalınca eski
        # sapma sönümleniyordu ama bu AÇIK DÖNGÜ: aracın o an nereye baktığını
        # bilmiyor, mekanik trim ya da kalan direksiyon açısı yüzünden savruluyor.
        # Odometri varken doğrusu şu: şeridi kaybettiğin ANDAKİ yönü hedef al ve
        # kavşağı o yönü koruyarak geç.
        self.declare_parameter('heading_hold', True)
        # Yön hatası başına direksiyon (rad direksiyon / rad yön hatası)
        self.declare_parameter('kp_heading', 1.2)
        # Bu süreden uzun süre şerit gelmezse uyar (kavşak bu kadar uzun sürmez;
        # sürüyorsa şerit tespiti bozulmuş demektir)
        self.declare_parameter('heading_hold_max_sec', 6.0)
        # KAVŞAK DÖNÜŞ AÇISI (derece). Mecburi yön levhası görülmüşse, şerit
        # kaybolduğunda hedef yön bu kadar kaydırılır. Her kavşak tam 90° olmaz;
        # pistte ölçüp ayarlayın.
        self.declare_parameter('turn_angle_deg', 90.0)

        # DÜZ BAŞLANGIÇ: Sistem açıldıktan sonra bu süre boyunca (saniye)
        # direksiyon merkezde tutulur, şerit takibi devreye girmez.
        # Kamera ve tespit otursun diye - başlangıçtaki sağa/sola çekmeyi önler.
        self.declare_parameter('straight_start_sec', 3.0)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.i_limit = self.get_parameter('i_limit').value
        self.d_filter = self.get_parameter('d_filter').value
        self.heading_hold = bool(self.get_parameter('heading_hold').value)
        self.kp_heading = float(self.get_parameter('kp_heading').value)
        self.heading_hold_max_sec = float(self.get_parameter('heading_hold_max_sec').value)
        self.turn_angle_deg = float(self.get_parameter('turn_angle_deg').value)
        self.steering_direction = self.get_parameter('steering_direction').value
        self.steering_trim = self.get_parameter('steering_trim').value
        self.straight_start_sec = self.get_parameter('straight_start_sec').value
        self.start_time = time.time()
        self.straight_phase_done = False

        self.prev_error = 0.0
        self.integral = 0.0
        self.d_filtered = 0.0
        self.last_pid_time = None
        self.max_steering_angle = 0.5

        # PID'i CANLI ayarlayabilmek için. Bu callback olmadan 'ros2 param set'
        # parametreyi değiştiriyor ama düğüm onu bir daha okumadığı için hiçbir
        # etkisi olmuyordu - yukarıdaki "kod değişmeden ayarlanabilir" ancak
        # bununla doğru.  ros2 param set /uart_sender_node kp 1.3
        self.add_on_set_parameters_callback(self._on_parameter_update)

        # Lateral deviation ve manuel ackermann komutları
        self.current_lateral_deviation = 0.0
        self.manual_speed = 0.0

        # Yön tutma durumu
        self.serit_gecerli = True
        self.guncel_yaw = None          # odometriden gelen yön (rad)
        self.hedef_yaw = None           # şeridi kaybettiğimiz andaki yön
        self.yon_tutma_basladi = None
        self.yon_tutma_uyarildi = False
        self.bekleyen_donus = 0         # -1 sola, 0 duz, +1 saga (levhadan)

        # Mix portu. Acil durdurma BATARYAYI keserek yapıldığı için Arduino
        # USB'den düşüyor ve /dev/serial/by-id/... yolu kayboluyor. Eskiden düğüm
        # bunu fark etmiyordu: batarya geri gelse bile komut gitmiyordu ve tüm
        # sistemi yeniden başlatmak gerekiyordu (~30 sn model yükleme).
        # Artık kopmayı görüp kendi kendine yeniden bağlanıyor.
        self.mix_serial = None
        self.port_hazir_zamani = 0.0     # Arduino açılışta reset atar, o kadar bekle
        self.baglanti_uyarildi = False
        self._portu_ac(ilk=True)
        # Bağlantı denetçisi: kopmuşsa 2 saniyede bir yeniden dener
        self.baglanti_timer = self.create_timer(2.0, self._baglanti_kontrol)

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

        # Kavşak: şerit geçerli mi + odometriden yön
        self.create_subscription(Bool, '/lane/valid', self.serit_gecerli_callback, 10)
        self.create_subscription(Odometry, '/zed2i/odom', self.odom_callback, 10)
        self.create_subscription(Int32, '/route/turn', self.donus_callback, 10)

        # Decision making node'undan speed verisi almak için subscriber ekle
        self.speed_sub = self.create_subscription(
            Float32,
            '/speed',
            self.speed_callback,
            10
        )

        self.get_logger().info('🦾 UART Gönderici (Tek Mix Port + PID Lateral Control) başlatıldı. Komut bekleniyor...')

    # Pistte canlı ayarlanabilen parametreler (öznitelik adlarıyla birebir aynı)
    LIVE_PARAMS = ('kp', 'ki', 'kd', 'i_limit', 'd_filter', 'steering_direction',
                   'steering_trim', 'straight_start_sec', 'heading_hold',
                   'kp_heading', 'heading_hold_max_sec', 'turn_angle_deg')

    def _on_parameter_update(self, params):
        for p in params:
            if p.name in self.LIVE_PARAMS:
                # Mevcut değerin tipini koru: steering_trim int olmalı, yoksa
                # porta 'd,180.0' gibi bozuk komut gider.
                setattr(self, p.name, type(getattr(self, p.name))(p.value))
                if p.name in ('kp', 'ki', 'kd'):
                    self.integral = 0.0   # kazanç değişince eski birikim anlamsız
                    self.d_filtered = 0.0
                self.get_logger().info(f'⚙️  {p.name} = {getattr(self, p.name)}')
        return SetParametersResult(successful=True)

    def _portu_ac(self, ilk=False):
        """Mix portunu açmayı dener. Başarılıysa True.

        Arduino port açılınca RESET atıyor ve ~2 sn boyunca komut kabul etmiyor;
        bu yüzden 'hazır zamanı' işaretlenir ve o ana kadar komut gönderilmez.
        (Eskiden burada time.sleep(3) vardı - timer içinde kullanılamaz, düğümü
        kilitler.)
        """
        try:
            self.mix_serial = serial.Serial(self.MIX_PORT, self.BAUD_RATE, timeout=0.1)
            self.port_hazir_zamani = time.time() + 3.0
            self.baglanti_uyarildi = False
            # Güç kesilip gelmişse eski PID birikimi anlamsız
            self.integral = 0.0
            self.d_filtered = 0.0
            self.prev_error = 0.0
            self.last_pid_time = None
            # Araç yeni açıldı: düz başlangıç fazı baştan işlesin
            self.start_time = time.time()
            self.straight_phase_done = False
            self.get_logger().info('✅ Mix portu açıldı (Hız + Direksiyon + Fren).'
                                   if ilk else
                                   '🔌 Mix portu YENİDEN bağlandı - sistem çalışmaya devam ediyor.')
            return True
        except Exception as e:
            self.mix_serial = None
            if not self.baglanti_uyarildi:
                self.baglanti_uyarildi = True    # her 2 sn'de bir log basmasın
                self.get_logger().error(
                    f'❌ Mix portu açılamadı: {e}\n'
                    f'   Bağlantı bekleniyor, port gelince kendiliğinden bağlanacak.')
            return False

    def _baglanti_kontrol(self):
        """Port kopmuşsa yeniden bağlanmayı dener (2 sn'de bir)."""
        if self.mix_serial is None or not self.mix_serial.is_open:
            self._portu_ac()

    def _baglantiyi_dusur(self, hata):
        """Yazma hatasında portu kapat, yeniden bağlanma döngüsüne bırak."""
        self.get_logger().warn(f'🔌 Mix portu koptu ({hata}). Yeniden bağlanılacak.')
        try:
            if self.mix_serial:
                self.mix_serial.close()
        except Exception:
            pass
        self.mix_serial = None
        self.baglanti_uyarildi = True

    def send_command(self, prefix, value):
        """Mix porta 'harf,değer' formatında komut gönderir (örn: h,1  f,0  d,127).
        DİKKAT: Sonlandırıcı yok - komutlar porta ayraçsız art arda yazılır."""
        if not (self.mix_serial and self.mix_serial.is_open):
            return
        if time.time() < self.port_hazir_zamani:
            return          # Arduino henüz reset'ten çıkmadı
        command = f'{prefix},{value}'
        try:
            self.mix_serial.write(command.encode('utf-8'))
        except Exception as e:
            # Batarya kesildiğinde yazma burada patlıyor; bunu yakalamazsak
            # düğüm istisnayla düşer ve sistemi yeniden başlatmak gerekir.
            self._baglantiyi_dusur(e)

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
        PID kontrolör kullanarak lateral deviation'ı direksiyon açısına dönüştürür.

        ZAMAN TABANLI: integral error*dt ile birikir, türev de/dt olarak alınır.
        Böylece çıkış örnekleme hızından bağımsız; şerit tespiti 5 FPS'te de
        30 FPS'te de aynı direksiyonu üretir. (Eski sürüm dt kullanmadığı için
        FPS düştükçe damping artıyor, yükseldikçe kayboluyordu.)
        """
        error = lateral_deviation

        # Mesajlar arası gerçek süre. Sınır: ilk mesaj, takılma ya da kare
        # atlaması türevi/integrali patlatmasın.
        now = time.time()
        if self.last_pid_time is None:
            dt = 0.1
        else:
            dt = min(max(now - self.last_pid_time, 0.005), 0.5)
        self.last_pid_time = now

        p_term = self.kp * error

        # Türev: gürültü /dt ile büyüdüğü için EMA'dan geçirilir
        d_raw = self.kd * (error - self.prev_error) / dt
        self.d_filtered = (1.0 - self.d_filter) * self.d_filtered + self.d_filter * d_raw
        d_term = self.d_filtered
        self.prev_error = error

        # İntegral: katkısı radyan olarak sınırlı (i_limit)
        self.integral += error * dt
        i_term = max(-self.i_limit, min(self.ki * self.integral, self.i_limit))

        pid_output = self.steering_direction * -(p_term + i_term + d_term)
        steering_angle = max(-self.max_steering_angle,
                             min(pid_output, self.max_steering_angle))

        # ANTI-WINDUP: direksiyon zaten tavana dayandıysa integrali büyütmeye
        # devam etmek sadece geri dönüşü geciktirir (araç merkeze dönerken
        # birikmiş integral onu öteye taşırıp salınım başlatıyor). Doyma varsa
        # birikmeyi geri al.
        if abs(pid_output) > self.max_steering_angle:
            self.integral -= error * dt

        return steering_angle

    @staticmethod
    def _yaw(q):
        """Kuaterniyondan sapma açısı (rad), Z ekseni etrafında."""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_callback(self, msg: Odometry):
        self.guncel_yaw = self._yaw(msg.pose.pose.orientation)

    def donus_callback(self, msg: Int32):
        """Karar alma düğümünden gelen MECBURİ DÖNÜŞ emri (-1 sol, 0 düz, +1 sağ).

        Koridor tercihinden (/route/preferred_side) farklı: bu gerçek bir
        manevra. Şerit kaybolduğu anda hedef yön bu kadar kaydırılır.
        """
        yeni = int(msg.data)
        if yeni != self.bekleyen_donus:
            yon = {-1: 'SOLA', 0: 'DÜZ', 1: 'SAĞA'}.get(yeni, '?')
            self.get_logger().info(f'↩️  Kavşakta yapılacak: {yon}')
        self.bekleyen_donus = yeni

    def serit_gecerli_callback(self, msg: Bool):
        """Şerit tespiti ölçüm yapabiliyor mu. Kavşakta False olur."""
        yeni = bool(msg.data)
        if yeni == self.serit_gecerli:
            return

        if not yeni:
            # ŞERİT KAYBOLDU: o anki yönü hedef al ve kavşağı bu yönle geç.
            if self.guncel_yaw is None:
                self.hedef_yaw = None
            else:
                # Mecburi yön levhası varsa hedef yönü o kadar kaydır; yoksa
                # giriş yönünü koru (düz geçiş).
                kayma = math.radians(self.turn_angle_deg) * self.bekleyen_donus
                ham = self.guncel_yaw - kayma      # saat yönü: sağa dönüş yaw'ı azaltır
                self.hedef_yaw = math.atan2(math.sin(ham), math.cos(ham))
            self.yon_tutma_basladi = time.time()
            self.yon_tutma_uyarildi = False
            if self.hedef_yaw is None:
                self.get_logger().warn(
                    '⚠️  Şerit kayboldu ama ODOMETRİ YOK - yön tutulamıyor, '
                    'eski davranışa düşülüyor (araç savrulabilir).')
            else:
                self.get_logger().info(
                    f'🧭 Şerit kayboldu (kavşak?) - yön tutma: '
                    f'{math.degrees(self.hedef_yaw):+.0f}°')
        else:
            if self.yon_tutma_basladi is not None:
                sure = time.time() - self.yon_tutma_basladi
                self.get_logger().info(f'🛣️ Şerit geri geldi ({sure:.1f} sn sonra) - '
                                       f'şerit takibi devrede.')
            if self.bekleyen_donus:
                self.get_logger().info('↩️  Dönüş tamamlandı, emir tüketildi.')
                self.bekleyen_donus = 0
            self.hedef_yaw = None
            self.yon_tutma_basladi = None
            # Kavşak boyunca biriken PID durumu artık geçersiz
            self.integral = 0.0
            self.d_filtered = 0.0
            # prev_error'ı SIFIRLAMA: sıfırlarsak ilk türev adımı 0 -> sapma
            # sıçraması görüp direksiyonu tavana vuruyor (ölçüm: d,339). Son
            # bilinen sapmayla başlat ki türev sıfırdan başlasın.
            self.prev_error = self.current_lateral_deviation
            self.last_pid_time = None

        self.serit_gecerli = yeni

    def yon_tutma_direksiyonu(self):
        """Şerit yokken yönü koruyacak direksiyon açısı. Yapamıyorsa None.

        Eski davranış sapmayı 0'a sönümlüyordu; bu AÇIK DÖNGÜ - aracın nereye
        baktığını bilmediği için mekanik trim ya da kalan direksiyon açısı
        yüzünden kavşakta savruluyordu. Burada odometriden gelen yön kapalı
        döngüde tutuluyor.
        """
        if not self.heading_hold:
            return None
        if self.hedef_yaw is None or self.guncel_yaw is None:
            return None

        if (self.yon_tutma_basladi is not None
                and not self.yon_tutma_uyarildi
                and time.time() - self.yon_tutma_basladi > self.heading_hold_max_sec):
            self.yon_tutma_uyarildi = True
            self.get_logger().warn(
                f'⚠️  {self.heading_hold_max_sec:.0f} saniyedir şerit yok. Kavşak bu '
                f'kadar sürmez - şerit tespiti bozulmuş olabilir. Yön tutma sürüyor.')

        # Açı farkını -pi..pi aralığına indir
        hata = math.atan2(math.sin(self.hedef_yaw - self.guncel_yaw),
                          math.cos(self.hedef_yaw - self.guncel_yaw))
        aci = self.kp_heading * hata
        return max(-self.max_steering_angle, min(aci, self.max_steering_angle))

    def lateral_deviation_callback(self, msg: Float32):
        """Lateral deviation mesajını alır ve direksiyon kontrolü yapar.
        DİREKSİYONUN TEK SAHİBİ BU FONKSİYON - başka hiçbir yer d komutu göndermez."""
        self.current_lateral_deviation = msg.data

        # DÜZ BAŞLANGIÇ FAZI: ilk straight_start_sec saniye direksiyon merkezde
        # tutulur, PID devreye girmez (kamera/tespit otursun, araç düz kalksın).
        if time.time() - self.start_time < self.straight_start_sec:
            self.integral = 0.0
            self.prev_error = 0.0
            self.d_filtered = 0.0
            # Faz bitince ilk türev adımı bu bekleme süresini dt sanmasın
            self.last_pid_time = None
            self.send_command('d', self.angle_to_byte(0.0))
            return
        if not self.straight_phase_done:
            self.straight_phase_done = True
            self.get_logger().info('🛣️ Düz başlangıç fazı bitti - şerit takibi devrede.')

        # KAVŞAK: şerit ölçülemiyorsa PID'in girdisi tahmin olur. Odometri varsa
        # onun yerine yönü koru; yoksa eski davranışa (sönümlenen sapma) düş.
        yon_acisi = None if self.serit_gecerli else self.yon_tutma_direksiyonu()

        if yon_acisi is not None:
            steering_angle = yon_acisi
            angle_byte = self.angle_to_byte(steering_angle)
            self.send_command('d', angle_byte)
            hata_derece = math.degrees(math.atan2(
                math.sin(self.hedef_yaw - self.guncel_yaw),
                math.cos(self.hedef_yaw - self.guncel_yaw)))
            self.get_logger().info(
                f'🧭 YÖN TUTMA (şerit yok) | Hata: {hata_derece:+.1f}° | '
                f'Steering: {steering_angle:.3f} rad | Byte: d,{angle_byte}')
            return

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

    def guvenli_dur(self):
        """Motorları güvenli duruma getirip portu kapatır.

        Eskiden bu SADECE __del__ içindeydi. __del__'in ne zaman (hatta çalışıp
        çalışmayacağı) Python'un çöp toplayıcısına bağlıdır; süreç dışarıdan
        sonlandırıldığında hiç çağrılmayabilir. O durumda araç SON ALDIĞI hız
        komutuyla ilerlemeye devam eder. Artık kapanışta AÇIKÇA çağrılıyor.
        Tekrar çağrılması zararsız (port kapalıysa hiçbir şey yapmaz).
        """
        if getattr(self, 'mix_serial', None) and self.mix_serial.is_open:
            try:
                self.send_command('h', 0)      # hızı kes
                self.send_command('f', 1)      # freni uygula
                self.send_command('d', self.angle_to_byte(0.0))  # direksiyonu merkeze
            finally:
                self.mix_serial.close()

    def destroy_node(self):
        self.guvenli_dur()
        super().destroy_node()

    def __del__(self):
        # Son çare: destroy_node çağrılmadıysa yine de aracı durdurmayı dene.
        try:
            self.guvenli_dur()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = UartSenderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Launcher kritik düğüm ölünce hepsini kapatıyor; bu normal kapanış.
        pass
    finally:
        # destroy_node aracı durdurup portu kapatır (bkz. guvenli_dur)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
