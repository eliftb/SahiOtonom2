#!/usr/bin/env python3
import math
import os
import re
import serial
import sys
import time

# ROS bağımlılıkları İSTEĞE BAĞLI yükleniyor. Sebep: dosyanın sonundaki
# kalibrasyon modu ROS'suz çalışır ve tam da ROS kaynaklanmamış bir terminalde
# (ya da düğümler kapalıyken) kullanılır. Bunlar koşulsuz import edilirse dosya
# daha 2. satırda ImportError ile ölür ve "--kalibrasyon" hiç çalışmaz.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from ackermann_msgs.msg import AckermannDrive
    from std_msgs.msg import Float32, Bool, Int32
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import SetParametersResult
    ROS_HATASI = None
except ImportError as _hata:
    # Sınıf gövdesi yine de tanımlanabilsin diye yer tutucular (metot imzaları
    # bu adları tanım anında çözer). Düğüm olarak çalıştırılırsa main() açık bir
    # hata verir; kalibrasyon modu bunların hiçbirine dokunmaz.
    ROS_HATASI = _hata
    rclpy = None
    Node = object
    ExternalShutdownException = Exception
    AckermannDrive = Float32 = Bool = Int32 = Odometry = SetParametersResult = object

# --- DİREKSİYON BYTE EŞLEMESİ ------------------------------------------------
# Düğümden BAĞIMSIZ tutuluyor çünkü dosyanın sonundaki kalibrasyon modu da aynı
# eşlemeyi kullanıyor. Ölçüm ile sürüş aynı koddan geçmezse ikisi zamanla
# ayrışır ve sehpada doğruladığın merkez pistte başka bir yere düşer.
BYTE_MERKEZ = 180        # firmware protokolü: d,0-360. NOMİNAL merkez.
BYTE_UST = 360


def merkez_byte(steering_trim):
    """Tekerlerin GERÇEKTEN düz olduğu byte (kalibrasyonla ölçülür)."""
    return int(BYTE_MERKEZ + steering_trim)


def yari_aralik(steering_trim):
    """Merkezden iki yana da gidilebilen byte miktarı.

    Trim merkezi kaydırınca iki tarafın alanı eşitsizleşir (merkez 150 ise solda
    150, sağda 210 birim kalır). Küçük tarafla sınırlamazsak araç sağa, sola
    döndüğünden sert döner ve sebebi kırpmanın içinde görünmez kalır.
    """
    merkez = merkez_byte(steering_trim)
    return max(0, min(merkez, BYTE_UST - merkez))


def aci_to_byte(angle_rad, steering_trim, max_steering_angle):
    """Direksiyon açısını firmware'in d aralığına çevirir.

    Merkez = 180 + steering_trim; ±max_steering_angle o merkezden itibaren
    ±yari_aralik() birime düşer.

    ÖLÇEK max_steering_angle'DAN TÜRETİLİYOR (2026-08-18). Eskiden ±0.5 rad ↔
    0-360 eşlemesi gömülüydü ama doyum sınırı ayrı bir değişkendi; ikisi
    tesadüfen 0.5'te eşit olduğu için sorun görünmüyordu. Gerçek kilit ölçülüp
    max_steering_angle 0.35'e çekilseydi byte ancak 306'ya çıkacak, kilidin son
    %15'i erişilemez olacaktı.
    """
    if max_steering_angle <= 0.0:
        return merkez_byte(steering_trim)
    angle_rad = max(-max_steering_angle, min(angle_rad, max_steering_angle))
    value = int(round(merkez_byte(steering_trim)
                      + angle_rad / max_steering_angle * yari_aralik(steering_trim)))
    return max(0, min(value, BYTE_UST))


class UartSenderNode(Node):
    def __init__(self):
        super().__init__('uart_sender_node')

        # Tek port (MIX PORT) - hız, direksiyon ve fren aynı porttan gönderilir
        # Arduino (CH340 çipli) sabit kimlik yolu - takma sırasından etkilenmez.
        # NOT: brltty servisi CH340'ı çaldığı için mask'landı (2026-07-12).
        self.declare_parameter('mix_port',
                               '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud_rate', 38400)

        # Her komutun sonuna '\n' konsun mu. AÇIK olmalı: kapalıyken komutlar
        # porta ayraçsız akıyor ve firmware'in parseInt'i bir sonraki komutun
        # harfini yutuyor (bkz. send_command). Kapatma seçeneği sadece
        # firmware'in sonlandırıcıyı sevmediği ortaya çıkarsa diye var.
        self.declare_parameter('satir_sonu', True)

        self.MIX_PORT = self.get_parameter('mix_port').value
        self.BAUD_RATE = self.get_parameter('baud_rate').value
        self.satir_sonu = bool(self.get_parameter('satir_sonu').value)

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
        #
        # DEĞERLER ARTIK BURADA SABİT (kalibrasyon.yaml 2026-08-18'de kaldırıldı).
        # Pistte yeniden başlatmadan denemek için:
        #     ros2 param set /uart_sender_node kp 1.0
        # Beğendiğin değeri BURAYA YAZ - yoksa sistem kapanınca kaybolur.
        #
        # kp 0.8 (PİSTTE DOĞRULANMADI): benzetimden gelen 0.4 fazla çekingen
        # kaldı - sapma -0.22 iken direksiyona sadece 0.09 rad, yani tam
        # kilidin ~%20'si gidiyordu (byte 141 / merkez 180) ve teker gözle
        # "dönmüyor" görünüyordu. Eski 1.0 ise sönümsüz olduğu için
        # salınıyordu; aşağıdaki kd 0.3 ile birlikte 0.8 hem düzeltir hem
        # savurmaz. Salınırsa 0.6'ya çek, hâlâ tembelse 1.0'a çıkar.
        # ÖLÇÜME DAYALI (2026-08-19). kp 0.8 ile 0.9 m yanal hata direksiyonu
        # 26°'ye götürüyordu; oysa pistin gerçek virajı (R=4.9 m, odometriden
        # ölçüldü) sadece 3-5° istiyor. Araç şeridi tek hamlede geçip karşı
        # tarafa aşıyor, sonra geri geliyor - loglardaki salınım buydu.
        # 0.3 + hata ölçeği 4.0 ile 0.9 m hata ≈ 4° veriyor.
        # KANITLANMIŞ DEĞER (2026-08-19 turu). Bu kazançla sapma logda
        # -0.367 -> -0.226 -> -0.154 -> -0.078 -> 0.000 diye DÜZGÜNCE
        # yakınsadı ve direksiyon merkeze döndü. 0.8 ile aynı araç zikzak
        # çizip tam kilide dayanıyordu (Dev 1.000, Byte d,100).
        # SABAHKİ DEĞERE DÖNÜLDÜ (2026-08-19 akşam). Aşağıdaki 0.3
        # gerekçesi bir hatayı içeriyordu: 'pistin virajı 3-5° istiyor' doğru
        # ama o FEEDFORWARD - virajı TAKİP etmek için gereken açı. Yanal HATAYI
        # DÜZELTMEK için gereken açı ayrı bir şey. 0.9 m hataya da 4° verilince
        # araç hatayı virajın tamamından daha yavaş kapatır: şeritten çıkar ve
        # geri gelemez. Pistte görülen buydu - 0.3 ile şerit takibi durdu.
        # 0.8 + ölçek 2.5 ile araç zikzak çizebilir AMA şeritte kalır. Zikzak
        # varsa doğru yön 0.3'e inmek değil, aradan (0.5-0.6) denemektir.
        self.declare_parameter('kp', 0.8)
        # ki KAPALI. p ve i birlikte iterken frenleyen terim yoktu, araç şerit
        # merkezi etrafında salınıyordu. Kalıcı bir yana çekme varsa önce
        # steering_trim'e bakın; ki en son açılır (0.2 gibi küçük başlayın).
        # İNTEGRAL AÇILDI (2026-08-19). Sadece P ile SABİT bir bozucu asla
        # sıfırlanmaz: araç mekanik olarak bir yana çekiyorsa (direksiyon
        # merkezi tam ölçülmemişse, teker düzeni yamuksa, kamera ofseti varsa)
        # kontrol kalıcı bir sapmada dengeye oturur ve araç sürekli o tarafta
        # gider. Pistte tam bu görüldü: araç ısrarla SOL şeride kaçıyordu.
        # İntegral bu sabit farkı zamanla toplayıp kendi kapatır.
        # Anti-windup ve i_limit zaten var; salınım başlarsa önce bunu düşürün.
        # İNTEGRAL KAPALI KALSIN. Açıldığında ('full sola' turu) birikip
        # direksiyonu kilide dayadı. Sabit yanlılık varsa önce steering_trim
        # ölçülmeli; integral onu maskelemek için doğru araç değil.
        self.declare_parameter('ki', 0.0)
        # kd [s] - sönümleme. Salınım başlarsa artırın, tepki geç kalırsa azaltın.
        # TÜREV DÜŞÜRÜLDÜ. Ölçüm kare kare zıplıyor (çizgi kilidi değişiyor);
        # 20 Hz'de kd 0.3 bu zıplamayı direksiyona sıçrama olarak geçiriyordu.
        # SABAHKİ DEĞERE DÖNÜLDÜ. Yukarıdaki 'ölçüm zıplıyor' gözlemi geçerli;
        # zıplama sürüyorsa çözüm kd'yi kısmak DEĞİL d_filter'ı artırmaktır
        # (kd hatayı kapatan sönümü de birlikte götürüyor).
        self.declare_parameter('kd', 0.3)
        # İntegral teriminin direksiyona katkı SINIRI (radyan). Doğrudan radyan
        # olmasının sebebi: eskiden integralin kendisi sınırlanıyordu, o yüzden
        # sınırın direksiyona ne kadar etki ettiği ki'ye bağlıydı ve okunmuyordu.
        # İntegralin katkı sınırı (rad). 0.06 = 3.4° idi; ölçülmemiş bir
        # direksiyon merkezini kapatmaya yetmiyordu. 0.12 = ~7°.
        self.declare_parameter('i_limit', 0.06)
        # Türev filtresi (0-1). 30 Hz'de ham türev ölçüm gürültüsünü büyütüyor;
        # bu EMA katsayısı ne kadar küçükse türev o kadar yumuşak.
        self.declare_parameter('d_filter', 0.3)
        # Araç şeride doğru değil de ŞERİTTEN DIŞARI kırıyorsa bunun işaretini çevir
        # (2026-07-12: araç sürekli sağa kaçtığı için +1.0 -> -1.0 yapıldı)
        # FİZİKSEL EŞLEME - PİSTTE GÖZLE DOĞRULANDI (2026-08-19):
        #     byte > 230  =  teker SAĞA
        #     byte < 230  =  teker SOLA
        # Bu değer sapmayı byte'a çeviren İŞARETTİR. -1.0 ile 'sapma > 0'
        # (araç şeridin solunda, sağa kırmalı) byte'ı 230'un ÜSTÜNE çıkarır.
        #
        # BİR KEZ +1.0 DENENDİ VE TERS ÇIKTI (araç sağa kırması gerekirken
        # sola kırdı). Bir daha değiştirmeden önce şu ölçümü yapın - 30 sn
        # sürer ve şerit tespitinden tamamen bağımsızdır:
        #     python3 Haberlesme/uart_sender_node3.py --kalibrasyon --tarama
        # d,0 gönderildiğinde teker SAĞA dönüyorsa bu değer +1.0 olmalı.
        self.declare_parameter('steering_direction', -1.0)
        # TRİM: Tekerlerin GERÇEKTEN düz olduğu byte ile yazılımın varsaydığı
        # 180 arasındaki fark. Birim: d komutunun birimi. Araç SOLA çekiyorsa
        # artır (+5, +10...), SAĞA çekiyorsa azalt (-5, -10...). Merkez = 180+trim.
        #
        # ÖLÇÜLDÜ (2026-08-18, sehpada): tekerler byte 230'da düz, yani merkez
        # 180 DEĞİL. Aranan arıza buydu - yazılım "hafif sola kırıyorum" (byte
        # 158-165) derken teker gerçek merkezin ~70 birim solunda duruyordu ve
        # merkeze dönüş komutu (180) bile 50 birim soldaydı.
        #
        # BEDELİ: merkez 230 olunca sağda sadece 130 birim yer kalıyor, iki taraf
        # simetrik tutulduğu için kullanılabilir aralık ±180'den ±130'a düşüyor
        # (yetkinin %28'i). Kalıcı çözüm MEKANİK: servo kolunu/çubuğu ortalayıp
        # bu sayıyı birkaç birime indirmek. Ondan sonra tekrar ölçün:
        #     python3 uart_sender_node3.py --kalibrasyon
        self.declare_parameter('steering_trim', 50)
        # TAM KİLİT (rad). PID çıkışı buraya doyar VE angle_to_byte'ın ölçeği
        # budur: ±max_steering_angle, merkezden itibaren kullanılabilir byte
        # aralığının uçlarına düşer.
        #
        # ÖLÇÜLMEDİ. 0.5 rad (28.6°) bir varsayım; çoğu ölçekli araçta gerçek
        # kilit ±18-22°'dir. Yanlışsa logdaki "rad" değerleri fiziksel açı
        # değildir ve kp'yi ayarlamak kör atış olur. Ölçümü:
        #     python3 uart_sender_node3.py --kalibrasyon --kilit --trim <ölçülen>
        self.declare_parameter('max_steering_angle', 0.5)
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
        # HAREKET ETMEDEN DİREKSİYON KIRMASIN. Araç dururken direksiyonu
        # çevirmek hem işe yaramıyor (duran tekerlek yön değiştirmez, sadece
        # yerinde sürtünür) hem servoyu ve ön düzeni zorluyor. Sıra şu olmalı:
        # önce gaz -> araç gerçekten ilerlemeye başlasın -> sonra şerit takibi.
        # Odometri bu kadar metre yol gördüğünde direksiyon devreye girer.
        # GAZ DEĞERİ. Yazılım 'ilerle' derken porta 'h,<bu değer>' yazar.
        # 1 idi ve firmware bunu bir HIZ SEVİYESİ olarak okuyorsa (0-255 gibi)
        # 1 neredeyse sıfır gaz demek: motor boşta döner ama araç yükü
        # kaldıramaz - pistte tam bu görüldü (motor dönüyor, araç ilerlemiyor).
        # Firmware sadece 0/1 bekliyorsa 1 doğru değerdir ve değiştirmeye gerek
        # yoktur. .ino elimizde olmadığı için PİSTTE DENENİR:
        #     ros2 param set /uart_sender_node hiz_degeri 50
        #     ros2 param set /uart_sender_node hiz_degeri 150
        # Araç hangi değerde kalkıyorsa o değeri buraya yazın.
        self.declare_parameter('hiz_degeri', 1)
        # GAZ TEKRAR HIZI (Hz). Gaz komutu artık /speed mesajına DEĞİL, sabit
        # bir zamanlayıcıya bağlı. Sebebi: /speed karar düğümünden geldiği
        # sürece gidiyordu; düğüm yavaşlar, mesaj gecikir ya da yayın seyrelirse
        # gaz akışı da seyreliyordu. Firmware'de zaman aşımı varsa (komut
        # gelmeyince motoru kesen tür) bu doğrudan gaz kesintisi demek.
        # 0 yapılırsa zamanlayıcı kapanır, eski davranışa dönülür.
        self.declare_parameter('gaz_tekrar_hz', 10.0)
        # AÇILIŞTA GAZ (2026-08-19, istek üzerine).
        # Eskiden düğüm _son_hiz=0 / _son_fren=1 ile başlıyordu: ilk /speed
        # mesajı gelene kadar porta FREN yazılıyordu. Bu değer True iken düğüm
        # gaz basılı başlar, yani gaz porta giden İLK komut olur - direksiyon
        # komutundan da önce (direksiyon ancak şerit mesajı gelince gönderilir).
        #
        # DİKKAT - NE ZAMAN KAPATILMALI: engel/ışık/levha frenlerinin HEPSİ
        # karar düğümünde yaşıyor. launch_all_nodes.py bu düğümü EN SON
        # başlattığı için normal açılışta karar düğümü zaten ayakta ve o
        # korumalar aktif. Ama bu dosya TEK BAŞINA çalıştırılırsa araç hiçbir
        # koruma olmadan gaza basar. Sehpa/tek başına test için:
        #     ros2 param set /uart_sender_node acilista_gaz false
        self.declare_parameter('acilista_gaz', True)
        # DİREKSİYON KOMUT HIZI (Hz). Şerit düğümü ~20 kare/sn yayınlıyor ve
        # her karede bir 'd' komutu gidiyordu. Gaz komutlarıyla birlikte porta
        # yığılan trafik Arduino'nun giriş tamponunu taşırabilir; taşınca
        # komutlar bozulur ve gaz kesik alınır. 0 = sınırsız (her mesajda).
        self.declare_parameter('direksiyon_hz', 20.0)
        self.declare_parameter('hareket_esigi_m', 0.15)
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
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.straight_start_sec = self.get_parameter('straight_start_sec').value
        self.hiz_degeri = int(self.get_parameter('hiz_degeri').value)
        self.gaz_tekrar_hz = float(self.get_parameter('gaz_tekrar_hz').value)
        self.direksiyon_hz = float(self.get_parameter('direksiyon_hz').value)
        self._son_direksiyon_zamani = 0.0
        # Zamanlayıcının tekrarlayacağı son gaz/fren değeri.
        # acilista_gaz True ise gaz basılı, fren serbest başlar (bkz. yukarısı).
        self.acilista_gaz = bool(self.get_parameter('acilista_gaz').value)
        self._son_hiz = self.hiz_degeri if self.acilista_gaz else 0
        self._son_fren = 0 if self.acilista_gaz else 1
        self.hareket_esigi_m = float(self.get_parameter('hareket_esigi_m').value)
        self._ilk_konum = None
        self._hareket_basladi = False
        # DİREKSİYON YÖNÜ OTOMATİK DENETİMİ (bkz. odom_callback)
        self._son_direksiyon_acisi = 0.0
        self._yon_onceki = None          # (zaman, yaw)
        self._yon_oy = 0                 # + ters, - dogru
        self._yon_uyarildi = False
        self.start_time = time.time()
        self.straight_phase_done = False

        self.prev_error = 0.0
        self.integral = 0.0
        self.d_filtered = 0.0
        self.last_pid_time = None

        # PID'i CANLI ayarlayabilmek için. Bu callback olmadan 'ros2 param set'
        # parametreyi değiştiriyor ama düğüm onu bir daha okumadığı için hiçbir
        # etkisi olmuyordu - yukarıdaki "kod değişmeden ayarlanabilir" ancak
        # bununla doğru.  ros2 param set /uart_sender_node kp 1.3
        self.add_on_set_parameters_callback(self._on_parameter_update)

        # Lateral deviation ve manuel ackermann komutları
        # Porta ulaşmayan komutların sayacı (bkz. send_command)
        self._dusen_komut = 0
        self._dusen_uyari = 0.0
        self.current_lateral_deviation = 0.0
        self.manual_speed = 0.0

        # Yön tutma durumu
        self.serit_gecerli = True
        self.guncel_yaw = None          # odometriden gelen yön (rad)
        self.hedef_yaw = None           # şeridi kaybettiğimiz andaki yön
        self.yon_tutma_basladi = None
        self.yon_tutma_uyarildi = False
        self.bekleyen_donus = 0         # -1 sola, 0 duz, +1 saga (levhadan)
        # ZATEN YAPILMIŞ DÖNÜŞ. Karar düğümü emri direction_memory_sec (12 sn)
        # boyunca yayınlamaya devam ediyor ve dönüşün yapıldığını bilmiyor:
        # şerit geri gelince bekleyen_donus'u sıfırlıyorduk, bir sonraki
        # /route/turn mesajı onu HEMEN geri kuruyordu. Kavşağın hemen ardında
        # şerit bir kez daha kaybolsa (viraj, gölge, boyasız bölüm) araç aynı
        # levha yüzünden İKİNCİ bir 90° dönüş yapardı. Mandal, emir 0'a
        # dönene kadar aynı yönün yeniden kurulmasını engeller.
        self.tuketilen_donus = 0

        # Mix portu. Acil durdurma BATARYAYI keserek yapıldığı için Arduino
        # USB'den düşüyor ve /dev/serial/by-id/... yolu kayboluyor. Eskiden düğüm
        # bunu fark etmiyordu: batarya geri gelse bile komut gitmiyordu ve tüm
        # sistemi yeniden başlatmak gerekiyordu (~30 sn model yükleme).
        # Artık kopmayı görüp kendi kendine yeniden bağlanıyor.
        self.mix_serial = None
        self.port_hazir_zamani = 0.0     # Arduino açılışta reset atar, o kadar bekle
        self.baglanti_uyarildi = False
        self._portu_ac(ilk=True)
        # GAZ İLK KOMUT OLSUN. Zamanlayıcıyı beklemeden burada yazıyoruz ki
        # porta giden ilk bayt gaz olsun. Arduino henüz reset'ten çıkmadıysa
        # send_command sessizce düşer - sorun değil, aşağıdaki 10 Hz kalp
        # atışı hazır olur olmaz aynı değeri tekrar yazar.
        if self.acilista_gaz:
            self.send_command('h', self._son_hiz)
            self.send_command('f', self._son_fren)
            self.get_logger().warn(
                f'🏁 AÇILIŞTA GAZ AÇIK - h,{self._son_hiz} f,{self._son_fren} '
                f'gönderildi. Araç ilk komutla hareket edebilir.')
        # Bağlantı denetçisi: kopmuşsa 2 saniyede bir yeniden dener
        self.baglanti_timer = self.create_timer(2.0, self._baglanti_kontrol)
        # GAZ KALP ATIŞI: son hız/fren değerini sabit hızda tekrarlar.
        if self.gaz_tekrar_hz > 0:
            self.gaz_timer = self.create_timer(1.0 / self.gaz_tekrar_hz,
                                               self._gaz_tekrarla)

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
                   'steering_trim', 'max_steering_angle', 'straight_start_sec',
                   'heading_hold', 'kp_heading', 'heading_hold_max_sec',
                   'turn_angle_deg', 'satir_sonu', 'hareket_esigi_m',
                   'hiz_degeri', 'gaz_tekrar_hz', 'direksiyon_hz', 'acilista_gaz')

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
        """Mix porta 'harf,değer\\n' formatında komut gönderir (örn: h,1  f,0  d,127).

        SONLANDIRICI ZORUNLU (2026-08-18'de eklendi - "hız komutu gidiyor ama
        araç gitmiyor" arızasının sebebi buydu). Eskiden komutlar ayraçsız
        yazılıyordu, yani porta kesintisiz şu akış gidiyordu:

            d,230d,230h,1f,0d,230...

        Firmware Serial.parseInt() kullanıyorsa sayıyı BİTİREN karakteri okuyup
        ATAR. Yukarıdaki akışta 230'u bitiren karakter bir sonraki komutun
        HARFİDİR: 'd' sayısını okuyan parseInt arkasından gelen 'h'yi yutuyor,
        geride kalan ',1' harfsiz kaldığı için yok sayılıyordu. Direksiyon (d)
        ve fren (f) geçiyor, HIZ (h) hiç ulaşmıyordu - loglarda 'h,1' görünmesine
        rağmen araç hareket etmiyordu.

        Kalibrasyon modunda gizli kalmasının sebebi: orada komutlar tuş tuş,
        aralarında saniyeler geçerek gidiyor. Akış durunca parseInt kendi
        zaman aşımıyla sayıyı bitiriyor, sonraki harfi yutmuyor. Arıza SADECE
        sürüşteki 20 Hz kesintisiz akışta ortaya çıkıyor.

        Firmware'in \\n'i sevmediği ortaya çıkarsa pistte kapatılabilir:
            ros2 param set /uart_sender_node satir_sonu false
        """
        # SESSİZ DÜŞME GÖRÜNÜR OLMALI. speed_callback / lateral_callback log
        # satırını send_command'dan SONRA basıyor ve bu fonksiyon port kapalıyken
        # sessizce dönüyordu: loglarda kesintisiz "Sinyal: h,1 f,0" görünürken
        # porta TEK BİR BAYT gitmemiş olabiliyordu. Araç durduğu hâlde loglar
        # dolu akınca arıza yanlış yerde aranıyor.
        if not (self.mix_serial and self.mix_serial.is_open):
            self._dusen_komut += 1
            simdi = time.time()
            if simdi - self._dusen_uyari > 2.0:
                self._dusen_uyari = simdi
                self.get_logger().error(
                    f'⛔ PORT KAPALI - komutlar gönderilmiyor ({self._dusen_komut} '
                    f'komut düştü). Loglardaki "Sinyal: h,1" satırları porta '
                    f'ULAŞMIYOR. Arduino bağlı mı: ls /dev/serial/by-id/')
            return
        if time.time() < self.port_hazir_zamani:
            return          # Arduino henüz reset'ten çıkmadı
        command = f'{prefix},{value}' + ('\n' if self.satir_sonu else '')
        try:
            self.mix_serial.write(command.encode('utf-8'))
            # flush: komut çekirdek arabelleğinde bekleyip sonrakiyle aynı
            # yazmada birleşmesin. Birleşince sonlandırıcı araya girse bile
            # firmware iki komutu tek okumada görür.
            self.mix_serial.flush()
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
            brake_signal = 0 if speed_signal != 0 else 1

            # SADECE DEĞERİ SAKLA. Porta yazan TEK yer _gaz_tekrarla.
            # Eskiden hem burası hem zamanlayıcı yazıyordu: aynı komut iki
            # kaynaktan, düzensiz aralıklarla gidiyordu ve porta saniyede ~80
            # komut yığılıyordu. Arduino'nun 64 baytlık giriş tamponu bunu
            # yetiştiremezse taşar, komutlar bozulur - gaz kesik kesik alınır.
            degisti = (speed_signal != self._son_hiz or brake_signal != self._son_fren)
            self._son_hiz, self._son_fren = speed_signal, brake_signal

            # Log her mesajda değil, DEĞİŞİNCE. Saniyede 10 satır 'h,1' hiçbir
            # şey söylemiyordu ve gerçek olayları (port kopması gibi) gizliyordu.
            if degisti:
                self.get_logger().info(
                    f'📡 HIZ | {speed:.2f} m/s | Sinyal: h,{speed_signal} f,{brake_signal}')

        except Exception as e:
            self.get_logger().error(f'Speed callback hatası: {e}')

    def _gaz_tekrarla(self):
        """Son gaz/fren değerini sabit hızda porta tekrar yazar.

        Gaz komutunun sürekliliğini /speed mesajlarının ritmine bırakmamak için.
        Karar düğümü yavaşlasa da, mesaj gecikse de porta kesintisiz akar.
        """
        self.send_command('h', self._son_hiz)
        self.send_command('f', self._son_fren)

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

        # --- DİREKSİYON YÖNÜ DENETİMİ ---------------------------------
        # Komut edilen direksiyon ile aracın GERÇEKTEN döndüğü yön tutuyor mu?
        # ROS kuralı: yaw ARTARSA araç SOLA dönüyor. Kodda pozitif direksiyon
        # açısı = byte > merkez = fiziksel SAĞ = yaw AZALIR. Yani doğru
        # bağlantıda aci * yaw_hizi < 0 olmalı. Üst üste tersi çıkıyorsa
        # steering_direction yanlıştır ve araç her düzeltmeyi ters yapar.
        # Bu denetim gözle tahmin etmeyi bitirir: araç kendi kendini ölçer.
        simdi_t = time.time()
        if self._yon_onceki is not None:
            dt = simdi_t - self._yon_onceki[0]
            if dt > 0.05:
                dyaw = math.atan2(math.sin(self.guncel_yaw - self._yon_onceki[1]),
                                  math.cos(self.guncel_yaw - self._yon_onceki[1]))
                yaw_hizi = dyaw / dt
                aci = self._son_direksiyon_acisi
                # Yalnızca anlamlı direksiyon + gerçek dönüş varken oy ver
                if abs(aci) > 0.05 and abs(yaw_hizi) > math.radians(5):
                    self._yon_oy += 1 if (aci * yaw_hizi) > 0 else -1
                    self._yon_oy = max(-40, min(self._yon_oy, 40))
                    if self._yon_oy >= 20 and not self._yon_uyarildi:
                        self._yon_uyarildi = True
                        self.get_logger().error(
                            '🔄 DİREKSİYON TERS BAĞLI. Komut edilen yön ile aracın '
                            'gerçekten döndüğü yön 20 ölçümde ters çıktı. '
                            f'steering_direction şu an {self.steering_direction:+.0f}; '
                            f'{-self.steering_direction:+.0f} olmalı:  ros2 param set '
                            f'/uart_sender_node steering_direction {-self.steering_direction:.1f}')
                    elif self._yon_oy <= -20 and not self._yon_uyarildi:
                        self._yon_uyarildi = True
                        self.get_logger().info(
                            '✅ Direksiyon yönü DOĞRU (komut ile gerçek dönüş uyuşuyor).')
                self._yon_onceki = (simdi_t, self.guncel_yaw)
        else:
            self._yon_onceki = (simdi_t, self.guncel_yaw)

        # HAREKET BAŞLADI MI: direksiyon bunu bekliyor (bkz. hareket_esigi_m).
        k = msg.pose.pose.position
        if self._ilk_konum is None:
            self._ilk_konum = (k.x, k.y)
        elif not self._hareket_basladi:
            if math.hypot(k.x - self._ilk_konum[0],
                          k.y - self._ilk_konum[1]) >= self.hareket_esigi_m:
                self._hareket_basladi = True
                self.get_logger().info(
                    f'🚗 Araç hareket etti ({self.hareket_esigi_m:.2f} m) - '
                    f'şerit takibi devrede.')

    def donus_callback(self, msg: Int32):
        """Karar alma düğümünden gelen MECBURİ DÖNÜŞ emri (-1 sol, 0 düz, +1 sağ).

        Koridor tercihinden (/route/preferred_side) farklı: bu gerçek bir
        manevra. Şerit kaybolduğu anda hedef yön bu kadar kaydırılır.
        """
        yeni = int(msg.data)

        if yeni == 0:
            # Emir geri çekildi (levha zaman aşımına uğradı ya da 'ileri
            # mecburi yön' görüldü): mandal serbest, sonraki kavşak dönebilir.
            self.tuketilen_donus = 0
        elif yeni == self.tuketilen_donus:
            # Aynı yön hâlâ yayınlanıyor ama o dönüş YAPILDI. Yok say.
            return

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
                # Mandalı kur: karar düğümü aynı emri yayınlamaya devam edecek.
                self.tuketilen_donus = self.bekleyen_donus
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

        # steering_direction BURADA DA UYGULANIR. Eskiden uygulanmıyordu ve bu
        # düğümde İKİ ayrı işaret kuralı oluşuyordu: PID yolu -1.0 ile çarpıyor,
        # bu yol hiç çarpmıyordu - yani ikisi tam TERS. Hangi yolun aktif olduğu
        # /lane/valid'e bağlı olduğu için araç şerit görünürken doğru, şerit
        # kaybolunca (viraj/kavşak) TERS kırıyordu.
        #
        # İŞARETİN KANITI (2026-08-19, kayıttan): karar düğümü 'SOLA DÖN'
        # yayınladığında bekleyen_donus=-1 -> hedef_yaw = guncel+90° ->
        # hata=+90° -> aci pozitif -> byte>230. Şerit takibinden byte>230'un
        # fiziksel olarak SAĞ olduğunu biliyoruz (PID'de sapma>0 'SOLA git'
        # demek ve byte 230'un üstüne çıkıyor, düz yolda şerit tutuluyor).
        # Yani sola emri sağa kırdırıyordu. steering_direction ile düzelir ve
        # bundan sonra iki yol TEK knob'a bağlı kalır.
        aci = self.steering_direction * (self.kp_heading * hata)
        return max(-self.max_steering_angle, min(aci, self.max_steering_angle))

    def lateral_deviation_callback(self, msg: Float32):
        """Lateral deviation mesajını alır ve direksiyon kontrolü yapar.
        DİREKSİYONUN TEK SAHİBİ BU FONKSİYON - başka hiçbir yer d komutu göndermez."""
        self.current_lateral_deviation = msg.data

        # ÖNCE GAZ, SONRA DİREKSİYON.
        #  - Odometri varsa: araç hareket_esigi_m kadar yol gidene kadar
        #    direksiyon MERKEZDE kalır. Duran araçta direksiyon kırmak boşuna.
        #  - Odometri yoksa (ZED kapalı): eski davranış, süreye bakılır.
        odometri_var = self._ilk_konum is not None
        bekle = (not self._hareket_basladi) if odometri_var else (
            time.time() - self.start_time < self.straight_start_sec)
        if bekle:
            # NEDEN BEKLEDİĞİ GÖRÜNSÜN. Bu dalda 'LATERAL KONTROL' logu
            # basılmıyor; sebebi yazılmazsa pistte "direksiyon çalışmıyor"
            # sanılıyor. Saniyede bir hatırlatır.
            simdi = time.time()
            if simdi - getattr(self, '_bekleme_logu', 0.0) > 1.0:
                self._bekleme_logu = simdi
                self.get_logger().info(
                    f'⏸️  Direksiyon BEKLEMEDE - araç henüz '
                    f'{self.hareket_esigi_m:.2f} m ilerlemedi '
                    f'(gaz veriliyor, hareket bekleniyor).'
                    if odometri_var else
                    '⏸️  Direksiyon BEKLEMEDE - düz başlangıç fazı.')
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

        # KOMUT HIZI SINIRI: porta gereğinden sık yazmak Arduino'nun giriş
        # tamponunu taşırıyor (bkz. direksiyon_hz).
        simdi = time.time()
        if self.direksiyon_hz > 0:
            if simdi - self._son_direksiyon_zamani < 1.0 / self.direksiyon_hz:
                return
            self._son_direksiyon_zamani = simdi

        self._son_direksiyon_acisi = steering_angle
        angle_byte = self.angle_to_byte(steering_angle)
        self.send_command('d', angle_byte)   # d -> direksiyon
        self.get_logger().info(f'🎯 LATERAL KONTROL | Dev: {self.current_lateral_deviation:.3f} | '
                             f'Steering: {steering_angle:.3f} rad | Byte: d,{angle_byte} | '
                             f'Durum: {"SAĞ tarafta→SOLA" if self.current_lateral_deviation < 0 else "SOL tarafta→SAĞA" if self.current_lateral_deviation > 0 else "MERKEZ"}')

    def speed_to_digital_signal(self, speed_ms):
        """Gelen hıza göre porta yazılacak h değeri: hiz_degeri (git) ya da 0 (dur).

        hiz_degeri varsayılan 1'dir, yani eski davranış. Firmware h'yi bir hız
        SEVİYESİ olarak okuyorsa 1 yetmeyebilir (bkz. parametre notu).
        """
        return self.hiz_degeri if speed_ms > 0.1 else 0

    def angle_to_byte(self, angle_rad):
        """Direksiyon açısını firmware'in d aralığına çevirir.

        Eşlemenin kendisi modül düzeyinde (bkz. aci_to_byte): kalibrasyon modu
        da aynı fonksiyonu kullansın diye.
        """
        return aci_to_byte(angle_rad, self.steering_trim, self.max_steering_angle)

    def ackermann_callback(self, msg: AckermannDrive):
        """Ackermann komutunu alır - sadece hız kontrolü için kullanılır"""
        try:
            # Hız komutunu işle ve GÖNDER
            speed_signal = self.speed_to_digital_signal(msg.speed)
            brake_signal = 0 if speed_signal != 0 else 1

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

# --- KALİBRASYON MODU --------------------------------------------------------
# python3 uart_sender_node3.py --kalibrasyon [--tarama|--kilit] [--sonlandiricisiz]
#
# ROS GEREKTİRMEZ, porta doğrudan yazar. Düğümler KAPALI olmalı (port tek
# kullanıcılı), tekerler HAVADA/SEHPADA, tahrik motoru kapalı.
#
# NEDEN DÜĞÜMÜN İÇİNDE: ölçtüğün merkez ile sürüşte kullanılan merkez aynı
# fonksiyondan (aci_to_byte) geçsin diye. Ayrı bir dosyada dursaydı byte
# aralığı iki yerde tanımlanır ve biri değişince diğeri sessizce eskirdi.

def _kalibrasyon_tus_oku():
    """Enter beklemeden tek tuş okur. Ok tuşları 3 baytlık ESC dizisi gönderir."""
    import termios
    import tty
    fd = sys.stdin.fileno()
    eski = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, eski)


class _KalibrasyonPortu:
    """Porta d komutu yazar. Arduino resetini ve sonlandırıcıyı yönetir."""

    def __init__(self, port_adi, baud, satirsonu=True):
        self.satirsonu = satirsonu
        self.port = serial.Serial(port_adi, baud, timeout=1)
        # Port açılınca Arduino DTR ile reset atar ve ~2 sn komut kabul etmez.
        # Beklemezsek ilk komutlar sessizce düşer ve "teker dönmüyor" sanılır -
        # aranan hatanın ta kendisi.
        print('  Arduino resetinden çıkması bekleniyor (3 sn)...')
        time.sleep(3.0)

    def gonder(self, deger):
        deger = int(max(0, min(deger, BYTE_UST)))
        komut = f'd,{deger}' + ('\n' if self.satirsonu else '')
        self.port.write(komut.encode('utf-8'))
        self.port.flush()
        return deger

    def kapat(self, park_byte=None):
        """Çıkarken tekerleri GERÇEK merkeze park eder.

        Nominal 180'e park etmek trim ölçüldükten sonra yanlış: tekerleri
        kalibrasyonun bulduğu düz konumdan trim kadar uzağa bırakırdı.
        """
        try:
            self.gonder(BYTE_MERKEZ if park_byte is None else park_byte)
        finally:
            self.port.close()


def _kalibrasyon_nereye_yazilir(ad, deger):
    print(f'\n  {"=" * 58}')
    print(f'  {ad} = {deger}')
    print(f'  {"=" * 58}')
    print('  OTOMATİK KAYIT YOK (kalibrasyon.yaml kaldırıldı). Bu dosyadaki')
    print(f'  declare_parameter satırını güncelleyin:')
    print(f"      self.declare_parameter('{ad}', {deger})")
    print('  Kalıcı olmayan deneme için:')
    print(f'      ros2 param set /uart_sender_node {ad} {deger}\n')


def _kalibrasyon_tarama(port, bekle):
    """Uç değerleri sırayla gönderir - komut ulaşıyor mu, hangi yöne?"""
    for deger in (BYTE_MERKEZ, 0, BYTE_MERKEZ, BYTE_UST, BYTE_MERKEZ):
        etiket = {0: 'tam SOL beklenir', BYTE_UST: 'tam SAĞ beklenir'}.get(deger, 'MERKEZ')
        print(f'  d,{deger:3d}  {etiket}')
        port.gonder(deger)
        time.sleep(bekle)
    print("""
  Ne gördün?
    Uçlarda döndü        -> haberleşme sağlam, sorun genlik/merkez.
                            --tarama'sız çalıştırıp merkezi ara.
    Sadece \\n ile döndü  -> komutlar bitişik gidiyor (d,158h,1f,0);
                            send_command'a sonlandırıcı eklenmeli.
    Hiç dönmedi          -> komut uygulanmıyor: kablo, güç ya da firmware'in
                            d aralığı. .ino görülmeli.""")


def _kodda_yazan(ad, varsayilan):
    """Bu dosyadaki declare_parameter varsayılanını kaynaktan okur.

    kalibrasyon.yaml kaldırıldığı için ayarların tek doğru kaynağı o satırlar.
    Kalibrasyon modu ROS'suz çalıştığından parametreyi düğüme soramaz; elle
    kopyalanan sayı ise sessizce eskir.
    """
    try:
        kaynak = open(os.path.abspath(__file__), encoding='utf-8').read()
    except OSError:
        return varsayilan
    m = re.search(r"declare_parameter\(\s*'%s'\s*,\s*(-?[\d.]+)\s*\)" % ad, kaynak)
    return type(varsayilan)(float(m.group(1))) if m else varsayilan


def _kalibrasyon_merkez_bul(port, trim=0):
    """Tuşla byte'ı kaydırıp tekerlerin GERÇEKTEN düz olduğu değeri bulur.

    KODDA YAZILI merkezden başlar, 180'den değil: merkez bir kez ölçüldükten
    sonra iş onu itme testiyle bir-iki birim hassaslaştırmaya döner ve her
    seferinde 180'den başlamak o ayarı sıfırdan aratırdı.
    """
    deger = port.gonder(merkez_byte(trim))
    print(f"""
  MERKEZ ARAMA - kodda yazılı merkez: {merkez_byte(trim)} (trim {trim:+d})
  Tekerlere bak, tam düz olunca 'm'

    ← / a   1 birim sol        → / d   1 birim sağ
    A       10 birim sol       D       10 birim sağ
    0       kodda yazılı merkeze dön
    q / e   uca git (0 / 360) - dönüyor mu diye bak
    n       satır sonu (\\n) aç/kapa - şu an: {'AÇIK' if port.satirsonu else 'KAPALI'}
    m       BURASI DÜZ -> ölç ve bitir
    x       kaydetmeden çık
""")
    adimlar = {'a': -1, '\x1b[D': -1, 'd': +1, '\x1b[C': +1, 'A': -10, 'D': +10}
    while True:
        print(f'\r  byte = {deger:3d}   (180\'e göre {deger - BYTE_MERKEZ:+d})   ',
              end='', flush=True)
        t = _kalibrasyon_tus_oku()
        if t in adimlar:
            deger += adimlar[t]
        elif t == '0':
            deger = merkez_byte(trim)
        elif t == 'q':
            deger = 0
        elif t == 'e':
            deger = BYTE_UST
        elif t == 'n':
            port.satirsonu = not port.satirsonu
            print(f'\n  satır sonu: {"AÇIK" if port.satirsonu else "KAPALI"}')
        elif t == 'm':
            print()
            return deger
        elif t in ('x', '\x03'):
            print('\n  Vazgeçildi.')
            return None
        else:
            continue
        deger = port.gonder(deger)


def _kalibrasyon_merkez_yorumla(merkez):
    trim = merkez - BYTE_MERKEZ
    print(f'  Tekerler byte {merkez} iken düz. Yazılımın varsayımı {BYTE_MERKEZ} idi.')
    _kalibrasyon_nereye_yazilir('steering_trim', trim)
    yarim = yari_aralik(trim)
    print(f'  Kullanılabilir simetrik aralık: merkez ± {yarim} birim '
          f'(solda {merkez}, sağda {BYTE_UST - merkez} birim yer var).')
    print('  HASSASLAŞTIRMA: aracı yere indirip düz zeminde 3-4 m ileri itin.')
    print("  Sağa kaçıyorsa trim'i AZALTIN, sola kaçıyorsa ARTIRIN (1-2 birim),")
    print("  bu modu tekrar çalıştırıp deneyin. Göz kararı ±3 birim tutturur,")
    print("  itme testi ±1'e indirir.")
    if abs(trim) > 30:
        print(f'  ⚠️  Trim {trim} - BÜYÜK. Yazılımda kapatmak kilidin bir kısmını')
        print('      kaybettirir. Önce mekanik bağlantıyı ortalayın, yazılıma')
        print('      sadece kalan birkaç birimi bırakın.')
    return trim


def _kalibrasyon_kilit_olc(port, trim):
    """Uçlara gönderip tekerin GERÇEK açısını sorar -> max_steering_angle."""
    merkez = merkez_byte(trim)
    yarim = yari_aralik(trim)
    print(f'\n  KİLİT ÖLÇÜMÜ (merkez {merkez}, simetrik aralık ±{yarim})')
    print('  Telefonun açıölçeri yeter. Tekerin DÜZ konuma göre açısını ölç.\n')

    olcumler = []
    for ad, hedef in (('SOL', merkez - yarim), ('SAĞ', merkez + yarim)):
        port.gonder(hedef)
        print(f'  d,{hedef} gönderildi -> teker {ad} uçta olmalı.')
        ham = input(f'  {ad} uçtaki açı kaç derece? (bilmiyorsan boş geç): ').strip()
        if ham:
            try:
                olcumler.append(abs(float(ham.replace(',', '.'))))
            except ValueError:
                print('    Sayı okunamadı, atlanıyor.')
    port.gonder(merkez)

    if not olcumler:
        print('\n  Ölçüm girilmedi; max_steering_angle değişmemeli.')
        return None

    derece = sum(olcumler) / len(olcumler)
    if len(olcumler) == 2 and abs(olcumler[0] - olcumler[1]) > 3.0:
        print(f'\n  ⚠️  İki taraf {abs(olcumler[0] - olcumler[1]):.1f}° farklı. '
              'Mekanik simetri bozuk; küçük taraf esas alınır.')
        derece = min(olcumler)
    rad = round(math.radians(derece), 3)
    print(f'\n  Gerçek kilit ≈ {derece:.1f}° = {rad} rad '
          '(yazılımın varsayımı 0.5 rad = 28.6° idi).')
    _kalibrasyon_nereye_yazilir('max_steering_angle', rad)
    return rad


def kalibrasyon_modu(argv):
    """--kalibrasyon ile çağrılır. ROS başlatılmadan önce, onun yerine çalışır."""
    import argparse
    ap = argparse.ArgumentParser(prog='uart_sender_node3.py --kalibrasyon')
    ap.add_argument('--kalibrasyon', action='store_true')
    ap.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    ap.add_argument('--baud', type=int, default=38400)
    # Sonlandırıcı artık VARSAYILAN (sürüşte de öyle - bkz. send_command).
    # Ölçüm ile sürüş aynı biçimde yazmazsa sehpada bulunan merkez pistte
    # başka yere düşer. --satirsonu geriye dönük uyum için kabul ediliyor.
    ap.add_argument('--satirsonu', action='store_true',
                    help='(artık varsayılan - etkisiz, uyumluluk için duruyor)')
    ap.add_argument('--sonlandiricisiz', action='store_true',
                    help="komutları \\n OLMADAN gönder (eski hatalı davranış; "
                         "sadece arızayı yeniden üretmek için)")
    ap.add_argument('--tarama', action='store_true',
                    help='sadece uç değerleri gönder (komut ulaşıyor mu?)')
    ap.add_argument('--kilit', action='store_true',
                    help='gerçek kilidi ölç (max_steering_angle)')
    ap.add_argument('--trim', type=int, default=None,
                    help='steering_trim (verilmezse kodda yazılı değer)')
    ap.add_argument('--bekle', type=float, default=2.0)
    args = ap.parse_args(argv)
    if args.trim is None:
        args.trim = _kodda_yazan('steering_trim', 0)

    print('\n  ⚠️  TEKERLER HAVADA/SEHPADA OLMALI, tahrik motoru KAPALI.')
    print('      ROS düğümleri kapalı olmalı (port tek kullanıcılı).\n')

    try:
        port = _KalibrasyonPortu(args.port, args.baud,
                                 satirsonu=not args.sonlandiricisiz)
    except serial.SerialException as e:
        # Hangi hata olduğu teşhisi tamamen değiştiriyor; tek bir "port
        # açılamadı" satırı yanlış yere baktırıyordu. (2026-08-18: Arduino
        # oturum boyunca üç kez düştü, Errno 5 alındı ve "düğüm açık mı" diye
        # arandı - oysa kablo/besleme sorunuydu.)
        print(f'  Port açılamadı: {e}')
        metin = str(e)
        if 'Errno 5' in metin or 'Input/output' in metin:
            print('  Errno 5 = cihaz düğümü var ama Arduino cevap vermiyor.')
            print('  KABLO/BESLEME: USB\'yi çıkar-tak, sonra: dmesg | tail -20')
            print('  (Yazılım sorunu DEĞİL - komut cihaza hiç ulaşmıyor.)')
        elif 'Errno 2' in metin or 'No such file' in metin:
            print('  Errno 2 = cihaz hiç bağlı değil. Kontrol:')
            print('      ls -l /dev/serial/by-id/')
        elif 'Errno 16' in metin or 'busy' in metin.lower():
            print('  Port başkası tarafından tutuluyor. Çalışan düğümleri kapatın:')
            print('      pgrep -af "launch_all_nodes|uart_sender_node3"')
        elif 'Errno 13' in metin or 'Permission' in metin:
            print('  İzin yok. Kullanıcı dialout grubunda mı?')
            print('      sudo usermod -aG dialout $USER   (sonra oturumu kapat-aç)')
        return 1

    try:
        if args.tarama:
            _kalibrasyon_tarama(port, args.bekle)
        elif args.kilit:
            _kalibrasyon_kilit_olc(port, args.trim)
        else:
            merkez = _kalibrasyon_merkez_bul(port, args.trim)
            if merkez is None:
                return 0
            trim = _kalibrasyon_merkez_yorumla(merkez)
            if input('  Kilidi de şimdi ölçelim mi? [e/H] ').strip().lower() == 'e':
                _kalibrasyon_kilit_olc(port, trim)
    except KeyboardInterrupt:
        print('\n  Kesildi.')
    finally:
        port.kapat(merkez_byte(args.trim))
    return 0


def main(args=None):
    # Kalibrasyon modu ROS'suz çalışır ve düğümün YERİNE geçer; port tek
    # kullanıcılı olduğu için ikisi aynı anda açılamaz.
    if '--kalibrasyon' in sys.argv[1:]:
        return kalibrasyon_modu(sys.argv[1:])

    if ROS_HATASI is not None:
        print(f'ROS yüklenemedi: {ROS_HATASI}')
        print('Düğüm olarak çalıştırmak için ROS ortamını kaynaklayın:')
        print('    source /opt/ros/*/setup.bash')
        print('(Kalibrasyon modu ROS gerektirmez: --kalibrasyon)')
        return 1

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
    sys.exit(main() or 0)
