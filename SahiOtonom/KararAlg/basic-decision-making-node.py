#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32, Bool, String, Int32
from rcl_interfaces.msg import SetParametersResult
from enum import Enum
import numpy as np
import json
import math
import time


# Levha sınıfları (run_tracker.py'deki model isimleriyle birebir aynı olmalı)
CLS_RED = 'kirmizi-isik'
CLS_YELLOW = 'sari-isik'
CLS_GREEN = 'yesil-isik'
CLS_STOP = 'dur'
# Levhalar İKİ FARKLI şey söyler; ilk sürümde ikisi aynı kovaya konmuştu ve
# 'sagdan-gidiniz' ile 'saga-mecburi-yon' aynı davranışı üretiyordu. Oysa:
#
#   KORİDOR TERCİHİ  = "refüjün/adanın şu tarafından geç" -> DÖNÜŞ DEĞİL,
#                      sadece yol ikiye ayrılınca hangi kolun seçileceği
#   MECBURİ DÖNÜŞ    = "burada sağa/sola dön" -> gerçek 90° manevra
#
DIRECTION_SIGNS = {          # koridor tercihi (-1 sol, +1 sağ)
    'soldan-gidiniz': -1,
    'serit-duzenlemesi-sol': -1,
    'saga-donulmez': -1,          # sağa dönülmez -> sola/ileri yönel
    'sagdan-gidiniz': +1,
    'serit-duzenlemesi-sag': +1,
    'sola-donulmez': +1,          # sola dönülmez -> sağa/ileri yönel
}

TURN_SIGNS = {               # mecburi dönüş (-1 sola dön, +1 sağa dön, 0 düz)
    'sola-mecburi-yon': -1,
    'ileriden-sola-mecburi-yon': -1,
    'ileri-ve-sola-mecburi-yon': -1,
    'saga-mecburi-yon': +1,
    'ileriden-saga-mecburi-yon': +1,
    'ileri-ve-saga-mecburi-yon': +1,
    'ileri-mecburi-yon': 0,       # açıkça DÜZ: bekleyen dönüşü iptal eder
}


def sinif_adini_sadelestir(ad):
    """Model sınıf adını tabloların ASCII yazımına indirger.

    NEDEN VAR: modelin sınıf adı 'ileriden-sağa-mecburi-yon' (Türkçe ğ ile),
    buradaki tablo ise 'saga' yazıyordu. Eşleşme tutmadığı için o levha
    tespit ediliyor, ekranda kutusu çiziliyor ama sign_callback onu HİÇBİR
    dala sokmadan atlıyordu - yani araç levhayı görüp yok sayıyordu ve
    ortada tek bir hata mesajı yoktu. Ad eşleştirmesini harf harf doğru
    yazmaya bırakmak bu hatayı model her yeniden eğitildiğinde geri getirir;
    normalleştirme sınıfın tamamını kapatır.
    """
    if not ad:
        return ''
    # Küçültmeden ÖNCE çevir: 'İ'.lower() birleşik noktalı i üretir ve
    # karşılaştırmayı yine bozar.
    return (ad.strip()
            .translate(str.maketrans('ğüşıöçĞÜŞİÖÇ', 'gusiocGUSIOC'))
            .lower()
            .replace('_', '-'))


class DriveState(Enum):
    """Aracın sürüş durumu. Durum makinesi olmadan 'kırmızıda dur, yeşilde kalk'
    ve 'dur levhasında 10 sn bekle' davranışları yazılamıyor: ikisi de zamana
    ve geçmişe bağlı, tek karelik bir 'if' ile ifade edilemez."""
    SURUYOR = 0
    KIRMIZI_BEKLIYOR = 1     # kırmızı ışıkta duruyor, yeşili bekliyor
    DUR_BEKLIYOR = 2         # 'dur' levhasında sayıyor


class DecisionMakingNode(Node):
    """
    Basit karar verme node'u: Şerit takibi, engel önleme, trafik ışığı ve levhalar
    """
    def __init__(self):
        super().__init__('decision_making_node')

        # Parametreler
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('steering_gain', 1.5)
        # ENGEL DAVRANIŞI. Eski değerler (dur 10 m, yavaşla 4 m) pratikte tek
        # kademeliydi: engel 10 m'den yakın görülünce hız doğrudan 0 oluyor,
        # slow_down_distance ise hiç kullanılmıyordu.
        # Yeni davranış: 5 m'den itibaren HER METREDE bir kademe yavaşla,
        # 1.5 m'de tam dur.
        self.declare_parameter('emergency_stop_distance', 1.5)
        self.declare_parameter('slow_down_distance', 5.0)
        # Yavaşlama kademesinin boyu (metre)
        self.declare_parameter('slow_down_step_m', 1.0)

        # --- LEVHA / IŞIK MESAFESİ ---------------------------------------
        # ZED derinlik KAPALI başlatılıyor (bkz. zedi2connect_port.py), yani
        # levhaya uzaklık ölçülemiyor. Kutu yüksekliği mesafeyle ters orantılı
        # olduğu için tek bir referans ölçümüyle mesafe türetilir:
        #     mesafe = ref_mesafe * ref_yukseklik / guncel_yukseklik
        # KALİBRASYON: aracı ışıktan ref_distance_m kadar uzağa koy, ekrandaki
        # kutunun piksel yüksekliğini oku, ref_box_height_px'e yaz.
        self.declare_parameter('ref_distance_m', 4.0)
        # TAHMİN - ölçülmedi. Bu yüzden "4 metre" gerçekte 4 m değil.
        self.declare_parameter('ref_box_height_px', 60.0)
        # Kırmızı ışıkta bu mesafede durulur
        self.declare_parameter('stop_distance_m', 4.0)
        # 'dur' levhasında beklenecek süre
        self.declare_parameter('stop_sign_wait_sec', 10.0)
        # Aynı 'dur' levhasında tekrar durmamak için: kalktıktan sonra bu süre
        # boyunca yeni bir 'dur' tetiklenmez (takip numarası değişirse diye).
        self.declare_parameter('stop_sign_cooldown_sec', 15.0)
        # Işık/levha bu kadar süre görünmezse "artık yok" sayılır. Tek karelik
        # tespit boşluğu aracı yanlışlıkla kaldırmasın diye var.
        self.declare_parameter('sign_memory_sec', 1.0)
        # KIRMIZIDA DURURKEN kalkmak için kırmızının bu kadar süre HİÇ
        # görünmemesi gerekir. sign_memory_sec'ten uzun olmalı: kırmızıda
        # beklerken tespit bir saniyeliğine kesilince araç kalkıp ışıkta
        # geçiyordu. Takılı kalmamak için bir üst sınır lazım ama kırmızıda
        # geçmek, birkaç saniye fazla beklemekten kötü.
        self.declare_parameter('red_release_sec', 3.0)
        # Yön levhası bu süre boyunca geçerli sayılır (kavşağa varana kadar)
        self.declare_parameter('direction_memory_sec', 12.0)

        self.BASE_SPEED = self.get_parameter('base_speed').value
        self.MAX_STEERING_ANGLE = self.get_parameter('max_steering_angle').value
        self.STEERING_GAIN = self.get_parameter('steering_gain').value
        self.EMERGENCY_STOP_DISTANCE = self.get_parameter('emergency_stop_distance').value
        self.SLOW_DOWN_DISTANCE = self.get_parameter('slow_down_distance').value
        self.SLOW_DOWN_STEP_M = self.get_parameter('slow_down_step_m').value
        self.ref_distance_m = self.get_parameter('ref_distance_m').value
        self.ref_box_height_px = self.get_parameter('ref_box_height_px').value
        self.stop_distance_m = self.get_parameter('stop_distance_m').value
        self.stop_sign_wait_sec = self.get_parameter('stop_sign_wait_sec').value
        self.stop_sign_cooldown_sec = self.get_parameter('stop_sign_cooldown_sec').value
        self.sign_memory_sec = self.get_parameter('sign_memory_sec').value
        self.red_release_sec = self.get_parameter('red_release_sec').value
        self.direction_memory_sec = self.get_parameter('direction_memory_sec').value

        # DİKKAT: eskiden burada sabit 1.0 vardı ve calculate_speed bunu
        # kullanıyordu; base_speed parametresi okunuyor ama HİÇ kullanılmıyordu.
        # Yani aracı yazılımdan durdurmanın yolu yoktu - sistem çalışırken
        # aracın önüne bir şey koymak tehlikeliydi. Artık base_speed geçerli
        # ve canlı değiştirilebilir:  ros2 param set /decision_making_node base_speed 0.0
        self.manual_speed = self.BASE_SPEED
        self.lateral_deviation = 0.0
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

        # Levha/ışık durumu
        self.state = DriveState.SURUYOR
        self.state_since = time.time()
        self.last_red = None          # (zaman, mesafe)
        self.last_green = None
        self.last_stop_sign = None    # (zaman, mesafe, takip_id)
        self.handled_stop_ids = set()
        self.stop_sign_released_at = 0.0
        self.preferred_side = 0       # -1 sol, 0 yok, +1 sağ
        self.preferred_side_at = 0.0
        # Kuralı olmayan levhalar: her karede uyarı basmamak için bir kez tutulur
        self.kurali_olmayan = set()
        self.bekleyen_donus = 0       # -1 sola dön, 0 yok, +1 sağa dön
        self.bekleyen_donus_at = 0.0
        self.last_state_log = None

        # Subscribers
        self.lateral_sub = self.create_subscription(
            Float32, '/lane/lateral_deviation', self.lateral_callback, 10)

        self.obstacle_detected_sub = self.create_subscription(
            Bool, '/obstacle_detected', self.obstacle_detected_callback, 10)

        self.obstacle_distance_sub = self.create_subscription(
            Float32, '/obstacle_distance', self.obstacle_distance_callback, 10)

        # LEVHA TESPİTİ. Bu abonelik yoktu: run_tracker.py levhaları yayınlıyor
        # ama sadece combined_view (ekran) dinliyordu, yani hiçbir levha ya da
        # ışık aracın davranışını etkilemiyordu.
        self.sign_sub = self.create_subscription(
            String, '/sign_detection/boxes', self.sign_callback, 10)

        # Publishers
        self.speed_pub = self.create_publisher(
            Float32, '/speed', 10)

        # Kavşakta hangi kolun seçileceği: şerit tespit düğümü bunu dinleyip
        # yol ikiye ayrıldığında ilgili koridoru tercih eder.
        self.side_pub = self.create_publisher(
            Int32, '/route/preferred_side', 10)

        # MECBURİ DÖNÜŞ. Koridor tercihinden ayrı: bu gerçek bir manevra emri.
        # UART düğümü şerit kaybolduğunda hedef yönü buna göre ±90° kaydırır.
        self.turn_pub = self.create_publisher(Int32, '/route/turn', 10)

        # Pistte canlı ayar: base_speed 0.0 -> araç durur ama sistem (ve kayıt)
        # çalışmaya devam eder. Kutu/levha yerleştirirken bunu kullanın.
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.timer = self.create_timer(0.1, self.decision_loop)  # 10 Hz

        self.get_logger().info('🚗 Decision Making Node başlatıldı (levha + ışık aktif).')

    LIVE_PARAMS = ('base_speed', 'emergency_stop_distance', 'slow_down_distance',
                   'stop_distance_m', 'stop_sign_wait_sec', 'ref_distance_m',
                   'ref_box_height_px', 'red_release_sec')

    def _on_parameter_update(self, params):
        for p in params:
            if p.name == 'base_speed':
                self.BASE_SPEED = float(p.value)
                self.manual_speed = self.BASE_SPEED
                if self.BASE_SPEED == 0.0:
                    self.get_logger().warn('⏸️  base_speed = 0 -> ARAÇ DURDURULDU '
                                           '(sistem ve kayıt çalışmaya devam ediyor)')
                else:
                    self.get_logger().info(f'▶️  base_speed = {self.BASE_SPEED}')
            elif p.name in self.LIVE_PARAMS:
                ad = {'emergency_stop_distance': 'EMERGENCY_STOP_DISTANCE',
                      'slow_down_distance': 'SLOW_DOWN_DISTANCE'}.get(p.name, p.name)
                setattr(self, ad, type(getattr(self, ad))(p.value))
                self.get_logger().info(f'⚙️  {p.name} = {getattr(self, ad)}')
        return SetParametersResult(successful=True)

    def lateral_callback(self, msg):
        """Şerit sapma bilgisini al"""
        self.lateral_deviation = msg.data

    def obstacle_detected_callback(self, msg):
        """Engel tespit durumunu al"""
        self.obstacle_detected = msg.data

    def obstacle_distance_callback(self, msg):
        """Engel mesafesini al"""
        if msg.data > 0:
            self.obstacle_distance = msg.data
        else:
            self.obstacle_distance = float('inf')

    def estimate_distance(self, box):
        """Levhanın piksel yüksekliğinden mesafe tahmini (metre).

        ZED derinlik kapalı olduğu için gerçek ölçüm yok. Kutu yüksekliği
        mesafeyle ters orantılıdır, o yüzden bilinen bir referanstan ölçeklenir.
        Kalibre edilmemişse sayı yanlış olur ama SIRALAMA doğru kalır (yakın
        levha her zaman daha büyük görünür).
        """
        h = float(box.get('y2', 0) - box.get('y1', 0))
        if h <= 1.0:
            return float('inf')
        return self.ref_distance_m * self.ref_box_height_px / h

    def sign_callback(self, msg):
        """Levha/ışık tespitlerini işler ve en yakın olanları hafızaya yazar."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        now = time.time()
        for box in data.get('boxes', []):
            # 'cls' alanı run_tracker.py tarafından eklendi. Eski sürümle
            # uyum için etiketten de ayrıştırılabiliyor ("ID:3 dur").
            cls = box.get('cls')
            if cls is None:
                label = box.get('label', '')
                cls = label.split(' ', 1)[1] if ' ' in label else label
            cls = sinif_adini_sadelestir(cls)
            distance = self.estimate_distance(box)

            if cls == CLS_RED:
                if self.last_red is None or distance < self.last_red[1] or now - self.last_red[0] > self.sign_memory_sec:
                    self.last_red = (now, distance)
            elif cls == CLS_GREEN:
                self.last_green = (now, distance)
            elif cls == CLS_STOP:
                track_id = box.get('id')
                if track_id not in self.handled_stop_ids:
                    self.last_stop_sign = (now, distance, track_id)
            elif cls in TURN_SIGNS:
                self.bekleyen_donus = TURN_SIGNS[cls]
                self.bekleyen_donus_at = now
                yon = {-1: 'SOLA DÖN', 0: 'DÜZ DEVAM', 1: 'SAĞA DÖN'}[self.bekleyen_donus]
                self.get_logger().info(f'↩️  Mecburi yön: {cls} -> {yon} ({distance:.1f} m)')
            elif cls in DIRECTION_SIGNS:
                self.preferred_side = DIRECTION_SIGNS[cls]
                self.preferred_side_at = now
                self.get_logger().info(
                    f'🧭 Koridor tercihi: {cls} -> {"SOL" if self.preferred_side < 0 else "SAĞ"} '
                    f'({distance:.1f} m)')
            elif cls not in self.kurali_olmayan:
                # SESSİZ ATLAMAYI GÖRÜNÜR KIL. Yukarıdaki dalların hiçbirine
                # girmeyen levha, araç için YOK demektir. Bunu bir kez de olsa
                # yazmazsak "levhayı görüyor ama uymuyor" arızası ancak pistte
                # ve tahminle bulunur.
                self.kurali_olmayan.add(cls)
                self.get_logger().warn(
                    f'👀 Levha tanındı ama KURALI YOK, davranış değişmeyecek: {cls}')

    def _fresh(self, record):
        """Kayıt hâlâ geçerli mi (son sign_memory_sec içinde görüldü mü)."""
        return record is not None and (time.time() - record[0]) <= self.sign_memory_sec

    def update_traffic_state(self):
        """Işık ve 'dur' levhasına göre durum makinesini ilerletir."""
        now = time.time()

        if self.state == DriveState.DUR_BEKLIYOR:
            if now - self.state_since >= self.stop_sign_wait_sec:
                self.stop_sign_released_at = now
                self.last_stop_sign = None
                self._set_state(DriveState.SURUYOR, 'dur levhası beklemesi bitti')
            return

        if self.state == DriveState.KIRMIZI_BEKLIYOR:
            # Kalkmanın normal yolu YEŞİL ışıktır. Kırmızının kaybolması da
            # kabul edilir ama sadece uzun bir süre sonra (red_release_sec):
            # kısa tespit boşlukları aracı kırmızıda kaldırmasın.
            if self._fresh(self.last_green):
                self._set_state(DriveState.SURUYOR, 'yeşil ışık')
            elif self.last_red is None or (now - self.last_red[0]) > self.red_release_sec:
                self._set_state(DriveState.SURUYOR,
                                f'kırmızı {self.red_release_sec:.0f} sn görünmedi')
            return

        # --- SURUYOR: durmayı gerektiren bir şey var mı? ---
        if self._fresh(self.last_red) and self.last_red[1] <= self.stop_distance_m:
            # Yeşil de aynı anda görünüyorsa (iki ışık kadrajda) kırmızıya uy
            self._set_state(DriveState.KIRMIZI_BEKLIYOR,
                            f'kırmızı ışık {self.last_red[1]:.1f} m')
            return

        if (self._fresh(self.last_stop_sign)
                and self.last_stop_sign[1] <= self.stop_distance_m
                and now - self.stop_sign_released_at > self.stop_sign_cooldown_sec):
            track_id = self.last_stop_sign[2]
            if track_id is not None:
                self.handled_stop_ids.add(track_id)
            self._set_state(DriveState.DUR_BEKLIYOR,
                            f'dur levhası {self.last_stop_sign[1]:.1f} m '
                            f'({self.stop_sign_wait_sec:.0f} sn bekleniyor)')

    def _set_state(self, state, reason):
        self.state = state
        self.state_since = time.time()
        icon = {DriveState.SURUYOR: '🟢', DriveState.KIRMIZI_BEKLIYOR: '🛑',
                DriveState.DUR_BEKLIYOR: '⏸️'}[state]
        self.get_logger().info(f'{icon} Durum: {state.name} - {reason}')

    def calculate_steering_angle(self):
        """Direksiyon açısını hesapla - Düz yol için basit şerit takibi"""
        # Basit orantısal kontrol
        steering_angle = -self.lateral_deviation * self.STEERING_GAIN
        
        # Maksimum direksiyon açısı sınırlaması
        steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        
        return steering_angle

    def engel_hiz_carpani(self, mesafe):
        """Engel mesafesine göre hız çarpanı (0.0 - 1.0), METRE KADEMELİ.

        5 m ve ötesi  -> 1.00 (tam hız)
        4.0 - 5.0 m   -> 0.80
        3.0 - 4.0 m   -> 0.60
        2.0 - 3.0 m   -> 0.40
        1.5 - 2.0 m   -> 0.20
        1.5 m ve altı -> 0.00 (dur)

        Sürekli bir rampa yerine kademe kullanılıyor: LiDAR mesafesi kare kare
        birkaç santim oynuyor ve sürekli rampa gaz komutunu titretiyor.
        """
        if mesafe <= self.EMERGENCY_STOP_DISTANCE:
            return 0.0
        if mesafe >= self.SLOW_DOWN_DISTANCE:
            return 1.0
        adim = max(self.SLOW_DOWN_STEP_M, 0.1)
        kademe = math.floor(mesafe / adim) * adim
        return min(1.0, max(0.0, kademe / self.SLOW_DOWN_DISTANCE))

    def calculate_speed(self):
        """Hız hesapla: durum makinesi + engel"""
        # Kırmızı ışık ya da 'dur' levhası beklemesi -> tam duruş
        if self.state in (DriveState.KIRMIZI_BEKLIYOR, DriveState.DUR_BEKLIYOR):
            return 0.0

        speed = self.manual_speed  # Manuel hızı kullan

        if self.obstacle_detected and self.obstacle_distance < float('inf'):
            carpan = self.engel_hiz_carpani(self.obstacle_distance)
            speed *= carpan
            if carpan == 0.0:
                self.get_logger().warn(
                    f'🛑 DUR! Engel {self.obstacle_distance:.2f} m '
                    f'(sınır {self.EMERGENCY_STOP_DISTANCE:.1f} m)')
            elif carpan < 1.0:
                self.get_logger().info(
                    f'🐢 Yavaşla: engel {self.obstacle_distance:.2f} m '
                    f'-> hız %{carpan * 100:.0f}')

        return max(speed, 0.0)

    def decision_loop(self):
        """Ana karar verme döngüsü"""
        try:
            # Işık/levha durumunu ilerlet (hızı bu belirliyor)
            self.update_traffic_state()

            # Kavşak tercihini yayınla; süresi dolduysa sıfırla
            simdi = time.time()
            if self.preferred_side and (simdi - self.preferred_side_at) > self.direction_memory_sec:
                self.preferred_side = 0
            self.side_pub.publish(Int32(data=int(self.preferred_side)))

            # Mecburi dönüş de zamanla unutulur: levhayı görüp kavşağa hiç
            # varmadıysak (yanlış tespit, yol değişikliği) sonsuza dek bekleyen
            # bir dönüş emri taşımak tehlikeli.
            if self.bekleyen_donus and (simdi - self.bekleyen_donus_at) > self.direction_memory_sec:
                self.bekleyen_donus = 0
                self.get_logger().info('↩️  Bekleyen dönüş zaman aşımıyla iptal edildi.')
            self.turn_pub.publish(Int32(data=int(self.bekleyen_donus)))

            # Direksiyon açısını hesapla
            steering_angle = self.calculate_steering_angle()

            # Hızı hesapla
            speed = self.calculate_speed()

            # Speed mesajı hazırla ve yayınla
            speed_msg = Float32()
            speed_msg.data = speed
            self.speed_pub.publish(speed_msg)
            
            # Debug bilgisi
            self.get_logger().debug(
                f'Speed: {speed:.2f} m/s | Steering: {steering_angle:.3f} rad | '
                f'Deviation: {self.lateral_deviation:.3f} | Obstacle: {self.obstacle_detected} | '
                f'Distance: {self.obstacle_distance:.2f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Decision loop error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakingNode()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Launcher kritik düğüm ölünce hepsini kapatıyor; bu normal kapanış.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()