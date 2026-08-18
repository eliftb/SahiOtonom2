#!/usr/bin/env python3
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np

# Kalıcı kalibrasyon değerleri proje kökündeki kalibrasyon.yaml'dan gelir.
# Bu düğüm alt klasörde olduğu için kök dizin arama yoluna ekleniyor.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kalibrasyon import kalibrasyon

KAL = kalibrasyon('lidar_obstacle_detector')


class LidarObstacleDetector(Node):
    """Aracın ÖNÜNDEKİ koridorda engel arar ve mesafesini yayınlar.

    Eski sürüm aracın ARKASINA bakıyordu (180° etrafındaki koni) ve öndeki
    engelleri hiç görmüyordu. Ayrıca sadece açı aralığına bakıyordu: 5 m'de
    ±15°'lik bir koni ±1.3 m genişliğindedir, yani pistin YAN BARİYERLERİ
    sürekli "engel" sayılıp aracı durduruyordu.

    Bu sürüm ölçümleri kartezyene çevirip aracın gerçek geçiş koridoruna
    (ileri x, yanal |y| < yarı genişlik) bakar. Böylece yandaki bariyer, ne
    kadar yakın olursa olsun, aracın önünü kesmiyorsa engel sayılmaz.
    """

    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        # --- MONTAJ ---------------------------------------------------------
        # LiDAR'ın hangi açısı aracın ÖNÜ. Cihaz öne bakacak şekilde düz monte
        # edilirse 0, ters monte edilirse 180. Montajdan sonra kalibre edin:
        # debug_nearest'i açın, aracın tam önüne bir kutu koyun ve loglarda
        # görünen açıyı buraya yazın.
        self.declare_parameter('forward_angle_deg', KAL('forward_angle_deg', 0.0))

        # --- KORİDOR --------------------------------------------------------
        # Aracın geçeceği şeridin yarı genişliği. Araç genişliğinin yarısı +
        # güvenlik payı. Büyütmek yan bariyerleri engel saymaya başlar.
        self.declare_parameter('corridor_half_width_m', KAL('corridor_half_width_m', 0.5))
        # Bu mesafeden yakındaki ölçümler aracın kendi gövdesi/tamponu sayılır
        self.declare_parameter('min_check_distance_m', 0.15)

        # --- KARAR MESAFELERİ ------------------------------------------------
        # Engel bu mesafede fark edilmeye başlar (yavaşlama buradan itibaren)
        self.declare_parameter('detect_distance_m', 5.0)
        # Bu mesafede tam duruş
        self.declare_parameter('stop_distance_m', 1.5)

        # --- GÜRÜLTÜ BASTIRMA -------------------------------------------------
        # Koridorda YAN YANA en az bu kadar ışın engeli görmeli. Tek başına
        # gelen bir ölçüm (toz, yansıma, kenar ışını) aracı durdurmasın.
        self.declare_parameter('min_cluster_points', 3)
        # Engel "var" demek için üst üste kaç tarama gerekli
        self.declare_parameter('confirm_frames', 2)
        # Engel "yok" demek için üst üste kaç temiz tarama gerekli. Bilerek
        # daha büyük: engeli kaybetmek, yanlışlıkla durmaktan tehlikelidir.
        self.declare_parameter('release_frames', 5)

        # Kalibrasyon/tanı için: en yakın noktanın AÇISINI ve mesafesini basar
        self.declare_parameter('debug_nearest', False)

        for ad in self.LIVE_PARAMS:
            setattr(self, ad, self.get_parameter(ad).value)

        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.hit_streak = 0
        self.clear_streak = 0
        self.obstacle_active = False
        self.last_distance = float('inf')
        self.scan_count = 0
        self.last_log = None

        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)

        self.get_logger().info(
            f'🛡️ Engel tespiti başlatıldı | koridor ±{self.corridor_half_width_m:.2f} m | '
            f'fark et {self.detect_distance_m:.1f} m | dur {self.stop_distance_m:.1f} m')

    LIVE_PARAMS = ('forward_angle_deg', 'corridor_half_width_m', 'min_check_distance_m',
                   'detect_distance_m', 'stop_distance_m', 'min_cluster_points',
                   'confirm_frames', 'release_frames', 'debug_nearest')

    def _on_parameter_update(self, params):
        for p in params:
            if p.name in self.LIVE_PARAMS:
                setattr(self, p.name, type(getattr(self, p.name))(p.value))
                self.get_logger().info(f'⚙️  {p.name} = {getattr(self, p.name)}')
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges or msg.angle_increment == 0.0:
            return

        self.scan_count += 1
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = ranges.size
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        gecerli = (np.isfinite(ranges)
                   & (ranges > max(msg.range_min, 0.0))
                   & (ranges < msg.range_max))

        # Açıyı aracın ÖN yönüne göre normalize et (-pi..pi)
        sapma = angles - np.deg2rad(self.forward_angle_deg)
        sapma = np.arctan2(np.sin(sapma), np.cos(sapma))

        # Geçersiz ölçümler (inf/nan) çarpımda nan üretip uyarı basıyordu;
        # maskeyle zaten eleniyorlar, o yüzden önce sıfırlanıyorlar.
        temiz = np.where(gecerli, ranges, 0.0)
        ileri = temiz * np.cos(sapma)     # x: aracın ilerisi (metre)
        yanal = temiz * np.sin(sapma)     # y: sola pozitif

        koridorda = (gecerli
                     & (ileri > self.min_check_distance_m)
                     & (ileri <= self.detect_distance_m)
                     & (np.abs(yanal) <= self.corridor_half_width_m))

        mesafe = self._en_yakin_kume(koridorda, ileri)

        if self.debug_nearest:
            self._kalibrasyon_logu(gecerli, ranges, sapma)

        self._durumu_guncelle(mesafe)

    def _en_yakin_kume(self, koridorda, ileri):
        """Koridordaki en yakın GERÇEK engelin ileri mesafesi.

        Tek tük ölçümler engel sayılmaz: yan yana en az min_cluster_points ışın
        aynı cismi görmeli. Tek bir yansıma yüzünden acil fren yapmak, pistte
        aracı durduran en sinir bozucu hata türü.
        """
        idx = np.flatnonzero(koridorda)
        if idx.size == 0:
            return float('inf')

        # Ardışık ışın gruplarına ayır
        gruplar = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        en_yakin = float('inf')
        for g in gruplar:
            if g.size >= self.min_cluster_points:
                en_yakin = min(en_yakin, float(np.min(ileri[g])))
        return en_yakin

    def _kalibrasyon_logu(self, gecerli, ranges, sapma):
        """MONTAJ KALİBRASYONU: en yakın cismin açısını basar.

        Aracın tam önüne bir kutu koyun; burada görünen açı 0'a yakın değilse
        forward_angle_deg'i o açı kadar kaydırın.
        """
        if self.scan_count % 10 != 0:
            return
        idx = np.flatnonzero(gecerli)
        if idx.size == 0:
            return
        en = idx[np.argmin(ranges[idx])]
        self.get_logger().info(
            f'🔎 En yakın cisim: {ranges[en]:.2f} m @ ön yöne göre '
            f'{np.rad2deg(sapma[en]):+.1f}°  (0° = aracın önü)')

    def _durumu_guncelle(self, mesafe):
        """Anlık ölçümü zaman filtresinden geçirip yayınlar."""
        var = mesafe <= self.detect_distance_m

        if var:
            self.hit_streak += 1
            self.clear_streak = 0
        else:
            self.clear_streak += 1
            self.hit_streak = 0

        if not self.obstacle_active and self.hit_streak >= self.confirm_frames:
            self.obstacle_active = True
            self.get_logger().warn(f'🚧 Engel: {mesafe:.2f} m')
        elif self.obstacle_active and self.clear_streak >= self.release_frames:
            self.obstacle_active = False
            self.get_logger().info('✅ Yol temiz')

        self.last_distance = mesafe if var else float('inf')

        detected_msg = Bool()
        detected_msg.data = bool(self.obstacle_active)
        self.obstacle_detected_pub.publish(detected_msg)

        distance_msg = Float32()
        # Engel yokken -1: karar alma düğümü bunu "sonsuz" olarak yorumluyor
        distance_msg.data = float(mesafe) if self.obstacle_active and np.isfinite(mesafe) else -1.0
        self.obstacle_distance_pub.publish(distance_msg)

        # Log sadece mesafe kademesi değişince (her taramada basmak terminali boğuyordu).
        # np.isfinite kontrolü ŞART: histerezis engeli hâlâ "aktif" tutarken
        # güncel ölçüm inf olabiliyor (engel görüşten çıktı ama release_frames
        # dolmadı) ve int(inf) düğümü çökertiyordu.
        if self.obstacle_active and np.isfinite(mesafe):
            kademe = int(mesafe)
            if kademe != self.last_log:
                self.last_log = kademe
                self.get_logger().info(f'🚧 Engel {mesafe:.2f} m')
        elif not self.obstacle_active:
            self.last_log = None


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
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
