#!/usr/bin/env python3
"""ZED odometrisini test/doğrulama aracı.

Aracı sürmeye gerek yok: kamerayı elinize alıp hareket ettirerek de test
edilebilir. /zed2i/odom'u dinler ve konumu insanın okuyabileceği biçimde
gösterir (kuaterniyon yerine derece, ayrıca kat edilen toplam yol).

KULLANIM:
    # 1) Kamerayı odometri AÇIK başlatın (tek başına, launcher olmadan):
    python3 zedi2connect_port.py --ros-args -p enable_odometry:=true

    # 2) Başka bir terminalde:
    python3 odometri_test.py

    SIFIRLA: ENTER'a basın (o anki konum yeni başlangıç olur)
    ÇIKIŞ  : CTRL+C
"""
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def kuaterniyon_to_yaw(q):
    """Kuaterniyondan sapma açısı (derece). Z ekseni etrafındaki dönüş."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class OdometriTest(Node):
    def __init__(self):
        super().__init__('odometri_test')
        self.baslangic = None       # sıfırlama noktası
        self.son = None
        self.toplam_yol = 0.0
        self.onceki_ham = None
        self.mesaj_sayisi = 0
        self.ilk_zaman = None

        self.create_subscription(Odometry, '/zed2i/odom', self.odom_callback, 10)
        self.create_timer(0.3, self.yaz)
        print('/zed2i/odom bekleniyor...')
        print('(veri gelmiyorsa: kamera enable_odometry:=true ile mi baslatildi?)')

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ham = (p.x, p.y, p.z)

        # Kat edilen toplam yol: ardışık konumlar arası mesafelerin toplamı.
        # Duruyorken bunun artması = tracking gürültüsü (drift) demektir.
        if self.onceki_ham is not None:
            d = math.dist(ham, self.onceki_ham)
            self.toplam_yol += d
        self.onceki_ham = ham

        if self.baslangic is None:
            self.baslangic = ham
            self.ilk_zaman = self.get_clock().now()
        self.son = (ham, kuaterniyon_to_yaw(q))
        self.mesaj_sayisi += 1

    def sifirla(self):
        if self.onceki_ham is not None:
            self.baslangic = self.onceki_ham
            self.toplam_yol = 0.0
            self.ilk_zaman = self.get_clock().now()

    def yaz(self):
        if self.son is None:
            return
        (x, y, z), yaw = self.son
        bx, by, bz = self.baslangic
        dx, dy, dz = x - bx, y - by, z - bz
        duz = math.hypot(dx, dy)
        gecen = (self.get_clock().now() - self.ilk_zaman).nanoseconds / 1e9

        print('\033[2J\033[H', end='')
        print('  ZED ODOMETRI TESTI')
        print('  ' + '-' * 52)
        print(f'  ILERI  (x) : {dx:+7.3f} m      <- one dogru hareket +')
        print(f'  SOL    (y) : {dy:+7.3f} m      <- sola dogru hareket +')
        print(f'  YUKARI (z) : {dz:+7.3f} m')
        print(f'  SAPMA      : {yaw:+7.1f} derece')
        print('  ' + '-' * 52)
        print(f'  duz mesafe (baslangictan)  : {duz:7.3f} m')
        print(f'  kat edilen toplam yol      : {self.toplam_yol:7.3f} m')
        print(f'  mesaj sayisi / sure        : {self.mesaj_sayisi} / {gecen:.1f} sn'
              f'  ({self.mesaj_sayisi / gecen:.1f} Hz)' if gecen > 0 else '')
        print('  ' + '-' * 52)
        if duz < 0.05 and self.toplam_yol > 0.20:
            print('  ! DURUYORKEN TOPLAM YOL ARTIYOR = drift.')
            print('    Ortam az dokulu/karanlik olabilir; ZED duvar gibi duz')
            print('    yuzeylerde konum kaybeder.')
        print('\n  ENTER = sifirla     CTRL+C = cikis')


def main(args=None):
    rclpy.init(args=args)
    node = OdometriTest()

    def enter_dinle():
        for _ in sys.stdin:
            node.sifirla()

    t = threading.Thread(target=enter_dinle, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
