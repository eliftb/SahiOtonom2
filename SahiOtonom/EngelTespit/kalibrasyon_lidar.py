#!/usr/bin/env python3
"""LiDAR MONTAJ KALİBRASYONU - pistte tek başına çalışır.

Araç hareket etmez, Arduino'ya gerek yoktur. Sadece LiDAR'ın hangi açısının
aracın ÖNÜ olduğunu ve koridorun ne kadar temiz göründüğünü ölçer.

KULLANIM (iki terminal):
  1) LiDAR sürücüsü:
       source /opt/ros/humble/setup.bash
       ros2 launch rplidar_ros rplidar_s2_launch.py \
            serial_port:=$(ls /dev/serial/by-id/*CP2102* | head -1) \
            serial_baudrate:=1000000
  2) Bu araç:
       python3 kalibrasyon_lidar.py

Aracın TAM ÖNÜNE, ~2 metre uzağa bir kutu koyun. Ekrandaki haritada kutuyu
görün, sonra CTRL+C'ye basın: önerilen forward_angle_deg değerini ve girmeniz
gereken komutu yazar.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# kalibrasyon.yaml KALDIRILDI (2026-08-18). Bu arac olcumu yapar ama artik
# hicbir yere yazamaz: cikan degeri ilgili dugumun icine ELLE yazin.
PARAM_YERI = {
    'lane_detection_node': 'SeritTespit/serit-tespitcopy.py',
    'lidar_obstacle_detector': 'EngelTespit/engel-tespit.py',
    'uart_sender_node': 'Haberlesme/uart_sender_node3.py',
    'decision_making_node': 'KararAlg/basic-decision-making-node.py',
    'zed_publisher_node': 'Kamera/zedi2connect_port.py',
}


def kalibrasyon_kaydet(dugum, ad, deger):
    yer = PARAM_YERI.get(dugum, dugum)
    print('\n  OTOMATIK KAYIT YOK - kalibrasyon.yaml kaldirildi.')
    print(f"  KALICI yapmak icin {yer} icinde su satiri guncelleyin:")
    print(f"      self.declare_parameter('{ad}', {deger})")
    print('  Sistem CALISIRKEN denemek icin (kapaninca kaybolur):')
    print(f"      ros2 param set /{dugum} {ad} {deger}")


HARITA_YARICAP_M = 5.0     # haritanın kapsadığı yarıçap
HARITA_GENISLIK = 61       # karakter (tek sayı olmalı ki merkez tam ortada olsun)
HARITA_YUKSEKLIK = 25


class LidarKalibrasyon(Node):
    def __init__(self):
        super().__init__('lidar_kalibrasyon')
        self.declare_parameter('hedef_mesafe_m', 2.0)   # kutuyu koyduğunuz mesafe
        self.declare_parameter('arama_toleransi_m', 1.0)
        self.declare_parameter('min_kume', 3)

        self.hedef = self.get_parameter('hedef_mesafe_m').value
        self.tolerans = self.get_parameter('arama_toleransi_m').value
        self.min_kume = self.get_parameter('min_kume').value

        self.acilar = []          # ölçülen "en yakın cisim" açıları
        self.tarama_sayisi = 0
        self.son_yazim = 0.0

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        print('LiDAR verisi bekleniyor... (sürücü çalışıyor mu? ros2 topic hz /scan)')

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges or msg.angle_increment == 0.0:
            return
        self.tarama_sayisi += 1

        r = np.asarray(msg.ranges, dtype=np.float64)
        n = r.size
        a = msg.angle_min + np.arange(n) * msg.angle_increment
        gecerli = np.isfinite(r) & (r > max(msg.range_min, 0.0)) & (r < msg.range_max)
        if not gecerli.any():
            return

        # Hedef mesafe civarındaki EN YAKIN KÜME: kutuyu ararız, uzaktaki
        # duvarı/bariyeri değil. Tek ışın gürültüsü elenir.
        aday = gecerli & (np.abs(r - self.hedef) <= self.tolerans)
        aci = None
        if aday.any():
            idx = np.flatnonzero(aday)
            gruplar = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
            gruplar = [g for g in gruplar if g.size >= self.min_kume]
            if gruplar:
                en = min(gruplar, key=lambda g: float(np.min(r[g])))
                aci = float(np.mean(a[en]))
                self.acilar.append(aci)

        simdi = self.get_clock().now().nanoseconds / 1e9
        if simdi - self.son_yazim >= 0.7:
            self.son_yazim = simdi
            self.ciz(r, a, gecerli, aci)

    def ciz(self, r, a, gecerli, aci):
        harita = [[' '] * HARITA_GENISLIK for _ in range(HARITA_YUKSEKLIK)]
        olcek_x = (HARITA_GENISLIK - 1) / (2 * HARITA_YARICAP_M)
        olcek_y = (HARITA_YUKSEKLIK - 1) / (2 * HARITA_YARICAP_M)

        ileri = r[gecerli] * np.cos(a[gecerli])
        yanal = r[gecerli] * np.sin(a[gecerli])
        for x, y in zip(ileri, yanal):
            if abs(x) > HARITA_YARICAP_M or abs(y) > HARITA_YARICAP_M:
                continue
            # ekranda: yukarı = LiDAR'ın 0 derecesi, sola = +y
            sut = int(round((HARITA_GENISLIK - 1) / 2 - y * olcek_x))
            sat = int(round((HARITA_YUKSEKLIK - 1) / 2 - x * olcek_y))
            if 0 <= sat < HARITA_YUKSEKLIK and 0 <= sut < HARITA_GENISLIK:
                harita[sat][sut] = '#'
        orta_s, orta_c = (HARITA_YUKSEKLIK - 1) // 2, (HARITA_GENISLIK - 1) // 2
        harita[orta_s][orta_c] = 'L'          # LiDAR

        print('\033[2J\033[H', end='')        # ekranı temizle
        print(f'  LIDAR KALIBRASYON   tarama #{self.tarama_sayisi}   '
              f'(harita yaricapi {HARITA_YARICAP_M:.0f} m, yukari = LiDAR 0 derece)')
        print('  ' + '-' * HARITA_GENISLIK)
        for satir in harita:
            print('  |' + ''.join(satir) + '|')
        print('  ' + '-' * HARITA_GENISLIK)
        print(f'  L = LiDAR   # = olcum')
        if aci is None:
            print(f'\n  {self.hedef:.1f} m civarinda kume YOK.')
            print(f'  Aracin onune ~{self.hedef:.1f} m uzaga bir kutu koyun.')
        else:
            print(f'\n  En yakin kume: {math.degrees(aci):+7.1f} derece')
            if self.acilar:
                med = math.degrees(float(np.median(self.acilar)))
                print(f'  Medyan ({len(self.acilar)} olcum): {med:+7.1f} derece')
        print('\n  Bitirmek ve sonucu almak icin CTRL+C')

    def sonuc(self):
        print('\033[2J\033[H', end='')
        print('=' * 62)
        print('  KALIBRASYON SONUCU')
        print('=' * 62)
        if not self.acilar:
            print('\n  Hic olcum alinamadi.')
            print('  - LiDAR surucusu calisiyor mu?   ros2 topic hz /scan')
            print(f'  - Kutu aracin onunde ~{self.hedef:.1f} m mesafede mi?')
            return
        med = float(np.median(self.acilar))
        derece = math.degrees(med)
        sapma = math.degrees(float(np.std(self.acilar)))
        print(f'\n  Olcum sayisi      : {len(self.acilar)}')
        print(f'  Medyan aci        : {derece:+.1f} derece')
        print(f'  Standart sapma    : {sapma:.1f} derece')
        if sapma > 5.0:
            print('\n  ! Sapma yuksek. Kutu sabit duruyor mu, baska cisim')
            print('    ayni mesafede mi? Olcumu tekrarlayin.')
        if '--kaydet' in sys.argv:
            kalibrasyon_kaydet('lidar_obstacle_detector', 'forward_angle_deg',
                               round(derece, 1))
        else:
            print('\n  Degeri koda nasil isleyeceginizi gormek icin:')
            print('    python3 kalibrasyon_lidar.py --kaydet')
            print('\n  Ya da sadece bu oturum icin:')
            print(f'    ros2 param set /lidar_obstacle_detector forward_angle_deg {derece:.1f}')
        print('=' * 62)


def main(args=None):
    rclpy.init(args=args)
    node = LidarKalibrasyon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sonuc()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
