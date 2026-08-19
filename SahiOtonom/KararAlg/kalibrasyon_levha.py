#!/usr/bin/env python3
"""LEVHA MESAFE KALİBRASYONU - ref_box_height_px ölçer.

NE ÖLÇÜYOR: karar düğümü levhaya uzaklığı kutunun PİKSEL YÜKSEKLİĞİNDEN
türetiyor (ZED derinliği o düğüme bağlı değil):

    mesafe = ref_distance_m * ref_box_height_px / kutu_yuksekligi_px

Yani "ref_distance_m metrede levha ref_box_height_px piksel görünür" demek.
Kodda şu an 4.0 m <-> 60 px yazıyor ve 60 ÖLÇÜLMEDİ, tahmin. Yanlışsa
'kırmızı ışıkta 4 m'de dur' kuralı başka bir mesafede tetiklenir.

KULLANIM:
  1) Kamera + levha düğümü çalışsın (launcher ya da tek tek).
  2) Aracı levhadan ÖLÇTÜĞÜN mesafeye park et. Mesafe KAMERADAN levhaya,
     tampondan değil - kutu yüksekliğini belirleyen kamera uzaklığı.
  3) python3 KararAlg/kalibrasyon_levha.py --mesafe 4.0

TEK REFERANS SORUNU: kod bütün sınıflar için AYNI ref_box_height_px'i
kullanıyor. Trafik ışığı ile 'dur' tabelası fiziksel olarak aynı boyda
değilse biri kalibre olurken diğeri oranı kadar hatalı kalır. Bu araç
gördüğü her sınıfı ayrı listeler ve seçtiğin referansla diğerlerinin kaç
metre görüneceğini yazar - hatayı gizlemek yerine sayıyla gösterir.
"""
import argparse
import json
import statistics
import sys
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PARAM_DOSYASI = 'KararAlg/basic-decision-making-node.py'


class LevhaKalibrasyon(Node):
    def __init__(self):
        super().__init__('kalibrasyon_levha')
        self.yukseklikler = defaultdict(list)
        self.kare = 0
        self.create_subscription(String, '/sign_detection/boxes',
                                 self.boxes_callback, 10)

    def boxes_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.kare += 1
        for box in data.get('boxes', []):
            cls = box.get('cls')
            if cls is None:
                etiket = box.get('label', '')
                cls = etiket.split(' ', 1)[1] if ' ' in etiket else etiket
            h = float(box.get('y2', 0)) - float(box.get('y1', 0))
            if h > 1.0:
                self.yukseklikler[cls].append(h)


def olc(node, sure):
    """sure saniye boyunca topic'i dinler."""
    hedef = node.get_clock().now().nanoseconds + sure * 1e9
    son_yazi = 0
    while rclpy.ok() and node.get_clock().now().nanoseconds < hedef:
        rclpy.spin_once(node, timeout_sec=0.1)
        kalan = (hedef - node.get_clock().now().nanoseconds) / 1e9
        if int(kalan) != son_yazi:
            son_yazi = int(kalan)
            toplam = sum(len(v) for v in node.yukseklikler.values())
            print(f'\r  olculuyor... {kalan:4.0f} sn  |  {node.kare} kare, '
                  f'{toplam} tespit  ', end='', flush=True)
    print()


def main():
    ap = argparse.ArgumentParser(description='ref_box_height_px olcumu')
    ap.add_argument('--mesafe', type=float, required=True,
                    help='KAMERADAN levhaya olculen gercek mesafe (m)')
    ap.add_argument('--sure', type=float, default=10.0,
                    help='kac saniye olculecek (varsayilan 10)')
    ap.add_argument('--sinif', default=None,
                    help='referans alinacak sinif (varsayilan: en cok gorulen)')
    args = ap.parse_args()

    rclpy.init()
    node = LevhaKalibrasyon()
    try:
        print(f'\n  /sign_detection/boxes dinleniyor ({args.sure:.0f} sn)...')
        print(f'  Levha KAMERADAN {args.mesafe:.2f} m uzakta olmali.\n')
        olc(node, args.sure)

        if not node.yukseklikler:
            print('\n  HIC TESPIT YOK.')
            if node.kare == 0:
                print('  Topic hic gelmedi: levha dugumu calisiyor mu?')
                print('    python3 GoruntuIsleme/run_tracker.py')
            else:
                print(f'  {node.kare} kare geldi ama kutu yok: levha kadrajda mi,')
                print('  yeterince aydinlik mi? (model conf esigi 0.4)')
            return 1

        print('\n  ' + '-' * 62)
        print(f'  {"sinif":<28} {"tespit":>7} {"medyan px":>10} {"sapma":>8}')
        print('  ' + '-' * 62)
        ozet = {}
        for cls, hs in sorted(node.yukseklikler.items(), key=lambda x: -len(x[1])):
            med = statistics.median(hs)
            sap = statistics.pstdev(hs) if len(hs) > 1 else 0.0
            ozet[cls] = med
            print(f'  {cls:<28} {len(hs):>7} {med:>10.1f} {sap:>8.1f}')
        print('  ' + '-' * 62)

        referans = args.sinif or max(node.yukseklikler, key=lambda c: len(node.yukseklikler[c]))
        if referans not in ozet:
            print(f'\n  Istenen sinif goruntude yok: {referans}')
            return 1
        ref_px = round(ozet[referans], 1)

        print(f'\n  REFERANS SINIF: {referans}')
        print(f'\n  {"=" * 62}')
        print(f'  ref_distance_m    = {args.mesafe}')
        print(f'  ref_box_height_px = {ref_px}')
        print(f'  {"=" * 62}')
        print(f'  {PARAM_DOSYASI} icinde su iki satiri guncelleyin:')
        print(f"      self.declare_parameter('ref_distance_m', {args.mesafe})")
        print(f"      self.declare_parameter('ref_box_height_px', {ref_px})")
        print('  Sistem calisirken denemek icin (kapaninca kaybolur):')
        print(f'      ros2 param set /decision_making_node ref_box_height_px {ref_px}')

        digerleri = {c: m for c, m in ozet.items() if c != referans}
        if digerleri:
            print(f'\n  BU REFERANSLA DIGER SINIFLAR ne mesafede gorunur')
            print(f'  (hepsi gercekte {args.mesafe:.2f} m uzakta):')
            for cls, med in digerleri.items():
                tahmin = args.mesafe * ref_px / med
                hata = (tahmin - args.mesafe) / args.mesafe * 100
                isaret = '  <-- %20 uzeri, ayri referans gerekir' if abs(hata) > 20 else ''
                print(f'    {cls:<28} {tahmin:5.2f} m  (%{hata:+.0f}){isaret}')
            print('\n  Kod TEK ref_box_height_px kullaniyor; yukaridaki sapma')
            print('  o sinif icin dogrudan durma mesafesi hatasidir.')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
