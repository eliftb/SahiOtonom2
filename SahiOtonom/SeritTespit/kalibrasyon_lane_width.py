#!/usr/bin/env python3
"""ŞERİT GENİŞLİĞİ KALİBRASYONU - lane_width_frac ölçer.

NEDEN: Bariyer/bordür gibi çizgi-benzeri kenarları elemenin son savunma hattı
genişlik makullük kontrolü (bkz. serit-tespitcopy.py _lane_centers_at_rows,
"0.5 * expected < measured < 1.8 * expected"). 'expected' lane_width_frac'tan
türüyor; bu hiç ölçülmediği (tahmini 0.40 olduğu) için bariyer ile gerçek bir
çizgi arasındaki mesafe tesadüfen bu geniş aralığa denk gelirse bariyer
'şerit' sayılıyor. Doğru ölçülmüş lane_width_frac bu aralığı gerçek şerit
genişliğine oturtur, yanlış eşleşmeleri daha güvenilir eler.

KULLANIM:
  1) Sistemi normal başlat:      python3 launch_all_nodes.py
  2) ARACI ŞERİDİN TAM ORTASINA park et, düz bir yerde, HER İKİ şerit çizgisi
     de (bariyer değil, gerçek boyalı çizgi) net görünsün
  3) Bu aracı çalıştır:          python3 kalibrasyon_lane_width.py --kaydet

  Birkaç saniye bekleyip CTRL+C. Ölçülen değer ekrana basılır; artık otomatik
  kaydedilmez - araç hangi dosyada hangi satırı güncelleyeceğinizi söyler.
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

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

# serit-tespitcopy.py ile AYNI sabitler (declare_parameter varsayılanları).
# lane_width_frac bu satırdaki genişlikle tanımlı: sample_rows[0]=0.80
GENISLIK = 1280.0      # ZED HD720
YUKSEKLIK = 720.0
HORIZON_FRAC = 0.55
OLCUM_SATIRI_FRAC = 0.80   # sample_rows[0]
REF_FRAC = 0.85            # _expected_lane_width'teki normalizasyon satırı


class SeritGenisligiKalibrasyon(Node):
    def __init__(self):
        super().__init__('serit_genisligi_kalibrasyon')
        self.olcumler = []
        self.create_subscription(Float32, '/lane/width_px', self.width_callback, 10)
        print('/lane/width_px bekleniyor... (sistem çalışıyor mu?)')
        self.create_timer(0.5, self.yaz)

    def width_callback(self, msg):
        self.olcumler.append(float(msg.data))

    @staticmethod
    def _lane_width_frac(measured_px):
        horizon = HORIZON_FRAC * YUKSEKLIK
        di = max(OLCUM_SATIRI_FRAC * YUKSEKLIK - horizon, 1.0)
        ref_denom = max(REF_FRAC * YUKSEKLIK - horizon, 1.0)
        return measured_px * ref_denom / (GENISLIK * di)

    def yaz(self):
        print('\033[2J\033[H', end='')
        print('  ŞERİT GENİŞLİĞİ KALİBRASYONU')
        print('  ' + '-' * 52)
        n = len(self.olcumler)
        print(f'  olcum sayisi : {n}')
        if n < 5:
            print('  Veri bekleniyor... sistem calisiyor mu, iki cizgi de goruluyor mu?')
        else:
            med = float(np.median(self.olcumler))
            sap = float(np.std(self.olcumler))
            frac = self._lane_width_frac(med)
            print(f'  Genislik (medyan)  : {med:7.1f} px  (0.80 satirinda, dik duzeltilmis)')
            print(f'  standart sapma     : {sap:7.1f} px')
            print()
            print(f'  >>> lane_width_frac = {frac:.3f}')
            if sap > 40:
                print('\n  ! Sapma yuksek. Iki cizgi de net goruluyor mu, arac')
                print('    seridin tam ortasinda mi? Olcumu tekrarlayin.')
        print('\n  CTRL+C = bitir ve kaydet (--kaydet verildiyse)')

    def sonuc(self):
        print('\033[2J\033[H', end='')
        print('=' * 60)
        print('  KALİBRASYON SONUCU')
        print('=' * 60)
        kaydet_mi = '--kaydet' in sys.argv

        if len(self.olcumler) < 5:
            print('\n  Yeterli olcum yok. Sistem calisiyor mu?')
            print('  Kontrol: ros2 topic hz /lane/width_px')
            return

        med = float(np.median(self.olcumler))
        sap = float(np.std(self.olcumler))
        frac = self._lane_width_frac(med)
        print(f'\n  olcum sayisi     : {len(self.olcumler)}')
        print(f'  Genislik medyani : {med:.1f} px   (sapma {sap:.1f})')
        print(f'\n  lane_width_frac = {frac:.3f}')

        # MAKULLUK SINIRI: gercek bir seridin genisligi goruntu genisliginin
        # ne %5'i ne de %80'i olabilir. Bunun disi olcum hatasidir.
        makul = 0.10 <= frac <= 0.70
        if sap > 40:
            print('\n  ! Sapma yuksek, deger guvenilir olmayabilir.')
        if not makul:
            print(f'\n  {"#" * 56}')
            print(f'  #  ŞÜPHELİ DEĞER: {frac:.3f}')
            print(f'  {"#" * 56}')
            print('  Neredeyse her zaman OLCUM hatasidir:')
            print('   - arac seridin tam ortasinda miydi?')
            print('   - HER IKI cizgi de gercek boyali serit miydi (bariyer degil)?')
            print('   - DUZ bir kesimde miydi (virajda degil)?')
            print('  Duzeltip tekrar olcun.')

        if kaydet_mi and (makul or '--zorla' in sys.argv):
            kalibrasyon_kaydet('lane_detection_node', 'lane_width_frac', round(frac, 3))
            print('\n  Kaydedildi. Sistemi yeniden baslatin.')
        elif kaydet_mi and not makul:
            print('\n  KAYDEDILMEDI. Gercekten dogru oldugundan eminseniz:')
            print('    python3 kalibrasyon_lane_width.py --kaydet --zorla')
        elif not kaydet_mi:
            print('\n  Kaydetmek icin --kaydet ile calistirin.')
        print('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = SeritGenisligiKalibrasyon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sonuc()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
