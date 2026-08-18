#!/usr/bin/env python3
"""KAMERA KALİBRASYONU - camera_center_offset_px ve hood_frac ölçer.

NEDEN: Kamera aracın tam ortasında/tam ileri bakacak şekilde değilse, şerit
merkezi görüntünün ortasına denk gelmez. Bu SABİT bir sapma üretir; araç düz
yolda bile direksiyonu kırık tutar. Ekrandaki "Merkez / ref" farkına göz kararı
bakmak yanıltır çünkü değer kare kare oynar - bu araç medyan alır.

KULLANIM:
  1) Sistemi normal başlat:      python3 launch_all_nodes.py
  2) ARACI ŞERİDİN TAM ORTASINA park et, düz bir yerde, şerit/yol net görünsün
  3) Bu aracı çalıştır:          python3 kalibrasyon_kamera.py --kaydet

  Birkaç saniye bekleyip CTRL+C. Değer kalibrasyon.yaml'a yazılır.

KAPUT SINIRI (hood_frac) için:
  Aynı aracı --kaput ile çalıştır ve aracı YAVAŞÇA İLERİ SÜR/İT. Kaput sabit
  durduğu için görüntünün alt bandı hiç değişmez; araç bunu tespit eder.
"""
import sys
import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kalibrasyon import kaydet as kalibrasyon_kaydet


class KameraKalibrasyon(Node):
    def __init__(self, kaput_modu=False):
        super().__init__('kamera_kalibrasyon')
        self.kaput_modu = kaput_modu
        self.merkezler = []
        self.genislik = None

        # Kaput tespiti için: satır bazında zaman içindeki değişim birikimi
        self.satir_toplam = None
        self.satir_kare = 0
        self.onceki_gri = None

        if kaput_modu:
            self.create_subscription(Image, '/zed2i_rgb/image_raw',
                                     self.image_callback, 10)
            print('KAPUT MODU: aracı yavaşça ileri sürün/itin...')
        else:
            self.create_subscription(Float32, '/lane/center_px',
                                     self.center_callback, 10)
            print('/lane/center_px bekleniyor... (sistem çalışıyor mu?)')

        self.create_timer(0.5, self.yaz)

    # ---------------- merkez kayması ----------------
    def center_callback(self, msg):
        self.merkezler.append(float(msg.data))
        if self.genislik is None:
            self.genislik = 1280.0      # ZED HD720; görüntüden de teyit edilebilir

    # ---------------- kaput sınırı ----------------
    def image_callback(self, msg):
        import cv2
        from cv_bridge import CvBridge
        if not hasattr(self, '_br'):
            self._br = CvBridge()
        frame = self._br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gri = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        self.genislik = float(gri.shape[1])
        if self.onceki_gri is not None:
            # Satır başına ortalama mutlak değişim. Kaput araçla birlikte
            # hareket ettiği için görüntüde SABİT kalır -> değişim ~0.
            fark = np.abs(gri - self.onceki_gri).mean(axis=1)
            if self.satir_toplam is None:
                self.satir_toplam = fark
            else:
                self.satir_toplam += fark
            self.satir_kare += 1
        self.onceki_gri = gri

    def kaput_siniri(self):
        """Alttan yukarı: değişimin belirgin arttığı ilk satır = kaputun üstü."""
        if self.satir_toplam is None or self.satir_kare < 10:
            return None
        ort = self.satir_toplam / self.satir_kare
        yukseklik = ort.size
        # Üst yarının hareketliliğini referans al (yol/çevre burada akıyor)
        referans = float(np.median(ort[: yukseklik // 2]))
        if referans <= 1e-6:
            return None
        esik = referans * 0.25          # bunun altı "sabit" sayılır
        sinir = yukseklik
        for y in range(yukseklik - 1, yukseklik // 2, -1):
            if ort[y] > esik:
                sinir = y + 1
                break
        return sinir / yukseklik

    def yaz(self):
        print('\033[2J\033[H', end='')
        print('  KAMERA KALIBRASYONU')
        print('  ' + '-' * 52)
        if self.kaput_modu:
            oran = self.kaput_siniri()
            print(f'  islenen kare : {self.satir_kare}')
            if oran is None:
                print('  Henuz yeterli hareket yok. Araci ileri surun/itin.')
            else:
                print(f'  KAPUT SINIRI : {oran:.3f}  (hood_frac)')
                print(f'  yani goruntunun alt %{(1 - oran) * 100:.0f} kadari kaput')
                if oran > 0.97:
                    print('  ! Kaput bulunamadi (kamera kaputu gormuyor olabilir).')
        else:
            n = len(self.merkezler)
            print(f'  olcum sayisi : {n}')
            if n < 5:
                print('  Veri bekleniyor... sistem calisiyor mu?')
            else:
                med = float(np.median(self.merkezler))
                sap = float(np.std(self.merkezler))
                ref = self.genislik / 2.0
                print(f'  Merkez (medyan) : {med:7.1f} px')
                print(f'  ref (goruntu orta): {ref:7.1f} px')
                print(f'  standart sapma  : {sap:7.1f} px')
                print()
                print(f'  >>> camera_center_offset_px = {med - ref:+.1f}')
                if sap > 40:
                    print('\n  ! Sapma yuksek. Arac gercekten seridin ortasinda mi,')
                    print('    yol/serit net goruluyor mu? Olcumu tekrarlayin.')
        print('\n  CTRL+C = bitir ve kaydet (--kaydet verildiyse)')

    def sonuc(self):
        print('\033[2J\033[H', end='')
        print('=' * 60)
        print('  KALIBRASYON SONUCU')
        print('=' * 60)
        kaydet_mi = '--kaydet' in sys.argv

        if self.kaput_modu:
            oran = self.kaput_siniri()
            if oran is None:
                print('\n  Yeterli olcum yok. Araci hareket ettirerek tekrarlayin.')
                return
            print(f'\n  hood_frac = {oran:.3f}   ({self.satir_kare} kare)')
            if kaydet_mi:
                kalibrasyon_kaydet('lane_detection_node', 'hood_frac', round(oran, 3))
            else:
                print('\n  Kaydetmek icin --kaydet ile calistirin.')
        else:
            if len(self.merkezler) < 5:
                print('\n  Yeterli olcum yok. Sistem calisiyor mu?')
                print('  Kontrol: ros2 topic hz /lane/center_px')
                return
            med = float(np.median(self.merkezler))
            sap = float(np.std(self.merkezler))
            ref = self.genislik / 2.0
            offset = med - ref
            print(f'\n  olcum sayisi   : {len(self.merkezler)}')
            print(f'  Merkez medyani : {med:.1f} px   (sapma {sap:.1f})')
            print(f'  goruntu ortasi : {ref:.1f} px')
            print(f'\n  camera_center_offset_px = {offset:+.1f}')
            # MAKULLUK SINIRI. Bu deger kameranin araca gore FIZIKSEL kaymasi;
            # birkac derecelik montaj hatasi ~50-100 px eder. 150 px'in ustu
            # (>12 derece) neredeyse her zaman HATALI OLCUMDUR: arac seridin
            # ortasinda degildi, serides acili duruyordu ya da model seridi
            # bulamadi. Boyle bir degeri kaydetmek araci surekli direksiyon
            # kirar hale getirir, o yuzden acikca zorlanmadikca kaydedilmez.
            makul = abs(offset) <= 150
            if sap > 40:
                print('\n  ! Sapma yuksek, deger guvenilir olmayabilir.')
            if not makul:
                print(f'\n  {"#" * 56}')
                print(f'  #  SUPHELI DEGER: {offset:+.1f} px')
                print(f'  {"#" * 56}')
                print('  Bu, kameranin ~12 dereceden fazla kaymis olmasi demek.')
                print('  Neredeyse her zaman OLCUM hatasidir:')
                print('   - arac seridin tam ortasinda miydi?')
                print('   - serie PARALEL miydi (acili degil)?')
                print('   - DUZ bir kesimde miydi (virajda degil)?')
                print('   - ekranda serit cizgileri net goruluyor muydu?')
                print('  Duzeltip tekrar olcun.')

            if kaydet_mi and (makul or '--zorla' in sys.argv):
                kalibrasyon_kaydet('lane_detection_node',
                                   'camera_center_offset_px', round(offset, 1))
                print('\n  Kaydedildi. Sistemi yeniden baslatin.')
            elif kaydet_mi and not makul:
                print('\n  KAYDEDILMEDI. Gercekten dogru oldugundan eminseniz:')
                print('    python3 kalibrasyon_kamera.py --kaydet --zorla')
            elif not kaydet_mi:
                print('\n  Kaydetmek icin --kaydet ile calistirin.')
        print('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = KameraKalibrasyon(kaput_modu='--kaput' in sys.argv)
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
