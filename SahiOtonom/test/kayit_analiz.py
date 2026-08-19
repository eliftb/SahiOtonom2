#!/usr/bin/env python3
"""GERÇEK PİST KAYDI ÜZERİNDE ŞERİT TAKİBİ ANALİZİ.

NEDEN: Sentetik maskelerle yazılan testler geçiyor ama araç pistte şerit
değiştiriyor - yani sentetik model gerçek arızayı temsil etmiyor. Bu araç
GERÇEK kamera karelerini (kayitlar/*.db3) alıp GERÇEK düğüm kodunu çalıştırır;
her karede hangi çizgilerin bulunduğunu, hangisinin seçildiğini ve rotanın
nereye kurulduğunu tabloya döker. Piste çıkmadan, tekrar tekrar.

KULLANIM:
  python3 test/kayit_analiz.py kayitlar/pist_20260818_111612
  python3 test/kayit_analiz.py kayitlar/pist_... --kare 300 --atla 5
  python3 test/kayit_analiz.py kayitlar/pist_... --ayar route_smoothing=0.7

  --ayar ile parametre değiştirip AYNI kayıtta sonucu kıyaslayabilirsiniz;
  pistte deneme yanılma yapmaya gerek kalmaz.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def bag_kareleri(klasor, konu='/zed2i_rgb/image_raw', gecici='/tmp/kayit_analiz'):
    """Kayıttaki görüntü mesajlarını sırayla verir.

    Kayıtlar zstd ile DOSYA modunda sıkıştırılıyor; rosbag2'nin sqlite3 eklentisi
    .db3.zstd'yi doğrudan açamıyor. Parçalar tek tek geçici klasöre açılıp
    sqlite ile okunur - böylece 16 dakikalık kayıt için 40 GB yer gerekmez.
    """
    import glob, shutil, sqlite3, subprocess
    os.makedirs(gecici, exist_ok=True)
    parcalar = sorted(glob.glob(os.path.join(klasor, '*.db3.zstd')),
                      key=lambda p: int(p.split('_')[-1].split('.')[0]))
    if not parcalar:                      # sıkıştırılmamış kayıt
        parcalar = sorted(glob.glob(os.path.join(klasor, '*.db3')),
                          key=lambda p: int(p.split('_')[-1].split('.')[0]))
    for parca in parcalar:
        if parca.endswith('.zstd'):
            hedef = os.path.join(gecici, 'parca.db3')
            subprocess.run(['zstd', '-d', '-q', '-f', parca, '-o', hedef], check=True)
        else:
            hedef = parca
        try:
            baglanti = sqlite3.connect(hedef)
            sorgu = ("select m.timestamp, m.data from messages m "
                     "join topics t on m.topic_id = t.id where t.name = ? "
                     "order by m.timestamp")
            for zaman, veri in baglanti.execute(sorgu, (konu,)):
                yield zaman, deserialize_message(bytes(veri), Image)
            baglanti.close()
        finally:
            if hedef.startswith(gecici):
                os.remove(hedef)


def dugumu_kur(ayarlar):
    """Gerçek LaneDetectionNode'u kurar (model dahil), ROS döngüsü olmadan."""
    import importlib.util
    yol = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'SeritTespit', 'serit-tespitcopy.py')
    sys.path.insert(0, os.path.dirname(yol))
    spec = importlib.util.spec_from_file_location('serit_tespit', yol)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['serit_tespit'] = mod
    spec.loader.exec_module(mod)

    rclpy.init()
    dugum = mod.LaneDetectionNode()
    # Debug görüntüsü üretmek analiz için gereksiz; kapatınca çok daha hızlı
    dugum.debug_every_n = 0
    dugum.debug_rows_log = False
    for ad, deger in ayarlar.items():
        mevcut = getattr(dugum, ad, None)
        if mevcut is None:
            print(f'! bilinmeyen ayar: {ad}', file=sys.stderr)
            continue
        setattr(dugum, ad, type(mevcut)(deger))
        print(f'  ayar: {ad} = {getattr(dugum, ad)}')
    return dugum


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kayit', help='kayitlar/pist_... klasörü')
    ap.add_argument('--kare', type=int, default=400, help='işlenecek kare sayısı')
    ap.add_argument('--atla', type=int, default=1, help='her N karede bir işle')
    ap.add_argument('--ayar', action='append', default=[],
                    help='parametre=deger (birden çok kez verilebilir)')
    ap.add_argument('--satirlar', action='store_true',
                    help='her karede satır satır aday/seçim dökümü')
    a = ap.parse_args()

    ayarlar = dict(s.split('=', 1) for s in a.ayar)
    dugum = dugumu_kur(ayarlar)

    print(f'\nkayit: {a.kayit}')
    print(f'{"kare":>5} {"merkez":>7} {"sapma":>7} {"viraj":>7} {"kaynak":>7} '
          f'{"genislik":>9} {"satir":>5}  sicrama')
    print('-' * 72)

    onceki_merkez = None
    merkezler, sicramalar = [], []
    islenen = 0

    for i, (_, msg) in enumerate(bag_kareleri(a.kayit)):
        if i % a.atla:
            continue
        if islenen >= a.kare:
            break
        dugum.image_callback(msg)
        islenen += 1

        merkez = dugum.debug_center[0] if dugum.debug_center else None
        sapma = dugum.deviation_history[-1] if dugum.deviation_history else 0.0
        gen = (float(np.median(dugum.width_samples))
               if len(dugum.width_samples) else float('nan'))

        sicrama = ''
        if merkez is not None and onceki_merkez is not None:
            d = abs(merkez - onceki_merkez)
            sicramalar.append(d)
            # Bir şerit genişliğinin yarısından büyük tek kare sıçraması =
            # neredeyse kesin ŞERİT DEĞİŞTİRME, ölçüm değil.
            if not np.isnan(gen) and d > 0.5 * gen:
                sicrama = f'<<< {d:.0f} px SICRAMA (serit degisimi?)'
        if merkez is not None:
            merkezler.append(merkez)
            onceki_merkez = merkez

        print(f'{islenen:>5} {("-" if merkez is None else f"{merkez:.0f}"):>7} '
              f'{sapma:>+7.3f} {dugum.debug_curve:>+7.3f} {dugum.debug_source:>7} '
              f'{("-" if np.isnan(gen) else f"{gen:.0f}"):>9} '
              f'{len(dugum.debug_rows):>5}  {sicrama}')

        if a.satirlar and dugum.debug_rows:
            adaylar = {y: p for y, p in getattr(dugum, 'debug_points', [])}
            for y, sol, sag, mrk in dugum.debug_rows:
                p = adaylar.get(y, [])
                print(f'        y={y:3d} sol={"-" if sol is None else int(sol):>5} '
                      f'sag={"-" if sag is None else int(sag):>5} mrk={mrk:6.0f}'
                      f'  adaylar=[{" ".join(str(int(x)) for x in p)}]')

    print('-' * 72)
    if sicramalar:
        s = np.array(sicramalar)
        gen = (float(np.median(dugum.width_samples))
               if len(dugum.width_samples) else float('nan'))
        buyuk = int((s > 0.5 * gen).sum()) if not np.isnan(gen) else 0
        print(f'islenen kare      : {islenen}')
        print(f'ogrenilen genislik: {gen:.0f} px')
        print(f'kare-kare oynaklik: ortalama {s.mean():.1f} px, medyan {np.median(s):.1f} px')
        print(f'BUYUK SICRAMA     : {buyuk} kare  (serit genisliginin yarisindan fazla)')
        if merkezler:
            print(f'merkez araligi    : {min(merkezler):.0f} - {max(merkezler):.0f} px '
                  f'(yayilim {max(merkezler) - min(merkezler):.0f} px)')

    dugum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
