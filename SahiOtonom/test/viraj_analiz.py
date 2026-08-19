#!/usr/bin/env python3
"""VIRAJ ANALIZI - kayittan, ARAC GEREKMEZ.

NEDEN AYRI BIR ARAC: test/kayit_analiz.py yalnizca /zed2i_rgb/image_raw
besliyor. Serit dugumu route_source='mesafe' modunda calisiyor ve o modda
sag cizginin mesafesi de viraj tespiti de DERINLIGE dayaniyor; derinlik
beslenmezse fx/depth_image None kalir, _sag_cizgi_mesafesi her karede None
doner ve kayit "arac hicbir sey gormedi" gibi gorunur - oysa sorun analiz
aracindadir. Bu arac derinlik + camera_info + odometriyi de zaman sirasiyla
besler, yani kayit pistteki gibi islenir.

IKI GECIS:
  --ozet   Sadece /zed2i/odom okur (goruntu cozmez, YOLOP calistirmaz).
           Saniyeler icinde "arac ne zaman dondu, ne zaman durdu" cikarir.
           347 saniyelik kayitta virajin yerini boyle buluruz.
  (tam)    Secilen zaman penceresinde GERCEK dugum kodunu calistirir ve
           her karede olculen mesafeyi, virajin gorulup gorulmedigini,
           yanliligi ve serit gecerliligini doker.

KULLANIM:
    python3 test/viraj_analiz.py kayitlar/pist_20260819_104142 --ozet
    python3 test/viraj_analiz.py kayitlar/pist_... --basla 250 --sure 30
    python3 test/viraj_analiz.py kayitlar/pist_... --basla 250 --sure 30 \
            --ayar viraj_donus_mesafesi_m=4.0
"""
import argparse
import glob
import math
import os
import subprocess
import sqlite3
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry

TIPLER = {
    '/zed2i_rgb/image_raw': Image,
    '/zed2i/depth': Image,
    '/zed2i/camera_info': CameraInfo,
    '/zed2i/odom': Odometry,
}


def parcalar(klasor, bas=None, son=None):
    """Kayit parcalari, SIRA NUMARASINA gore.

    Aralik secilebiliyor cunku bir kayit 19-25 GB: hepsini acmak dakikalar
    suruyor ve viraj genelde SONDA. Parcalar zaman sirali oldugu icin
    '--parca -4' demek 'son ~80 saniye' demektir.
    """
    p = sorted(glob.glob(os.path.join(klasor, '*.db3.zstd')),
               key=lambda x: int(x.split('_')[-1].split('.')[0]))
    p = p or sorted(glob.glob(os.path.join(klasor, '*.db3')),
                    key=lambda x: int(x.split('_')[-1].split('.')[0]))
    if bas is not None or son is not None:
        p = p[bas:son]
    return p


def mesajlar(klasor, konular, gecici='/tmp/viraj_analiz', t0=None, t1=None,
             p_bas=None, p_son=None):
    """(zaman_sn, konu, mesaj) - ZAMAN SIRASINDA, konular arasi karisik.

    Sira onemli: derinlik karesi kendi goruntusunden SONRA beslenirse dugum
    bir kare eski derinlikle olcum yapar. Pistte de oyle oluyor ama analizde
    sirayi bozmak, olculen mesafeyi gercekte olmayan bir gecikmeyle kaydirir.
    """
    os.makedirs(gecici, exist_ok=True)
    baslangic = None
    yer_tutucu = ','.join('?' * len(konular))
    for parca in parcalar(klasor, p_bas, p_son):
        if parca.endswith('.zstd'):
            hedef = os.path.join(gecici, 'parca.db3')
            # BOZUK PARCA OLABILIR. Kayit zorla kesilince (ya da yetim kalan
            # kaydedici oldurulunce) sikistirilmakta olan son parca yarim
            # kalir ve zstd 'premature end' der. Tek bozuk parca yuzunden
            # analizin tamamini kaybetmek sacma: uyar, atla, devam et.
            sonuc = subprocess.run(['zstd', '-d', '-q', '-f', parca, '-o', hedef],
                                   capture_output=True)
            if sonuc.returncode != 0:
                print(f'  ! BOZUK PARCA atlandi: {os.path.basename(parca)}',
                      file=sys.stderr)
                if os.path.exists(hedef):
                    os.remove(hedef)
                continue
        else:
            hedef = parca
        try:
            baglanti = sqlite3.connect(hedef)
            sorgu = (f"select m.timestamp, t.name, m.data from messages m "
                     f"join topics t on m.topic_id = t.id "
                     f"where t.name in ({yer_tutucu}) order by m.timestamp")
            for zaman, konu, veri in baglanti.execute(sorgu, tuple(konular)):
                if baslangic is None:
                    baslangic = zaman
                sn = (zaman - baslangic) / 1e9
                if t1 is not None and sn > t1:
                    baglanti.close()
                    return
                if t0 is not None and sn < t0:
                    continue
                yield sn, konu, deserialize_message(bytes(veri), TIPLER[konu])
            baglanti.close()
        finally:
            if hedef.startswith(gecici) and os.path.exists(hedef):
                os.remove(hedef)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ozet(klasor, p_bas=None, p_son=None):
    """Sadece odometri: arac ne zaman ilerledi, ne zaman dondu."""
    print('  /zed2i/odom okunuyor (goruntu cozulmuyor)...\n')
    kayitlar = []
    for sn, _, msg in mesajlar(klasor, ['/zed2i/odom'], p_bas=p_bas, p_son=p_son):
        p = msg.pose.pose.position
        kayitlar.append((sn, p.x, p.y, math.degrees(yaw_of(msg.pose.pose.orientation))))
    if not kayitlar:
        print('  Odometri yok.')
        return

    print(f'  {len(kayitlar)} odometri mesaji, {kayitlar[-1][0]:.1f} sn\n')
    print(f'  {"sn":>6} {"yol_m":>7} {"hiz_m/s":>8} {"yaw":>7} {"donus/sn":>9}  durum')
    print('  ' + '-' * 62)

    PENCERE = 1.0
    onceki = kayitlar[0]
    yol = 0.0
    yazilacak = []
    son_yazim = -PENCERE
    onceki_yaw = kayitlar[0][3]
    for sn, x, y, yaw in kayitlar[1:]:
        dt = sn - onceki[0]
        d = math.hypot(x - onceki[1], y - onceki[2])
        if d < 2.0:
            yol += d
        if sn - son_yazim >= PENCERE:
            hiz = d / dt if dt > 0 else 0.0
            dyaw = math.degrees(math.atan2(math.sin(math.radians(yaw - onceki_yaw)),
                                           math.cos(math.radians(yaw - onceki_yaw))))
            hiz_p = (yol - (yazilacak[-1][1] if yazilacak else 0.0)) / PENCERE
            durum = []
            if hiz_p < 0.05:
                durum.append('DURUYOR')
            if abs(dyaw) > 8:
                durum.append(f'DONUYOR {"SOLA" if dyaw > 0 else "SAGA"}')
            yazilacak.append((sn, yol, hiz_p, yaw, dyaw, ' '.join(durum)))
            son_yazim = sn
            onceki_yaw = yaw
        onceki = (sn, x, y, yaw)

    for sn, yol_m, hiz, yaw, dyaw, durum in yazilacak:
        print(f'  {sn:>6.1f} {yol_m:>7.2f} {hiz:>8.2f} {yaw:>+7.1f} {dyaw:>+9.1f}  {durum}')

    donusler = [z for z in yazilacak if abs(z[4]) > 8]
    print('\n  ' + '-' * 62)
    print(f'  toplam yol: {yol:.1f} m')
    if donusler:
        ilk, son = donusler[0][0], donusler[-1][0]
        toplam = yazilacak[-1][3] - yazilacak[0][3]
        print(f'  DONUS PENCERESI: {ilk:.0f} - {son:.0f} sn '
              f'(net yaw degisimi {toplam:+.0f} derece)')
        print(f'\n  Ayrintili analiz icin:')
        print(f'    python3 test/viraj_analiz.py {klasor} '
              f'--basla {max(0, ilk - 10):.0f} --sure {son - ilk + 20:.0f}')
    else:
        print('  Belirgin donus yok (yaw hep sabit).')


def dugumu_kur(ayarlar):
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
    dugum.debug_every_n = 0
    dugum.debug_rows_log = False
    for ad, deger in ayarlar.items():
        mevcut = getattr(dugum, ad, None)
        if mevcut is None:
            print(f'  ! bilinmeyen ayar: {ad}', file=sys.stderr)
            continue
        setattr(dugum, ad, type(mevcut)(deger))
        print(f'  ayar: {ad} = {getattr(dugum, ad)}')
    return dugum


def tam_analiz(klasor, t0, sure, ayarlar, atla, p_bas=None, p_son=None):
    dugum = dugumu_kur(ayarlar)
    t1 = t0 + sure
    print(f'\n  pencere: {t0:.0f} - {t1:.0f} sn   '
          f'(hedef sag mesafe {dugum.hedef_sag_mesafe_m} m)\n')
    print(f'  {"sn":>6} {"mesafe":>7} {"sapma":>7} {"gecerli":>7} {"kaynak":>13}')
    print('  ' + '-' * 72)

    kare = 0
    satirlar = []
    for sn, konu, msg in mesajlar(klasor, list(TIPLER), t0=t0, t1=t1,
                                  p_bas=p_bas, p_son=p_son):
        if konu == '/zed2i/depth':
            dugum.depth_callback(msg)
        elif konu == '/zed2i/camera_info':
            dugum.camera_info_callback(msg)
        elif konu == '/zed2i/odom':
            # Serit dugumu artik odometri kullanmiyor (viraj algilama
            # kaldirildi); mesaj okunur ama dugume beslenmez.
            pass
        elif konu == '/zed2i_rgb/image_raw':
            kare += 1
            if atla > 1 and kare % atla:
                continue
            dugum.image_callback(msg)
            mesafe = dugum.son_mesafe_m
            sapma = dugum.deviation_history[-1] if dugum.deviation_history else 0.0
            satirlar.append((sn, mesafe, sapma, dugum.lane_valid, dugum.debug_source))
            print(f'  {sn:>6.1f} '
                  f'{("-" if mesafe is None else f"{mesafe:.2f}"):>7} '
                  f'{sapma:>+7.3f} {str(dugum.lane_valid):>7} {dugum.debug_source:>13}')

    if not satirlar:
        print('\n  Bu pencerede goruntu yok.')
        return

    print('\n  ' + '-' * 72)
    olculen = [s[1] for s in satirlar if s[1] is not None]
    gecerli = sum(1 for s in satirlar if s[3])
    kaynaklar = {}
    for s in satirlar:
        kaynaklar[s[4]] = kaynaklar.get(s[4], 0) + 1

    print(f'  islenen kare        : {len(satirlar)}')
    print(f'  serit gecerli       : {gecerli}/{len(satirlar)} '
          f'(%{gecerli / len(satirlar) * 100:.0f})')
    print(f'  mesafe olculen kare : {len(olculen)}/{len(satirlar)} '
          f'(%{len(olculen) / len(satirlar) * 100:.0f})')
    if olculen:
        print(f'  mesafe medyan/min/max: {np.median(olculen):.2f} / '
              f'{min(olculen):.2f} / {max(olculen):.2f} m '
              f'(hedef {dugum.hedef_sag_mesafe_m})')
    print(f'  kaynak dagilimi     : ' +
          ', '.join(f'{k}={v}' for k, v in sorted(kaynaklar.items(), key=lambda x: -x[1])))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kayit')
    ap.add_argument('--ozet', action='store_true',
                    help='sadece odometri: virajin kayittaki yerini bul')
    ap.add_argument('--basla', type=float, default=0.0, help='pencere baslangici (sn)')
    ap.add_argument('--sure', type=float, default=20.0, help='pencere suresi (sn)')
    ap.add_argument('--atla', type=int, default=1, help='her N karede bir isle')
    ap.add_argument('--ayar', action='append', default=[], help='ad=deger')
    ap.add_argument('--parca', type=int, default=None, metavar='N',
                    help='sadece son N parca (negatif) ya da ilk N parca (pozitif)')
    a = ap.parse_args()

    p_bas = p_son = None
    if a.parca is not None:
        if a.parca < 0:
            p_bas = a.parca          # son N parca
        else:
            p_son = a.parca          # ilk N parca

    if a.ozet:
        ozet(a.kayit, p_bas, p_son)
        return 0
    tam_analiz(a.kayit, a.basla, a.sure,
               dict(s.split('=', 1) for s in a.ayar), a.atla, p_bas, p_son)
    return 0


if __name__ == '__main__':
    sys.exit(main())
