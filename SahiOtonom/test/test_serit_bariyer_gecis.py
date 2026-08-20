#!/usr/bin/env python3
"""Iki duzeltmeyi de kanitlar. ROS/ARAC gerektirmez."""
import importlib.util, os, sys, types
import numpy as np
import time

KOK = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
SRC = os.path.join(KOK, 'SeritTespit/serit-tespitcopy.py')

for n in ['rclpy','rclpy.node','rclpy.executors','sensor_msgs','sensor_msgs.msg',
          'std_msgs','std_msgs.msg','rcl_interfaces','rcl_interfaces.msg',
          'cv_bridge','torch','utils','utils.utils']:
    sys.modules.setdefault(n, types.ModuleType(n))
sys.modules['rclpy.node'].Node = type('Node', (), {})
sys.modules['rclpy.executors'].ExternalShutdownException = Exception
class _M:
    def __init__(s, data=None): s.data = data
for a in ('Float32','Int32','Bool'): setattr(sys.modules['std_msgs.msg'], a, _M)
sys.modules['sensor_msgs.msg'].Image = object
sys.modules['sensor_msgs.msg'].CameraInfo = object
sys.modules['rcl_interfaces.msg'].SetParametersResult = object
sys.modules['cv_bridge'].CvBridge = object
sys.modules['utils.utils'].select_device = lambda *a, **k: None
sys.modules['utils.utils'].driving_area_mask = lambda *a: None
sys.modules['utils.utils'].lane_line_mask = lambda *a: None

spec = importlib.util.spec_from_file_location('serit', SRC)
mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
LDN = mod.LaneDetectionNode

class Log:
    def __init__(s): s.satirlar = []
    def info(s, m): s.satirlar.append(m)
    def warn(s, m): s.satirlar.append(m)
    def error(s, m): s.satirlar.append(m)

H, W_PX = 720, 1280
FX = 700.0

def dugum():
    n = object.__new__(LDN)
    n._log = Log(); n.get_logger = lambda: n._log
    n.sample_rows = (0.80, 0.76, 0.72, 0.68, 0.64, 0.60)
    n.camera_center_offset_px = 0.0
    n.paint_inside_only = True
    n.derinlik_pencere_px = 7
    n.fx = FX; n.cx = 640.0
    n.depth_image = np.full((H, W_PX), 4.0, dtype=np.float32)
    n.olcum_ileri_mesafe_m = 3.0
    n.ayni_cizgi_tol_m = 1.0
    # _mesafe_sapmasi icin gerekenler
    n.mesafe_alt_sinir_m = 0.5; n.mesafe_ust_sinir_m = 2.5
    n.hedef_sag_mesafe_m = 1.5; n.mesafe_hata_olcegi_m = 2.5
    n.serit_genisligi_m = 3.0
    n.merkez_bandi_m = 0.5; n.kenar_kazanci = 1.0
    n.mesafe_sicrama_esigi_m = 0.8; n.mesafe_sicrama_kabul_kare = 8
    n.gecerlilik_kayip_kare = 12
    n.deviation_deadband = 0.02; n.max_deviation_rate = 2.5
    n.deviation_history = []; n.max_history_size = 5; n.last_update_time = None
    n._sicrama_sayaci = 0; n.lost_frames = 0; n.lane_valid = False
    n.son_mesafe_m = None; n.debug_source = '-'
    n.debug_fit = None; n.debug_center = None; n.debug_curve = 0.0
    return n

def maske(sutunlar):
    m = np.zeros((H, W_PX), np.uint8)
    for x0, x1 in sutunlar: m[:, x0:x1] = 1
    return m

def da_maskesi(x0, x1):
    d = np.zeros((H, W_PX), np.uint8); d[:, x0:x1] = 1; return d

ok = True
def kontrol(ad, kosul, detay):
    global ok
    print(f'  {"OK  " if kosul else "HATA"}  {ad:<52} {detay}')
    if not kosul: ok = False

print('=' * 82)
print('BUG 1 - Bariyer filtresi (metrik mod, _sag_cizgi_mesafesi)')
print('=' * 82)
# Sag virajda uzak satirda yol tamamen merkezin SAGINDA: surulebilir alan
# [750,1100]. Omuzdaki korkuluk x=700'de (alan DISINDA), gercek sag serit
# cizgisi x=1050'de (alan ICINDE).
lane = maske([(695, 705), (1045, 1055)])
da = da_maskesi(750, 1100)

n = dugum()
sonuc = n._sag_cizgi_mesafesi(lane, da)
bekl = (1050 - 640) * 4.0 / FX
kontrol('bariyer elenir, GERCEK cizgi olculur',
        sonuc is not None and abs(sonuc[0] - bekl) < 0.05,
        f'{sonuc[0]:.2f} m (beklenen {bekl:.2f})' if sonuc else 'None')

# Filtresiz (paint_inside_only kapali) ESKI davranis: bariyeri olcerdi
n2 = dugum(); n2.paint_inside_only = False
eski = n2._sag_cizgi_mesafesi(lane, da)
bekl_eski = (700 - 640) * 4.0 / FX
kontrol('filtre KAPALI iken eski (hatali) deger geri geliyor',
        eski is not None and abs(eski[0] - bekl_eski) < 0.05,
        f'{eski[0]:.2f} m = bariyer (kontrol grubu)' if eski else 'None')

# Regresyon: cizgi alanin icindeyken hicbir sey elenmemeli
n3 = dugum()
lane3 = maske([(895, 905)]); da3 = da_maskesi(300, 950)
s3 = n3._sag_cizgi_mesafesi(lane3, da3)
b3 = (900 - 640) * 4.0 / FX
kontrol('normal durum: alan icindeki cizgi hala olculuyor',
        s3 is not None and abs(s3[0] - b3) < 0.05,
        f'{s3[0]:.2f} m (beklenen {b3:.2f})' if s3 else 'None')

# da_mask None (testlerin piksel yolu) cokmemeli
n4 = dugum()
s4 = n4._sag_cizgi_mesafesi(lane3, None)
kontrol('da_mask None ise cokmez, eleme yapilmaz',
        s4 is not None and abs(s4[0] - b3) < 0.05,
        f'{s4[0]:.2f} m' if s4 else 'None')

print()
print('=' * 82)
print('BUG 2 - SAG cizgi gecisi (simetrik -W dali)')
print('=' * 82)
n = dugum()
# Serit 3.0 m. Kendi sag cizgimiz +0.60 m'de (seridin sag kenarina yakiniz),
# komsu seridin cizgisi +3.60 m'de. Arac 0.70 m saga kayar: kendi cizgimiz
# -0.10 m'ye (SOLUMUZA) gecer ve artik 'en yakin sag cizgi' komsununki (2.90).
olcumler = [0.60] + [2.90] * 8
etiket = (['sag cizgiye yakin (ham 0.60 m)',
           'SAG CIZGI GECILDI (ham 2.90 m = komsu seridin cizgisi)']
          + [f'hala komsu seritte (kare {k})' for k in range(2, 9)])
for ham, ad in zip(olcumler, etiket):
    time.sleep(0.06)          # ~17 FPS: hiz limiti gercekci olsun
    n._sag_cizgi_mesafesi = lambda lm, dm, _h=ham: (_h, 900, 500)
    sapma = n._mesafe_sapmasi(lane3, da3)
    print(f'    {ad}')
    ref = 'YOK' if n.son_mesafe_m is None else f'{n.son_mesafe_m:+.2f} m'
    if ad.startswith('hala') and not ad.endswith('8)'):
        continue
    print(f'      -> kaynak={n.debug_source:<14} referans={ref}'
          f'  sapma={sapma:+.3f}  ({"SOLA kir" if sapma < 0 else "SAGA kir"})')
kontrol('gecis yakalandi (kaynak serit-gecildi)',
        n.debug_source == 'serit-gecildi', n.debug_source)
kontrol('referans NEGATIF: kendi cizgimizin sagindayiz',
        n.son_mesafe_m < 0, f'{n.son_mesafe_m:+.2f} m')
kontrol('sapma doyuma rampaladi (-1.6 m hata / 2.5 = -0.64)',
        sapma < -0.6, f'{sapma:+.3f}')
kontrol('log SAG gecisi bildiriyor',
        any('SAĞ ŞERİT ÇİZGİSİ GEÇİLDİ' in s for s in n._log.satirlar),
        [s for s in n._log.satirlar if 'GEÇİLDİ' in s][:1])

print()
print('  --- SOL cizgi gecisi (mevcut dal, regresyon) ---')
n = dugum()
# Kendi sag cizgimiz +2.40 m'de (sol cizgi -0.60 m). Arac 0.70 m sola kayar:
# sol cizgi +0.10 m'ye gecip 'en yakin sag cizgi' olur.
for ham in [2.40] + [0.10] * 8:
    time.sleep(0.06)
    n._sag_cizgi_mesafesi = lambda lm, dm, _h=ham: (_h, 900, 500)
    sapma = n._mesafe_sapmasi(lane3, da3)
print(f'      -> kaynak={n.debug_source:<14} referans={n.son_mesafe_m:+.2f} m'
      f'  sapma={sapma:+.3f}  ({"SOLA kir" if sapma < 0 else "SAGA kir"})')
kontrol('SOL gecis hala yakalaniyor', n.debug_source == 'serit-gecildi', n.debug_source)
kontrol('sapma doyuma rampaladi (+1.6 m hata / 2.5 = +0.64)',
        sapma > 0.6, f'{sapma:+.3f}')

print()
print('  --- normal surus: gecis dali TETIKLENMEMELI (yanlis pozitif yok) ---')
n = dugum()
for ham in (1.50, 1.55, 1.62, 1.70, 1.45):
    n._sag_cizgi_mesafesi = lambda lm, dm, _h=ham: (_h, 900, 500)
    sapma = n._mesafe_sapmasi(lane3, da3)
kontrol('kucuk hareketlerde kaynak "mesafe" kaliyor',
        n.debug_source == 'mesafe', n.debug_source)
kontrol('hic gecis logu yok',
        not any('GEÇİLDİ' in s for s in n._log.satirlar),
        f'{len([s for s in n._log.satirlar if "GEÇİLDİ" in s])} log')

print()
print('=' * 82)
print('TUMU GECTI' if ok else 'BASARISIZ')
sys.exit(0 if ok else 1)
