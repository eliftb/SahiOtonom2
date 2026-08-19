#!/usr/bin/env python3
"""Viraj hafızası regresyon testi - ARAÇ/KAMERA/ROS GEREKTİRMEZ.

DÜZELTİLEN HATA: viraja girildiği anda (viraj_mesafe_m < 0) yanlılık
    oran = 1.0 - max(viraj_mesafe_m, 0.0) / viraj_donus_mesafesi_m
ifadesindeki max(...) yüzünden 1.0'a KİLİTLENİYOR ve viraj_hafiza_m (12 m)
boyunca tam değerde kalıyordu. Dönüşün bittiğini anlayan hiçbir koşul yoktu:
tek çıkış 12 m yol gitmek ya da çizginin geri görünmesiydi. Çizgi görünmezse
araç tam kilitte dönmeye devam ediyor, yoldan çıktıkça çizgi daha da görünmez
oluyordu - kendi kendini besleyen kilit.

Artık çıkış ölçüsü ODOMETRİDEN DÖNÜLEN AÇI. Bu test onu doğrular.

    python3 SahiOtonom/test/test_viraj_hafiza.py
"""
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_lane_geometry import load_lane_module, LANE_SRC   # noqa: E402

mod = load_lane_module()
KAYNAK = open(LANE_SRC, encoding='utf-8').read()


def kodda_yazan(ad, varsayilan):
    """declare_parameter satırındaki GERÇEK değeri kaynaktan okur.

    Testin "araç şu an bunu çalıştırıyor" iddiası ancak böyle doğru kalır;
    elle kopyalanan sayı sessizce eskir.
    """
    m = re.search(r"declare_parameter\(\s*'%s'\s*,\s*([-\d.]+)\s*\)" % ad, KAYNAK)
    return float(m.group(1)) if m else varsayilan


class SahteLogger:
    def __init__(self):
        self.mesajlar = []

    def info(self, m):
        self.mesajlar.append(('info', m))

    def warn(self, m):
        self.mesajlar.append(('warn', m))


class SahteOdom:
    """nav_msgs/Odometry'nin bu düğümün kullandığı alanları."""
    def __init__(self, x, y, yaw):
        q = type('Q', (), {'w': math.cos(yaw / 2), 'x': 0.0,
                           'y': 0.0, 'z': math.sin(yaw / 2)})()
        p = type('P', (), {'x': x, 'y': y, 'z': 0.0})()
        poz = type('Poz', (), {'position': p, 'orientation': q})()
        self.pose = type('Pose', (), {'pose': poz})()


def dugum_kur(mesafe_m=1.5, yon=1):
    """__init__ çalıştırmadan viraj hafızası alanlarıyla düğüm kurar.

    Parametreler KAYNAKTAN okunur: pistte yüklü değerlerle test edilir.
    """
    n = object.__new__(mod.LaneDetectionNode)
    n.get_logger = lambda: n._logger
    n._logger = SahteLogger()

    n.viraj_donus_mesafesi_m = kodda_yazan('viraj_donus_mesafesi_m', 2.0)
    n.viraj_donus_kazanci = kodda_yazan('viraj_donus_kazanci', 0.45)
    n.viraj_hafiza_m = kodda_yazan('viraj_hafiza_m', 12.0)
    n.viraj_donus_acisi_deg = kodda_yazan('viraj_donus_acisi_deg', 90.0)
    n.viraj_birakma_orani = kodda_yazan('viraj_birakma_orani', 0.7)
    n.viraj_donus_yolu_m = kodda_yazan('viraj_donus_yolu_m', 4.0)
    n.viraj_zaman_asimi_s = kodda_yazan('viraj_zaman_asimi_s', 3.0)

    n.viraj_mesafe_m = mesafe_m
    n.viraj_yon = yon
    n.viraj_olculdu = True
    n.viraj_donulen = None
    n.viraj_veri_zamani = time.monotonic()
    n._son_konum = None
    n._son_yaw = None
    return n


def sur(n, adim_m, d_yaw, kare):
    """Aracı adım adım ilerletir. Dönen: her adımdan sonraki yanlılık listesi."""
    x, y, yaw = getattr(n, '_tx', 0.0), getattr(n, '_ty', 0.0), getattr(n, '_tyaw', 0.0)
    cikti = []
    for _ in range(kare):
        yaw += d_yaw
        x += adim_m * math.cos(yaw)
        y += adim_m * math.sin(yaw)
        n.odom_callback(SahteOdom(x, y, yaw))
        cikti.append(n._viraj_yanliligi())
    n._tx, n._ty, n._tyaw = x, y, yaw
    return cikti


SONUCLAR = []


def kontrol(ad, kosul, detay):
    SONUCLAR.append((ad, bool(kosul), detay))


# --- 1) ESKİ HATA: viraja girince tam yanlılıkta kilitlenmek ---------------
# 90 dereceyi dönecek kadar sürülür. Eski kodda yanlılık bu sürecin tamamında
# 0.45'te sabit kalır ve 12 m dolana kadar da öyle kalırdı.
n = dugum_kur()
tam = n.viraj_donus_kazanci
sur(n, 0.05, 0.0, 40)                       # viraja yaklaş (2 m yol)
girisdeki = n._viraj_yanliligi()
# 90 derece dön: adım başına 1.5 derece, 60 adım
egri = sur(n, 0.05, math.radians(1.5), 70)
son = n._viraj_yanliligi()

kontrol('viraja girişte yanlılık tam',
        abs(girisdeki - tam) < 1e-6,
        f'{girisdeki:.3f} (tam {tam:.2f})')

kontrol('90° dönülünce yanlılık BİTER (eski hata)',
        abs(son) < 1e-9,
        f'{son:.3f}')

kontrol('dönüş bitince viraj unutulur',
        n.viraj_mesafe_m is None and n.viraj_yon == 0,
        f'mesafe={n.viraj_mesafe_m} yon={n.viraj_yon}')

kontrol('bitiş mesajı basıldı ve AÇI ile bittiğini yazıyor',
        any('Viraj bitti' in m and 'açı' in m for _, m in n._logger.mesajlar),
        next((m.split('|')[0].strip() for _, m in n._logger.mesajlar), 'yok'))

# Dönüş boyunca yanlılık önce sabit sonra azalmalı; ASLA artmamalı
artan = [i for i in range(1, len(egri)) if egri[i] > egri[i - 1] + 1e-9]
kontrol('dönüş boyunca yanlılık hiç artmaz',
        not artan,
        f'{len(artan)} artış')

# --- 2) BIRAKMA FAZI: sonda sıfıra iner, ortada tam kalır ------------------
n2 = dugum_kur()
sur(n2, 0.05, 0.0, 40)
orta = sur(n2, 0.05, math.radians(1.5), 30)   # ~45 derece = %50
n2b = n2._viraj_yanliligi()
kontrol('dönüşün ortasında (%50) yanlılık hâlâ tam',
        abs(n2b - tam) < 1e-6,
        f'{n2b:.3f}')

n3 = dugum_kur()
sur(n3, 0.05, 0.0, 40)
sur(n3, 0.05, math.radians(1.5), 55)          # ~82 derece = %91
n3b = n3._viraj_yanliligi()
kontrol('dönüşün sonunda (%91) yanlılık azalmış ama bitmemiş',
        0.0 < n3b < tam,
        f'{n3b:.3f} (0 < x < {tam:.2f})')

# --- 3) EMNİYET: bilgi bayatlarsa viraj unutulur ---------------------------
n4 = dugum_kur()
n4.viraj_veri_zamani = time.monotonic() - (n4.viraj_zaman_asimi_s + 1.0)
bayat = n4._viraj_yanliligi()
kontrol('bayat viraj bilgisi kullanılmaz',
        abs(bayat) < 1e-9 and n4.viraj_mesafe_m is None,
        f'{bayat:.3f}, mesafe={n4.viraj_mesafe_m}')

# --- 4) ARAÇ HİÇ DÖNMÜYORSA: mesafe emniyeti dönüşü bitirir ---------------
# Direksiyon ters/takılıysa dönülen açı hiç ilerlemez. Açı tek ölçü olsaydı
# dönüş asla bitmez, düzeltmeye çalıştığımız kilit geri gelirdi.
n5 = dugum_kur()
sur(n5, 0.05, 0.0, 40)                        # viraja gir
donmeden = sur(n5, 0.05, 0.0, 200)            # 10 m DÜMDÜZ git (hiç dönmüyor)
bitti_mi = donmeden[-1]
bitis_adimi = next((i for i, v in enumerate(donmeden) if v == 0.0), None)
kontrol('araç dönmezse mesafe emniyeti bitirir',
        abs(bitti_mi) < 1e-9 and bitis_adimi is not None,
        f'{(bitis_adimi or 0) * 0.05:.1f} m sonra bitti')

kontrol('emniyet hafıza sınırından (12 m) ÖNCE devreye girer',
        bitis_adimi is not None and bitis_adimi * 0.05 < n5.viraj_hafiza_m,
        f'{(bitis_adimi or 0) * 0.05:.1f} m < {n5.viraj_hafiza_m:.0f} m')

# --- 5) YAKLAŞMA FAZI değişmedi -------------------------------------------
n6 = dugum_kur(mesafe_m=2.0)
kontrol('eşik dışında yanlılık yok',
        abs(n6._viraj_yanliligi()) < 1e-9,
        f'{n6._viraj_yanliligi():.3f}')
n6.viraj_mesafe_m = 1.0
yarim = n6._viraj_yanliligi()
kontrol('yaklaşmada yanlılık kademeli artar',
        abs(yarim - tam * 0.5) < 1e-6,
        f'{yarim:.3f} (beklenen {tam * 0.5:.3f})')

# --- 6) Taze kamera ölçümü dönüş sayacını sıfırlar -------------------------
n7 = dugum_kur()
sur(n7, 0.05, 0.0, 40)
sur(n7, 0.05, math.radians(1.5), 30)
onceki = n7.viraj_donulen
n7.viraj_mesafe_m = 1.2            # kamera virajı YENİDEN ölçtü (ileride)
n7.viraj_donulen = None            # _viraj_guncelle'nin yaptığı
kontrol('yeniden ölçülen viraj yaklaşma fazına döner',
        n7._viraj_tamamlanma() is None and onceki is not None,
        f'donulen {math.degrees(onceki):.0f}° -> sıfırlandı')

# --- RAPOR ----------------------------------------------------------------
print()
print(f'{"test":<52}{"detay":<28}sonuç')
print('-' * 88)
hata = 0
for ad, ok, detay in SONUCLAR:
    if not ok:
        hata += 1
    print(f'{ad:<52}{detay:<28}{"OK" if ok else "HATA <<<"}')
print('-' * 88)
print('TÜMÜ GEÇTİ' if not hata else f'{hata} TEST BAŞARISIZ')
sys.exit(1 if hata else 0)
