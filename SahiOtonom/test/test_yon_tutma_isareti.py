#!/usr/bin/env python3
"""Direksiyon İŞARET tutarlılığı testi - ARAÇ/ROS GEREKTİRMEZ.

DÜZELTİLEN HATA: uart_sender_node3.py'de direksiyon açısı üreten İKİ yol var
ve işaret kuralları TERSTİ:

    PID yolu       :  steering_direction * -(p+i+d)     -> -1.0 uygulanıyor
    Yön tutma yolu :  kp_heading * hata                 -> HİÇ uygulanmıyor

Hangisinin aktif olduğu /lane/valid'e bağlı: şerit görünürken PID, şerit
kaybolunca (viraj/kavşak) yön tutma. Yani araç düz yolda doğru, virajda TERS
kırıyordu. Kayıttaki belirti: 'SOLA DÖN' emri verilirken araç sağa aktı.

Bu test ikisinin AYNI fiziksel yöne komut verdiğini doğrular.

    python3 SahiOtonom/test/test_yon_tutma_isareti.py
"""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_direksiyon_byte import load_uart_module, UART_SRC   # noqa: E402

mod = load_uart_module()
KAYNAK = open(UART_SRC, encoding='utf-8').read()


def kodda_yazan(ad, varsayilan):
    m = re.search(r"declare_parameter\(\s*'%s'\s*,\s*(-?[\d.]+)\s*\)" % ad, KAYNAK)
    return float(m.group(1)) if m else varsayilan


TRIM = int(kodda_yazan('steering_trim', 50))
MERKEZ = mod.merkez_byte(TRIM)          # tekerlerin GERÇEKTEN düz olduğu byte


class SahteLogger:
    def info(self, m): pass
    def warn(self, m): pass


class Msg:
    def __init__(self, data): self.data = data


def dugum_kur():
    """__init__ çalıştırmadan, PİSTTE YÜKLÜ parametrelerle düğüm kurar."""
    n = object.__new__(mod.UartSenderNode)
    n.get_logger = lambda: SahteLogger()

    n.steering_direction = kodda_yazan('steering_direction', -1.0)
    n.steering_trim = TRIM
    n.max_steering_angle = kodda_yazan('max_steering_angle', 0.5)
    n.kp = kodda_yazan('kp', 0.8)
    n.ki = kodda_yazan('ki', 0.0)
    n.kd = kodda_yazan('kd', 0.3)
    n.i_limit = kodda_yazan('i_limit', 0.06)
    n.d_filter = kodda_yazan('d_filter', 0.3)
    n.kp_heading = kodda_yazan('kp_heading', 1.2)
    n.turn_angle_deg = kodda_yazan('turn_angle_deg', 90.0)
    n.heading_hold = True
    n.heading_hold_max_sec = kodda_yazan('heading_hold_max_sec', 6.0)

    n.integral = 0.0
    n.prev_error = 0.0
    n.d_filtered = 0.0
    n.last_pid_time = None
    n.current_lateral_deviation = 0.0
    n.serit_gecerli = True
    n.guncel_yaw = 0.0
    n.hedef_yaw = None
    n.bekleyen_donus = 0
    n.yon_tutma_basladi = None
    n.yon_tutma_uyarildi = False
    return n


def kavsakta_byte(donus_emri, donuldu_rad=0.0):
    """Kavşak emri verilip şerit kaybolduğunda gidecek direksiyon byte'ı."""
    n = dugum_kur()
    n.bekleyen_donus = donus_emri
    n.guncel_yaw = 0.0
    n.serit_gecerli_callback(Msg(False))       # şerit kayboldu
    n.guncel_yaw = donuldu_rad                 # araç bu kadar dönmüş durumda
    aci = n.yon_tutma_direksiyonu()
    return n.angle_to_byte(aci), n.hedef_yaw


def seritte_byte(sapma):
    """Şerit görünürken verilen sapmanın gideceği direksiyon byte'ı."""
    n = dugum_kur()
    return n.angle_to_byte(n.lateral_deviation_to_steering_angle(sapma))


SONUC = []


def kontrol(ad, kosul, detay):
    SONUC.append((ad, bool(kosul), detay))


# --- 1) ŞERİT YOLU: yön kuralının kendisi ---------------------------------
# Log biçimi: sapma > 0 = "SOL tarafta -> SAĞA". Sağ = byte MERKEZ'in üstü.
sag = seritte_byte(+0.4)
sol = seritte_byte(-0.4)
kontrol('şerit: sapma>0 (araç solda) SAĞA kırar',
        sag > MERKEZ, f'd,{sag} > {MERKEZ}')
kontrol('şerit: sapma<0 (araç sağda) SOLA kırar',
        sol < MERKEZ, f'd,{sol} < {MERKEZ}')

# --- 2) YÖN TUTMA YOLU: aynı fiziksel yön kuralına uymalı -----------------
sola_byte, sola_hedef = kavsakta_byte(-1)      # -1 = SOLA emri
saga_byte, saga_hedef = kavsakta_byte(+1)      # +1 = SAĞA emri

kontrol('kavşak: SOLA emri hedef yönü +90° kaydırır',
        abs(math.degrees(sola_hedef) - 90.0) < 1.0,
        f'{math.degrees(sola_hedef):+.0f}°')
kontrol('kavşak: SAĞA emri hedef yönü -90° kaydırır',
        abs(math.degrees(saga_hedef) + 90.0) < 1.0,
        f'{math.degrees(saga_hedef):+.0f}°')

kontrol('kavşak: SOLA emri SOLA kırar (eski hata: sağa kırıyordu)',
        sola_byte < MERKEZ, f'd,{sola_byte} < {MERKEZ}')
kontrol('kavşak: SAĞA emri SAĞA kırar',
        saga_byte > MERKEZ, f'd,{saga_byte} > {MERKEZ}')

# --- 3) İKİ YOL BİRBİRİYLE TUTARLI ----------------------------------------
# 'Sola git' iki yolda da merkezin AYNI tarafına düşmeli.
kontrol('iki yol da SOLA için aynı tarafa komut verir',
        (sol - MERKEZ) * (sola_byte - MERKEZ) > 0,
        f'şerit d,{sol} / kavşak d,{sola_byte}')
kontrol('iki yol da SAĞA için aynı tarafa komut verir',
        (sag - MERKEZ) * (saga_byte - MERKEZ) > 0,
        f'şerit d,{sag} / kavşak d,{saga_byte}')

# --- 4) Yön tutma KAPALI DÖNGÜ: hedefe varınca merkeze döner --------------
varildi, _ = kavsakta_byte(-1, donuldu_rad=math.radians(90))
kontrol('kavşak: hedef yöne varınca direksiyon merkeze döner',
        abs(varildi - MERKEZ) <= 1, f'd,{varildi} ~ {MERKEZ}')

# Hedefi AŞARSA ters yöne düzeltmeli (kapalı döngü olduğunun kanıtı)
asti, _ = kavsakta_byte(-1, donuldu_rad=math.radians(110))
kontrol('kavşak: hedefi aşarsa ters yöne düzeltir',
        asti > MERKEZ, f'd,{asti} > {MERKEZ}')

# --- RAPOR ---------------------------------------------------------------
print()
print(f'  merkez byte = {MERKEZ} (trim {TRIM:+d}), '
      f'steering_direction = {kodda_yazan("steering_direction", -1.0)}')
print()
print(f'{"test":<52}{"detay":<30}sonuç')
print('-' * 90)
hata = 0
for ad, ok, detay in SONUC:
    if not ok:
        hata += 1
    print(f'{ad:<52}{detay:<30}{"OK" if ok else "HATA <<<"}')
print('-' * 90)
print('TÜMÜ GEÇTİ' if not hata else f'{hata} TEST BAŞARISIZ')
sys.exit(1 if hata else 0)
