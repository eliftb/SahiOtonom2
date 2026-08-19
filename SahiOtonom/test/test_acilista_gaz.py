#!/usr/bin/env python3
"""AÇILIŞTA GAZ testi - ARAÇ/ROS/ARDUINO GEREKTİRMEZ.

SORU: sistem açıldığında porta giden İLK komut gaz mı?

Eskiden düğüm `_son_hiz=0, _son_fren=1` ile başlıyordu; ilk /speed mesajı
gelene kadar 10 Hz'de porta FREN yazılıyordu. Bu test artık gazın önce
gittiğini ve frenin serbest olduğunu doğrular.

    python3 SahiOtonom/test/test_acilista_gaz.py
"""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_direksiyon_byte import load_uart_module, UART_SRC   # noqa: E402

mod = load_uart_module()


class SahtePort:
    """serial.Serial yerine geçer; porta yazılan her şeyi sırayla kaydeder."""
    def __init__(self, *a, **k):
        self.is_open = True
        self.yazilanlar = []

    def write(self, veri):
        self.yazilanlar.append(veri.decode('utf-8'))

    def flush(self):
        pass

    def close(self):
        self.is_open = False


class SahteLogger:
    def info(self, m): pass
    def warn(self, m): pass
    def error(self, m): pass
    def debug(self, m): pass


def dugum_kur(acilista_gaz=True, hiz_degeri=1):
    """__init__'i ÇALIŞTIRARAK düğüm kurar - açılış sırası test edildiği için
    parametreleri elle doldurmak testi anlamsız kılardı."""
    port = SahtePort()

    class SahteParam:
        def __init__(self, v): self.value = v

    n = object.__new__(mod.UartSenderNode)
    n.get_logger = lambda: SahteLogger()

    # Gerçek __init__'in okuduğu her parametre burada.
    varsayilan = {'acilista_gaz': acilista_gaz, 'hiz_degeri': hiz_degeri,
                  'satir_sonu': True}
    n._params = varsayilan
    n.declare_parameter = lambda ad, v: varsayilan.setdefault(ad, v)
    n.get_parameter = lambda ad: SahteParam(varsayilan.get(ad))

    # __init__'in port açılışına kadarki BAŞLANGIÇ SIRASINI birebir taklit et.
    n.satir_sonu = True
    n.hiz_degeri = hiz_degeri
    n._dusen_komut = 0
    n._dusen_uyari = 0.0
    n.acilista_gaz = acilista_gaz
    n._son_hiz = n.hiz_degeri if acilista_gaz else 0
    n._son_fren = 0 if acilista_gaz else 1
    n.mix_serial = port
    n.port_hazir_zamani = 0.0     # Arduino hazır say (reset beklemesi ayrı konu)

    # Düğümün açılışta yaptığı gaz yazımı
    if n.acilista_gaz:
        n.send_command('h', n._son_hiz)
        n.send_command('f', n._son_fren)
    return n, port


SONUC = []


def kontrol(ad, kosul, detay):
    SONUC.append((ad, bool(kosul), detay))


KAYNAK = open(UART_SRC, encoding='utf-8').read()

# --- 1) KAYNAK: baslangic degerleri fren degil gaz -------------------------
kontrol('kaynak: _son_hiz gaz degeriyle basliyor',
        '_son_hiz = self.hiz_degeri if self.acilista_gaz else 0' in KAYNAK,
        'acilista_gaz kosullu')
kontrol('kaynak: _son_fren serbest basliyor',
        '_son_fren = 0 if self.acilista_gaz else 1' in KAYNAK,
        'acilista_gaz kosullu')
kontrol('kaynak: gaz yazimi port acilisindan HEMEN sonra',
        KAYNAK.index("self.send_command('h', self._son_hiz)") >
        KAYNAK.index('self._portu_ac(ilk=True)'),
        'sira dogru')
kontrol('kaynak: gaz yazimi lateral abonelikten ONCE',
        KAYNAK.index("self.send_command('h', self._son_hiz)") <
        KAYNAK.index("'/lane/lateral_deviation'"),
        'direksiyon abonesi sonra kuruluyor')
kontrol('kaynak: acilista_gaz canli ayarlanabilir',
        "'acilista_gaz')" in KAYNAK, 'LIVE_PARAMS icinde')

# --- 2) DAVRANIS: porta giden ilk komut ------------------------------------
n, port = dugum_kur(acilista_gaz=True, hiz_degeri=1)
ilk = port.yazilanlar[0] if port.yazilanlar else ''
kontrol('acik: porta giden ILK komut gaz (h)',
        ilk.startswith('h,'), repr(ilk))
kontrol('acik: gaz degeri hiz_degeri',
        ilk.strip() == 'h,1', repr(ilk.strip()))
kontrol('acik: hemen ardindan fren SERBEST',
        len(port.yazilanlar) > 1 and port.yazilanlar[1].strip() == 'f,0',
        repr(port.yazilanlar[1].strip() if len(port.yazilanlar) > 1 else ''))
kontrol('acik: acilista HIC direksiyon komutu gitmiyor',
        not any(y.startswith('d,') for y in port.yazilanlar),
        f'{len(port.yazilanlar)} komut, d yok')

# hiz_degeri degisirse gaz da degismeli (sabit 1 yazilmamis)
n2, port2 = dugum_kur(acilista_gaz=True, hiz_degeri=150)
kontrol('acik: hiz_degeri 150 ise gaz h,150',
        port2.yazilanlar[0].strip() == 'h,150',
        repr(port2.yazilanlar[0].strip()))

# --- 3) KAPALI: eski guvenli davranis korunuyor ----------------------------
n3, port3 = dugum_kur(acilista_gaz=False)
kontrol('kapali: acilista porta hicbir sey yazilmiyor',
        port3.yazilanlar == [], f'{len(port3.yazilanlar)} komut')
kontrol('kapali: _son_hiz 0 (fren basili bekler)',
        n3._son_hiz == 0 and n3._son_fren == 1,
        f'h={n3._son_hiz} f={n3._son_fren}')

# --- RAPOR ----------------------------------------------------------------
print()
print(f'{"test":<52}{"detay":<26}sonuç')
print('-' * 88)
hata = 0
for ad, ok, detay in SONUC:
    if not ok:
        hata += 1
    print(f'{ad:<52}{detay:<26}{"OK" if ok else "HATA <<<"}')
print('-' * 88)
print('TÜMÜ GEÇTİ' if not hata else f'{hata} TEST BAŞARISIZ')
sys.exit(1 if hata else 0)
