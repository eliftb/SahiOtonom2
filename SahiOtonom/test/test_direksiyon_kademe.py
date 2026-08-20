#!/usr/bin/env python3
"""DİREKSİYON KADEME SINIRI testi - ARAÇ/ROS/ARDUINO GEREKTİRMEZ.

SORU: porta yazılan iki ardışık 'd' komutu arasındaki fark, Arduino'nun
giriş tamponunu taşıracak kadar büyüyebiliyor mu?

NEDEN ÖNEMLİ - "kesik kesik gaz" arızası buradan geliyordu:
communication.ino'daki stepAt() bloklayan bir döngü (derece başına ~17.8 ms) ve
o süre boyunca Arduino seri portu HİÇ okumuyor. Porta akan trafik ~266 bayt/sn,
Arduino'nun tamponu 64 bayt -> 0.24 saniyede doluyor, yani tek komutta ~13.5
DERECEDEN büyük her hareket tamponu taşırıyor. Taşınca akıştan bayt düşüyor,
'h,1' bozulup 'h,0' olarak okunabiliyor ve firmware

    analogWrite(9, deger == 1 ? 134 : 0);

dediği için GAZ SIFIRLANIYOR. Bu test, hiçbir komutun o sınırı aşmadığını
doğrular - yani arızanın kaynağını kapalı tutar.

    python3 SahiOtonom/test/test_direksiyon_kademe.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_direksiyon_byte import load_uart_module            # noqa: E402


# NOT: bu sahteler test_acilista_gaz.py'de de var ama oradan ICE AKTARILMIYOR -
# o dosya bir BETIK (modul duzeyinde testlerini calistirip sys.exit ediyor),
# import etmek onun ciktisini bu testin ortasina basar ve sys.exit bu testi
# yarida keser.
class SahtePort:
    """serial.Serial yerine gecer; porta yazilan her seyi sirayla kaydeder."""
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

mod = load_uart_module()

# communication.ino'dan: adim = derece * 3200/360, adim basina 2 x 1000 us
MS_PER_DERECE = (3200.0 / 360.0) * 2.0        # ~17.8 ms
TAMPON_BAYT = 64                              # Arduino donanim RX tamponu
AKIS_BAYT_SN = 266.0                          # d 31 Hz + h 10 Hz + f 10 Hz
TASMA_DERECE = (TAMPON_BAYT / AKIS_BAYT_SN) / (MS_PER_DERECE / 1000.0)

hatalar = []


def kontrol(ad, kosul, detay=''):
    print(f'  {ad:<52} {detay:<26} {"OK" if kosul else "HATA"}')
    if not kosul:
        hatalar.append(ad)


def dugum(adim=12, baslangic=None):
    n = object.__new__(mod.UartSenderNode)
    port = SahtePort()
    n.get_logger = lambda: SahteLogger()
    n.satir_sonu = True
    n.mix_serial = port
    n.port_hazir_zamani = 0.0
    n._dusen_komut = 0
    n._dusen_uyari = 0.0
    n.direksiyon_max_adim_byte = adim
    n._son_direksiyon_byte = (mod.FIRMWARE_ACILIS_BYTE if baslangic is None
                              else baslangic)
    return n, port


def yazilan_d(port):
    return [int(y.strip().split(',')[1]) for y in port.yazilanlar
            if y.startswith('d,')]


print(__doc__.split('\n')[0])
print(f'\nTasma sinirin hesabi: {TAMPON_BAYT} bayt / {AKIS_BAYT_SN:.0f} bayt-sn '
      f'/ {MS_PER_DERECE:.1f} ms-derece = {TASMA_DERECE:.1f} derece\n')

print('1) Baslangic referansi firmware ile ayni')
kontrol('acilista _son_direksiyon_byte = 210',
        mod.FIRMWARE_ACILIS_BYTE == 210, f'{mod.FIRMWARE_ACILIS_BYTE}')

print('\n2) Tek komutta buyuk sicrama YOK')
n, port = dugum()
n._direksiyon_gonder(360)              # tam kilit - mumkun en buyuk hedef
kontrol('210 -> 360 istegi tek adimda gitmiyor',
        yazilan_d(port) == [222], f'yazilan d,{yazilan_d(port)[0]}')

print('\n3) Ardisik komutlarda hedefe ULASIYOR')
n, port = dugum()
for _ in range(20):
    n._direksiyon_gonder(360)
d = yazilan_d(port)
kontrol('20 komutta hedefe varildi', d[-1] == 360, f'son d,{d[-1]}')
farklar = [b - a for a, b in zip(d, d[1:])]
kontrol('hicbir adim sinirdan buyuk degil',
        all(f <= 12 for f in farklar), f'en buyuk {max(farklar)}')

print('\n4) Ters yon de sinirli')
n, port = dugum(baslangic=300)
n._direksiyon_gonder(100)
kontrol('300 -> 100 istegi sinirli', yazilan_d(port) == [288],
        f'yazilan d,{yazilan_d(port)[0]}')

print('\n5) Kucuk fark AYNEN geciyor (gereksiz kademe yok)')
n, port = dugum(baslangic=230)
n._direksiyon_gonder(235)
n._direksiyon_gonder(230)
kontrol('5 birimlik istekler dogrudan gidiyor',
        yazilan_d(port) == [235, 230], f'{yazilan_d(port)}')

print('\n6) HICBIR adim tampon tasma sinirini asmiyor (asil guvence)')
n, port = dugum()
hedefler = [360, 100, 230, 360, 100, 230, 205, 255]   # sert savrulmalar
for h in hedefler:
    for _ in range(6):
        n._direksiyon_gonder(h)
d = [mod.FIRMWARE_ACILIS_BYTE] + yazilan_d(port)
en_buyuk = max(abs(b - a) for a, b in zip(d, d[1:]))
kontrol(f'en buyuk tek hareket < {TASMA_DERECE:.1f} derece',
        en_buyuk < TASMA_DERECE, f'{en_buyuk} derece')
kontrol('bloklama suresi tampondan kisa',
        en_buyuk * MS_PER_DERECE < (TAMPON_BAYT / AKIS_BAYT_SN) * 1000,
        f'{en_buyuk * MS_PER_DERECE:.0f} ms')

print('\n7) Sinir 0/negatif verilse bile dugum kilitlenmiyor')
n, port = dugum(adim=0)
for _ in range(5):
    n._direksiyon_gonder(360)
kontrol('adim 0 -> en az 1 birim ilerliyor',
        yazilan_d(port) == [211, 212, 213, 214, 215], f'{yazilan_d(port)[:3]}...')

print('\n' + '-' * 88)
if hatalar:
    print(f'BASARISIZ: {len(hatalar)} kontrol')
    for h in hatalar:
        print(f'   - {h}')
else:
    print('TUMU GECTI')
