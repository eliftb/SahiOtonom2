#!/usr/bin/env python3
"""Direksiyon byte eşlemesi regresyon testi - ARAÇ/ROS GEREKTİRMEZ.

uart_sender_node3.angle_to_byte'ı sentetik trim/kilit değerleriyle çalıştırır.
Buradaki iddialar aracı sehpaya çıkarmadan doğrulanabilen tek şeyler:
merkezin nereye düştüğü, ölçeğin max_steering_angle ile birlikte değişip
değişmediği ve kırpmanın iki tarafı eşit bırakıp bırakmadığı.

    python3 SahiOtonom/test/test_direksiyon_byte.py
"""
import importlib.util
import os
import re
import sys
import types

UART_SRC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'Haberlesme',
    'uart_sender_node3.py'))


def load_uart_module():
    """ROS/serial bağımlılıklarını sahteleyerek düğüm dosyasını yükler."""
    for name in ['rclpy', 'rclpy.node', 'rclpy.executors', 'ackermann_msgs',
                 'ackermann_msgs.msg', 'std_msgs', 'std_msgs.msg', 'nav_msgs',
                 'nav_msgs.msg', 'rcl_interfaces', 'rcl_interfaces.msg', 'serial']:
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy.node'].Node = type('Node', (), {})
    sys.modules['rclpy.executors'].ExternalShutdownException = Exception
    sys.modules['ackermann_msgs.msg'].AckermannDrive = object
    sys.modules['nav_msgs.msg'].Odometry = object
    sys.modules['rcl_interfaces.msg'].SetParametersResult = object

    class _Msg:
        def __init__(self, data=None):
            self.data = data
    for ad in ('Float32', 'Int32', 'Bool'):
        setattr(sys.modules['std_msgs.msg'], ad, _Msg)
    sys.modules['serial'].Serial = object
    sys.modules['serial'].SerialException = Exception

    spec = importlib.util.spec_from_file_location('uart_node', UART_SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def dugum(trim=0, kilit=0.5):
    """__init__ çalıştırmadan sadece byte eşlemesi için gereken alanları kurar.

    __init__ portu açmaya ve ROS'a bağlanmaya çalışır; test edilen fonksiyonun
    ikisiyle de işi yok.
    """
    mod = load_uart_module()
    n = object.__new__(mod.UartSenderNode)
    n.steering_trim = trim
    n.max_steering_angle = kilit
    return n


def esit(ad, bulunan, beklenen):
    if bulunan != beklenen:
        print(f'  BAŞARISIZ  {ad}: {bulunan} != {beklenen}')
        return 1
    print(f'  tamam      {ad} = {bulunan}')
    return 0


def main():
    hata = 0

    # BEKLENENLER communication.ino'DAN TÜRETİLDİ (2026-08-20):
    #     if (deger < 0 || deger > 420) return;   int hedef = deger - 210;
    # yani merkez 210, tavan 420. Bu test eskiden 180/360 bekliyordu ve
    # GEÇİYORDU - çünkü kodda da aynı yanlış sabit vardı. Test hatayı
    # doğruluyordu; firmware kaynağı gelince ikisi birden düzeltildi.
    print('\n1) Trim 0, kilit 0.5 - FIRMWARE PROTOKOLÜ (merkez 210, tavan 420)')
    n = dugum()
    hata += esit('merkez', n.angle_to_byte(0.0), 210)
    hata += esit('tam sol', n.angle_to_byte(-0.5), 0)
    hata += esit('tam sağ', n.angle_to_byte(0.5), 420)
    # Merkez firmware'in sıfırı: d,210 gonderilince hedef = 210-210 = 0 derece
    hata += esit('-0.061 rad', n.angle_to_byte(-0.061), 184)

    print('\n2) Trim -30: merkez kayar, komutlar merkez etrafında SİMETRİK kalır')
    n = dugum(trim=-30)
    hata += esit('merkez', n.angle_to_byte(0.0), 180)
    merkez = 180
    sol = n.angle_to_byte(-0.25)
    sag = n.angle_to_byte(0.25)
    hata += esit('sol sapma', merkez - sol, sag - merkez)
    print('\n   Uçlar KIRPILMIYOR: küçük taraf (180) esas alınır, yoksa araç')
    print('   sağa sola döndüğünden sert dönerdi.')
    hata += esit('tam sol', n.angle_to_byte(-0.5), 0)
    hata += esit('tam sağ', n.angle_to_byte(0.5), 360)

    print('\n3) Kilit ölçülüp 0.35 rad yapılınca ÖLÇEK de onunla değişir')
    n = dugum(kilit=0.35)
    hata += esit('tam sol (0.35)', n.angle_to_byte(-0.35), 0)
    hata += esit('tam sağ (0.35)', n.angle_to_byte(0.35), 420)
    print('   (ölçek kilitten türemeseydi tam sağ 420\'ye çıkamaz, kilidin')
    print('    son dilimi ölü kalırdı)')

    print('\n4) Doyum: kilidin ötesindeki komut uca kırpılır')
    n = dugum(trim=10, kilit=0.4)
    hata += esit('aşırı sol', n.angle_to_byte(-5.0), n.angle_to_byte(-0.4))
    hata += esit('aşırı sağ', n.angle_to_byte(5.0), n.angle_to_byte(0.4))
    # Hiçbir açı protokol aralığının dışına taşmamalı (firmware'e bozuk
    # sayı gitmesi tekerin nereye gideceğini tahmin edilemez yapar).
    disari = [a for a in (-9.0, -0.7, 0.0, 0.7, 9.0)
              if not 0 <= n.angle_to_byte(a) <= 420]
    hata += esit('aralık dışına taşan açı yok', disari, [])

    print('\n5) Bozuk kilit (0) düğümü çökertmez, merkezde tutar')
    n = dugum(kilit=0.0)
    hata += esit('merkez', n.angle_to_byte(-0.3), 210)

    # --- 6. FIRMWARE İLE SABİTLER UYUŞUYOR MU ------------------------------
    # Sabitleri iki dosyaya elle kopyalamak, .ino değişince Python'un sessizce
    # ESKİ protokolü konuşmasına yol açar - en pahalı hata türü bu. Burada
    # firmware kaynağı okunup karşılaştırılıyor; araç/ROS/Arduino gerekmez.
    print('\n6) communication.ino ile sabitler uyuşuyor mu')
    mod = load_uart_module()
    ino_yolu = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'Haberlesme',
        'communication.ino'))

    def dogru(ad, kosul, ayrinti=''):
        sonuc = 0
        if kosul:
            print(f'  tamam      {ad}' + (f' ({ayrinti})' if ayrinti else ''))
        else:
            print(f'  BAŞARISIZ  {ad}' + (f': {ayrinti}' if ayrinti else ''))
            sonuc = 1
        return sonuc

    if not os.path.exists(ino_yolu):
        hata += dogru('communication.ino repoda var', False, ino_yolu)
        print('             Karta yükledikten sonra SİLMEYİN: bu düğümün')
        print('             konuştuğu protokolün tek doğrulanabilir kaynağı o.')
    else:
        kaynak = open(ino_yolu, encoding='utf-8').read()

        def sayi(desen, varsayilan=0):
            m = re.search(desen, kaynak)
            return int(m.group(1)) if m else varsayilan

        hata += dogru('baud aynı', f'Serial.begin({mod._kodda_yazan("baud_rate", 0)})'
                      in kaynak, '38400')
        hata += dogru('d üst sınırı aynı',
                      f'deger > {mod.BYTE_UST}) return' in kaynak, str(mod.BYTE_UST))
        hata += dogru('d merkezi aynı',
                      f'deger - {mod.BYTE_MERKEZ};' in kaynak, str(mod.BYTE_MERKEZ))
        # Firmware `int stepangle = 0` ile açılır, yani "d,<merkez> konumundayım".
        # Düğüm de yeniden bağlanınca sayacını oraya çeker (bkz. _portu_ac).
        hata += dogru('açılış konumu aynı',
                      'int stepangle = 0;' in kaynak
                      and mod.FIRMWARE_ACILIS_BYTE == mod.BYTE_MERKEZ,
                      f'd,{mod.FIRMWARE_ACILIS_BYTE}')
        # Firmware SADECE h,1'i gaz sayar; hiz_degeri başka bir şey olursa
        # düğüm gaz verdiğini sanır, firmware analogWrite(9, 0) yapar.
        hata += dogru('gaz sadece h,1',
                      'deger == 1 ? 134 : 0' in kaynak
                      and mod._kodda_yazan('hiz_degeri', 0) == 1, 'analogWrite 134')
        # En uzun komut ('d,' + üst sınır + '\n') firmware tamponuna sığmalı.
        tampon = sayi(r'char buf\[(\d+)\]', 0)
        gerekli = len(f'd,{mod.BYTE_UST}') + 1
        hata += dogru('en uzun komut firmware tamponuna sığıyor',
                      tampon >= gerekli, f'buf[{tampon}] >= {gerekli}')
        # KALP ATIŞI ZAMAN AŞIMINDAN HIZLI OLMALI: firmware bu süre sessiz
        # kalırsa gazı kesip freni basıyor.
        zaman_asimi_s = sayi(r'KOMUT_ZAMAN_ASIMI_MS = (\d+)', 0) / 1000.0
        kalp_hz = mod._kodda_yazan('gaz_tekrar_hz', 0.0)
        hata += dogru('gaz kalp atışı firmware zaman aşımından hızlı',
                      kalp_hz > 0 and 1.0 / kalp_hz < zaman_asimi_s,
                      f'{1.0/kalp_hz:.2f} sn < {zaman_asimi_s:.2f} sn')
        # ASIL BAĞ: stepAt() bloklarken Arduino seri portu okumuyor. Tek
        # komutta gidilen byte, tamponun dolma süresinden UZUN sürmemeli -
        # yoksa akıştan bayt düşer, 'h,1' bozulup 'h,0' okunur ve gaz kesilir.
        adim_us = sayi(r'#define stepdelays (\d+)', 0)
        m = re.search(r'\* (\d+)L / (\d+)L', kaynak)
        tur_adim, tur_derece = (int(m.group(1)), int(m.group(2))) if m else (0, 1)
        s_derece = (tur_adim / tur_derece) * 2 * adim_us / 1e6
        TAMPON_BAYT, AKIS_BAYT_SN = 64, 266.0
        sinir = (TAMPON_BAYT / AKIS_BAYT_SN) / s_derece if s_derece else 0
        kademe = mod._kodda_yazan('direksiyon_max_adim_byte', 0)
        hata += dogru('direksiyon kademesi tampon taşma sınırının altında',
                      0 < kademe < sinir, f'{kademe} < {sinir:.1f} birim')

    print('\n' + ('  HEPSİ GEÇTİ' if hata == 0 else f'  {hata} BAŞARISIZ'))
    return 1 if hata else 0


if __name__ == '__main__':
    sys.exit(main())
