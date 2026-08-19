#!/usr/bin/env python3
"""HIZ KOMUTU ARIZASINI SEHPADA KANITLAR - ROS GEREKTİRMEZ, porta doğrudan yazar.

NEDEN VAR: 2026-08-18'de loglar "h,1" bastığı hâlde araç hareket etmiyordu.
Kalibrasyon modunda (tuş tuş, aralarında saniyeler geçerek) komutlar geçiyordu,
yani "komut ulaşmıyor" tek başına doğru değildi. Fark AKIŞ YOĞUNLUĞU:

    sürüşte  ->  d 20 Hz + h/f 10 Hz, kesintisiz:  d,230d,230h,1f,0d,230...
    ölçümde  ->  tek komut, sonra sessizlik

Firmware Serial.parseInt() kullanıyorsa sayıyı bitiren karakteri okur ve ATAR.
Kesintisiz akışta 230'u bitiren karakter bir sonraki komutun HARFİDİR: 'h' yutulur,
geride kalan ',1' harfsiz kalıp yok sayılır. Ölçümde akış durduğu için parseInt
kendi zaman aşımıyla biter, harf yutulmaz - arıza gizlenir.

Bu betik iki akışı arka arkaya gönderir. Beklenen sonuç:

    1. AYRAÇSIZ (eski davranış)   -> tahrik dönmez  (h yutuluyor)
    2. SONLANDIRICILI (\\n, yeni) -> tahrik döner   (h ulaşıyor)

İkisinde de dönmüyorsa arıza YAZILIMDA DEĞİL: besleme / ESC / motor sürücüsü /
acil durdurma tarafına bakın. İkisinde de dönüyorsa firmware sonlandırıcıya
ihtiyaç duymuyor, arıza başka yerde.

    ⚠️  TEKERLER HAVADA/SEHPADA OLMALI. Bu betik TAHRİK MOTORUNU ÇALIŞTIRIR.
    ⚠️  ROS düğümleri KAPALI olmalı (port tek kullanıcılı).

    python3 SahiOtonom/test/test_hiz_akisi.py
    python3 SahiOtonom/test/test_hiz_akisi.py --sure 4 --sadece yeni
"""
import argparse
import sys
import time

import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
BAUD = 38400
BYTE_MERKEZ_OLCULEN = 230    # sehpada ölçülen düz konum (steering_trim 50)


def akis_gonder(port, sonlandirici, sure, direksiyon_byte):
    """Sürüşteki komut akışını taklit eder: d 20 Hz, h/f 10 Hz.

    Oranı korumak önemli: arıza 'h'nin bir 'd' sayısının hemen ardından
    gelmesinden doğuyor. Sadece h göndersek akış seyrekleşir ve arıza kaybolur -
    tam da kalibrasyon modunda olduğu gibi.
    """
    son = ''
    baslangic = time.time()
    sayac = 0
    while time.time() - baslangic < sure:
        for komut in (f'd,{direksiyon_byte}', f'd,{direksiyon_byte}', 'h,1', 'f,0'):
            veri = komut + sonlandirici
            port.write(veri.encode('utf-8'))
            port.flush()
            son += veri
            time.sleep(0.05)      # 20 Hz - sürüşteki lateral kontrol hızı
        sayac += 1
    return son, sayac


def dur(port, sonlandirici):
    """Motoru kes, freni uygula, direksiyonu ölçülen merkeze bırak."""
    for komut in ('h,0', 'f,1', f'd,{BYTE_MERKEZ_OLCULEN}'):
        port.write((komut + sonlandirici).encode('utf-8'))
        port.flush()
        time.sleep(0.05)


def onay(metin):
    try:
        return input(metin).strip().lower()
    except (EOFError, KeyboardInterrupt):
        return 'q'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--sure', type=float, default=3.0,
                    help='her akışın saniyesi (varsayılan 3)')
    ap.add_argument('--direksiyon', type=int, default=BYTE_MERKEZ_OLCULEN,
                    help='akışa karışacak d byte değeri (varsayılan: ölçülen merkez)')
    ap.add_argument('--sadece', choices=['eski', 'yeni'],
                    help='tek akış çalıştır (eski = ayraçsız, yeni = \\n)')
    args = ap.parse_args()

    print(__doc__.split('    python3')[0])
    if onay('  Tekerler HAVADA ve kimse aracın yanında değil mi? [e/H] ') != 'e':
        print('  İptal edildi.')
        return 1

    try:
        port = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f'\n  Port açılamadı: {e}')
        metin = str(e)
        if 'Errno 16' in metin or 'busy' in metin.lower():
            print('  Port meşgul: ROS düğümleri hâlâ açık. CTRL+C ile kapatın.')
        elif 'Errno 5' in metin or 'Input/output' in metin:
            print('  Errno 5 = cihaz düğümü var ama Arduino cevap vermiyor.')
            print('  KABLO/BESLEME: USB\'yi çıkar-tak, sonra: dmesg | tail -20')
        elif 'Errno 2' in metin or 'No such file' in metin:
            print('  Arduino bağlı değil. Kontrol: ls -l /dev/serial/by-id/')
        return 1

    # Port açılınca Arduino DTR ile reset atar ve ~2 sn komut kabul etmez.
    # Beklemezsek ilk akış sessizce düşer ve "yine dönmedi" sanılır.
    print('\n  Arduino resetinden çıkması bekleniyor (3 sn)...')
    time.sleep(3.0)

    denemeler = [
        ('1/2  AYRAÇSIZ AKIŞ (eski hatalı davranış)', '',
         'BEKLENEN: tahrik DÖNMEZ - h harfi parseInt tarafından yutuluyor'),
        ('2/2  SONLANDIRICILI AKIŞ (\\n - yeni davranış)', '\n',
         'BEKLENEN: tahrik DÖNER - h komutu ulaşıyor'),
    ]
    if args.sadece == 'eski':
        denemeler = denemeler[:1]
    elif args.sadece == 'yeni':
        denemeler = denemeler[1:]

    sonuclar = []
    try:
        for baslik, sonlandirici, beklenen in denemeler:
            print(f'\n{"=" * 68}\n  {baslik}\n  {beklenen}\n{"=" * 68}')
            if onay(f'  {args.sure:.0f} sn boyunca gönderilecek. Başlat? [e/H] ') != 'e':
                print('  Atlandı.')
                continue

            akis, tur = akis_gonder(port, sonlandirici, args.sure, args.direksiyon)
            dur(port, sonlandirici)

            gorunum = akis[:60].replace('\n', '\\n')
            print(f'\n  Porta yazılan ({tur} tur): {gorunum}...')
            cevap = onay('  Tahrik motoru DÖNDÜ mü? [e/h] ')
            sonuclar.append((baslik, cevap == 'e'))
            print('  Motor kesildi, fren uygulandı.')
    finally:
        # Betik nasıl biterse bitsin araç komutsuz kalmasın: son aldığı hız
        # komutuyla dönmeye devam etmesin.
        try:
            dur(port, '\n')
            dur(port, '')
            port.close()
        except Exception:
            pass

    print(f'\n{"=" * 68}\n  SONUÇ\n{"=" * 68}')
    for baslik, dondu in sonuclar:
        print(f'  {baslik:<52} {"DÖNDÜ" if dondu else "dönmedi"}')

    if len(sonuclar) == 2:
        eski, yeni = sonuclar[0][1], sonuclar[1][1]
        print()
        if yeni and not eski:
            print('  ✅ TEŞHİS DOĞRULANDI: sebep eksik sonlandırıcıydı.')
            print('     send_command artık \\n ekliyor - düzeltme yerinde, sürüşe hazır.')
        elif yeni and eski:
            print('  Firmware sonlandırıcı olmadan da anlıyor; arıza BURADA DEĞİL.')
            print('     Sırada: karar düğümü gerçekten /speed yayınlıyor mu')
            print('     (ros2 topic echo /speed) ve base_speed 0 kalmış olabilir mi')
            print('     (./devam.sh 1.0).')
        elif not yeni and not eski:
            print('  ⚠️  Hiçbir akış motoru döndürmedi - arıza YAZILIMDA DEĞİL.')
            print('     Sırada: motor beslemesi, ESC/motor sürücüsü, acil durdurma')
            print('     rölesi, h komutunun firmware\'de gerçekten tahriki sürdüğü.')
            print('     .ino dosyası olmadan bundan ötesi ölçülemez.')
        else:
            print('  Beklenmedik: ayraçsız çalışıp sonlandırıcılı çalışmıyor.')
            print('     Firmware \\n\'i geçersiz komut sayıyor olabilir:')
            print('     ros2 param set /uart_sender_node satir_sonu false')
    return 0


if __name__ == '__main__':
    sys.exit(main())
