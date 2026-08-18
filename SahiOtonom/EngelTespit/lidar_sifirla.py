#!/usr/bin/env python3
"""RPLIDAR RESET + SAGLIK KONTROLU - fisi cekmeden denemek icin.

NEDEN: 'RPLidar health status : 2' cihazin KENDI koruma hatasidir; surucuyu
yeniden baslatmak temizlemez cunku hata cihazin icinde mandalli kalir. RPLIDAR
protokolunde bir RESET komutu var (0xA5 0x40) - once onu deneriz, olmazsa
fiziksel olarak fisi cekmek gerekir.

KULLANIM:
    python3 lidar_sifirla.py            # portu otomatik bul
    python3 lidar_sifirla.py /dev/ttyUSB0

NOT: Bu araci calistirmadan once rplidar surucusunun KAPALI oldugundan emin
olun (launch_all_nodes.py'yi durdurun) - yoksa port mesgul olur.
"""
import glob
import sys
import time

try:
    import serial
except ImportError:
    print("HATA: pyserial yok  ->  pip install pyserial")
    sys.exit(1)

BAUD = 1000000          # RPLIDAR S2
CMD_RESET = b'\xA5\x40'
CMD_HEALTH = b'\xA5\x52'
CMD_STOP = b'\xA5\x25'

SAGLIK = {0: 'IYI (0)', 1: 'UYARI (1)', 2: 'HATA (2)'}


def port_bul():
    for kalip in ('/dev/sahi_lidar',
                  '/dev/serial/by-id/*CP2102*',
                  '/dev/serial/by-id/*Silicon_Labs*'):
        b = sorted(glob.glob(kalip))
        if b:
            return b[0]
    return None


def saglik_oku(ser):
    """GET_HEALTH gonderir, (durum, hata_kodu) doner. Okunamazsa (None, None)."""
    ser.reset_input_buffer()
    ser.write(CMD_HEALTH)
    ser.flush()
    time.sleep(0.2)
    # Yanit tanimlayicisi: A5 5A + 5 bayt
    basli = ser.read(7)
    if len(basli) < 7 or basli[0] != 0xA5 or basli[1] != 0x5A:
        return None, None
    veri = ser.read(3)
    if len(veri) < 3:
        return None, None
    durum = veri[0]
    hata_kodu = veri[1] | (veri[2] << 8)
    return durum, hata_kodu


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else port_bul()
    if not port:
        print("HATA: LiDAR portu bulunamadi. Takili mi?")
        print("  ls /dev/serial/by-id/ | grep -i cp2102")
        return 1

    print(f"Port: {port}")
    try:
        ser = serial.Serial(port, BAUD, timeout=1.0)
    except Exception as e:
        print(f"HATA: port acilamadi: {e}")
        print("  rplidar surucusu calisiyor olabilir - once onu durdurun.")
        return 1

    with ser:
        ser.setDTR(False)          # motor kontrol hatti

        durum, kod = saglik_oku(ser)
        print(f"Reset ONCESI saglik : {SAGLIK.get(durum, f'okunamadi ({durum})')}"
              + (f"  hata kodu: {kod}" if kod else ""))

        print("RESET gonderiliyor (0xA5 0x40)...")
        ser.write(CMD_STOP)        # once taramayi durdur
        ser.flush()
        time.sleep(0.1)
        ser.write(CMD_RESET)
        ser.flush()
        time.sleep(2.5)            # cihaz yeniden aciliyor
        ser.reset_input_buffer()

        durum2, kod2 = saglik_oku(ser)
        print(f"Reset SONRASI saglik: {SAGLIK.get(durum2, f'okunamadi ({durum2})')}"
              + (f"  hata kodu: {kod2}" if kod2 else ""))

    print()
    if durum2 == 0:
        print("  DUZELDI. Sistemi normal baslatabilirsiniz.")
        return 0
    if durum2 is None:
        print("  Cihaz yanit vermiyor. Baud/port yanlis olabilir ya da")
        print("  surucu portu tutuyor olabilir.")
        return 1
    print("  HALA HATALI. Yazilimsal reset yetmedi. Sirasiyla:")
    print("   1) USB kablosunu cikarip 10 sn bekleyip takin")
    print("   2) Duzelmezse BESLEME yetersiz olabilir: S2'nin motoru laptop USB")
    print("      portunun verebileceginden fazla akim cekiyor olabilir.")
    print("      Harici beslemeli bir USB hub'a ya da kendi guc girisine baglayin.")
    print("   3) LiDAR'in onunde donmesini engelleyen bir sey var mi bakin")
    print("      (kablo, bant, kapak) - motor tikanirsa cihaz kendini korumaya alir.")
    return 1


if __name__ == '__main__':
    sys.exit(main())
