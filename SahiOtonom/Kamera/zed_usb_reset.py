#!/usr/bin/env python3
"""ZED 2i'yi USB seviyesinde resetler.

Düğüm akış halindeyken CTRL+C / kill ile kesilirse kamera 'CAMERA STREAM
FAILED TO START' durumunda kalır: lsusb cihazı görür, V4L2 kare bile okur
ama ZED SDK açamaz. Kabloyu çıkarıp takmakla aynı işi yapar.

Kullanım:  sudo python3 zed_usb_reset.py
"""
import fcntl
import re
import subprocess
import sys

USBDEVFS_RESET = 0x5514
ZED_IDS = ("2b03:f880", "2b03:f881")  # video arayüzü + HID arayüzü


def zed_cihazlarini_bul():
    """lsusb çıktısından ZED'in /dev/bus/usb yollarını çıkarır."""
    try:
        cikti = subprocess.check_output(["lsusb"], text=True)
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        sys.exit(f"lsusb çalıştırılamadı: {e}")

    bulunan = []
    for satir in cikti.splitlines():
        eslesme = re.match(r"Bus (\d+) Device (\d+): ID (\S+)", satir)
        if eslesme and eslesme.group(3) in ZED_IDS:
            bus, dev, uid = eslesme.groups()
            bulunan.append((f"/dev/bus/usb/{bus}/{dev}", uid, satir.strip()))
    return bulunan


def main():
    cihazlar = zed_cihazlarini_bul()
    if not cihazlar:
        sys.exit("ZED USB'de görünmüyor (lsusb'de 2b03 yok). Kabloyu kontrol et.")

    hata = False
    for yol, uid, satir in cihazlar:
        print(f"Resetleniyor: {satir}")
        try:
            with open(yol, "wb") as fd:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            print(f"  ✅ {yol} ({uid}) resetlendi")
        except PermissionError:
            print(f"  ❌ {yol}: yetki yok — 'sudo' ile çalıştır")
            hata = True
        except OSError as e:
            print(f"  ❌ {yol}: {e}")
            hata = True

    if hata:
        sys.exit(1)

    print("\nBitti. Kameranın yeniden enumerate olması için ~3 saniye bekle,")
    print("sonra doğrula:  python3 -c \"import pyzed.sl as sl;"
          " print(len(sl.Camera.get_device_list()))\"")


if __name__ == "__main__":
    main()
