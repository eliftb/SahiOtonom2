#!/usr/bin/env python3
"""LiDAR seri port ayarları — tek yerden.

Port neden sabit değil?
    /dev/ttyUSB0 numarası takılma sırasına göre değişir. Başka bir USB-seri
    cihaz önce takılırsa lidar /dev/ttyUSB1 olur ve kod patlar.

Çözüm sırası:
    1) /dev/rplidar        -> udev kuralı kuruluysa (bkz. 99-rplidar.rules)
    2) /dev/serial/by-id/  -> her zaman var, cihazın seri numarasına bağlı
    3) /dev/ttyUSB0        -> son çare
"""
import glob
import os

# Cihazın USB seri numarası (udevadm ile okundu, cihaza özgü ve sabit).
USB_SERIAL = '323ebac4046ff011823b549b1045c30f'

BY_ID = ('/dev/serial/by-id/'
         f'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_{USB_SERIAL}-if00-port0')

# Bu cihaz (model 0x71) 1 Mbaud konuşur. S1'in 256000 değeri BU CİHAZDA ÇALIŞMAZ.
BAUDRATE = 1000000


def find_port():
    """Kullanılabilir ilk sabit port yolunu döndürür."""
    for path in ('/dev/rplidar', BY_ID):
        if os.path.exists(path):
            return path

    # by-id yolu ürün adı değişirse diye seri numarasıyla da ara
    for path in glob.glob(f'/dev/serial/by-id/*{USB_SERIAL}*'):
        return path

    return '/dev/ttyUSB0'


if __name__ == '__main__':
    port = find_port()
    print(f'Port    : {port}')
    print(f'Gerçek  : {os.path.realpath(port)}')
    print(f'Baudrate: {BAUDRATE}')
