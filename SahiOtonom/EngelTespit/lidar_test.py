#!/usr/bin/env python3
from rplidar import RPLidar

from lidar_port import BAUDRATE, find_port

PORT_NAME = find_port()

lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)

try:
    print(f"Lidar başlatıldı ({PORT_NAME} @ {BAUDRATE}). Ölçümler geliyor...\n")
    for scan in lidar.iter_scans():
        # Her scan: [(quality, angle, distance), ...]
        for (_, angle, distance) in scan:
            print(f"Açı: {angle:.2f}°, Mesafe: {distance:.2f} mm")
except KeyboardInterrupt:
    print("\nDurduruluyor...")
finally:
    print("Bağlantı kapatılıyor...")
    lidar.stop()
    lidar.disconnect()
