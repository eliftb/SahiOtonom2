#!/usr/bin/env python3
from rplidar import RPLidar

# Lidar portu (senin durumunda /dev/ttyUSB0)
PORT_NAME = '/dev/ttyUSB0'

# S1 genelde 256000 baud kullanır
lidar = RPLidar(PORT_NAME, baudrate=256000)

try:
    print("Lidar başlatıldı. Ölçümler geliyor...\n")
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
