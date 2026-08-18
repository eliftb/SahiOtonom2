import os
import sys

from rplidar import RPLidar

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'SahiOtonom', 'EngelTespit'))
from lidar_port import BAUDRATE, find_port

PORT = find_port()

lidar = RPLidar(PORT, baudrate=BAUDRATE)
info = lidar.get_info()
print(f"Port: {PORT}")
print("Lidar bilgisi:", info)
lidar.disconnect()
