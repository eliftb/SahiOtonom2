from rplidar import RPLidar

PORT = '/dev/ttyUSB0'
BAUD = 256000

lidar = RPLidar(PORT, baudrate=BAUD)
info = lidar.get_info()
print("Lidar bilgisi:", info)
lidar.disconnect()
