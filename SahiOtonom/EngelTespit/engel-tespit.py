#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np

class LidarObstacleDetector(Node):
    """
    Aracın önündeki engelleri LiDAR ile tespit eder; engel var mı bilgisini ve
    mesafesini yayınlar.

    Taranan yön 'forward_angle_deg' parametresiyle belirlenir: bu, aracın ileri
    yönünün LiDAR'ın KENDİ açı sisteminde nereye denk geldiğidir. RPLIDAR S2'nin
    gövdesindeki 0° işareti aracın önüne bakacak şekilde monte edildiği için
    varsayılan 0.0'dır. LiDAR döndürülerek monte edilirse yalnızca bu parametre
    değiştirilir, koda dokunulmaz.
    """
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.declare_parameter('obstacle_threshold', 5.0)
        self.declare_parameter('forward_angle_deg', 180.0)
        self.declare_parameter('sector_width_deg', 120.0)
        self.declare_parameter('corridor_width_m', 1.5)
        self.declare_parameter('min_valid_distance', 0.10)
        self.declare_parameter('lidar_on_ofset_m', 0.50)
        self.OBSTACLE_THRESHOLD = self.get_parameter('obstacle_threshold').value
        self.FORWARD_ANGLE_DEG = self.get_parameter('forward_angle_deg').value
        self.SECTOR_WIDTH_DEG = self.get_parameter('sector_width_deg').value
        self.CORRIDOR_WIDTH_M = self.get_parameter('corridor_width_m').value
        self.MIN_VALID_DISTANCE = self.get_parameter('min_valid_distance').value
        self.LIDAR_ON_OFSET_M = max(0.0, float(self.get_parameter('lidar_on_ofset_m').value))
        self.sector_logged = False

        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)

        half = self.SECTOR_WIDTH_DEG / 2.0
        koridor = (f'{self.CORRIDOR_WIDTH_M:.2f} m'
                   if self.CORRIDOR_WIDTH_M > 0 else 'KAPALI (saf koni)')
        self.get_logger().info(
            f'LiDAR Engel Tespiti başlatıldı | İleri yön: {self.FORWARD_ANGLE_DEG:.1f}° | '
            f'Taranan koni: {self.FORWARD_ANGLE_DEG - half:.1f}° … {self.FORWARD_ANGLE_DEG + half:.1f}° '
            f'({self.SECTOR_WIDTH_DEG:.0f}°) | Koridor: {koridor} | '
            f'Eşik: {self.OBSTACLE_THRESHOLD:.1f} m | '
            f'Min geçerli: {self.MIN_VALID_DISTANCE:.2f} m | '
            f'LiDAR→tampon payı: {self.LIDAR_ON_OFSET_M:.2f} m')
        if self.CORRIDOR_WIDTH_M <= 0 and self.SECTOR_WIDTH_DEG > 60:
            self.get_logger().warn(
                'Koridor filtresi kapalı ve koni 60°den geniş: aracın YANINDAKİ '
                'nesneler de engel sayılacak, gereksiz acil duruş olasıdır.')

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        angle_increment = msg.angle_increment
        if angle_increment == 0.0:
            self.get_logger().warn("Invalid angle_increment in LaserScan.")
            return

        num_ranges = len(msg.ranges)
        angle_min = msg.angle_min

        center_index = int(round(
            (np.deg2rad(self.FORWARD_ANGLE_DEG) - angle_min) / angle_increment))

        half_indices = int(round(
            (np.deg2rad(self.SECTOR_WIDTH_DEG) / 2.0) / abs(angle_increment)))

        indices = np.arange(center_index - half_indices, center_index + half_indices + 1)

        full_circle = (msg.angle_max - angle_min) >= (2.0 * np.pi - abs(angle_increment))
        if full_circle:
            indices = np.mod(indices, num_ranges)
        else:
            indices = indices[(indices >= 0) & (indices < num_ranges)]

        if indices.size == 0:
            self.publish_obstacle_status(False, float('inf'))
            return

        front_ranges = np.asarray(msg.ranges, dtype=float)[indices]

       
        alt_sinir = max(msg.range_min, self.MIN_VALID_DISTANCE)
        gecerli = (np.isfinite(front_ranges)
                   & (front_ranges > alt_sinir)
                   & (front_ranges < msg.range_max))

        if self.CORRIDOR_WIDTH_M > 0:
            r = front_ranges[gecerli]
            # theta = ışının İLERİ YÖNDEN sapması; koni merkezli hesaplanır ki
            # LiDAR döndürülerek monte edilse bile doğru kalsın.
            theta = (angle_min + indices[gecerli] * angle_increment
                     - np.deg2rad(self.FORWARD_ANGLE_DEG))
            ileri = r * np.cos(theta)      # araç ekseni boyunca
            yanal = r * np.sin(theta)      # araca dik
            koridorda = ((np.abs(yanal) <= self.CORRIDOR_WIDTH_M / 2.0)
                         & (ileri > alt_sinir))
            front_ranges = ileri[koridorda]
        else:
            front_ranges = front_ranges[gecerli]

        if not self.sector_logged:
            self.sector_logged = True
            self.get_logger().info(
                f'Tarama doğrulaması | Nokta sayısı: {num_ranges} | '
                f'Merkez indeks: {center_index} | Koni: ±{half_indices} indeks '
                f'({indices.size} ışın) | Koridorda kalan: {front_ranges.size} ışın | '
                f'Sarma: {"evet" if full_circle else "hayır"}')

        if front_ranges.size == 0:
            self.publish_obstacle_status(False, float('inf'))
            return

        min_dist = float(np.min(front_ranges))
        TEMAS = 0.01   # "temas" - pozitif kalmalı, sentinel ile çakışmasın
        tampon_mesafe = max(TEMAS, min_dist - self.LIDAR_ON_OFSET_M)
        obstacle_detected = tampon_mesafe < self.OBSTACLE_THRESHOLD

        self.publish_obstacle_status(obstacle_detected, tampon_mesafe)

        if obstacle_detected:
            self.get_logger().info(
                f"Önde engel: {tampon_mesafe:.2f} m (tampondan) | "
                f"{min_dist:.2f} m (LiDAR'dan)")

    def publish_obstacle_status(self, detected, distance: float):
        detected_msg = Bool()
        detected_msg.data = bool(detected)
        self.obstacle_detected_pub.publish(detected_msg)
        
        distance_msg = Float32()
        distance_msg.data = float(distance) if detected else -1.0
        self.obstacle_distance_pub.publish(distance_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()