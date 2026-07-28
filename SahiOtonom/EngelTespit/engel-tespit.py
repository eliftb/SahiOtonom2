#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np

class LidarObstacleDetector(Node):
    """
    Detects obstacles directly behind the vehicle using LiDAR data and
    publishes whether an obstacle is present and its distance.
    """
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.declare_parameter('obstacle_threshold', 5.0)
        self.declare_parameter('rear_angle_range_deg', 30.0)
        
        self.OBSTACLE_THRESHOLD = self.get_parameter('obstacle_threshold').value
        self.REAR_ANGLE_RANGE_DEG = self.get_parameter('rear_angle_range_deg').value
        
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        
        self.get_logger().info('LiDAR Obstacle Detector initialized.')

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        angle_increment = msg.angle_increment
        if angle_increment == 0.0:
            self.get_logger().warn("Invalid angle_increment in LaserScan.")
            return

        num_ranges = len(msg.ranges)
        
        # 180° açısının (arka) hangi indekste olduğunu hesapla
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        pi_angle_index = int((np.pi - angle_min) / angle_increment)
        
        # Eğer 180° açısı veri aralığının dışındaysa, en yakın ucu kullan
        if pi_angle_index < 0:
            pi_angle_index = 0
        elif pi_angle_index >= num_ranges:
            pi_angle_index = num_ranges - 1
        
        # Arka bölge için açı aralığını hesapla
        angle_range_rad = np.deg2rad(self.REAR_ANGLE_RANGE_DEG) / 2.0
        range_indices = int(angle_range_rad / abs(angle_increment))
        
        # Arka bölge indekslerini belirle
        start_index = max(0, pi_angle_index - range_indices)
        end_index = min(num_ranges, pi_angle_index + range_indices + 1)
        
        # Arka bölgedeki mesafe değerlerini al
        rear_ranges = np.array(msg.ranges[start_index:end_index])
        
        # Geçersiz değerleri filtrele
        rear_ranges = rear_ranges[np.isfinite(rear_ranges)]
        rear_ranges = rear_ranges[rear_ranges > msg.range_min]
        rear_ranges = rear_ranges[rear_ranges < msg.range_max]

        if rear_ranges.size == 0:
            self.publish_obstacle_status(False, float('inf'))
            return

        min_dist = np.min(rear_ranges)
        obstacle_detected = min_dist < self.OBSTACLE_THRESHOLD
        
        self.publish_obstacle_status(obstacle_detected, min_dist)
        
        if obstacle_detected:
            self.get_logger().info(f"Obstacle detected at {min_dist:.2f}m")

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()