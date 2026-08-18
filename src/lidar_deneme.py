#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist

class LidarAngleFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_angle_filter')
        
        # Parametreleri declare et
        self.declare_parameter('min_angle', -30.0)
        self.declare_parameter('max_angle', 30.0)
        self.declare_parameter('min_distance', 1.0)
        self.declare_parameter('scan_topic', '/scan')
        
        # Parametreleri al
        self.min_angle = self.get_parameter('min_angle').get_parameter_value().double_value
        self.max_angle = self.get_parameter('max_angle').get_parameter_value().double_value
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        
        # Publisher'lar
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/closest_distance', 10)
        self.control_pub = self.create_publisher(String, '/control_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f"LiDAR Angle Filter Node başlatıldı")
        self.get_logger().info(f"Açı aralığı: {self.min_angle}° - {self.max_angle}°")
        self.get_logger().info(f"Minimum güvenli mesafe: {self.min_distance}m")
        self.get_logger().info(f"Scan topic: {self.scan_topic}")
    
    def scan_callback(self, scan_msg):
        """
        LaserScan mesajını işler
        """
        try:
            # Scan verilerini çıkar
            ranges = np.array(scan_msg.ranges)
            angle_min = scan_msg.angle_min
            angle_increment = scan_msg.angle_increment
            
            # Açıları hesapla (radyandan dereceye çevir)
            angles_rad = angle_min + np.arange(len(ranges)) * angle_increment
            angles_deg = np.degrees(angles_rad)
            
            # Sonsuz ve NaN değerleri temizle
            valid_indices = np.isfinite(ranges) & (ranges > 0)
            valid_ranges = ranges[valid_indices]
            valid_angles = angles_deg[valid_indices]
            
            if len(valid_ranges) == 0:
                self.get_logger().warn("Geçerli LiDAR verisi yok")
                return
            
            # Açı filtreleme yap
            filtered_angles, filtered_distances = self.filter_lidar_data(
                valid_angles, valid_ranges
            )
            
            # Engel tespiti
            obstacle_detected, closest_distance, obstacle_angle = self.get_front_obstacles(
                filtered_angles, filtered_distances
            )
            
            # Kontrol mantığı
            control_command = self.simple_control_logic(
                obstacle_detected, closest_distance, obstacle_angle
            )
            
            # Sonuçları yayınla
            self.publish_results(obstacle_detected, closest_distance, control_command)
            
            # Log bilgileri
            if obstacle_detected:
                self.get_logger().info(f"ENGEL! Mesafe: {closest_distance:.2f}m, "
                                     f"Açı: {obstacle_angle:.1f}°, Komut: {control_command}")
            
        except Exception as e:
            self.get_logger().error(f"Scan callback hatası: {str(e)}")
    
    def filter_lidar_data(self, angles, distances):
        """
        LiDAR verilerini belirtilen açı aralığına göre filtreler
        """
        angles = np.array(angles)
        distances = np.array(distances)
        
        # Açı aralığına göre filtrele
        mask = (angles >= self.min_angle) & (angles <= self.max_angle)
        
        filtered_angles = angles[mask]
        filtered_distances = distances[mask]
        
        return filtered_angles, filtered_distances
    
    def get_front_obstacles(self, angles, distances):
        """
        Önde bulunan engelleri tespit eder
        """
        if len(distances) == 0:
            return False, float('inf'), None
        
        # En yakın engeli bul
        closest_idx = np.argmin(distances)
        closest_distance = distances[closest_idx]
        obstacle_angle = angles[closest_idx]
        
        # Engel tespit edildi mi?
        obstacle_detected = closest_distance < self.min_distance
        
        return obstacle_detected, closest_distance, obstacle_angle
    
    def simple_control_logic(self, obstacle_detected, closest_distance, obstacle_angle):
        """
        Basit araç kontrol mantığı
        """
        if obstacle_detected:
            if closest_distance < 0.5:
                return "DUR!"
            elif closest_distance < 1.0:
                if obstacle_angle is not None:
                    if obstacle_angle > 0:
                        return "SOLA_DON"
                    else:
                        return "SAGA_DON"
                else:
                    return "GERI_GIT"
            else:
                return "YAVASLA"
        else:
            return "ILERLE"
    
    def publish_results(self, obstacle_detected, closest_distance, control_command):
        """
        Sonuçları ROS topic'lerine yayınlar
        """
        # Bool mesajı
        obstacle_msg = Bool()
        obstacle_msg.data = bool(obstacle_detected)
        self.obstacle_pub.publish(obstacle_msg)

        # Float32 mesajı
        distance_msg = Float32()
        distance_msg.data = float(closest_distance) if closest_distance != float('inf') else -1.0
        self.distance_pub.publish(distance_msg)
        
        # String mesajı
        control_msg = String()
        control_msg.data = control_command
        self.control_pub.publish(control_msg)
        
        # Cmd_vel mesajı (isteğe bağlı)
        self.publish_cmd_vel(control_command)
    
    def publish_cmd_vel(self, control_command):
        """
        Kontrol komutuna göre cmd_vel mesajı yayınlar
        """
        twist = Twist()
        
        if control_command == "ILERLE":
            twist.linear.x = 0.5  # İleri hız
        elif control_command == "YAVASLA":
            twist.linear.x = 0.2  # Yavaş ileri
        elif control_command == "DUR!":
            twist.linear.x = 0.0  # Dur
        elif control_command == "SOLA_DON":
            twist.linear.x = 0.1
            twist.angular.z = 0.5  # Sola dön
        elif control_command == "SAGA_DON":
            twist.linear.x = 0.1
            twist.angular.z = -0.5  # Sağa dön
        elif control_command == "GERI_GIT":
            twist.linear.x = -0.2  # Geri git
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Node'u başlat
        node = LidarAngleFilterNode()
        
        node.get_logger().info("LiDAR Angle Filter Node çalışıyor...")
        node.get_logger().info("Çıkmak için Ctrl+C")
        
        # ROS2 spin
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info("Node kapatılıyor...")
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()