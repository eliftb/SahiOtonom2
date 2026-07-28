#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive
from enum import Enum
import numpy as np



class DecisionMakingNode(Node):
    """
    Basit karar verme node'u: Düz yolda şerit takibi ve engel önleme
    """
    def __init__(self):
        super().__init__('decision_making_node')
        
        # Parametreler
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('steering_gain', 1.5)
        self.declare_parameter('emergency_stop_distance', 10.0)
        self.declare_parameter('slow_down_distance', 4.0)
        
        self.BASE_SPEED = self.get_parameter('base_speed').value
        self.MAX_STEERING_ANGLE = self.get_parameter('max_steering_angle').value
        self.STEERING_GAIN = self.get_parameter('steering_gain').value
        self.EMERGENCY_STOP_DISTANCE = self.get_parameter('emergency_stop_distance').value
        self.SLOW_DOWN_DISTANCE = self.get_parameter('slow_down_distance').value
        
        self.manual_speed = 1.0  # Manuel hız kontrolü için
        self.lateral_deviation = 0.0
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        
        # Subscribers
        self.lateral_sub = self.create_subscription(
            Float32, '/lane/lateral_deviation', self.lateral_callback, 10)
        
        self.obstacle_detected_sub = self.create_subscription(
            Bool, '/obstacle_detected', self.obstacle_detected_callback, 10)
        
        self.obstacle_distance_sub = self.create_subscription(
            Float32, '/obstacle_distance', self.obstacle_distance_callback, 10)
        
        # Publishers
        self.ackermann_pub = self.create_publisher(
            AckermannDrive, '/ackermann_cmd', 10)
        
        self.speed_pub = self.create_publisher(
            Float32, '/speed', 10)

        self.timer = self.create_timer(0.1, self.decision_loop)  # 10 Hz
        
        self.get_logger().info('🚗 Decision Making Node başlatıldı.')

    def lateral_callback(self, msg):
        """Şerit sapma bilgisini al"""
        self.lateral_deviation = msg.data

    def obstacle_detected_callback(self, msg):
        """Engel tespit durumunu al"""
        self.obstacle_detected = msg.data

    def obstacle_distance_callback(self, msg):
        """Engel mesafesini al"""
        if msg.data > 0:
            self.obstacle_distance = msg.data
        else:
            self.obstacle_distance = float('inf')

    def calculate_steering_angle(self):
        """Direksiyon açısını hesapla - Düz yol için basit şerit takibi"""
        # Basit orantısal kontrol
        steering_angle = -self.lateral_deviation * self.STEERING_GAIN
        
        # Maksimum direksiyon açısı sınırlaması
        steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
        
        return steering_angle

    def calculate_speed(self):
        """Hız hesapla - Düz yol için"""
        speed = self.manual_speed  # Manuel hızı kullan
        
        # Engel durumunda hız kontrolü
        if self.obstacle_detected:
            if self.obstacle_distance <= self.EMERGENCY_STOP_DISTANCE:
                # Acil dur
                self.get_logger().info('Acil duruş! Engel çok yakın!')
                speed = 0.0
                self.get_logger().warn('Acil duruş! Engel çok yakın!')        
        return max(speed, 0.0)

    def decision_loop(self):
        """Ana karar verme döngüsü"""
        try:
            # Direksiyon açısını hesapla
            steering_angle = self.calculate_steering_angle()
            
            # Hızı hesapla
            speed = self.calculate_speed()
            
            # Ackermann mesajı hazırla ve yayınla
            ackermann_msg = AckermannDrive()
            ackermann_msg.speed = speed
            ackermann_msg.steering_angle = steering_angle
            ackermann_msg.steering_angle_velocity = 0.0
            ackermann_msg.acceleration = 0.0
            ackermann_msg.jerk = 0.0
            
            self.ackermann_pub.publish(ackermann_msg)
            
            # Speed mesajı hazırla ve yayınla
            speed_msg = Float32()
            speed_msg.data = speed
            self.speed_pub.publish(speed_msg)
            
            # Debug bilgisi
            self.get_logger().debug(
                f'Speed: {speed:.2f} m/s | Steering: {steering_angle:.3f} rad | '
                f'Deviation: {self.lateral_deviation:.3f} | Obstacle: {self.obstacle_detected} | '
                f'Distance: {self.obstacle_distance:.2f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Decision loop error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DecisionMakingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()