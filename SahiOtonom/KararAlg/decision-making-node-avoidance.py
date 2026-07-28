#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive
from enum import Enum
import numpy as np
import time

class VehicleState(Enum):
    """Araç durumu enum'u"""
    NORMAL = "normal"
    OBSTACLE_DETECTED = "obstacle_detected"
    EMERGENCY_STOP = "emergency_stop"
    OBSTACLE_AVOIDANCE_LEFT = "obstacle_avoidance_left"  # Sola kaçınma
    OBSTACLE_AVOIDANCE_FOLLOW_LANE = "obstacle_avoidance_follow_lane"  # Şerit takibi
    OBSTACLE_AVOIDANCE_RIGHT = "obstacle_avoidance_right"  # Sağa dönüş

class DecisionMakingNode(Node):
    """
    Gelişmiş karar verme node'u: Vehicle state ile durum yönetimi ve engel kaçınma
    """
    def __init__(self):
        super().__init__('decision_making_node')
        
        # Parametreler
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('steering_gain', 1.5)
        self.declare_parameter('emergency_stop_distance', 20.0)
        self.declare_parameter('slow_down_distance', 4.0)
        self.declare_parameter('emergency_stop_duration', 4.0)
        self.declare_parameter('avoidance_left_deviation', 0.3)  # Sola kaçınma sapma değeri
        self.declare_parameter('avoidance_right_deviation', -0.3)  # Sağa kaçınma sapma değeri
        self.declare_parameter('avoidance_phase_duration', 5.0)  # Her aşamanın süresi
        self.declare_parameter('obstacle_is_right', True)  # MANUEL PARAMETRE: Engel hangi tarafta (True: sağda, False: solda)
        
        self.BASE_SPEED = self.get_parameter('base_speed').value
        self.MAX_STEERING_ANGLE = self.get_parameter('max_steering_angle').value
        self.STEERING_GAIN = self.get_parameter('steering_gain').value
        self.EMERGENCY_STOP_DISTANCE = self.get_parameter('emergency_stop_distance').value
        self.SLOW_DOWN_DISTANCE = self.get_parameter('slow_down_distance').value
        self.EMERGENCY_STOP_DURATION = self.get_parameter('emergency_stop_duration').value
        self.AVOIDANCE_LEFT_DEVIATION = self.get_parameter('avoidance_left_deviation').value
        self.AVOIDANCE_RIGHT_DEVIATION = self.get_parameter('avoidance_right_deviation').value
        self.AVOIDANCE_PHASE_DURATION = self.get_parameter('avoidance_phase_duration').value
        self.obstacle_is_right = self.get_parameter('obstacle_is_right').value  # MANUEL DEĞER
        
        # Vehicle State Management
        self.vehicle_state = VehicleState.NORMAL
        self.emergency_stop_start_time = None
        self.avoidance_start_time = None
        self.current_phase_start_time = None
        
        # Obstacle detection variables
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        
        # Deviation variables
        self.lateral_deviation = 0.0  # Şerit algılama sisteminden gelen
        self.new_lateral_deviation = 0.0  # Lane following'e gönderilecek
        
        # Speed control
        self.manual_speed = self.BASE_SPEED
        
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
        
        # Yeni lateral deviation publisher - lane following'e gönderilecek
        self.new_lateral_deviation_pub = self.create_publisher(
            Float32, '/lane/lateral_new_deviation', 10)
        
        # Debug publishers
        self.vehicle_state_pub = self.create_publisher(
            Float32, '/vehicle_state', 10)

        self.timer = self.create_timer(0.1, self.decision_loop)  # 10 Hz
        
        side_str = "sağda" if self.obstacle_is_right else "solda"
        self.get_logger().info(f'🚗 Gelişmiş Decision Making Node başlatıldı. Engel pozisyonu: {side_str}')

    def lateral_callback(self, msg):
        """Şerit sapma bilgisini al (lane detection'dan)"""
        self.lateral_deviation = msg.data

    def obstacle_detected_callback(self, msg):
        """Engel tespit durumunu al"""
        prev_obstacle_detected = self.obstacle_detected
        self.obstacle_detected = msg.data
        
        if not prev_obstacle_detected and self.obstacle_detected:
            side_str = "sağda" if self.obstacle_is_right else "solda"
            self.get_logger().info(f'🚨 Engel tespit edildi! Pozisyon: {side_str}')
        elif prev_obstacle_detected and not self.obstacle_detected:
            self.get_logger().info('✅ Engel kayboldu, normal duruma dönülüyor.')

    def obstacle_distance_callback(self, msg):
        """Engel mesafesini al"""
        if msg.data > 0:
            self.obstacle_distance = msg.data
        else:
            self.obstacle_distance = float('inf')

    def update_vehicle_state(self):
        """Vehicle state'i güncelle"""
        current_time = time.time()
        
        # Emergency Stop Logic
        if self.obstacle_detected and self.obstacle_distance <= self.EMERGENCY_STOP_DISTANCE:
            if self.vehicle_state != VehicleState.EMERGENCY_STOP:
                # Emergency stop'a geçiş
                self.vehicle_state = VehicleState.EMERGENCY_STOP
                self.emergency_stop_start_time = current_time
                self.get_logger().warn(f'⛔ EMERGENCY STOP başladı! Mesafe: {self.obstacle_distance:.2f}m')
            
            elif self.emergency_stop_start_time and \
                 (current_time - self.emergency_stop_start_time) >= self.EMERGENCY_STOP_DURATION:
                # Emergency stop süresi doldu, engel kaçınma moduna geç
                if self.obstacle_is_right:
                    self.vehicle_state = VehicleState.OBSTACLE_AVOIDANCE_LEFT
                    self.get_logger().warn('🔄 OBSTACLE AVOIDANCE LEFT moduna geçildi!')
                else:
                    self.vehicle_state = VehicleState.OBSTACLE_AVOIDANCE_RIGHT
                    self.get_logger().warn('🔄 OBSTACLE AVOIDANCE RIGHT moduna geçildi!')
                
                self.avoidance_start_time = current_time
                self.current_phase_start_time = current_time
        
        # Obstacle Avoidance State Machine
        elif self.vehicle_state in [VehicleState.OBSTACLE_AVOIDANCE_LEFT, 
                                    VehicleState.OBSTACLE_AVOIDANCE_FOLLOW_LANE,
                                    VehicleState.OBSTACLE_AVOIDANCE_RIGHT]:
            
            phase_elapsed = current_time - self.current_phase_start_time
            
            if phase_elapsed >= self.AVOIDANCE_PHASE_DURATION:
                # Bir sonraki aşamaya geç
                if self.vehicle_state == VehicleState.OBSTACLE_AVOIDANCE_LEFT:
                    self.vehicle_state = VehicleState.OBSTACLE_AVOIDANCE_FOLLOW_LANE
                    self.get_logger().info('🛣️  Şerit takibi aşamasına geçildi!')
                    
                elif self.vehicle_state == VehicleState.OBSTACLE_AVOIDANCE_RIGHT:
                    self.vehicle_state = VehicleState.OBSTACLE_AVOIDANCE_FOLLOW_LANE
                    self.get_logger().info('🛣️  Şerit takibi aşamasına geçildi!')
                    
                elif self.vehicle_state == VehicleState.OBSTACLE_AVOIDANCE_FOLLOW_LANE:
                    # Şerit takibinden sonra ters yöne sap
                    if self.obstacle_is_right:
                        self.vehicle_state = VehicleState.OBSTACLE_AVOIDANCE_RIGHT
                        self.get_logger().info('➡️  Sağa dönüş aşamasına geçildi!')
                    else:
                        self.vehicle_state = VehicleState.OBSTACLE_AVOIDANCE_LEFT
                        self.get_logger().info('⬅️  Sola dönüş aşamasına geçildi!')
                
                self.current_phase_start_time = current_time
            
            # Toplam kaçınma süresi kontrolü (3 aşama × süre)
            total_avoidance_elapsed = current_time - self.avoidance_start_time
            if total_avoidance_elapsed >= (3 * self.AVOIDANCE_PHASE_DURATION):
                # Kaçınma tamamlandı, normal moda dön
                self.vehicle_state = VehicleState.NORMAL
                self.avoidance_start_time = None
                self.current_phase_start_time = None
                self.emergency_stop_start_time = None
                self.get_logger().info('✅ Engel kaçınma tamamlandı! NORMAL moduna dönüldü!')
                
        elif self.obstacle_detected:
            # Engel var ama uzakta
            if self.vehicle_state == VehicleState.NORMAL:
                self.vehicle_state = VehicleState.OBSTACLE_DETECTED
                
        elif not self.obstacle_detected:
            # Engel yok, normal duruma dön
            if self.vehicle_state != VehicleState.NORMAL:
                self.vehicle_state = VehicleState.NORMAL
                self.emergency_stop_start_time = None
                self.avoidance_start_time = None
                self.current_phase_start_time = None

    def calculate_new_lateral_deviation(self):
        """State'e göre yeni lateral deviation hesapla"""
        if self.vehicle_state == VehicleState.OBSTACLE_AVOIDANCE_LEFT:
            # Sola kaçınma
            self.new_lateral_deviation = self.AVOIDANCE_LEFT_DEVIATION
            self.get_logger().debug(f'⬅️  Sola kaçınma: {self.new_lateral_deviation:.3f}')
            
        elif self.vehicle_state == VehicleState.OBSTACLE_AVOIDANCE_RIGHT:
            # Sağa kaçınma
            self.new_lateral_deviation = self.AVOIDANCE_RIGHT_DEVIATION
            self.get_logger().debug(f'➡️  Sağa kaçınma: {self.new_lateral_deviation:.3f}')
            
        elif self.vehicle_state == VehicleState.OBSTACLE_AVOIDANCE_FOLLOW_LANE:
            # Şerit takibi sırasında gerçek lateral deviation kullan
            self.new_lateral_deviation = self.lateral_deviation
            self.get_logger().debug(f'🛣️  Şerit takibi: {self.new_lateral_deviation:.3f}')
            
        else:
            # Normal durumda ve diğer durumlarda gerçek değeri kullan
            self.new_lateral_deviation = self.lateral_deviation

    def calculate_steering_angle(self):
        """new_lateral_deviation'a göre direksiyon açısını hesapla"""
        if self.vehicle_state == VehicleState.EMERGENCY_STOP:
            steering_angle = 0.0
        else:
            steering_angle = -self.new_lateral_deviation * self.STEERING_GAIN
            steering_angle = np.clip(steering_angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)
            
        return steering_angle

    def calculate_speed(self):
        """Durum bazlı hız hesapla"""
        if self.vehicle_state == VehicleState.EMERGENCY_STOP:
            speed = 0.0
        else:
            # Diğer tüm durumlarda base speed kullan
            speed = self.manual_speed
        
        return max(speed, 0.0)

    def decision_loop(self):
        """Ana karar verme döngüsü"""
        try:
            # Vehicle state'i güncelle
            self.update_vehicle_state()
            
            # Yeni lateral deviation hesapla
            self.calculate_new_lateral_deviation()
            
            # Direksiyon açısını hesapla
            steering_angle = self.calculate_steering_angle()
            
            # Hızı hesapla
            speed = self.calculate_speed()
            
            # Mesajları yayınla
            # 1. Ackermann mesajı
            ackermann_msg = AckermannDrive()
            ackermann_msg.speed = speed
            ackermann_msg.steering_angle = steering_angle
            ackermann_msg.steering_angle_velocity = 0.0
            ackermann_msg.acceleration = 0.0
            ackermann_msg.jerk = 0.0
            self.ackermann_pub.publish(ackermann_msg)
            
            # 2. Speed mesajı
            speed_msg = Float32()
            speed_msg.data = speed
            self.speed_pub.publish(speed_msg)
            
            # 3. New Lateral Deviation mesajı (lane following'e gönderilecek)
            new_lateral_msg = Float32()
            new_lateral_msg.data = self.new_lateral_deviation
            self.new_lateral_deviation_pub.publish(new_lateral_msg)
            
            # 4. Vehicle state mesajı (debug için)
            state_msg = Float32()
            state_msg.data = float(list(VehicleState).index(self.vehicle_state))
            self.vehicle_state_pub.publish(state_msg)
            
            # Debug bilgisi
            if self.vehicle_state != VehicleState.NORMAL:
                self.get_logger().info(
                    f'State: {self.vehicle_state.value} | Speed: {speed:.2f} | '
                    f'Steering: {steering_angle:.3f} | Original Dev: {self.lateral_deviation:.3f} | '
                    f'New Dev: {self.new_lateral_deviation:.3f} | Obstacle: {self.obstacle_detected} | '
                    f'Distance: {self.obstacle_distance:.2f}m | Side: {"Right" if self.obstacle_is_right else "Left"}'
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