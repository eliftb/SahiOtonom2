import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
from ultrafastLaneDetector import UltrafastLaneDetector, ModelType
import numpy as np
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/serit_takip_kamerasi/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher = self.create_publisher(Image, 'lane_detection_output', 10)
    
        self.br = CvBridge()
        self.model_path = "/home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/serit-tespit/models/tusimple_18.pt"
        self.model_type = ModelType.CULANE
        self.use_gpu = True
        self.lane_detector = UltrafastLaneDetector(self.model_path, self.model_type, self.use_gpu)
        self.get_logger().info('Lane detection node has been started.')

        self.pid_controller = PIDController(kp=0.0005, ki=0.0001, kd=0.0)
        self.previous_time = time.time()

        self.driving_lane_center = None
        self.left_lane_points = None
        self.right_lane_points = None
        self.turn_left = False
        self.turn_right = False

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data)
        output_img = self.lane_detector.detect_lanes(frame)
        output_msg = self.br.cv2_to_imgmsg(output_img, encoding="bgr8")
        self.publisher.publish(output_msg)
        cv2.imshow("Detected lanes", output_img)
        cv2.waitKey(1)
        
        num_lanes_detected = np.sum(self.lane_detector.lanes_detected)
        self.get_logger().info(f'Number of lanes detected: {num_lanes_detected}')
        
        driving_lane_center = self.lane_detector.get_driving_lane_center()
        if driving_lane_center:
            self.get_logger().info(f'Driving lane center: {driving_lane_center}')
            
            self.left_lane_points = self.lane_detector.lanes_points[1]
            self.right_lane_points = self.lane_detector.lanes_points[2]
            
            if self.left_lane_points and self.right_lane_points:
                self.driving_lane_center = driving_lane_center
            
            # Şeritler algılandı, sola ve sağa dönme flag'lerini sıfırla
            self.turn_left = False
            self.turn_right = False
        else:
            self.get_logger().info('Driving lane not detected.')
            self.driving_lane_center = None
            self.left_lane_points = None
            self.right_lane_points = None
            
            # Şerit algılanmadığında düz gitmek için flag'i ayarla
            self.turn_left = True

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # 0.5 m/s hızla ilerle
        
        if self.driving_lane_center and self.left_lane_points and self.right_lane_points:
            left_x_points = [point[0] for point in self.left_lane_points]
            right_x_points = [point[0] for point in self.right_lane_points]
            left_avg_x = np.mean(left_x_points)
            right_avg_x = np.mean(right_x_points)
            center_x = (left_avg_x + right_avg_x) / 2
            image_center_x = 640  # Görüntü genişliği varsayılan olarak 1280
            
            error = image_center_x - center_x  # Normal hata hesaplaması

            current_time = time.time()
            delta_time = current_time - self.previous_time
            self.previous_time = current_time
            
            control_action = self.pid_controller.update(error, delta_time)
            msg.angular.z = control_action
        else:
            if not self.left_lane_points and self.right_lane_points:
                msg.angular.z = -0.5  # Sağ şeridi algılayamıyorsa sağa dönme hareketi
                self.get_logger().info('Left lane not detected, turning right.')
            elif self.turn_left:
                msg.angular.z = 0.0  # Şerit algılanmadığında düz git
                self.get_logger().info('Lane not detected, going straight.')
            else:
                msg.angular.z = 0.0  # Şerit algılanmadığında düz git
            
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()