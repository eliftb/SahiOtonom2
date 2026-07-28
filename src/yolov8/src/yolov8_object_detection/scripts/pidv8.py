import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import DetectionWithDepth
import cv2
import numpy as np

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Subscription to the /front_left_min_distance topic
        self.subscription = self.create_subscription(
            Float32,
            '/front_left_min_distance',
            self.wall_distance_callback,
            10)

        # Subscription to the /detection_with_depth topic
        self.stop_sign_subscription = self.create_subscription(
            DetectionWithDepth,
            '/detection_with_depth',
            self.stop_sign_callback,
            10)

        # Publisher to control the robot's velocity
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.serit_tespit_publisher = self.create_publisher(String, '/start_serit_tespit', 10)
        self.ada_etrafinda_donunuz_publisher = self.create_publisher(String, '/ada_etrafinda_don', 10)

        # Desired distance to the left wall
        self.desired_distance = 1.65
        # Control parameters
        self.kp = 1.0  # Proportional gain

        # State to track if stop sign is detected
        self.stop_sign_detected = False
        self.turn_left_detected = False
        self.circle_around_detected = False
        self.go_straight_after_turn = False
        self.stop_time = None
        self.waiting = False
        self.waiting_start_time = None  # To track the start time of waiting phase
        self.timer = self.create_timer(1.0, self.timer_callback)  # Update timer interval to 1 second

        # Variable to store the last wall-following velocity message
        self.velocity_msg = Twist()

        # Variables for displaying information
        self.last_detected_sign = None
        self.last_detected_distance = None
        self.last_detected_confidence = None

        # States for stop sign handling
        self.first_stop_done = False
        self.passenger_info = None

        # State to track if wall following should be stopped
        self.wall_following_active = True

        # Flag for parking mode
        self.parking_mode = False


    def wall_distance_callback(self, msg):
        if self.wall_following_active:
            left_min_dist = msg.data
            error = self.desired_distance - left_min_dist

            # Check if in parking mode
            if self.parking_mode:
                linear_velocity = 0.6
            else:
                linear_velocity = 1.0
            
            angular_velocity = self.kp * error

            self.velocity_msg.linear.x = linear_velocity
            self.velocity_msg.angular.z = angular_velocity

            if not self.stop_sign_detected and not self.waiting and not self.turn_left_detected and not self.circle_around_detected and not self.go_straight_after_turn:
                self.velocity_publisher.publish(self.velocity_msg)

    def stop_sign_callback(self, msg):
        self.last_detected_sign = msg.class_name
        self.last_detected_distance = msg.depth_value
        self.last_detected_confidence = msg.confidence
        
        if msg.class_name == "durak" and not self.stop_sign_detected and not self.waiting:
            # if msg.depth_value <= 7.0:
                # self.velocity_msg.linear.x = 0.3  # Slow down when within 7 meters
                # self.velocity_publisher.publish(self.velocity_msg)
            if msg.depth_value <= 3.0:  # Stop completely when within 3 meters
                if not self.first_stop_done:
                    self.passenger_info = "Arac yolcu alma gorevini gercekleştiriyor..."
                    self.get_logger().info(self.passenger_info)
                else:
                    self.passenger_info = "Arac yolcu bırakma gorevini gercekleştiriyor..."
                    self.get_logger().info(self.passenger_info)
                self.stop_sign_detected = True
                self.stop_time = self.get_clock().now() + Duration(seconds=2.2)
                self.velocity_msg.linear.x = 0.0
                self.velocity_publisher.publish(self.velocity_msg)
        elif msg.class_name == "ada-etrafinda-donunuz" and msg.depth_value <= 7.3:
            pass
        elif msg.class_name == "park-yeri" and msg.depth_value <= 20.0 and msg.confidence>0.65:
            self.get_logger().info("Park yapılır tabelası algılandı, duvar mesafesi 4 metreye çıkarılıyor.")
            self.desired_distance = 4.0
            self.velocity_msg.linear.x = 0.6  # Set speed to 0.6 upon recognizing the sign
            self.velocity_publisher.publish(self.velocity_msg)
            self.parking_mode = True  # Set parking mode to true
            if msg.depth_value < 4.5:
                self.velocity_msg.linear.x = 0.0
                self.velocity_msg.angular.z = 0.0
                self.velocity_publisher.publish(self.velocity_msg)
                self.get_logger().info("Araç Park etti. Hayırlı uğurlu olsun!!!")
                self.wall_following_active = False
    
    def timer_callback(self):
        if self.stop_sign_detected:
            if self.get_clock().now() < self.stop_time:
                # Continue waiting for the specified stop time duration
                self.velocity_publisher.publish(self.velocity_msg)
            else:
                self.get_logger().info("Robot şimdi duruyor...")
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                self.velocity_publisher.publish(stop_msg)
                self.stop_sign_detected = False
                self.waiting = True
                self.waiting_start_time = self.get_clock().now()
                self.stop_time = self.waiting_start_time + Duration(seconds=30)  # 30 seconds wait time
                if not self.first_stop_done:
                    self.first_stop_done = True
        elif self.waiting:
            elapsed_time = (self.get_clock().now() - self.waiting_start_time).nanoseconds / 1e9
            remaining_time = 30 - int(elapsed_time)
            if remaining_time > 0:
                if int(elapsed_time) != int(elapsed_time + 0.1):  # Log every second
                    self.get_logger().info(f"30 saniye sayimi başlat... {remaining_time} saniye kaldı")
            else:
                self.get_logger().info("Hareket devam ediyor...")
                self.velocity_msg.linear.x = 0.6  # Set speed to 0.6 after waiting
                self.velocity_publisher.publish(self.velocity_msg)
                self.waiting = False
        elif self.turn_left_detected:
            if self.get_clock().now() - self.turn_start_time < self.turn_duration:
                # Execute left turn
                turn_msg = Twist()
                turn_msg.linear.x = 0.5  # Forward speed
                turn_msg.angular.z = 0.37  # Left turn
                self.velocity_publisher.publish(turn_msg)
                self.wall_following_active = False 
                self.go_straight_after_turn= True
            elif self.go_straight_after_turn:
                # Continue straight after left turn
                self.get_logger().info("Sola dönüş sonrası düz devam ediyor...")
                straight_msg = Twist()
                straight_msg.linear.x = 0.5
                straight_msg.angular.z = 0.0
                self.velocity_publisher.publish(straight_msg)
                self.go_straight_after_turn = False
                self.turn_left_detected = False
                self.wall_following_active = False
                self.ada_etrafinda_donunuz_publisher.publish(String(data="true"))

        elif self.circle_around_detected:
            if self.last_detected_distance > 8.0:
                pass
            else:
                # Stop when within 7 meters of the sign
                self.get_logger().info("Ada etrafında dönünüz tabelasına 7 metre yaklaşıldı, duruyor...")
                self.ada_etrafinda_donunuz_publisher.publish(String(data="true"))

        # Display the information using OpenCV
        self.display_info()

    def display_info(self):
        # Create a blank image
        display_img = np.zeros((400, 600, 3), dtype=np.uint8)

        # Add text for the last detected sign and distance
        if self.last_detected_sign is not None:
            sign_text = f"Son Tabela: {self.last_detected_sign}"
            cv2.putText(display_img, sign_text, (10, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)
        if self.last_detected_distance is not None:
            if self.last_detected_distance == 40.0:
                distance_text = "Mesafe olculemedi"
            else:
                distance_text = f"Mesafe: {self.last_detected_distance:.2f} metre"
            cv2.putText(display_img, distance_text, (10, 100), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 2)

        # Add text if the robot is in the stop sign waiting phase
        if self.waiting:
            elapsed_time = (self.get_clock().now() - self.waiting_start_time).nanoseconds / 1e9
            remaining_time = 30 - int(elapsed_time)
            if remaining_time > 0:
                countdown_text = f"30 saniye sayimi başlat... {remaining_time} saniye kaldi"
                cv2.putText(display_img, countdown_text, (10, 150), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
            else:
                cv2.putText(display_img, "Hareket devam ediyor...", (10, 150), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

        # Add text for passenger information
        if self.passenger_info is not None:
            cv2.putText(display_img, self.passenger_info, (10, 200), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

        # Display the image in a window
        cv2.imshow('Duvar Takip Bilgisi', display_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
