import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from yolov8_msgs.msg import Yolov8Inference, DetectionWithDepth
import numpy as np
import cv2

bridge = CvBridge()

class DepthImageSubscriber(Node):

    def __init__(self):
        super().__init__('depth_image_subscriber')

        self.bridge = CvBridge()
        self.detections = []
        self.left = None
        self.top = None
        self.bottom = None
        self.right = None

        self.depth_pub = self.create_publisher(DetectionWithDepth, '/detection_with_depth', 10)
        self.angle_pub = self.create_publisher(Float32, '/sign_angle', 10)

        self.subscription_depth = self.create_subscription(
            Image,
            '/zed2i_depth/depth/image_raw',
            self.depth_image_callback,
            10)
        
        self.subscription = self.create_subscription(
            Image,
            '/zed2i_depth/image_raw',
            self.camera_callback,
            10)
        
        self.subscription_yolo = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)

        cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)

    def yolo_callback(self, data):
        self.detections = []
        
        if not data.yolov8_inference:
            depth_msg = DetectionWithDepth()
            depth_msg.class_name = "none"
            depth_msg.depth_value = 40.0
            self.depth_pub.publish(depth_msg)
            self.get_logger().warn(f"Levha: {depth_msg.class_name}, Derinlik Değeri: {depth_msg.depth_value:.2f} metre")
        else:
            for result in data.yolov8_inference:
                class_name = result.class_name
                self.top = result.top
                self.left = result.left
                self.bottom = result.bottom
                self.right = result.right

                center_x = int((self.left + self.right) / 2)
                center_y = int((self.top + self.bottom) / 2)
                
                self.detections.append((class_name, center_x, center_y, self.left, self.top, self.right, self.bottom))

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        detected_images = []
        
        if self.detections:
            for detection in self.detections:
                class_name, center_x, center_y, left, top, right, bottom = detection
                cropped_img = img[top:bottom, left:right]
                detected_images.append(cropped_img)

        if detected_images:
            combined_image = self.combine_images(detected_images)
            if combined_image is not None and combined_image.size > 0:
                cv2.imshow("Detected Objects", combined_image)
        cv2.waitKey(1)

    def combine_images(self, images, max_width=1000):
        """
        Combines a list of images into a single image, arranging them in a grid.
        """
        if not images:
            return None

        widths, heights = zip(*(i.shape[1::-1] for i in images))
        total_width = sum(widths)
        max_height = max(heights)

        if total_width > max_width:
            # Scale images to fit within max_width
            scale = max_width / total_width
            images = [cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale))) for img in images]
            widths, heights = zip(*(i.shape[1::-1] for i in images))
            total_width = sum(widths)
            max_height = max(heights)

        combined_image = np.zeros((max_height, total_width, 3), dtype=np.uint8)
        x_offset = 0
        for img in images:
            combined_image[:img.shape[0], x_offset:x_offset + img.shape[1]] = img
            x_offset += img.shape[1]

        return combined_image

    def depth_image_callback(self, data):
        depth_image = bridge.imgmsg_to_cv2(data)
        height, width = depth_image.shape

        display_image = np.zeros((height, width, 3), dtype=np.uint8)

        for class_name, center_x, center_y, left, top, right, bottom in self.detections:
            depth_value = depth_image[center_x, center_y]
            if isinstance(depth_value, (np.float32, np.float64, float, int)):
                depth_value = float(depth_value)  # Ensure depth_value is a Python float

                self.get_logger().warn(f"Levha: {class_name}, Derinlik Değeri: {depth_value:.2f} metre")
                
                depth_msg = DetectionWithDepth()
                depth_msg.class_name = class_name
                depth_msg.depth_value = depth_value
                self.depth_pub.publish(depth_msg)

                # Draw rectangle and label on the display image
                cv2.rectangle(display_image, (left, top), (right, bottom), (255, 0, 0), 2)
                label = f"{class_name} {depth_value:.2f}m"
                cv2.putText(display_image, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Detected Objects", display_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    depth_image_subscriber = DepthImageSubscriber()
    rclpy.spin(depth_image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
