import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class DepthImageSubscriber(Node):
    def __init__(self):
        super().__init__('depth_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/zed2i_depth/image_raw',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()
        self.last_save_time = time.time()
        self.frame_id = 0

    def image_callback(self, msg):
        try:
            # Derinlik görüntüsünü alıp RGB formatına dönüştürüyoruz
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_image_rgb = cv2.cvtColor(depth_image, cv2.COLOR_BGR2RGB)  # BGR'den RGB'ye dönüşüm
        except Exception as e:
            self.get_logger().info(f"Failed to convert image: {e}")
            return

        cv2.imshow('Depth Image', depth_image_rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

        # Save image every 0.5 seconds
        if time.time() - self.last_save_time >= 0.5:
            save_path = f'depth_image_282{self.frame_id}.png'
            cv2.imwrite(save_path, depth_image_rgb)
            self.get_logger().info(f"Saved depth image {self.frame_id} to {save_path}")
            self.frame_id += 1
            self.last_save_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthImageSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
