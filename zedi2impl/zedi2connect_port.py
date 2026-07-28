import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2

class ZEDPublisherNode(Node):
    def __init__(self):
        super().__init__('zed_publisher_node')
        self.publisher_ = self.create_publisher(Image, '/zed2i_rgb/image_raw', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
        self.bridge = CvBridge()
        self.get_logger().info("ZEDPublisherNode başlatılıyor...")
        
        # ZED kamera başlat
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED kamerayı açamadı: {err}")
            rclpy.shutdown()
            return
        
        self.image = sl.Mat()
        self.get_logger().info("ZED kamera başarıyla başlatıldı.")
    
    def timer_callback(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = self.image.get_data()
            
            # ZED genellikle BGRA formatında döndürür, BGR2RGB dönüşümü yap
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
            
            # ROS Image mesajı olarak yayınla
            msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            self.publisher_.publish(msg)
            self.get_logger().info("Görüntü yayınlandı.")
        else:
            self.get_logger().warning("ZED'den görüntü alınamadı.")
    
    def destroy_node(self):
        self.get_logger().info("ZED kamera kapatılıyor...")
        self.zed.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZEDPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import pyzed.sl as sl 
# import cv2
# class ZEDPublisherNode(Node):
#     def __init__(self):
#         super().__init__('zed_publisher_node')
#         self.publisher_ = self.create_publisher(Image, '/zed2i_rgb/image_raw', 10)
#         self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
#         self.bridge = CvBridge()

#         self.get_logger().info("ZEDPublisherNode başlatılıyor...")

#         # ZED kamera başlat
#         self.zed = sl.Camera()
#         init_params = sl.InitParameters()
#         init_params.camera_resolution = sl.RESOLUTION.HD720
#         init_params.camera_fps = 30

#         err = self.zed.open(init_params)
#         if err != sl.ERROR_CODE.SUCCESS:
#             self.get_logger().error(f"ZED kamerayı açamadı: {err}")
#             rclpy.shutdown()
#             return

#         self.image = sl.Mat()
#         self.get_logger().info("ZED kamera başarıyla başlatıldı.")

#     def timer_callback(self):
#         if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
#             self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
#             frame = self.image.get_data()
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

#             # ROS Image mesajı olarak yayınla
#             msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
#             self.publisher_.publish(msg)
#             self.get_logger().info("Görüntü yayınlandı.")
#         else:
#             self.get_logger().warning("ZED'den görüntü alınamadı.")

#     def destroy_node(self):
#         self.get_logger().info("ZED kamera kapatılıyor...")
#         self.zed.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = ZEDPublisherNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import zmq
# import numpy as np
# import cv2

# class ZedRgbSubscriber(Node):
#     def __init__(self):
#         super().__init__('zed_rgb_subscriber')
        
#         # RGB görüntü için topic
#         self.publisher_ = self.create_publisher(Image, '/zed2i_rgb/image_raw', 10)
#         self.bridge = CvBridge()
        
#         # ZeroMQ subscriber
#         context = zmq.Context()
#         self.socket = context.socket(zmq.SUB)
#         self.socket.connect("tcp://172.30.64.1:5555")  # Windows IP adresi
#         self.socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Tüm mesajları al
        
#         # Timer - 30Hz
#         self.timer = self.create_timer(0.03, self.timer_callback)
        
#         self.get_logger().info("ZED RGB Subscriber başlatıldı")
#         self.get_logger().info(f"ZeroMQ bağlantısı: tcp://172.30.64.1:5555")
#         self.get_logger().info(f"Yayın topic'i: /zed2i_rgb/image_raw")
    
#     def timer_callback(self):
#         try:
#             msg = self.socket.recv(flags=zmq.NOBLOCK)
#             self.get_logger().info(f"RGB veri alındı! Boyut: {len(msg)} bytes")
#         except zmq.Again:
#             return  # Veri yoksa hemen çık
        
#         # JPEG verisini RGB olarak decode et
#         np_arr = np.frombuffer(msg, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR formatında
        
#         if cv_image is None:
#             self.get_logger().warning("RGB görüntü decode edilemedi!")
#             return
        
#         self.get_logger().info(f"RGB görüntü işlendi: {cv_image.shape}, dtype: {cv_image.dtype}")
        
#         # ROS mesajına çevir (BGR8 encoding)
#         ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
#         ros_image.header.stamp = self.get_clock().now().to_msg()
#         ros_image.header.frame_id = "zed2i_rgb_frame"
        
#         # Yayınla
#         self.publisher_.publish(ros_image)
#         self.get_logger().info("RGB görüntü topic'te yayınlandı!")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ZedRgbSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()