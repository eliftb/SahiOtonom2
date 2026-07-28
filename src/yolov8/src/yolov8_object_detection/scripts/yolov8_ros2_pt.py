#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
import cv2

bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/home/sahi/sahi_otonom-main/src/yolov8/src/yolov8_object_detection/scripts/custom_yolov8_model/model12m.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/zed2i_depth/image_raw',
            self.camera_callback,
            10)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        cv2.namedWindow("Tespit Edilen Nesneler", cv2.WINDOW_NORMAL)

    def camera_callback(self, data):
        
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        # Değişkenleri başlangıç değerleri ile tanımla
        merkez_x = 0
        merkez_y = 0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                conf = box.conf.item()  # Get the confidence of the detection and convert to float
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.inference_result.confidence = round(conf,2)
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

                cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (255, 0, 0), 2)

                # Dikdörtgenin merkez noktasını al
                merkez_x = int((b[0] + b[2]) / 2)
                merkez_y = int((b[1] + b[3]) / 2)

                # Merkeze kırmızı bir nokta çiz
                cv2.circle(img, (merkez_x, merkez_y), 5, (0, 0, 255), -1)
                label = f"{self.model.names[int(c)]} {conf:.2f}"
                cv2.putText(img, label, (int(b[0]), int(b[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            camera_subscriber.get_logger().info(f"{self.yolov8_inference}, center x,y: {merkez_x}, {merkez_y}")

        cv2.imshow("Tespit Edilen Nesneler", img)
        cv2.waitKey(1)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        
        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
