#!/usr/bin/env python3
# Birleşik izleme düğümü (TEK KARE):
# Şerit tespiti görüntüsünün (lane_detection_output) ÜSTÜNE levha kutularını
# (/sign_detection/boxes) çizer ve tek pencerede gösterir. Ekran bölünmez.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np

BOX_COLOR = (255, 165, 0)  # BGR - levha kutu rengi

class CombinedViewNode(Node):
    def __init__(self):
        super().__init__('combined_view_node')
        self.br = CvBridge()
        self.lane_frame = None
        self.sign_frame = None      # şerit görüntüsü hiç gelmezse yedek
        self.sign_boxes = None      # {"w":..,"h":..,"boxes":[{x1,y1,x2,y2,label}]}

        self.create_subscription(Image, 'lane_detection_output', self.lane_callback, 10)
        self.create_subscription(String, '/sign_detection/boxes', self.boxes_callback, 10)
        self.create_subscription(Image, '/sign_detection/output', self.sign_callback, 10)

        # Gösterimi sabit aralıkla yenile (30 FPS)
        self.create_timer(1.0 / 30.0, self.show)
        self.get_logger().info('🖥️ Birleşik görüntü (tek kare: Yol + Levhalar) başlatıldı. Kapatmak için pencerede q.')

    def lane_callback(self, msg):
        self.lane_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def sign_callback(self, msg):
        self.sign_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def boxes_callback(self, msg):
        try:
            self.sign_boxes = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def draw_sign_boxes(self, frame):
        """Levha kutularını kare üstüne çizer (çözünürlük farkını ölçekler)."""
        if not self.sign_boxes or not self.sign_boxes.get("boxes"):
            return frame
        fh, fw = frame.shape[:2]
        sx = fw / self.sign_boxes["w"]
        sy = fh / self.sign_boxes["h"]
        for b in self.sign_boxes["boxes"]:
            x1, y1 = int(b["x1"] * sx), int(b["y1"] * sy)
            x2, y2 = int(b["x2"] * sx), int(b["y2"] * sy)
            cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, 2)
            label = b["label"]
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - 20), (x1 + tw, y1), BOX_COLOR, -1)
            cv2.putText(frame, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return frame

    def show(self):
        if self.lane_frame is not None:
            frame = self.draw_sign_boxes(self.lane_frame.copy())
        elif self.sign_frame is not None:
            # Şerit görüntüsü yoksa en azından levha görüntüsünü göster
            frame = self.sign_frame.copy()
        else:
            frame = np.zeros((480, 848, 3), dtype=np.uint8)
            cv2.putText(frame, 'Goruntu bekleniyor...', (30, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow('SAHI OTONOM - Yol + Levhalar', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
