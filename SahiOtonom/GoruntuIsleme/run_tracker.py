# Dosya Adı: sign_detector_node.py
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
from ultralytics import YOLO
from collections import defaultdict, deque, Counter
import os

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')
        self.br = CvBridge()
        
        # --- Parametreler ---
        self.declare_parameter('model_name', 'UltraConservative_BEST_mAP0.9248_20250801_115028.pt')
        
        # YENİ: Takip stabilizasyonu için ayarlanabilir parametreler
        # Aracın hızı yavaş (5km/s) olduğu için daha uzun bir geçmişe bakabiliriz.
        # history_length: Bir nesnenin sınıfını belirlemek için kaç kare geriye bakılacağı.
        self.declare_parameter('history_length', 10) 
        # min_confidence_frames: history_length içinde bir sınıfın en az kaç kere
        # görülmesi gerektiği ki "stabil" olarak kabul edilsin.
        self.declare_parameter('min_confidence_frames', 4) 

        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.history_length = self.get_parameter('history_length').get_parameter_value().integer_value
        self.min_confidence_frames = self.get_parameter('min_confidence_frames').get_parameter_value().integer_value

        # Her bir takip ID'sinin sınıf geçmişini tutmak için deque kullanıyoruz.
        # deque, maxlen parametresi sayesinde otomatik olarak eski verileri atar.
        self.track_class_history = defaultdict(lambda: deque(maxlen=self.history_length))
        
        # --- Model Yükleme ---
        try:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            model_path = os.path.join(script_dir, model_name)
            self.model = YOLO(model_path)
            self.get_logger().info(f'✅ Levha modeli yüklendi: {model_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Levha modeli yüklenemedi: {e}')
            rclpy.shutdown()
            return
            
        self.subscription = self.create_subscription(Image, '/zed2i_rgb/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/sign_detection/output', 10)
        # Stabil levha kutularını JSON olarak yayınlar; birleşik görüntü (combined_view.py)
        # bu kutuları şerit görüntüsünün üstüne çizer.
        self.boxes_pub = self.create_publisher(String, '/sign_detection/boxes', 10)
        self.get_logger().info('✅ Gelişmiş Levha Tespit Düğümü Başlatıldı.')

    def image_callback(self, msg):
        try:
            # Görüntüyü ROS'tan alıp OpenCV formatına çeviriyoruz (BGR)
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLO ile takip et (henüz çizim yapma)
            results = self.model.track(frame, persist=True, conf=0.4, iou=0.5, verbose=False)
            
            # Geliştirilmiş fonksiyonumuzla sonuçları işle, stabilize et ve çiz
            annotated_frame, stable_boxes = self.draw_stabilized_annotations(results[0], frame.copy())

            # Görüntü artık ayrı pencerede gösterilmiyor; birleşik izleme düğümü
            # (combined_view.py) kutuları şerit görüntüsünün üstüne çizip tek pencerede gösterir.
            self.publisher.publish(self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8"))
            h, w = frame.shape[:2]
            self.boxes_pub.publish(String(data=json.dumps(
                {"w": w, "h": h, "boxes": stable_boxes})))
        except Exception as e:
            self.get_logger().error(f'Levha Tespit Callback Hatası: {e}')
            import traceback
            traceback.print_exc()

    def draw_stabilized_annotations(self, result, frame):
        """
        Tespit sonuçlarını alır, sınıflandırmayı stabilize eder ve
        sonuçları manuel olarak OpenCV ile görüntü üzerine çizer.
        """
        stable_boxes = []
        # Eğer takip edilen bir nesne yoksa, orijinal görüntüyü direkt döndür.
        if not hasattr(result.boxes, 'id') or result.boxes.id is None:
            return frame, stable_boxes

        track_ids = result.boxes.id.int().cpu().tolist()
        boxes = result.boxes.xywh.cpu()
        class_ids = result.boxes.cls.int().cpu().tolist()

        # Her bir tespit edilen nesne için döngü
        for box, track_id, class_id in zip(boxes, track_ids, class_ids):
            # 1. Bu nesnenin sınıflandırma geçmişini güncelle
            self.track_class_history[track_id].append(class_id)
            
            # 2. Geçmişi analiz et
            history = self.track_class_history[track_id]
            if not history:
                continue

            # 3. Geçmişteki en yaygın sınıfı ve görülme sayısını bul
            most_common_class, confidence_count = Counter(history).most_common(1)[0]

            # 4. Sadece yeterince emin olduğumuz (stabil) sınıfları çiz
            if confidence_count >= self.min_confidence_frames:
                x, y, w, h = box
                x1, y1 = int(x - w / 2), int(y - h / 2)
                x2, y2 = int(x + w / 2), int(y + h / 2)
                
                # Stabil sınıfın adını modelden al
                stable_class_name = self.model.names[most_common_class]
                label = f"ID:{track_id} {stable_class_name}"

                # RENK DÜZELTMESİ: OpenCV BGR formatında renk bekler (Mavi, Yeşil, Kırmızı)
                color = (255, 165, 0) # Güzel bir mavi tonu (BGR)

                # Dikdörtgeni çiz
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # Etiket için daha okunaklı bir arka plan çiz
                (w_text, h_text), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, y1 - 20), (x1 + w_text, y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) # Yazı Beyaz

                # label ekrana çizim için (combined_view onu kullanır); cls ve id
                # ise KARAR ALMA için: sınıf adını "ID:3 dur" metninden ayrıştırmak
                # zorunda kalmasın, levhayı takip numarasıyla tanıyabilsin
                # (aynı 'dur' levhasında ikinci kez durmamak için gerekiyor).
                stable_boxes.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2,
                                     "label": label,
                                     "cls": stable_class_name,
                                     "id": int(track_id)})

        return frame, stable_boxes

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()
        self.get_logger().info("Levha Tespit Düğümü kapatıldı.")

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Launcher kritik düğüm ölünce hepsini kapatıyor; bu normal kapanış.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()