# Dosya Adı: dashboard_node.py
# Konum: /home/sahi/SahiOtonom/dashboard_node.py (launch_all_nodes.py ile aynı yere koyabilirsin)

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QTimer, Qt
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from cv_bridge import CvBridge
import numpy as np

class Dashboard(QWidget):
    # GUI'ı güncellemek için sinyaller oluştur
    update_sign_image_signal = pyqtSignal(np.ndarray)
    update_lane_image_signal = pyqtSignal(np.ndarray)
    update_steering_angle_signal = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.bridge = CvBridge()
        self.init_ui()

        # Sinyalleri güncelleme fonksiyonlarına bağla
        self.update_sign_image_signal.connect(self.set_sign_image)
        self.update_lane_image_signal.connect(self.set_lane_image)
        self.update_steering_angle_signal.connect(self.set_steering_angle)

    def init_ui(self):
        self.setWindowTitle('SAHI OTONOM - GÖREV KONTROL PANELİ')
        self.setGeometry(100, 100, 1280, 560) # Pencere boyutu: Genişlik x Yükseklik
        self.setStyleSheet("background-color: #2E2E2E; color: white;")

        # --- Ana Layout'lar ---
        main_layout = QVBoxLayout() # Dikey ana layout (üstte videolar, altta veri)
        video_layout = QHBoxLayout() # Yatay video layout'u

        # --- Video Görüntüleri ---
        self.sign_video_label = self.create_video_label("LEVHA TESPİT (YOLO)")
        self.lane_video_label = self.create_video_label("ŞERİT TESPİT")
        
        video_layout.addWidget(self.sign_video_label)
        video_layout.addWidget(self.lane_video_label)
        
        # --- Veri Alanı ---
        self.steering_label = self.create_data_label("DİREKSİYON AÇISI (RAD)")

        main_layout.addLayout(video_layout)
        main_layout.addWidget(self.steering_label)
        self.setLayout(main_layout)

    def create_video_label(self, title):
        label = QLabel(f'{title}\n\nVeri bekleniyor...')
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont('Arial', 12))
        label.setFixedSize(640, 480)
        label.setStyleSheet("background-color: black; border: 1px solid #555;")
        return label
    
    def create_data_label(self, title):
        label = QLabel(f'{title}: -')
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont('Arial', 16, QFont.Bold))
        label.setStyleSheet("background-color: #1E1E1E; border-radius: 5px; padding: 5px;")
        return label

    # --- Sinyal Slotları (Güncelleme Fonksiyonları) ---
    @pyqtSlot(np.ndarray)
    def set_sign_image(self, cv_img):
        self.update_video_label(self.sign_video_label, cv_img)

    @pyqtSlot(np.ndarray)
    def set_lane_image(self, cv_img):
        self.update_video_label(self.lane_video_label, cv_img)
        
    def update_video_label(self, label, cv_img):
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qt_img)
        label.setPixmap(pixmap.scaled(640, 480, Qt.KeepAspectRatio))

    @pyqtSlot(float)
    def set_steering_angle(self, angle):
        self.steering_label.setText(f"DİREKSİYON AÇISI (RAD): {angle:.3f}")

class RosNode(Node):
    def __init__(self, gui):
        super().__init__('dashboard_ros_node')
        self.gui = gui
        self.bridge = CvBridge()

        # İlgili topic'lere abone ol
        self.create_subscription(Image, '/sign_detection/output', self.sign_callback, 10)
        self.create_subscription(Image, 'lane_detection_output', self.lane_callback, 10)
        self.create_subscription(AckermannDrive, '/ackermann_cmd', self.steering_callback, 10)
        
        self.get_logger().info('Dashboard ROS Düğümü başlatıldı.')

    def sign_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.gui.update_sign_image_signal.emit(cv_image)

    def lane_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.gui.update_lane_image_signal.emit(cv_image)

    def steering_callback(self, msg):
        self.gui.update_steering_angle_signal.emit(msg.steering_angle)

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    dashboard_gui = Dashboard()
    ros_node = RosNode(dashboard_gui)
    
    # ROS düğümünü ayrı bir thread'de çalıştırmak yerine QTimer kullanmak daha güvenli
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros_node)
    
    def spin_ros():
        executor.spin_once(timeout_sec=0.01)

    timer = QTimer()
    timer.timeout.connect(spin_ros)
    timer.start(10) # 10 ms'de bir ROS'u kontrol et
    
    dashboard_gui.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()