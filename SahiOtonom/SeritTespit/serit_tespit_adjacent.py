#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os
import sys

from utils.utils import (
    select_device,
    driving_area_mask, lane_line_mask, show_seg_result
)

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.br = CvBridge()

        self.weights = '/home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/serit-tespit/models/tusimple_18.pt'
        self.device = select_device('0')
        self.half = self.device.type != 'cpu'
        self.img_size = 640

        # ROS Image subscription
        self.subscription = self.create_subscription(
            Image, '/zed2i_rgb/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, 'lane_detection_output', 10)

        # ROS Publishers
        self.lateral_pub = self.create_publisher(Float32, '/lane/lateral_deviation', 10)
        self.intersection_pub = self.create_publisher(Int32, '/lane/intersection_direction', 10)
        self.left_adjacent_pub = self.create_publisher(Float32, '/lane/left_adjacent_center', 10)
        self.right_adjacent_pub = self.create_publisher(Float32, '/lane/right_adjacent_center', 10)

        # Geçmiş veriler için
        self.previous_deviation = 0.0
        self.deviation_history = []
        self.max_history_size = 5

        self.intersection_history = []
        self.intersection_history_size = 3
        self.stable_intersection_count = 0
        self.min_stable_frames = 2

        # Sol ve sağ komşu şerit takibi için ayrı geçmiş
        self.left_adjacent_history = []
        self.right_adjacent_history = []
        self.adjacent_history_size = 3

        # Frame sayacı
        self.frame_count = 0

        self.load_model()
        self.get_logger().info('🚦 Lane Detection Node başlatıldı.')
    def load_model(self):
        if not os.path.exists(self.weights):
            self.get_logger().error(f'❌ Model dosyası bulunamadı: {self.weights}')
            sys.exit(1)
            
        self.model = torch.jit.load(self.weights).to(self.device)
        if self.half:
            self.model.half()
        self.model.eval()
        self.get_logger().info('✅ Model yüklendi.')

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        shape = img.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:
            r = min(r, 1.0)
        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        if auto:
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)
        elif scaleFill:
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
        dw /= 2
        dh /= 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, ratio, (dw, dh)

    def preprocess_image(self, im0):
        lab = cv2.cvtColor(im0, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        img, _, _ = self.letterbox(enhanced, new_shape=self.img_size, stride=32)
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        return img

    def find_all_lane_lines(self, lane_mask):
        """
        Tüm şerit çizgilerini tespit eder
        """
        height, width = lane_mask.shape
        
        # Alt yarıda odaklan (daha stabil sonuçlar için)
        roi_height_start = int(height * 0.6)
        roi = lane_mask[roi_height_start:, :]
        
        # Morfolojik işlemlerle gürültüyü temizle
        kernel = np.ones((3, 3), np.uint8)
        roi_cleaned = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        roi_cleaned = cv2.morphologyEx(roi_cleaned, cv2.MORPH_OPEN, kernel)
        
        # Konturları bul
        contours, _ = cv2.findContours(roi_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return []
        
        # Konturları boyut ve pozisyona göre filtrele
        valid_contours = []
        min_contour_area = 100  # Minimum kontur alanı
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_contour_area:
                # Konturun merkez x koordinatını hesapla
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + roi_height_start  # ROI offset'i ekle
                    valid_contours.append({'contour': contour, 'center_x': cx, 'center_y': cy, 'area': area})
        
        # X koordinatına göre sırala (soldan sağa)
        valid_contours = sorted(valid_contours, key=lambda x: x['center_x'])
        
        return valid_contours

    def find_lane_lines(self, lane_mask):
        """
        Mevcut şerit için sol/sağ şerit sınırlarını bulur
        """
        all_lanes = self.find_all_lane_lines(lane_mask)
        
        if len(all_lanes) < 2:
            return None, None
            
        height, width = lane_mask.shape
        center_x = width / 2
        
        # En yakın sol ve sağ şeritleri bul
        left_line = None
        right_line = None
        
        for lane in all_lanes:
            if lane['center_x'] < center_x and left_line is None:
                left_line = lane['contour']
            elif lane['center_x'] > center_x and right_line is None:
                right_line = lane['contour']
                
        return left_line, right_line

    def detect_adjacent_lane_centers(self, lane_mask):
        """
        Sadece sol ve sağ komşu şeritlerin merkezlerini tespit eder
        Döndürülen liste: [sol_komşu, sağ_komşu]
        """
        height, width = lane_mask.shape
        image_center = width / 2
        
        all_lanes = self.find_all_lane_lines(lane_mask)
        
        if len(all_lanes) < 2:
            return [None, None]  # [sol_komşu, sağ_komşu]
        
        # Mevcut şeridin sınırlarını bul
        current_left = None
        current_right = None
        
        for lane in all_lanes:
            if lane['center_x'] < image_center:
                current_left = lane
            elif lane['center_x'] > image_center and current_right is None:
                current_right = lane
                break
                
        # Tipik şerit genişliği
        typical_lane_width = width * 0.35
        
        # Sol ve sağ komşu şerit merkezleri
        left_adjacent_center = None
        right_adjacent_center = None
        
        # Sol komşu şeridi ara
        if current_left:
            # Mevcut sol şerit çizgisinin solundaki en yakın şerit çizgisini bul
            for lane in reversed(all_lanes):  # Soldan başlayarak ara
                if lane['center_x'] < current_left['center_x'] - typical_lane_width * 0.3:
                    # Sol komşu şeridin merkezi = bu şerit çizgisi + yarım şerit genişliği
                    left_adjacent_center = lane['center_x'] + typical_lane_width * 0.5
                    # Normalize et (-1 ile 1 arasında)
                    left_adjacent_center = (left_adjacent_center - image_center) / (width / 2)
                    left_adjacent_center = float(np.clip(left_adjacent_center, -1.0, 1.0))
                    break
        
        # Sağ komşu şeridi ara
        if current_right:
            # Mevcut sağ şerit çizgisinin sağındaki en yakın şerit çizgisini bul
            for lane in all_lanes:  # Soldan sağa ara
                if lane['center_x'] > current_right['center_x'] + typical_lane_width * 0.3:
                    # Sağ komşu şeridin merkezi = bu şerit çizgisi - yarım şerit genişliği
                    right_adjacent_center = lane['center_x'] - typical_lane_width * 0.5
                    # Normalize et (-1 ile 1 arasında)
                    right_adjacent_center = (right_adjacent_center - image_center) / (width / 2)
                    right_adjacent_center = float(np.clip(right_adjacent_center, -1.0, 1.0))
                    break
        
        # Geçmiş değerlerle yumuşatma
        self.left_adjacent_history.append(left_adjacent_center)
        self.right_adjacent_history.append(right_adjacent_center)
        
        if len(self.left_adjacent_history) > self.adjacent_history_size:
            self.left_adjacent_history.pop(0)
        if len(self.right_adjacent_history) > self.adjacent_history_size:
            self.right_adjacent_history.pop(0)
        
        # Stabil değerleri hesapla
        stable_left = None
        stable_right = None
        
        # Sol komşu için stabil değer
        valid_left_values = [x for x in self.left_adjacent_history if x is not None]
        if len(valid_left_values) >= 2:
            stable_left = float(np.mean(valid_left_values))
            
        # Sağ komşu için stabil değer
        valid_right_values = [x for x in self.right_adjacent_history if x is not None]
        if len(valid_right_values) >= 2:
            stable_right = float(np.mean(valid_right_values))
        
        return [stable_left, stable_right]

    def compute_lateral_deviation(self, lane_mask):
        """
        Geliştirilmiş lateral deviation hesaplama
        """
        height, width = lane_mask.shape
        image_center = width / 2
        
        # Şerit çizgilerini bul
        left_line, right_line = self.find_lane_lines(lane_mask)
        
        lane_center = None
        
        if left_line is not None and right_line is not None:
            # Her iki şerit de varsa, ortalarını hesapla
            left_M = cv2.moments(left_line)
            right_M = cv2.moments(right_line)
            
            if left_M["m00"] != 0 and right_M["m00"] != 0:
                left_cx = left_M["m10"] / left_M["m00"]
                right_cx = right_M["m10"] / right_M["m00"]
                lane_center = (left_cx + right_cx) / 2
                
        elif left_line is not None:
            # Sadece sol şerit varsa, tahmini sağ şeridi hesapla
            left_M = cv2.moments(left_line)
            if left_M["m00"] != 0:
                left_cx = left_M["m10"] / left_M["m00"]
                estimated_lane_width = width * 0.4
                estimated_right_cx = left_cx + estimated_lane_width
                lane_center = (left_cx + estimated_right_cx) / 2
                
        elif right_line is not None:
            # Sadece sağ şerit varsa, tahmini sol şeridi hesapla
            right_M = cv2.moments(right_line)
            if right_M["m00"] != 0:
                right_cx = right_M["m10"] / right_M["m00"]
                estimated_lane_width = width * 0.4
                estimated_left_cx = right_cx - estimated_lane_width
                lane_center = (estimated_left_cx + right_cx) / 2
        
        # Eğer hiçbir şerit bulunamazsa
        if lane_center is None:
            bottom_quarter = lane_mask[4*height//5:, :]
            indices = np.column_stack(np.where(bottom_quarter > 0))
            
            if indices.size == 0:
                if len(self.deviation_history) > 0:
                    return self.deviation_history[-1]
                return 0.0
            
            lane_center = np.mean(indices[:, 1])
        
        # Lateral deviation hesapla
        deviation = (lane_center - image_center) / (width / 2)
        deviation = float(np.clip(deviation, -1.0, 1.0))
        
        # Geçmiş değerlerle yumuşat
        self.deviation_history.append(deviation)
        if len(self.deviation_history) > self.max_history_size:
            self.deviation_history.pop(0)
            
        if len(self.deviation_history) >= 3:
            weights = np.array([0.2, 0.3, 0.5])
            smooth_deviation = np.average(self.deviation_history[-3:], weights=weights)
        else:
            smooth_deviation = deviation
            
        return float(smooth_deviation)

    def detect_brightness_level(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        return brightness

    def detect_horizontal_lines(self, lane_mask):
        height, width = lane_mask.shape

        roi_start = int(0.5 * height)
        roi_end = int(0.85 * height)
        roi = lane_mask[roi_start:roi_end, :]

        blurred = cv2.GaussianBlur(roi, (5,5), 0)
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                                minLineLength=width // 4, maxLineGap=20)

        has_horizontal = False
        strongest = 0

        if lines is not None:
            horizontal_lines = []
            for line in lines:
                x1,y1,x2,y2 = line[0]
                angle = np.arctan2((y2 - y1), (x2 - x1)) * 180 / np.pi
                if abs(angle) < 15:
                    horizontal_lines.append(line)

            strongest = len(horizontal_lines)
            has_horizontal = strongest > 0

        return has_horizontal, strongest

    def detect_intersection_direction(self, lane_mask, original_image):
        height, width = lane_mask.shape

        brightness = self.detect_brightness_level(original_image)
        has_horizontal_line, horizontal_strength = self.detect_horizontal_lines(lane_mask)

        if not has_horizontal_line:
            intersection_flags = 0
            self.stable_intersection_count = 0
        else:
            left_band = lane_mask[:, :width // 3]
            right_band = lane_mask[:, -width // 3:]

            left_sum = np.sum(left_band)
            right_sum = np.sum(right_band)

            direction_threshold = 1.3

            if left_sum > right_sum * direction_threshold:
                intersection_flags = 1
            elif right_sum > left_sum * direction_threshold:
                intersection_flags = 2
            else:
                intersection_flags = 4

        self.intersection_history.append(intersection_flags)
        if len(self.intersection_history) > self.intersection_history_size:
            self.intersection_history.pop(0)

        if len(self.intersection_history) >= self.min_stable_frames:
            if all(flag == intersection_flags for flag in self.intersection_history[-self.min_stable_frames:]):
                self.stable_intersection_count += 1
            else:
                self.stable_intersection_count = 0

        if self.stable_intersection_count >= self.min_stable_frames:
            final_direction = intersection_flags
        else:
            final_direction = 0

        return final_direction

    def visualize_adjacent_lanes(self, image, adjacent_centers):
        """
        Sol ve sağ komşu şerit merkezlerini görselleştir
        """
        height, width = image.shape[:2]
        image_center = width // 2
        
        labels = ['Left Adj', 'Right Adj']
        colors = [(0, 255, 255), (255, 0, 255)]  # Sol: Sarı, Sağ: Magenta
        
        for i, (center, label, color) in enumerate(zip(adjacent_centers, labels, colors)):
            if center is not None:
                pixel_x = int(image_center + center * (width / 2))
                cv2.line(image, (pixel_x, 0), (pixel_x, height), color, 2)
                cv2.putText(image, label, (pixel_x + 5, 30 + i*25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def image_callback(self, msg):
        """
        ROS Image topic callback'i
        """
        try:
            # ROS Image mesajını OpenCV formatına çevir
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
            
            # Frame işleme
            processed_frame = self.process_frame(frame)
            
            # İşlenmiş frame'i ROS topic'inde yayınla
            self.publisher.publish(self.br.cv2_to_imgmsg(processed_frame, encoding="bgr8"))
            
        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')

    def process_frame(self, frame):
        """
        Frame işleme ana fonksiyonu
        """
        try:
            img = self.preprocess_image(frame)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()
            img /= 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            with torch.no_grad():
                [pred, anchor_grid], seg, ll = self.model(img)

            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)
            ll_seg_mask = cv2.resize(ll_seg_mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)

            result_frame = frame.copy()
            show_seg_result(result_frame, (da_seg_mask, ll_seg_mask), is_demo=True)

            # Lateral deviation hesaplama
            lateral_deviation = self.compute_lateral_deviation(ll_seg_mask)
            self.lateral_pub.publish(Float32(data=lateral_deviation))

            # Sol ve sağ komşu şerit merkezlerini tespit et
            adjacent_centers = self.detect_adjacent_lane_centers(ll_seg_mask)
            
            # Sol komşu şeridi yayınla
            if adjacent_centers[0] is not None:
                self.left_adjacent_pub.publish(Float32(data=adjacent_centers[0]))
            
            # Sağ komşu şeridi yayınla
            if adjacent_centers[1] is not None:
                self.right_adjacent_pub.publish(Float32(data=adjacent_centers[1]))

            intersection_direction = self.detect_intersection_direction(ll_seg_mask, frame)
            self.intersection_pub.publish(Int32(data=intersection_direction))

            # Debug bilgilerini göster
            left_adj = f"{adjacent_centers[0]:.3f}" if adjacent_centers[0] is not None else "None"
            right_adj = f"{adjacent_centers[1]:.3f}" if adjacent_centers[1] is not None else "None"
            
            debug_text = f'Frame: {self.frame_count} | LatDev: {lateral_deviation:.3f} | Int: {intersection_direction} | L:{left_adj} R:{right_adj}'
            cv2.putText(result_frame, debug_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Kontrol bilgileri
            control_text = 'Q: Çıkış | ROS Image Topic Mode'
            cv2.putText(result_frame, control_text, (10, result_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Şerit merkezi gösterimi
            height, width = ll_seg_mask.shape
            image_center = width // 2
            cv2.line(result_frame, (image_center, 0), (image_center, height), (255, 0, 0), 2)
            
            # Komşu şerit merkezlerini görselleştir
            self.visualize_adjacent_lanes(result_frame, adjacent_centers)

            # Görselleştirme penceresi
            cv2.imshow('Lane Detection - ROS Image Topic', result_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('👋 Kullanıcı tarafından sonlandırıldı.')
                rclpy.shutdown()
                cv2.destroyAllWindows()

            return result_frame

        except Exception as e:
            self.get_logger().error(f'Frame processing error: {str(e)}')
            return frame

def main(args=None):
    rclpy.init(args=args)

    try:
        node = LaneDetectionNode()
        node.get_logger().info('📡 ROS image_raw topic modunda çalışıyor')
        node.get_logger().info('🎯 Topic: /zed2i_rgb/image_raw')

        rclpy.spin(node)

    except KeyboardInterrupt:
        print('\n👋 Programdan çıkılıyor...')
    except Exception as e:
        print(f'❌ Hata: {e}')
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()