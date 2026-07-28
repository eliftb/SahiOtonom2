import cv2
import numpy as np
import os
from collections import deque

class LaneDetector:
    def __init__(self, adjacent_history_size=5):
        self.adjacent_history_size = adjacent_history_size
        self.left_adjacent_history = deque(maxlen=adjacent_history_size)
        self.right_adjacent_history = deque(maxlen=adjacent_history_size)
        
    def get_logger(self):
        """Basit logger simülasyonu"""
        class Logger:
            def debug(self, msg):
                print(f"DEBUG: {msg}")
        return Logger()
    
    def preprocess_image(self, image):
        """Görüntüyü şerit tespiti için ön işle"""
        # Gri tonlamaya çevir
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Gürültüyü azalt
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Kenar tespiti
        edges = cv2.Canny(blurred, 50, 150)
        
        # ROI (Region of Interest) uygula - sadece alt yarıya odaklan
        height, width = edges.shape
        roi_vertices = np.array([
            [(0, height),
             (0, int(height * 0.6)),
             (width, int(height * 0.6)),
             (width, height)]
        ], dtype=np.int32)
        
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        return masked_edges
    
    def find_all_lane_lines(self, lane_mask):
        """Basit şerit çizgisi tespiti (orijinal kod için uyumluluk)"""
        lines = self.find_all_lane_lines_improved(lane_mask)
        # Format dönüşümü
        return [{'center_x': line['x_position']} for line in lines]
    
    def find_all_lane_lines_improved(self, lane_mask):
        """Geliştirilmiş şerit çizgi tespiti - daha stabil sonuçlar için"""
        height, width = lane_mask.shape
        
        # Alt yarıda odaklan (daha stabil)
        roi_height_start = int(height * 0.6)
        roi = lane_mask[roi_height_start:, :]
        
        # Morfolojik işlemlerle gürültüyü temizle
        kernel = np.ones((5, 5), np.uint8)
        roi_cleaned = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)
        roi_cleaned = cv2.morphologyEx(roi_cleaned, cv2.MORPH_OPEN, kernel)
        
        # Dikey çizgileri güçlendir
        vertical_kernel = np.ones((7, 1), np.uint8)
        roi_cleaned = cv2.morphologyEx(roi_cleaned, cv2.MORPH_CLOSE, vertical_kernel)
        
        # Konturları bul
        contours, _ = cv2.findContours(roi_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return []
        
        valid_lines = []
        min_contour_area = 150
        min_height = 20
        
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            
            # Dikey çizgileri filtrele
            if area > min_contour_area and h > min_height and h > w * 1.5:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + roi_height_start
                    
                    # Çizginin üst ve alt noktalarını bul
                    contour_points = contour.reshape(-1, 2)
                    top_points = contour_points[contour_points[:, 1] == np.min(contour_points[:, 1])]
                    bottom_points = contour_points[contour_points[:, 1] == np.max(contour_points[:, 1])]
                    
                    if len(top_points) > 0 and len(bottom_points) > 0:
                        top_x = np.mean(top_points[:, 0])
                        bottom_x = np.mean(bottom_points[:, 0])
                        avg_x = int((top_x + bottom_x) / 2)
                        
                        valid_lines.append({
                            'x_position': avg_x,
                            'confidence': area / (width * height) * 100,
                            'height': h,
                            'width': w
                        })
        
        # X koordinatına göre sırala
        valid_lines = sorted(valid_lines, key=lambda x: x['x_position'])
        
        # Çok yakın çizgileri birleştir
        merged_lines = []
        merge_threshold = width * 0.05
        
        for line in valid_lines:
            if not merged_lines or abs(line['x_position'] - merged_lines[-1]['x_position']) > merge_threshold:
                merged_lines.append(line)
            else:
                if line['confidence'] > merged_lines[-1]['confidence']:
                    merged_lines[-1] = line
        
        return merged_lines
    
    def calculate_lane_structure(self, lane_lines, image_width):
        """Şerit yapısını analiz et ve komşu şeritleri belirle"""
        image_center = image_width / 2
        
        if len(lane_lines) < 2:
            return None, None, None, None
        
        # Merkeze en yakın sol ve sağ çizgileri bul
        left_boundary = None
        right_boundary = None
        left_idx = -1
        right_idx = -1
        
        for i, line in enumerate(lane_lines):
            if line['x_position'] < image_center:
                left_boundary = line
                left_idx = i
            elif line['x_position'] > image_center and right_boundary is None:
                right_boundary = line
                right_idx = i
                break
        
        # Sol komşu şerit çizgisi
        left_adjacent_line = None
        if left_idx > 0:
            left_adjacent_line = lane_lines[left_idx - 1]
        
        # Sağ komşu şerit çizgisi
        right_adjacent_line = None
        if right_idx != -1 and right_idx < len(lane_lines) - 1:
            right_adjacent_line = lane_lines[right_idx + 1]
        
        return left_boundary, right_boundary, left_adjacent_line, right_adjacent_line
    
    def detect_adjacent_lane_centers(self, lane_mask):
        """Sol ve sağ komşu şeritlerin merkezlerini tespit eder"""
        height, width = lane_mask.shape
        image_center = width / 2
        
        all_lanes = self.find_all_lane_lines(lane_mask)
        
        if len(all_lanes) < 2:
            return [None, None]
        
        # Tüm şerit çizgilerini x koordinatına göre sırala
        lane_positions = [lane['center_x'] for lane in all_lanes]
        
        # Mevcut şeridin sol ve sağ sınırlarını bul
        current_left_line = None
        current_right_line = None
        current_left_idx = -1
        current_right_idx = -1
        
        for i, pos in enumerate(lane_positions):
            if pos < image_center:
                current_left_line = pos
                current_left_idx = i
            elif pos > image_center and current_right_line is None:
                current_right_line = pos
                current_right_idx = i
                break
        
        left_adjacent_center = None
        right_adjacent_center = None
        
        # Sol komşu şerit merkezi
        if current_left_idx > 0:
            left_neighbor_line = lane_positions[current_left_idx - 1]
            left_adjacent_center = (left_neighbor_line + current_left_line) / 2
            left_adjacent_center = (left_adjacent_center - image_center) / (width / 2)
            left_adjacent_center = float(np.clip(left_adjacent_center, -1.0, 1.0))
        
        # Sağ komşu şerit merkezi
        if current_right_idx != -1 and current_right_idx < len(lane_positions) - 1:
            right_neighbor_line = lane_positions[current_right_idx + 1]
            right_adjacent_center = (current_right_line + right_neighbor_line) / 2
            right_adjacent_center = (right_adjacent_center - image_center) / (width / 2)
            right_adjacent_center = float(np.clip(right_adjacent_center, -1.0, 1.0))
        
        # Geçmiş değerlerle yumuşatma
        self.left_adjacent_history.append(left_adjacent_center)
        self.right_adjacent_history.append(right_adjacent_center)
        
        # Stabil değerleri hesapla
        stable_left = None
        stable_right = None
        
        valid_left_values = [x for x in self.left_adjacent_history if x is not None]
        if len(valid_left_values) >= 2:
            stable_left = float(np.mean(valid_left_values))
        
        valid_right_values = [x for x in self.right_adjacent_history if x is not None]
        if len(valid_right_values) >= 2:
            stable_right = float(np.mean(valid_right_values))
        
        return [stable_left, stable_right]
    
    def detect_adjacent_lane_centers_v2(self, lane_mask):
        """Geliştirilmiş komşu şerit merkez tespiti"""
        height, width = lane_mask.shape
        image_center = width / 2
        
        # Geliştirilmiş çizgi tespiti kullan
        lane_lines = self.find_all_lane_lines_improved(lane_mask)
        
        if len(lane_lines) < 2:
            return [None, None]
        
        # Şerit yapısını analiz et
        left_bound, right_bound, left_adj_line, right_adj_line = self.calculate_lane_structure(lane_lines, width)
        
        left_adjacent_center = None
        right_adjacent_center = None
        
        # Sol komşu şerit merkezi
        if left_bound and left_adj_line:
            left_adjacent_center = (left_adj_line['x_position'] + left_bound['x_position']) / 2
            left_adjacent_center = (left_adjacent_center - image_center) / (width / 2)
            left_adjacent_center = float(np.clip(left_adjacent_center, -1.0, 1.0))
            
            self.get_logger().debug(f"Sol komşu: Çizgi1={left_adj_line['x_position']}, Çizgi2={left_bound['x_position']}, Merkez={left_adjacent_center:.3f}")
        
        # Sağ komşu şerit merkezi
        if right_bound and right_adj_line:
            right_adjacent_center = (right_bound['x_position'] + right_adj_line['x_position']) / 2
            right_adjacent_center = (right_adjacent_center - image_center) / (width / 2)
            right_adjacent_center = float(np.clip(right_adjacent_center, -1.0, 1.0))
            
            self.get_logger().debug(f"Sağ komşu: Çizgi1={right_bound['x_position']}, Çizgi2={right_adj_line['x_position']}, Merkez={right_adjacent_center:.3f}")
        
        # Geçmiş değerlerle yumuşatma
        self.left_adjacent_history.append(left_adjacent_center)
        self.right_adjacent_history.append(right_adjacent_center)
        
        # Stabil değerleri hesapla
        stable_left = None
        stable_right = None
        
        valid_left_values = [x for x in self.left_adjacent_history if x is not None]
        if len(valid_left_values) >= 2:
            stable_left = float(np.mean(valid_left_values))
        
        valid_right_values = [x for x in self.right_adjacent_history if x is not None]
        if len(valid_right_values) >= 2:
            stable_right = float(np.mean(valid_right_values))
        
        return [stable_left, stable_right]
    
    def calculate_lateral_deviation(self, lane_mask):
        """Mevcut şeritteki yanal sapma hesapla"""
        height, width = lane_mask.shape
        image_center = width / 2
        
        all_lanes = self.find_all_lane_lines(lane_mask)
        
        if len(all_lanes) < 2:
            return 0.0
        
        lane_positions = [lane['center_x'] for lane in all_lanes]
        
        # Merkeze en yakın sol ve sağ çizgileri bul
        left_line = None
        right_line = None
        
        for pos in lane_positions:
            if pos < image_center:
                left_line = pos
            elif pos > image_center and right_line is None:
                right_line = pos
                break
        
        if left_line is not None and right_line is not None:
            # Şerit merkezi
            lane_center = (left_line + right_line) / 2
            # Yanal sapma (-1 ile 1 arasında normalize edilmiş)
            lateral_deviation = (lane_center - image_center) / (width / 2)
            return float(np.clip(lateral_deviation, -1.0, 1.0))
        
        return 0.0
    
    def visualize_all_lanes(self, image, lane_mask, lateral_deviation, adjacent_centers):
        """Tüm şeritleri ve merkezlerini görselleştir"""
        height, width = image.shape[:2]
        image_center = width // 2
        
        # Tüm şerit çizgilerini göster
        all_lanes = self.find_all_lane_lines(lane_mask)
        
        # Şerit çizgilerini yeşil ile işaretle
        for i, lane in enumerate(all_lanes):
            x = lane['center_x']
            cv2.line(image, (x, 0), (x, height), (0, 255, 0), 1)
            cv2.putText(image, f"L{i}", (x-10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # Görüntü merkezini mavi ile göster
        cv2.line(image, (image_center, 0), (image_center, height), (255, 0, 0), 2)
        cv2.putText(image, "CENTER", (image_center-30, height-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Mevcut şerit merkezini kırmızı ile göster
        current_lane_x = int(image_center + lateral_deviation * (width / 2))
        cv2.line(image, (current_lane_x, 0), (current_lane_x, height), (0, 0, 255), 2)
        cv2.putText(image, "CURRENT", (current_lane_x-30, height-30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Sol komşu şerit merkezini sarı ile göster
        if adjacent_centers[0] is not None:
            left_x = int(image_center + adjacent_centers[0] * (width / 2))
            cv2.line(image, (left_x, 0), (left_x, height), (0, 255, 255), 2)
            cv2.circle(image, (left_x, height//2), 5, (0, 255, 255), -1)
            cv2.putText(image, f"LEFT ADJ: {adjacent_centers[0]:.2f}", 
                       (left_x-40, height//2-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Sağ komşu şerit merkezini magenta ile göster
        if adjacent_centers[1] is not None:
            right_x = int(image_center + adjacent_centers[1] * (width / 2))
            cv2.line(image, (right_x, 0), (right_x, height), (255, 0, 255), 2)
            cv2.circle(image, (right_x, height//2), 5, (255, 0, 255), -1)
            cv2.putText(image, f"RIGHT ADJ: {adjacent_centers[1]:.2f}", 
                       (right_x-40, height//2+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)


def main():
    """Ana fonksiyon - video dosyası veya kameradan şerit tespiti"""
    
    # Lane detector oluştur
    detector = LaneDetector(adjacent_history_size=5)
    
    # Video kaynağını seç (0: kamera, dosya yolu: video dosyası)
    video_source = 0  # Kamera için
    # video_source = "test_video.mp4"  # Video dosyası için
    
    cap = cv2.VideoCapture(video_source)
    
    if not cap.isOpened():
        print("Hata: Video kaynağı açılamadı!")
        return
    
    print("Şerit tespiti başlatıldı. Çıkmak için 'q' tuşuna basın.")
    print("Görselleştirme:")
    print("- Yeşil çizgiler: Tespit edilen şerit çizgileri")
    print("- Mavi çizgi: Görüntü merkezi")
    print("- Kırmızı çizgi: Mevcut şerit merkezi")
    print("- Sarı çizgi: Sol komşu şerit merkezi")
    print("- Magenta çizgi: Sağ komşu şerit merkezi")
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Video akışı sonlandı veya frame okunamadı.")
            break
        
        frame_count += 1
        
        # Orijinal frame'i kopyala (görselleştirme için)
        display_frame = frame.copy()
        
        # Görüntüyü ön işle
        lane_mask = detector.preprocess_image(frame)
        
        # Yanal sapmayı hesapla
        lateral_deviation = detector.calculate_lateral_deviation(lane_mask)
        
        # Komşu şerit merkezlerini tespit et (geliştirilmiş versiyon)
        adjacent_centers = detector.detect_adjacent_lane_centers_v2(lane_mask)
        
        # Sonuçları görselleştir
        detector.visualize_all_lanes(display_frame, lane_mask, lateral_deviation, adjacent_centers)
        
        # Bilgi metni ekle
        info_text = [
            f"Frame: {frame_count}",
            f"Lateral Deviation: {lateral_deviation:.3f}",
            f"Left Adjacent: {adjacent_centers[0]:.3f if adjacent_centers[0] else 'None'}",
            f"Right Adjacent: {adjacent_centers[1]:.3f if adjacent_centers[1] else 'None'}"
        ]
        
        for i, text in enumerate(info_text):
            cv2.putText(display_frame, text, (10, 30 + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Görüntüleri göster
        cv2.imshow('Lane Detection - Original', display_frame)
        cv2.imshow('Lane Detection - Processed', lane_mask)
        
        # Çıkış kontrolü
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Screenshot kaydet
            cv2.imwrite(f'lane_detection_frame_{frame_count}.jpg', display_frame)
            print(f"Frame {frame_count} kaydedildi.")
    
    # Temizlik
    cap.release()
    cv2.destroyAllWindows()
    print("Şerit tespiti sonlandırıldı.")


if __name__ == "__main__":
    print("Şerit Tespit Sistemi")
    print("1. Kamera/Video ile test için main() fonksiyonu çalışacak")
    print("2. Test görüntüsü ile denemek için test_with_sample_image() fonksiyonunu çağırın")
    print()
    
    # Ana fonksiyonu çalıştır
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram kullanıcı tarafından sonlandırıldı.")
    except Exception as e:
        print(f"Hata oluştu: {e}")
        print("Test görüntüsü ile deneniyor...")