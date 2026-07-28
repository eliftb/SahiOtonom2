#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

from utils.utils import (
    time_synchronized, select_device,
    driving_area_mask, lane_line_mask, show_seg_result
)

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.br = CvBridge()

        self.weights = '/home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/serit-tespit/models/tusimple_18.pt'
        self.device = select_device('0')
        self.half = self.device.type != 'cpu'
        self.img_size = 640

        self.subscription = self.create_subscription(
            Image, '/zed2i_rgb/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, 'lane_detection_output', 10)
        self.lateral_pub = self.create_publisher(Float32, '/lane/lateral_deviation', 10)
        self.intersection_pub = self.create_publisher(Int32, '/lane/intersection_direction', 10)
        self.control_pub = self.create_publisher(Float32, '/lane/pid_control_signal', 10)  # Yeni publisher (isteğe bağlı)

        self.previous_deviation = 0.0
        self.deviation_history = []
        self.max_history_size = 5

        # PID parametreleri - kendine göre ayarla
        self.pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.1, dt=0.05)

        # Kavşak için parametreler
        self.intersection_history = []
        self.intersection_history_size = 10
        self.stable_intersection_count = 0
        self.min_stable_frames = 6
        
        self.horizontal_line_positions = []
        self.max_line_history = 5

        self.load_model()
        self.get_logger().info('🚦 Lane Detection Node başlatıldı.')

    def load_model(self):
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

    def compute_lateral_deviation(self, lane_mask):
        height, width = lane_mask.shape
        center_x = width / 2

        bottom_quarter = lane_mask[3*height//4:, :]
        indices = np.column_stack(np.where(bottom_quarter > 0))

        if indices.size == 0:
            if len(self.deviation_history) > 0:
                return self.deviation_history[-1]
            return 0.0

        lane_center = np.mean(indices[:, 1])
        deviation = (lane_center - center_x) / (width / 2)
        deviation = float(np.clip(deviation, -1.0, 1.0))
        self.deviation_history.append(deviation)
        if len(self.deviation_history) > self.max_history_size:
            self.deviation_history.pop(0)
        if len(self.deviation_history) >= 3:
            smooth_deviation = np.mean(self.deviation_history[-3:])
        else:
            smooth_deviation = deviation
        return float(smooth_deviation)

    def detect_brightness_level(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        return brightness

    def detect_horizontal_lines(self, lane_mask):
        # (Bu kısımda değişiklik yapmadım, istersen çıkarabilirsin)
        height, width = lane_mask.shape

        roi_start = int(0.75 * height)
        roi_end = int(0.95 * height)
        roi = lane_mask[roi_start:roi_end, :]

        blurred = cv2.GaussianBlur(roi, (5,5), 0)
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)

        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi/180,
            threshold=30,
            minLineLength=width // 6,
            maxLineGap=20
        )

        has_horizontal = False
        strongest = 0
        min_y_position = float('inf')

        if lines is not None:
            horizontal_lines = []
            for line in lines:
                x1,y1,x2,y2 = line[0]
                angle = np.arctan2((y2 - y1), (x2 - x1)) * 180 / np.pi
                if abs(angle) < 10:
                    horizontal_lines.append(line)
                    avg_y = (y1 + y2) / 2
                    min_y_position = min(min_y_position, avg_y)
                    cv2.line(roi, (x1,y1), (x2,y2), (255,255,255), 2)

            strongest = len(horizontal_lines)

            if strongest >= 2:
                roi_height = roi_end - roi_start
                if min_y_position > roi_height * 0.5:
                    has_horizontal = True
                    self.horizontal_line_positions.append(min_y_position)
                    if len(self.horizontal_line_positions) > self.max_line_history:
                        self.horizontal_line_positions.pop(0)

        cv2.imshow('ROI Edges', edges)
        return has_horizontal, strongest

    def is_intersection_approaching(self):
        if len(self.horizontal_line_positions) < 3:
            return False
        recent_positions = self.horizontal_line_positions[-3:]
        is_approaching = all(recent_positions[i] <= recent_positions[i+1] for i in range(len(recent_positions)-1))
        return is_approaching

    def detect_intersection_direction(self, lane_mask, original_image):
        height, width = lane_mask.shape
        brightness = self.detect_brightness_level(original_image)
        has_horizontal_line, horizontal_strength = self.detect_horizontal_lines(lane_mask)

        if not has_horizontal_line:
            intersection_flags = 0
            self.stable_intersection_count = 0
        else:
            if not self.is_intersection_approaching():
                intersection_flags = 0
                self.stable_intersection_count = 0
            else:
                left_band = lane_mask[:, :width // 2]
                right_band = lane_mask[:, width // 2:]
                left_sum = np.sum(left_band)
                right_sum = np.sum(right_band)
                direction_threshold = 1.5
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
            last_frames = self.intersection_history[-self.min_stable_frames:]
            most_common = max(set(last_frames), key=last_frames.count)
            if last_frames.count(most_common) >= int(self.min_stable_frames * 0.8):
                self.stable_intersection_count += 1
                final_direction = most_common
            else:
                self.stable_intersection_count = 0
                final_direction = 0
        else:
            final_direction = 0

        if self.stable_intersection_count < 3:
            final_direction = 0

        self.get_logger().info(
            f'Brightness: {brightness:.1f} | Horizontal: {has_horizontal_line} ({horizontal_strength}) | '
            f'Direction: {final_direction} | Stable: {self.stable_intersection_count} | '
            f'Approaching: {self.is_intersection_approaching()}'
        )
        return final_direction

    def image_callback(self, msg):
        try:
            im0s = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = self.preprocess_image(im0s)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()
            img /= 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            with torch.no_grad():
                [pred, anchor_grid], seg, ll = self.model(img)

            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)
            ll_seg_mask = cv2.resize(ll_seg_mask, (im0s.shape[1], im0s.shape[0]), interpolation=cv2.INTER_NEAREST)

            im0 = im0s.copy()
            show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)

            lateral_deviation = self.compute_lateral_deviation(ll_seg_mask)
            self.lateral_pub.publish(Float32(data=lateral_deviation))

            # PID kontrol sinyali üret
            control_signal = self.pid.update(lateral_deviation)
            self.control_pub.publish(Float32(data=control_signal))

            intersection_direction = self.detect_intersection_direction(ll_seg_mask, im0s)
            self.intersection_pub.publish(Int32(data=intersection_direction))

            debug_text = f'LatDev: {lateral_deviation:.2f} | PID: {control_signal:.2f} | Int: {intersection_direction}'
            cv2.putText(im0, debug_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            cv2.imshow('LANE NODE', im0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
                cv2.destroyAllWindows()
                return

            self.publisher.publish(self.br.cv2_to_imgmsg(im0, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
