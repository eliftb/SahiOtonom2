#!/usr/bin/env python3
"""Şerit geometrisi regresyon testi - ARAÇ/KAMERA/ROS GEREKTİRMEZ.

serit-tespitcopy.py içindeki compute_lateral_deviation'ı sentetik şerit
maskeleriyle çalıştırır. Pistte bir şey denemeden önce burada koştur:
saniyeler sürer, aracı çıkarmak yarım saat.

    python3 SahiOtonom/test/test_lane_geometry.py

Sentetik maskeler kusursuzdur; bu test "algılama iyi mi"yi değil,
"geometri/mantık bozuldu mu"yu ölçer. Gerçek görüntüde doğrulama yerine geçmez.
"""
import importlib.util
import os
import sys
import types

import numpy as np

H, W = 720, 1280
HORIZON = 0.55 * H


def load_lane_module():
    """ROS/torch bağımlılıklarını sahteleyerek düğüm dosyasını yükler."""
    for name in ['rclpy', 'rclpy.node', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs',
                 'std_msgs.msg', 'cv_bridge', 'torch', 'utils', 'utils.utils',
                 'rcl_interfaces', 'rcl_interfaces.msg']:
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy.node'].Node = type('Node', (), {})
    sys.modules['sensor_msgs.msg'].Image = object
    sys.modules['std_msgs.msg'].Float32 = object
    sys.modules['std_msgs.msg'].Int32 = object
    sys.modules['cv_bridge'].CvBridge = object
    sys.modules['rcl_interfaces.msg'].SetParametersResult = object
    for fn in ['select_device', 'driving_area_mask', 'lane_line_mask', 'show_seg_result']:
        setattr(sys.modules['utils.utils'], fn, lambda *a, **k: None)

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'SeritTespit', 'serit-tespitcopy.py')
    spec = importlib.util.spec_from_file_location('lane_node', os.path.normpath(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mod = load_lane_module()


class FakeClock:
    """Kareler anında koştuğu için dt~0 çıkar; 30 FPS taklit et."""
    def __init__(self, fps=30.0):
        self.t, self.step = 0.0, 1.0 / fps

    def monotonic(self):
        self.t += self.step
        return self.t


def make_node(curve_ff=1.0, offset=0.0, fps=30.0):
    """__init__ çalıştırmadan (ROS gerektirmeden) düğüm örneği kurar."""
    mod.time = FakeClock(fps)
    n = object.__new__(mod.LaneDetectionNode)
    n.camera_center_offset_px = offset
    n.look_ahead_frac = 0.80
    n.deviation_deadband = 0.02
    n.max_deviation_rate = 2.5
    n.curve_feedforward = curve_ff
    n.horizon_frac = 0.55
    n.sample_rows = (0.95, 0.90, 0.85, 0.80, 0.75, 0.70, 0.65)
    n.lane_center_track = None
    n.lane_width_track = [None] * len(n.sample_rows)
    n.lost_frames = 0
    n.lane_slope = None
    n.deviation_history = []
    n.max_history_size = 5
    n.last_update_time = None
    n.debug_rows, n.debug_fit, n.debug_center, n.debug_curve = [], None, None, 0.0
    return n


def lane_mask(curvature=0.0, car_offset=0.0, dashed_left=False, dashed_right=False,
              drop_inner=False, extra_left_lane=False):
    """Perspektifli şerit maskesi.

    curvature : şeridin ufka doğru yanal kayması (px). + = sağa viraj
    car_offset: aracın şerit ortasından kayması (px). + = araç sağda
    """
    mask = np.zeros((H, W), np.uint8)
    for y in range(int(HORIZON) + 8, H):
        t = (H - y) / (H - HORIZON)          # 0 = altta, 1 = ufukta
        half = 380 * (1 - t)                  # perspektifle daralan yarı genişlik
        cx = 640 - car_offset + curvature * t * t
        for side, dashed in ((-1, dashed_left), (+1, dashed_right)):
            if dashed and (y // 45) % 2 == 0:
                continue
            if drop_inner and curvature and side == np.sign(curvature):
                continue                      # iç çizgi kadraj dışı
            x = int(cx + side * half)
            if 0 <= x < W:
                mask[y, max(0, x - 5):x + 5] = 1
        if extra_left_lane:
            x = int(cx - 3 * half)
            if 0 <= x < W:
                mask[y, max(0, x - 5):x + 5] = 1
    return mask


def run(node, mask, frames=12):
    for _ in range(frames):
        deviation = node.compute_lateral_deviation(mask)
    return deviation


# --- TESTLER -----------------------------------------------------------------
# Her giriş: (ad, ölçüm fonksiyonu, alt sınır, üst sınır)

def dashed_out_of_phase():
    """Sol ve sağ kesikli çizgiler farklı fazda - eski kontur yönteminin çöktüğü yer."""
    mask = np.zeros((H, W), np.uint8)
    for y in range(int(HORIZON) + 8, H):
        t = (H - y) / (H - HORIZON)
        half = 380 * (1 - t)
        side = -1 if (y // 45) % 2 == 0 else +1
        x = int(640 + side * half)
        if 0 <= x < W:
            mask[y, max(0, x - 5):x + 5] = 1
    return mask


TESTS = [
    # Düz yol: araç ortadayken sapma SIFIR olmalı
    ('düz, iki sürekli çizgi',
     lambda: run(make_node(), lane_mask()), -0.03, 0.03),
    ('düz, iki kesikli çizgi',
     lambda: run(make_node(), lane_mask(dashed_left=True, dashed_right=True)), -0.03, 0.03),
    ('düz, kesikli çizgiler ters fazda',
     lambda: run(make_node(), dashed_out_of_phase()), -0.03, 0.03),
    ('düz, komşu şerit çizgisi var',
     lambda: run(make_node(), lane_mask(extra_left_lane=True)), -0.03, 0.03),

    # Düz yol: araç yana kaymışsa sapma o yönü göstermeli
    ('düz, araç 100px solda -> sapma +',
     lambda: run(make_node(), lane_mask(car_offset=-100)), 0.10, 0.22),
    ('düz, araç 100px sağda -> sapma -',
     lambda: run(make_node(), lane_mask(car_offset=100)), -0.22, -0.10),

    # Viraj: ileri besleme yeterli direksiyon üretmeli
    ('viraj orta sağa',
     lambda: run(make_node(), lane_mask(curvature=200)), 0.15, 0.45),
    ('viraj keskin sağa',
     lambda: run(make_node(), lane_mask(curvature=350)), 0.30, 0.70),
    ('viraj keskin sola',
     lambda: run(make_node(), lane_mask(curvature=-350)), -0.70, -0.30),
    ('viraj keskin sağa, iç çizgi kadraj dışı',
     lambda: run(make_node(), lane_mask(curvature=350, drop_inner=True)), 0.20, 0.70),
    ('viraj keskin sola, iç çizgi kadraj dışı',
     lambda: run(make_node(), lane_mask(curvature=-350, drop_inner=True)), -0.70, -0.20),

    # Kamera kayması kalibrasyonu
    ('kamera 60px kaymış, offset girilmiş',
     lambda: run(make_node(offset=60.0), lane_mask(car_offset=-60)), -0.03, 0.03),
]


def test_lane_lost_decays():
    """Şerit kaybolunca sapma sıfıra sönmeli (araç dönmeye devam etmemeli)."""
    n = make_node()
    run(n, lane_mask(car_offset=-100), frames=10)
    return run(n, np.zeros((H, W), np.uint8), frames=12)


def test_s_curve():
    """Sağ viraj -> düz -> sol viraj: işaretler doğru, takip kilitlenmemeli."""
    n = make_node()
    results = []
    for curvature in (300, 0, -300, 0):
        results.append(run(n, lane_mask(curvature=curvature), frames=15))
    return results


def test_response_frames():
    """Düzden keskin viraja girişte sapmanın 0.20'ye kaç karede ulaştığı."""
    n = make_node()
    run(n, lane_mask(), frames=10)
    mask = lane_mask(curvature=300)
    for i in range(1, 16):
        if abs(n.compute_lateral_deviation(mask)) >= 0.20:
            return i
    return 99


def main():
    failures = 0
    print(f"{'test':<44}{'sonuç':>9}   {'beklenen aralık':>18}")
    print('-' * 76)
    for name, fn, low, high in TESTS:
        value = fn()
        ok = low <= value <= high
        failures += not ok
        print(f"{name:<44}{value:>+9.3f}   [{low:+.2f}, {high:+.2f}]  {'OK' if ok else '<<< HATA'}")

    lost = test_lane_lost_decays()
    ok = abs(lost) < 0.05
    failures += not ok
    print(f"{'şerit kaybı sonrası sönümleme':<44}{lost:>+9.3f}   {'|x| < 0.05':>18}  {'OK' if ok else '<<< HATA'}")

    s = test_s_curve()
    ok = s[0] > 0.20 and abs(s[1]) < 0.05 and s[2] < -0.20 and abs(s[3]) < 0.05
    failures += not ok
    print(f"{'S-viraj (sağ/düz/sol/düz)':<44}{str([round(v, 2) for v in s]):>9}"
          f"{'işaretler doğru':>28}  {'OK' if ok else '<<< HATA'}")

    frames = test_response_frames()
    ok = frames <= 6
    failures += not ok
    print(f"{'viraja tepki (0.20 sapmaya kaç kare)':<44}{frames:>9}   {'<= 6 kare':>18}  {'OK' if ok else '<<< HATA'}")

    print('-' * 76)
    print('TÜMÜ GEÇTİ' if not failures else f'{failures} TEST BAŞARISIZ')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
