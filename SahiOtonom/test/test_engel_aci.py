#!/usr/bin/env python3
"""engel-tespit.py aci mantigi testi - donanim gerektirmez.

Gercek RPLIDAR S2 geometrisi kullanilir: N=3240, angle_min=-pi, inc=2pi/3239.
Bilinen acilara engel yerlestirilip dogru sektorde gorulup gorulmedigi olculur.
"""
import importlib.util
import math
import os
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan

# YOL TESTIN KENDI KONUMUNDAN TURETILIYOR. Burada paketi hazirlayanin
# makinesine ait sabit bir yol yaziliydi (/media/elifnur/...) ve test baska
# hicbir bilgisayarda calismiyordu - FileNotFoundError ile duruyordu.
SRC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'EngelTespit',
    'engel-tespit.py'))

spec = importlib.util.spec_from_file_location("engel_tespit", SRC)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

# --- Gercek S2 tarama geometrisi ---
N = 3240
ANGLE_MIN = -math.pi
ANGLE_MAX = math.pi
INC = (ANGLE_MAX - ANGLE_MIN) / (N - 1)


def scan_with_obstacle_at(deg, dist=1.0, background=25.0):
    """Verilen acida tek bir yakin engel olan tarama uretir."""
    m = LaserScan()
    m.angle_min = ANGLE_MIN
    m.angle_max = ANGLE_MAX
    m.angle_increment = INC
    m.range_min = 0.05
    m.range_max = 30.0
    r = np.full(N, background, dtype=float)
    idx = int(round((math.radians(deg) - ANGLE_MIN) / INC)) % N
    r[idx] = dist                      # engelin tam merkezi
    m.ranges = r.tolist()
    return m


def run_case(forward_deg, obstacle_deg, beklenen, esik=5.0):
    rclpy.init()
    node = mod.LidarObstacleDetector()
    # Parametreleri test senaryosuna gore ayarla
    node.FORWARD_ANGLE_DEG = forward_deg
    node.SECTOR_WIDTH_DEG = 30.0
    node.OBSTACLE_THRESHOLD = esik
    node.sector_logged = True

    sonuc = {}
    node.publish_obstacle_status = lambda d, dist: sonuc.update(detected=d, dist=dist)
    node.scan_callback(scan_with_obstacle_at(obstacle_deg))
    node.destroy_node()
    rclpy.shutdown()

    bulundu = bool(sonuc.get("detected"))
    mesafe = sonuc.get("dist")
    ok = bulundu == beklenen
    durum = "GECTI" if ok else "KALDI"
    mstr = f"{mesafe:.2f} m" if bulundu else "-"
    print(f"  [{durum}] ileri={forward_deg:+6.1f}°  engel={obstacle_deg:+7.1f}°  "
          f"-> tespit={str(bulundu):5s} ({mstr})  beklenen={beklenen}")
    return ok


print("engel-tespit.py aci mantigi testi (RPLIDAR S2 geometrisi, N=3240)")
print("-" * 78)

testler = [
    # (ileri yon, engel acisi, beklenen)
    (0.0,    0.0,   True),    # onde engel -> gorulmeli
    (0.0,   10.0,   True),    # koni icinde (+-15) -> gorulmeli
    (0.0,  -10.0,   True),    # koninin diger yarisi -> gorulmeli
    (0.0,   40.0,   False),   # koni disinda -> gorulmemeli
    (0.0,  180.0,   False),   # ARKADA -> gorulmemeli (eski hata buydu)
    (0.0,  -90.0,   False),   # yanda -> gorulmemeli
    # --- sarma (wrap-around) testleri: ileri yon 180 secilirse koni
    #     dizinin sonu ile basi arasinda bolunur ---
    (180.0, 180.0,  True),    # tam arkada
    (180.0, 179.0,  True),    # dizinin sonu tarafi
    (180.0, -179.0, True),    # dizinin basi tarafi <- ESKI KOD BUNU KACIRIYORDU
    (180.0,   0.0,  False),   # onde -> gorulmemeli
    # --- yana montaj ---
    (90.0,   90.0,  True),
    (90.0,    0.0,  False),
]

sonuclar = [run_case(*t) for t in testler]

print("-" * 78)
gecen, toplam = sum(sonuclar), len(sonuclar)
print(f"  {gecen}/{toplam} test gecti")
sys.exit(0 if gecen == toplam else 1)
