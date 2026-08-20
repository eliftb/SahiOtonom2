#!/usr/bin/env python3
"""LiDAR canlı görselleştirici — üstten görünüm.

RPLIDAR S2 bir 2D LiDAR: dikey eksen etrafında döner, YATAY bir düzlemi
360° tarar. Dikey tarama YOKTUR. Bu pencere o yatay dilimi üstten gösterir.

Ekranda:
  gri noktalar   : tüm 360° taraması
  yeşil noktalar : engel tespitinin baktığı koridorun içi
  kırmızı çizgi  : forward_angle_deg (sistemin "ileri" saydığı yön)
  sarı koni      : taranan sektör (sector_width_deg)
  yeşil şerit    : koridor (corridor_width_m)

KULLANIM (LiDAR sürücüsü çalışırken):
    python3 EngelTespit/lidar_goster.py                # varsayilan 0°
    python3 EngelTespit/lidar_goster.py --forward -90  # aciyi deneyerek bul

--forward degerini degistirip hangi acinin yolu ortaladigini GOZLE gor,
dogru degeri engel-tespit.py'ye yaz.
"""
import argparse
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import matplotlib
# Backend ZORLANMIYOR. matplotlib.use("TkAgg") sabit yazilinca, Tk'nin
# calismadigi ortamlarda (ekransiz oturum, uzak baglanti) import ANINDA
# kilitleniyor ve program hicbir sey yazmadan duruyor. MPLBACKEND ortam
# degiskeni verilmisse ona saygi gosterilir; yoksa Tk denenir, olmazsa
# matplotlib kendi varsayilanina duser.
import os as _os
if not _os.environ.get("MPLBACKEND"):
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Polygon


class Goster(Node):
    def __init__(self, args):
        super().__init__('lidar_goster')
        self.a = args
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, m):
        self.scan = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--forward', type=float, default=0.0,
                    help='ileri yon (derece), engel-tespit.py forward_angle_deg')
    ap.add_argument('--sector', type=float, default=120.0)
    # engel-tespit.py'deki corridor_width_m ile AYNI olmali;
    # ekran gercekte tarananı gostermezse kalibrasyon yaniltir.
    ap.add_argument('--corridor', type=float, default=1.5)
    ap.add_argument('--range', type=float, default=8.0, help='gosterilecek yaricap (m)')
    args = ap.parse_args()

    rclpy.init()
    n = Goster(args)

    # Pencereyi acmadan ONCE tarama var mi diye bak. Onceki surum sessizce
    # bekliyordu; LiDAR takili degilken ekranda hicbir sey olmuyor ve
    # program "acilmadi" gibi gorunuyordu.
    import time as _t
    print(f"forward={args.forward:.0f}°  koni={args.sector:.0f}°  "
          f"koridor={args.corridor:.2f} m", flush=True)
    print("/scan bekleniyor...", flush=True)
    _bas = _t.time()
    while n.scan is None and _t.time() - _bas < 5.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.scan is None:
        print("\n⚠️  5 saniyedir /scan gelmiyor. Pencere yine de acilacak ama BOS olacak.")
        print("   1) LiDAR takili mi?   lsusb | grep 10c4")
        print("   2) Surucu BASKA BIR TERMINALDE calisiyor olmali:")
        print("      source /media/elifnur/Linux_150GB/sahi_deps/setup_sahi_env.sh")
        print("      PORT=$(ls /dev/serial/by-id/*CP2102*)")
        print("      ros2 launch sllidar_ros2 sllidar_s2_launch.py \\")
        print("           serial_port:=$PORT serial_baudrate:=1000000")
        print("\n   Surucuyu baslatinca bu pencere kendiliginden dolacak.\n", flush=True)
    else:
        print("✅ tarama geldi, pencere aciliyor.\n", flush=True)

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("LiDAR - ustten gorunum (arac merkezde)")

    # Tarama gelmeden de PENCEREYI AC. Onceki surum ilk /scan gelene kadar
    # hicbir sey cizmiyordu; LiDAR takili degilken program calisiyor ama
    # ekranda hicbir sey yok - "acilmadi" gibi gorunuyordu.
    import time as _t
    baslangic = _t.time()
    uyarildi = False
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.scan is None:
            gecen = _t.time() - baslangic
            ax.clear()
            ax.text(0.5, 0.58, "TARAMA BEKLENIYOR", ha='center', va='center',
                    fontsize=17, color='darkred', transform=ax.transAxes)
            ax.text(0.5, 0.44, f"/scan topic'inden veri gelmiyor  ({gecen:.0f} sn)",
                    ha='center', va='center', fontsize=10, color='0.35',
                    transform=ax.transAxes)
            ax.text(0.5, 0.30,
                    "LiDAR takili mi?  Surucu baska terminalde calisiyor mu?\n"
                    "ros2 launch sllidar_ros2 sllidar_s2_launch.py \\\n"
                    "    serial_port:=$(ls /dev/serial/by-id/*CP2102*) serial_baudrate:=1000000",
                    ha='center', va='center', fontsize=8, color='0.45',
                    family='monospace', transform=ax.transAxes)
            ax.set_xticks([]); ax.set_yticks([])
            plt.pause(0.2)
            if gecen > 5 and not uyarildi:
                uyarildi = True
                print("\n⚠️  5 saniyedir /scan gelmiyor.")
                print("   1) LiDAR USB'ye takili mi?   lsusb | grep 10c4")
                print("   2) Surucu calisiyor mu? Baska bir terminalde:")
                print("      PORT=$(ls /dev/serial/by-id/*CP2102*)")
                print("      ros2 launch sllidar_ros2 sllidar_s2_launch.py \\")
                print("           serial_port:=$PORT serial_baudrate:=1000000\n",
                      flush=True)
            continue
        m = n.scan
        r = np.asarray(m.ranges, dtype=float)
        aci = m.angle_min + np.arange(len(r)) * m.angle_increment
        g = np.isfinite(r) & (r > 0.10) & (r < m.range_max)
        r_g, a_g = r[g], aci[g]

        # ARAC cercevesine cevir: ileri = +Y, sag = +X
        #
        # ACIYI -pi..pi ARALIGINA SAR. Sarmadan |th| ile karsilastirmak,
        # forward=180 gibi degerlerde koninin YARISINI disarida sayiyordu:
        # -175° isin icin th = -355° cikip |th|>60 oluyor, oysa gercekte
        # ileri yonden yalnizca 5° sapma var. Ekranda koninin bir tarafi
        # gri (koni disi) gorunuyordu. Tespit dugumu bu hatadan etkilenmiyor;
        # o, indeks aritmetigiyle np.mod sarmasi yapiyor.
        th = a_g - math.radians(args.forward)
        th = (th + np.pi) % (2 * np.pi) - np.pi
        ileri = r_g * np.cos(th)
        yanal = r_g * np.sin(th)

        konide = np.abs(np.degrees(th)) <= args.sector / 2
        koridorda = konide & (np.abs(yanal) <= args.corridor / 2) & (ileri > 0.10)

        ax.clear()
        R = args.range
        # koni
        ax.add_patch(Wedge((0, 0), R, 90 - args.sector/2, 90 + args.sector/2,
                           facecolor='gold', alpha=0.10))
        # koridor
        ax.add_patch(Polygon([(-args.corridor/2, 0), (args.corridor/2, 0),
                              (args.corridor/2, R), (-args.corridor/2, R)],
                             facecolor='limegreen', alpha=0.12))
        # menzil halkalari
        for rad in range(1, int(R) + 1):
            ax.add_patch(plt.Circle((0, 0), rad, fill=False, color='0.85', lw=0.7))
            ax.text(0.06, rad, f"{rad}m", fontsize=7, color='0.5', va='bottom')

        ax.scatter(yanal[~konide], ileri[~konide], s=3, c='0.75', label='360° tarama')
        ax.scatter(yanal[konide & ~koridorda], ileri[konide & ~koridorda],
                   s=5, c='darkorange', label='koni içi, koridor dışı')
        ax.scatter(yanal[koridorda], ileri[koridorda], s=12, c='green',
                   label='KORİDOR (engel sayılır)')

        # ileri yon oku
        ax.arrow(0, 0, 0, R * 0.35, width=0.03, color='red', zorder=5)
        ax.text(0.12, R * 0.35, f"İLERİ\nforward={args.forward:.0f}°",
                color='red', fontsize=9, va='top')
        ax.plot(0, 0, 'ks', ms=9)

        if koridorda.sum():
            en = ileri[koridorda].min()
            ax.set_title(f"KORİDORDA EN YAKIN: {en:.2f} m   "
                         f"({koridorda.sum()} ışın)", color='darkred', fontsize=12)
        else:
            ax.set_title("KORİDOR TEMİZ", color='green', fontsize=12)

        ax.set_xlim(-R, R); ax.set_ylim(-R * 0.4, R)
        ax.set_aspect('equal'); ax.grid(alpha=0.2)
        ax.set_xlabel("yanal (m)   ←sol   sağ→")
        ax.set_ylabel("ileri (m)")
        ax.legend(loc='lower right', fontsize=8)
        plt.pause(0.05)


if __name__ == '__main__':
    main()
