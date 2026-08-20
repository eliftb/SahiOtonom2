#!/usr/bin/env python3
"""LiDAR hayalet engel teşhisi.

Kullanım (LiDAR sürücüsü çalışırken):
    python3 EngelTespit/lidar_tani.py

Boş yolda sabit bir mesafe raporlanıyorsa sebebi üç şeyden biridir ve
bu araç hangisi olduğunu AÇIYA GÖRE DAĞILIMA bakarak ayırt eder:

  1. ARACIN KENDİ GÖVDESİ  -> dönüşler DAR açı bantlarında toplanır,
                              aralarda boşluk vardır.
  2. YER YANSIMASI         -> LiDAR aşağı eğik. Dönüşler ön arkın
                              ÇOĞUNU kesintisiz kaplar ve mesafe
                              azimutla DÜZGÜN değişir (eğimin en dik
                              olduğu yönde en yakın).
  3. GERÇEK ENGEL          -> dar bant + arkasında serbest mesafe.

Not: yatay düzlemde TAM DÜZ monte edilmiş bir 2D LiDAR yeri hiç görmez;
yer dönüşü varsa mutlaka bir eğim vardır.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Tani(Node):
    def __init__(self):
        super().__init__('lidar_tani')
        self.kare = None
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, m):
        if self.kare is None:
            self.kare = m


def main():
    yakinlik = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    n = Tani()
    import time
    t0 = time.time()
    while time.time() - t0 < 15 and n.kare is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = n.kare
    if m is None:
        print("❌ /scan gelmiyor. LiDAR sürücüsü çalışıyor mu?")
        return 1

    r = np.asarray(m.ranges, dtype=float)
    aci = np.degrees(m.angle_min + np.arange(len(r)) * m.angle_increment)
    gec = np.isfinite(r) & (r > 0.10) & (r < m.range_max)

    print(f"Tarama: {len(r)} nokta | {np.degrees(m.angle_min):+.0f}°..{np.degrees(m.angle_max):+.0f}° "
          f"| geçerli %{100*gec.mean():.0f}")
    print()
    print("ÖN 120°'DE AÇIYA GÖRE PROFİL")
    print(f"{'açı':>14} {'eğik menzil':>12} {'ileri':>9} {'yanal':>9} {'ışın':>6}")
    for a0 in range(-60, 60, 10):
        s = gec & (aci >= a0) & (aci < a0 + 10)
        if not s.sum():
            print(f"  {a0:+3d}°..{a0+10:+3d}°      -- veri yok --")
            continue
        rr = r[s]; th = np.radians(aci[s])
        print(f"  {a0:+3d}°..{a0+10:+3d}° {np.median(rr):10.2f} m {np.median(rr*np.cos(th)):7.2f} m "
              f"{np.median(rr*np.sin(th)):7.2f} m {s.sum():6d}")

    on = gec & (np.abs(aci) <= 60)
    rr = r[on]; a = aci[on]
    yakin = rr < yakinlik
    print()
    print(f"{yakinlik:.1f} m'den yakın ışın: {yakin.sum()} / {on.sum()} (%{100*yakin.mean():.0f})")
    if yakin.sum() < 10:
        print("✅ Ön temiz - hayalet engel yok.")
        return 0

    d = rr[yakin]; ay = a[yakin]
    print(f"  mesafe: {d.min():.2f} .. {d.max():.2f} m (std {d.std():.3f})")
    print(f"  açı   : {ay.min():+.0f}° .. {ay.max():+.0f}°")
    kapsama = 100.0 * yakin.sum() / on.sum()
    print()

    if kapsama > 60:
        h = float(np.median(d))
        print("🔎 TEŞHİS: YER YANSIMASI (LiDAR aşağı eğik)")
        print(f"   Yakın dönüşler ön arkın %{kapsama:.0f}'ini kaplıyor. Tek bir cisim")
        print("   bu kadar geniş bir açıyı kaplayamaz; taranan düzlem zemine değiyor.")
        print()
        print(f"   Zemine ~{h:.2f} m'de değiyor. Montaj yüksekliğine göre eğim:")
        for yuk in (0.15, 0.25, 0.35, 0.50):
            if yuk < h:
                print(f"     {yuk:.2f} m yükseklikte  ->  {math.degrees(math.asin(yuk/h)):.1f}° aşağı eğim")
        print()
        print("   ÇÖZÜM: LiDAR'ı yatayla tam paralel hizala (su terazisi).")
        print("   Küçük bir eğim kalıyorsa min_valid_distance'ı zemin mesafesinin")
        print("   biraz ÜSTÜNE çek - ama bu, o mesafedeki GERÇEK engelleri de kör eder.")
    else:
        print("🔎 TEŞHİS: SABİT CİSİM (büyük olasılıkla aracın kendi parçası)")
        print(f"   Yakın dönüşler ön arkın yalnızca %{kapsama:.0f}'ini kaplıyor,")
        print("   yani belirli yönlerde bir şey var. Şu açılara fiziksel olarak bak:")
        print()
        idx = np.argsort(ay); a_s = ay[idx]; d_s = d[idx]
        kop = np.flatnonzero(np.diff(a_s) > 3.0)
        bas = 0
        for k in list(kop) + [len(a_s) - 1]:
            sa, sd = a_s[bas:k+1], d_s[bas:k+1]
            if len(sa) > 3:
                yanal = np.median(sd) * math.sin(math.radians(np.median(sa)))
                print(f"     {sa.min():+6.1f}°..{sa.max():+6.1f}°   {np.median(sd):.2f} m   "
                      f"(yanal {yanal:+.2f} m)   {len(sa)} ışın")
            bas = k + 1
        print()
        print("   ÇÖZÜM: o parçayı LiDAR'ın görüş alanından çıkar, ya da")
        print("   LiDAR'ı öne/yukarı taşı. Yazılımla maskelemek son çare -")
        print("   o açıdaki gerçek engelleri de kör eder.")
    return 0


if __name__ == '__main__':
    rclpy.init if False else None
    sys.exit(main())
