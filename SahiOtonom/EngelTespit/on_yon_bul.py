#!/usr/bin/env python3
"""forward_angle_deg kalibrasyonu — aracın önü LiDAR'ın kaç derecesi?

NEDEN GEREKLİ: engel tespiti "ileri" yönü forward_angle_deg parametresinden
alır ve varsayılan 0.0, yani "LiDAR'ın gövdesindeki 0° işareti aracın önüne
bakıyor" demektir. LiDAR döndürülerek monte edilmişse sistem yanlış yöne
bakar; boş yolda hayalet engel görür, gerçek engeli göremez.

KULLANIM (LiDAR sürücüsü çalışırken):
    python3 EngelTespit/on_yon_bul.py

Sonra ARACIN TAM ÖNÜNE geç ve 1-3 m mesafede dur. Araç sana bakan açıyı
ölçüp önerilen forward_angle_deg değerini yazacak.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class YonBul(Node):
    def __init__(self, dmin, dmax):
        super().__init__('on_yon_bul')
        self.dmin, self.dmax = dmin, dmax
        self.olcumler = []
        self.son = None
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, m):
        r = np.asarray(m.ranges, dtype=float)
        aci = np.degrees(m.angle_min + np.arange(len(r)) * m.angle_increment)
        g = np.isfinite(r) & (r > self.dmin) & (r < self.dmax)
        if g.sum() < 5:
            self.son = None
            return
        # Hedef = bu mesafe bandındaki EN GENİŞ bitişik küme (insan gövdesi),
        # tek tük gürültü pikselleri değil.
        a = aci[g]; d = r[g]
        idx = np.argsort(a); a, d = a[idx], d[idx]
        kop = np.flatnonzero(np.diff(a) > 3.0)
        bas, en_iyi = 0, None
        for k in list(kop) + [len(a) - 1]:
            sa, sd = a[bas:k+1], d[bas:k+1]
            if len(sa) >= 5 and (en_iyi is None or len(sa) > en_iyi[2]):
                en_iyi = (float(np.median(sa)), float(np.median(sd)), len(sa))
            bas = k + 1
        if en_iyi:
            self.son = en_iyi
            self.olcumler.append(en_iyi[0])


def main():
    dmin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
    dmax = float(sys.argv[2]) if len(sys.argv) > 2 else 3.5
    rclpy.init()
    n = YonBul(dmin, dmax)
    print(f"Hedef bandı: {dmin:.1f} – {dmax:.1f} m")
    print("ŞİMDİ ARACIN TAM ÖNÜNE GEÇ ve o mesafede sabit dur.")
    print("15 saniye ölçüyorum...\n")
    t0 = time.time(); son_yaz = 0
    while time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - son_yaz > 1.0:
            son_yaz = time.time()
            if n.son:
                a, d, k = n.son
                print(f"  {time.time()-t0:4.0f}s   açı {a:+7.1f}°   mesafe {d:5.2f} m   ({k} ışın)")
            else:
                print(f"  {time.time()-t0:4.0f}s   -- bu bantta hedef yok --")

    if len(n.olcumler) < 5:
        print("\n❌ Yeterli ölçüm yok. Mesafe bandını değiştirip tekrar dene:")
        print("   python3 EngelTespit/on_yon_bul.py 0.5 5.0")
        return 1

    o = np.array(n.olcumler)
    med = float(np.median(o))
    print(f"\n{'='*54}")
    print(f"ölçüm sayısı : {len(o)}")
    print(f"açı medyanı  : {med:+.1f}°   (std {o.std():.1f}°)")
    if o.std() > 8:
        print("⚠️  Dağılım geniş - sabit durduğundan ve bantta başka cisim")
        print("    olmadığından emin ol, tekrar ölç.")
    print()
    print(f">>> forward_angle_deg = {med:.1f}")
    print()
    print("EngelTespit/engel-tespit.py içinde:")
    print(f"    self.declare_parameter('forward_angle_deg', {med:.1f})")
    print(f"{'='*54}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
