#!/usr/bin/env python3
"""Slalom düğümü — şerit sapmasına kaydırma ekleyerek şerit değiştirir.

BORU HATTINDAKİ YERİ:

    şerit tespiti ──/lane/lateral_deviation_raw──▶ SLALOM ──/lane/lateral_deviation──▶ UART PID
                                                     ▲
                          /obstacle_detected ────────┘
                          /obstacle_distance
                          /speed

Şerit düğümünün yayın topic'i 'raw' olarak DEĞİŞTİRİLMELİ. İki düğüm aynı
topic'e yazarsa direksiyon çekişir ve davranış öngörülemez olur.

Haberlesme/uart_sender_node3.py'ye DOKUNULMAZ: oradaki PID zaten
/lane/lateral_deviation'ı sıfıra sürüyor, biz o değerin anlamını
kaydırıyoruz.

ÇALIŞTIRMA:
    python3 Slalom/slalom_node.py
    python3 Slalom/slalom_node.py --ros-args -p gecis_suresi_s:=2.5
"""
from __future__ import annotations

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool, Float32, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from slalom_durum import SlalomMakinesi, Durum  # noqa: E402


class SlalomNode(Node):
    def __init__(self):
        super().__init__('slalom_node')
        p = self.declare_parameter

        p('giris_topic', '/lane/lateral_deviation_raw')
        p('cikis_topic', '/lane/lateral_deviation')
        # Manevranın başladığı mesafe (tampondan). Karar düğümünün acil
        # duruş eşiğinden BÜYÜK olmalı, yoksa araç geçişe başlayamadan
        # fren yapar ve slalom akmaz.
        p('manevra_tetik_m', 3.0)
        p('ilk_engelde_dur', True)
        p('durma_suresi_s', 2.0)
        # Geçişin süresi. Şerit genişliği ve hıza göre PİSTTE AYARLANIR.
        p('gecis_suresi_s', 3.0)
        # Kaydırma büyüklüğü (-1..1 ölçeğinde). Büyütmek geçişi
        # sertleştirir; 1.0 tam direksiyon kilidi demek.
        p('gecis_sapmasi', 0.6)
        p('durma_hiz_esigi', 0.05)
        # Şerit sapması bu süredir gelmiyorsa yayın YAPILMAZ (bkz. aşağıda).
        p('giris_zaman_asimi_s', 0.5)

        g = lambda k: self.get_parameter(k).value
        self.GIRIS = g('giris_topic')
        self.CIKIS = g('cikis_topic')
        self.GIRIS_TIMEOUT = float(g('giris_zaman_asimi_s'))

        if self.GIRIS == self.CIKIS:
            self.get_logger().error(
                f"giris_topic ve cikis_topic AYNI ({self.GIRIS}) - dugum kendi "
                f"ciktisini dinler ve geri besleme dongusune girer. Serit "
                f"tespitinin yayin topic'ini '_raw' olarak degistir.")
            raise SystemExit(1)

        self.makine = SlalomMakinesi(
            manevra_tetik_m=float(g('manevra_tetik_m')),
            durma_suresi_s=float(g('durma_suresi_s')),
            gecis_suresi_s=float(g('gecis_suresi_s')),
            gecis_sapmasi=float(g('gecis_sapmasi')),
            durma_hiz_esigi=float(g('durma_hiz_esigi')),
            ilk_engelde_dur=bool(g('ilk_engelde_dur')),
        )

        self.sapma = None
        self.sapma_zamani = None
        self.engel_var = False
        self.engel_m = None
        self.hiz = None

        self.create_subscription(Float32, self.GIRIS, self._sapma_cb, 10)
        self.create_subscription(Bool, '/obstacle_detected', self._engel_cb, 10)
        self.create_subscription(Float32, '/obstacle_distance', self._mesafe_cb, 10)
        self.create_subscription(Float32, '/speed', self._hiz_cb, 10)

        self.pub = self.create_publisher(Float32, self.CIKIS, 10)
        self.pub_durum = self.create_publisher(String, '/slalom/durum', 10)

        # Şerit düğümüyle aynı hızda değil, sabit 20 Hz: kaydırma zamana
        # bağlı, kamera karesine değil.
        self.create_timer(0.05, self._dongu)

        self.get_logger().warn(
            f"🏁 SLALOM AKTİF | {self.GIRIS} -> {self.CIKIS} | "
            f"tetik {self.makine.tetik:.1f} m | duruş {self.makine.durma_suresi:.1f} sn | "
            f"geçiş {self.makine.gecis_suresi:.1f} sn | kaydırma ±{self.makine.gecis_sapmasi:.2f}")

    # ---------------- callback'ler ----------------

    def _sapma_cb(self, m):
        self.sapma = float(m.data)
        self.sapma_zamani = time.monotonic()

    def _engel_cb(self, m):
        self.engel_var = bool(m.data)

    def _mesafe_cb(self, m):
        self.engel_m = float(m.data) if m.data > 0 else None

    def _hiz_cb(self, m):
        self.hiz = float(m.data)

    # ---------------- ana döngü ----------------

    def _dongu(self):
        simdi = time.monotonic()

        # Şerit sapması gelmiyorsa HİÇBİR ŞEY YAYINLAMA.
        # Kendi başımıza bir değer uydurmak, şerit tespiti çökmüşken
        # araca sahte bir "yol böyle" bilgisi vermek olurdu. Yayını
        # kesmek, aşağı akıştaki davranışı şerit düğümü hiç yokmuş
        # gibi bırakır - yani mevcut davranışın aynısı.
        if self.sapma is None or (simdi - self.sapma_zamani) > self.GIRIS_TIMEOUT:
            self.get_logger().warn(
                f"{self.GIRIS} gelmiyor - sapma yayını kesildi.",
                throttle_duration_sec=2.0)
            return

        c = self.makine.adim(simdi, self.engel_var, self.engel_m, self.hiz)

        cikti = max(-1.0, min(1.0, self.sapma + c.kaydirma))
        self.pub.publish(Float32(data=cikti))
        self.pub_durum.publish(String(
            data=f"{c.durum.value} | şerit={c.serit.value} | "
                 f"geçilen={c.gecilen_engel} | {c.aciklama}"))

        if c.degisti:
            self.get_logger().warn(
                f"➡️  {c.durum.value.upper()} | şerit={c.serit.value} | "
                f"geçilen={c.gecilen_engel} | {c.aciklama}")
        elif c.durum is Durum.GECIS:
            self.get_logger().info(
                f"{c.aciklama} | sapma {self.sapma:+.3f} -> {cikti:+.3f}",
                throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SlalomNode()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
