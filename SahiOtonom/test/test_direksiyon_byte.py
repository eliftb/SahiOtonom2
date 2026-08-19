#!/usr/bin/env python3
"""Direksiyon byte eşlemesi regresyon testi - ARAÇ/ROS GEREKTİRMEZ.

uart_sender_node3.angle_to_byte'ı sentetik trim/kilit değerleriyle çalıştırır.
Buradaki iddialar aracı sehpaya çıkarmadan doğrulanabilen tek şeyler:
merkezin nereye düştüğü, ölçeğin max_steering_angle ile birlikte değişip
değişmediği ve kırpmanın iki tarafı eşit bırakıp bırakmadığı.

    python3 SahiOtonom/test/test_direksiyon_byte.py
"""
import importlib.util
import os
import sys
import types

UART_SRC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'Haberlesme',
    'uart_sender_node3.py'))


def load_uart_module():
    """ROS/serial bağımlılıklarını sahteleyerek düğüm dosyasını yükler."""
    for name in ['rclpy', 'rclpy.node', 'rclpy.executors', 'ackermann_msgs',
                 'ackermann_msgs.msg', 'std_msgs', 'std_msgs.msg', 'nav_msgs',
                 'nav_msgs.msg', 'rcl_interfaces', 'rcl_interfaces.msg', 'serial']:
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy.node'].Node = type('Node', (), {})
    sys.modules['rclpy.executors'].ExternalShutdownException = Exception
    sys.modules['ackermann_msgs.msg'].AckermannDrive = object
    sys.modules['nav_msgs.msg'].Odometry = object
    sys.modules['rcl_interfaces.msg'].SetParametersResult = object

    class _Msg:
        def __init__(self, data=None):
            self.data = data
    for ad in ('Float32', 'Int32', 'Bool'):
        setattr(sys.modules['std_msgs.msg'], ad, _Msg)
    sys.modules['serial'].Serial = object
    sys.modules['serial'].SerialException = Exception

    spec = importlib.util.spec_from_file_location('uart_node', UART_SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def dugum(trim=0, kilit=0.5):
    """__init__ çalıştırmadan sadece byte eşlemesi için gereken alanları kurar.

    __init__ portu açmaya ve ROS'a bağlanmaya çalışır; test edilen fonksiyonun
    ikisiyle de işi yok.
    """
    mod = load_uart_module()
    n = object.__new__(mod.UartSenderNode)
    n.steering_trim = trim
    n.max_steering_angle = kilit
    return n


def esit(ad, bulunan, beklenen):
    if bulunan != beklenen:
        print(f'  BAŞARISIZ  {ad}: {bulunan} != {beklenen}')
        return 1
    print(f'  tamam      {ad} = {bulunan}')
    return 0


def main():
    hata = 0

    print('\n1) Trim 0, kilit 0.5 - ESKİ DAVRANIŞ AYNEN KORUNUYOR')
    n = dugum()
    hata += esit('merkez', n.angle_to_byte(0.0), 180)
    hata += esit('tam sol', n.angle_to_byte(-0.5), 0)
    hata += esit('tam sağ', n.angle_to_byte(0.5), 360)
    # Logdaki gerçek örnek: -0.061 rad -> 158
    hata += esit('-0.061 rad', n.angle_to_byte(-0.061), 158)

    print('\n2) Trim -30: merkez kayar, komutlar merkez etrafında SİMETRİK kalır')
    n = dugum(trim=-30)
    hata += esit('merkez', n.angle_to_byte(0.0), 150)
    merkez = 150
    sol = n.angle_to_byte(-0.25)
    sag = n.angle_to_byte(0.25)
    hata += esit('sol sapma', merkez - sol, sag - merkez)
    print('\n   Uçlar KIRPILMIYOR: küçük taraf (150) esas alınır, yoksa araç')
    print('   sağa sola döndüğünden sert dönerdi.')
    hata += esit('tam sol', n.angle_to_byte(-0.5), 0)
    hata += esit('tam sağ', n.angle_to_byte(0.5), 300)

    print('\n3) Kilit ölçülüp 0.35 rad yapılınca ÖLÇEK de onunla değişir')
    n = dugum(kilit=0.35)
    hata += esit('tam sol (0.35)', n.angle_to_byte(-0.35), 0)
    hata += esit('tam sağ (0.35)', n.angle_to_byte(0.35), 360)
    print('   (eski gömülü ölçekte tam sağ 306 olurdu - kilidin %15\'i ölü)')

    print('\n4) Doyum: kilidin ötesindeki komut uca kırpılır')
    n = dugum(trim=10, kilit=0.4)
    hata += esit('aşırı sol', n.angle_to_byte(-5.0), n.angle_to_byte(-0.4))
    hata += esit('aşırı sağ', n.angle_to_byte(5.0), n.angle_to_byte(0.4))
    # Hiçbir açı protokol aralığının dışına taşmamalı (firmware'e bozuk
    # sayı gitmesi tekerin nereye gideceğini tahmin edilemez yapar).
    disari = [a for a in (-9.0, -0.7, 0.0, 0.7, 9.0)
              if not 0 <= n.angle_to_byte(a) <= 360]
    hata += esit('aralık dışına taşan açı yok', disari, [])

    print('\n5) Bozuk kilit (0) düğümü çökertmez, merkezde tutar')
    n = dugum(kilit=0.0)
    hata += esit('merkez', n.angle_to_byte(-0.3), 180)

    print('\n' + ('  HEPSİ GEÇTİ' if hata == 0 else f'  {hata} BAŞARISIZ'))
    return 1 if hata else 0


if __name__ == '__main__':
    sys.exit(main())
