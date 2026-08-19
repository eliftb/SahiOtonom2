#!/usr/bin/env python3
"""ZED odometrisini test/ÖLÇME aracı.

İki mod var:

  CANLI İZLEME (varsayılan)  - konumu ekranda gösterir, ENTER sıfırlar.
  ÖLÇÜM       (--mesafe/--donus/--bekle) - başlangıç/bitiş arasını ölçer ve
                               ÖLÇEK (okunan / gerçek) basar.

NEDEN ÖLÇEK ÖNEMLİ: odometri iki yerde KONTROLE giriyor, gösterge değil.
  1) Şerit düğümü virajı kaybettiğinde mesafeyi kat edilen yolla düşürür ve
     dönülen AÇIYI biriktirir; dönüşün bittiğine bu açı karar verir
     (viraj_donus_acisi_deg, viraj_donus_yolu_m).
  2) UART düğümü kavşakta hedef yönü ±turn_angle_deg kaydırıp odometri yaw'ına
     kapalı döngüde tutar.
Yani %20 hatalı bir açı ölçümü = 90° yerine 72° dönen bir araç.

KULLANIM:
    # 1) Kamerayı odometri AÇIK başlatın (tek başına, launcher olmadan):
    python3 zedi2connect_port.py --ros-args -p enable_odometry:=true

    # 2) Başka bir terminalde:
    python3 odometri_test.py                 # canlı izleme
    python3 odometri_test.py --mesafe 2.0    # 2 m düz it, ölçeği ölç
    python3 odometri_test.py --donus -90     # sağa 90° döndür (sağ = eksi)
    python3 odometri_test.py --bekle 30      # 30 sn DURARAK drift ölç

    SIFIRLA: ENTER'a basın (o anki konum yeni başlangıç olur)
    ÇIKIŞ  : CTRL+C
"""
import argparse
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def kuaterniyon_to_yaw(q):
    """Kuaterniyondan sapma açısı (derece). Z ekseni etrafındaki dönüş."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class OdometriTest(Node):
    def __init__(self):
        super().__init__('odometri_test')
        self.baslangic = None       # sıfırlama noktası
        self.son = None
        self.toplam_yol = 0.0
        self.onceki_ham = None
        self.mesaj_sayisi = 0
        self.ilk_zaman = None

        self.create_subscription(Odometry, '/zed2i/odom', self.odom_callback, 10)
        self.create_timer(0.3, self.yaz)
        print('/zed2i/odom bekleniyor...')
        print('(veri gelmiyorsa: kamera enable_odometry:=true ile mi baslatildi?)')

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ham = (p.x, p.y, p.z)

        # Kat edilen toplam yol: ardışık konumlar arası mesafelerin toplamı.
        # Duruyorken bunun artması = tracking gürültüsü (drift) demektir.
        if self.onceki_ham is not None:
            d = math.dist(ham, self.onceki_ham)
            self.toplam_yol += d
        self.onceki_ham = ham

        if self.baslangic is None:
            self.baslangic = ham
            self.ilk_zaman = self.get_clock().now()
        self.son = (ham, kuaterniyon_to_yaw(q))
        self.mesaj_sayisi += 1

    def sifirla(self):
        if self.onceki_ham is not None:
            self.baslangic = self.onceki_ham
            self.toplam_yol = 0.0
            self.ilk_zaman = self.get_clock().now()

    def yaz(self):
        if self.son is None:
            return
        (x, y, z), yaw = self.son
        bx, by, bz = self.baslangic
        dx, dy, dz = x - bx, y - by, z - bz
        duz = math.hypot(dx, dy)
        gecen = (self.get_clock().now() - self.ilk_zaman).nanoseconds / 1e9

        print('\033[2J\033[H', end='')
        print('  ZED ODOMETRI TESTI')
        print('  ' + '-' * 52)
        print(f'  ILERI  (x) : {dx:+7.3f} m      <- one dogru hareket +')
        print(f'  SOL    (y) : {dy:+7.3f} m      <- sola dogru hareket +')
        print(f'  YUKARI (z) : {dz:+7.3f} m')
        print(f'  SAPMA      : {yaw:+7.1f} derece')
        print('  ' + '-' * 52)
        print(f'  duz mesafe (baslangictan)  : {duz:7.3f} m')
        print(f'  kat edilen toplam yol      : {self.toplam_yol:7.3f} m')
        print(f'  mesaj sayisi / sure        : {self.mesaj_sayisi} / {gecen:.1f} sn'
              f'  ({self.mesaj_sayisi / gecen:.1f} Hz)' if gecen > 0 else '')
        print('  ' + '-' * 52)
        if duz < 0.05 and self.toplam_yol > 0.20:
            print('  ! DURUYORKEN TOPLAM YOL ARTIYOR = drift.')
            print('    Ortam az dokulu/karanlik olabilir; ZED duvar gibi duz')
            print('    yuzeylerde konum kaybeder.')
        print('\n  ENTER = sifirla     CTRL+C = cikis')


# --- ÖLÇÜM MODU --------------------------------------------------------------

class OlcumNode(Node):
    """Başlangıç ile bitiş arasındaki hareketi biriktirir.

    Canlı izlemeden farkı: burada sayı EKRANDA AKMAZ, iki ENTER arasındaki
    hareketin toplamı raporlanır. Göz kararı "yaklaşık bir metre gibi" yerine
    ölçek çıkar; ölçek olmadan parametre düzeltilemez.
    """

    def __init__(self):
        super().__init__('odometri_olcum')
        self.create_subscription(Odometry, '/zed2i/odom', self.odom_callback, 10)
        self.kilit = threading.Lock()
        self.sifirla()

    def sifirla(self):
        with self.kilit:
            self.ilk_konum = None
            self.ilk_yaw = None
            self.son_konum = None
            self.son_yaw = None
            self.onceki_konum = None
            self.onceki_yaw = None
            self.toplam_yol = 0.0
            self.donulen = 0.0       # BİRİKİMLİ: ±180° sınırından geçse de doğru
            self.sayac = 0
            self.ilk_zaman = None
            self.son_zaman = None

    def odom_callback(self, msg):
        k = msg.pose.pose.position
        konum = (k.x, k.y, k.z)
        yaw = math.radians(kuaterniyon_to_yaw(msg.pose.pose.orientation))
        simdi = self.get_clock().now()

        with self.kilit:
            if self.ilk_konum is None:
                self.ilk_konum, self.ilk_yaw = konum, yaw
                self.ilk_zaman = simdi
            else:
                self.toplam_yol += math.dist(konum[:2], self.onceki_konum[:2])
                # Açı farkı sarmalanarak toplanır - şerit düğümündeki
                # dönülen açı sayacıyla AYNI yöntem, ölçüm onu doğrulasın diye.
                self.donulen += math.atan2(math.sin(yaw - self.onceki_yaw),
                                           math.cos(yaw - self.onceki_yaw))
            self.onceki_konum, self.onceki_yaw = konum, yaw
            self.son_konum, self.son_yaw = konum, yaw
            self.son_zaman = simdi
            self.sayac += 1

    def rapor(self):
        with self.kilit:
            if self.ilk_konum is None or self.sayac < 2:
                return None
            dx = self.son_konum[0] - self.ilk_konum[0]
            dy = self.son_konum[1] - self.ilk_konum[1]
            sure = (self.son_zaman - self.ilk_zaman).nanoseconds / 1e9
            return {
                'ileri': dx, 'yanal': dy,
                'duz': math.hypot(dx, dy),
                'yol': self.toplam_yol,
                'donulen_deg': math.degrees(self.donulen),
                'sayac': self.sayac,
                'sure': sure,
                'hz': self.sayac / sure if sure > 0 else 0.0,
            }


def enter_bekle(mesaj):
    print(mesaj, end='', flush=True)
    sys.stdin.readline()


def spin_ederken_bekle(node, bitti):
    """ENTER beklerken düğümü döndürür; yoksa mesajlar birikir ve ölçüm boş kalır."""
    while rclpy.ok() and not bitti.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def veri_var_mi(node, saniye=5.0):
    """Ölçüme başlamadan önce topic gerçekten akıyor mu."""
    hedef = node.get_clock().now().nanoseconds + saniye * 1e9
    while rclpy.ok() and node.get_clock().now().nanoseconds < hedef:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.sayac > 0:
            return True
    return False


def yorumla(ad, okunan, gercek, birim, iyi=0.05, idare=0.15):
    """Ölçek ve tanı satırı. Eşikler: %5 iyi, %15 sınırda, üstü kullanılamaz."""
    if abs(gercek) < 1e-9:
        return
    olcek = okunan / gercek
    hata = abs(olcek - 1.0)
    durum = 'IYI' if hata <= iyi else ('SINIRDA' if hata <= idare else 'KOTU')
    print(f'  {ad:<12}: okunan {okunan:+8.3f} {birim} / gercek {gercek:+8.3f} {birim}')
    print(f'  {"olcek":<12}: {olcek:.3f}  (%{(olcek - 1.0) * 100:+.1f} sapma)  -> {durum}')
    return olcek


def olcum_yap(args):
    rclpy.init()
    node = OlcumNode()
    try:
        print('/zed2i/odom bekleniyor...')
        if not veri_var_mi(node):
            print('\n  HIC MESAJ GELMEDI. Kamera odometri ACIK baslatildi mi?')
            print('    python3 zedi2connect_port.py --ros-args -p enable_odometry:=true')
            print('  (launcher ile baslattiysaniz zaten acik; ros2 topic hz /zed2i/odom ile bakin)')
            return 1

        if args.bekle:
            print(f'\n  ARAC HIC HAREKET ETMEYECEK. {args.bekle:.0f} sn olculuyor...')
            node.sifirla()
            hedef = node.get_clock().now().nanoseconds + args.bekle * 1e9
            while rclpy.ok() and node.get_clock().now().nanoseconds < hedef:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            if args.mesafe:
                print(f'\n  HAZIRLIK: araci baslangic cizgisine koyun, {args.mesafe:.2f} m '
                      f'ilerisini yere isaretleyin.')
                print('  Araci DUZ itin/surun; donmeyin (yanal kayma ve sapma da olculuyor).')
            else:
                print(f'\n  HAZIRLIK: araci yerinde {abs(args.donus):.0f} derece dondurun.')
                print('  ROS kurali: SOLA donus + , SAGA donus -  (bu isaret UART yon '
                      'tutmasinda kullaniliyor).')
                print('  TEKRAR OLCERKEN: araci baslangic yonune geri dondurmeniz gerekir;')
                print(f'  o geri donusu olcuyorsaniz --donus {-args.donus:+.0f} kullanin.')
            enter_bekle('\n  Baslamak icin ENTER...')
            node.sifirla()
            bitti = threading.Event()
            t = threading.Thread(target=spin_ederken_bekle, args=(node, bitti), daemon=True)
            t.start()
            enter_bekle('  Hareket bitince ENTER...')
            bitti.set()
            t.join(timeout=1.0)

        r = node.rapor()
        if r is None:
            print('\n  Yeterli veri toplanmadi (mesaj gelmiyor).')
            return 1

        print('\n  ' + '-' * 58)
        print(f'  ileri (x)   : {r["ileri"]:+8.3f} m')
        print(f'  yanal (y)   : {r["yanal"]:+8.3f} m')
        print(f'  duz mesafe  : {r["duz"]:8.3f} m')
        print(f'  kat edilen  : {r["yol"]:8.3f} m   (yol > duz mesafe = zikzak/gurultu)')
        print(f'  donulen aci : {r["donulen_deg"]:+8.1f} derece')
        print(f'  veri        : {r["sayac"]} mesaj / {r["sure"]:.1f} sn = {r["hz"]:.1f} Hz')
        print('  ' + '-' * 58)

        if args.bekle:
            print('  DRIFT (arac dururken okunan hareket):')
            print(f'    konum kaymasi : {r["duz"]:.3f} m')
            print(f'    yol birikimi  : {r["yol"]:.3f} m')
            print(f'    aci kaymasi   : {r["donulen_deg"]:+.1f} derece')
            kotu = r['yol'] > 0.20 or abs(r['donulen_deg']) > 3.0
            print('\n  ' + ('! DRIFT YUKSEK. ZED dokusuz/karanlik ortamda konum kaybeder.'
                            if kotu else 'Drift kabul edilebilir.'))
            if kotu:
                print('    Viraj hafizasi ve kavsak yon tutmasi bu drifte dogrudan biner:')
                print('    viraj_mesafe_m yanlis dusar, hedef yaw kayar.')
        elif args.mesafe:
            # HANGI BUYUKLUK? Serit dugumu odom_callback'te ARDISIK ADIMLARI
            # topluyor (viraj_mesafe_m -= adim), yani KAT EDILEN YOL. Bastan
            # sona duz cizgi mesafesi (duz) sadece hareket gercekten duzse
            # ayni sey; olcegi ondan hesaplamak, itiste yasanan kucuk bir geri
            # kacisi 'odometri bozuk' gibi gosterir. Asil olcek yol integrali.
            oran = r['duz'] / r['yol'] if r['yol'] > 1e-6 else 0.0
            if oran < 0.95:
                print('  ! HAREKET DUZ DEGIL: bastan sona yer degistirme, kat')
                print(f'    edilen yolun %{oran * 100:.0f}\'i. Arac geri de gitmis,')
                print('    duraklamis ya da takip sicramis olabilir.')
                print('    Asagidaki "duz mesafe" olcegi bu yuzden dusuk cikar;')
                print('    sonuc supheliyse olcumu TEKRARLAYIN.\n')
            yorumla('yol (KULLANILAN)', r['yol'], args.mesafe, 'm')
            print()
            yorumla('duz mesafe', r['duz'], args.mesafe, 'm')
            print(f'\n  yanal kayma : {r["yanal"]:+.3f} m   (duz gittiyseniz ~0 olmali)')
            print(f'  sapma       : {r["donulen_deg"]:+.1f} derece (duz gittiyseniz ~0)')
            print('\n  KARAR "yol (KULLANILAN)" satirina gore verilir: serit dugumu')
            print('  odometriyi adim adim toplar (viraj_mesafe_m -= adim), bastan')
            print('  sona duz cizgiyi hic olcmez. Etkiledigi yerler:')
            print('    viraj_mesafe_m dususu (viraj ne zaman "gelir")')
            print('    viraj_donus_yolu_m = 6.0 emniyeti, viraj_hafiza_m = 12.0')
            print('  NOT: yol integrali dururken bile gurultuyle sisiyor')
            print('  (--bekle olcumundeki "yol birikimi" o gurultunun tabani).')
        else:
            olcek = yorumla('donus', r['donulen_deg'], args.donus, 'derece')
            if r['donulen_deg'] * args.donus < 0:
                # BUYUKLUK TUTUP ISARET TUTMUYORSA en olasi aciklama sensor
                # degil OLCUM: olcumu tekrarlamak icin araci geri dondurmek
                # gerekiyor ve o geri donus ters yondedir. Bunu 'sensor ters'
                # diye raporlamak gercek bir ariza varmis gibi gosterir -
                # olculen dort donusun ikisi geri donusse tablo alternatif
                # isaretli cikar ve insan sensoru sucladi.
                yakin = abs(abs(r['donulen_deg']) - abs(args.donus)) <= abs(args.donus) * 0.2
                print('\n  ! Okunan isaret beklenenin TERSI.')
                if yakin:
                    print('    Buyukluk tutuyor, yalnizca yon ters. EN OLASI SEBEP:')
                    print('    araci beklenenin ters yonune dondurdunuz (or. bir onceki')
                    print('    olcumden GERI dondurme). Oyleyse sensor dogru calisiyor:')
                    print(f'    ayni donusu --donus {-args.donus:+.0f} ile olcun.')
                    print('    Sensorun gercekten ters oldugunu soyleyebilmek icin')
                    print('    aracin FIZIKSEL yonunden emin olmalisiniz.')
                else:
                    print('    Buyukluk de tutmuyor; olcum supheli, tekrarlayin.')
                print('\n    GERCEKTEN ters ise: kavsakta hedef yaw hesabi ters doner')
                print('    (uart_sender_node3.py: ham = guncel_yaw - kayma).')
            elif olcek and abs(olcek - 1.0) > 0.15:
                print('\n  Duzeltme: dogru 90 derece donmesi icin')
                print(f'    turn_angle_deg      ~ {90 / olcek:.0f}   (uart_sender_node3.py)')
                print(f'    viraj_donus_acisi_deg ~ {90 / olcek:.0f} (serit-tespitcopy.py)')
                print('    NOT: once ZED montaji/titresimini kontrol edin; parametreyi')
                print('    egmek gercek sorunu gizleyebilir.')
        return 0
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main(args=None):
    ap = argparse.ArgumentParser(
        description='ZED odometri testi/olcumu',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--mesafe', type=float, metavar='M',
                   help='araci M metre DUZ ilerletin, mesafe olcegini olcer')
    g.add_argument('--donus', type=float, metavar='DERECE',
                   help='araci yerinde dondurun (sola +, saga -), aci olcegini olcer')
    g.add_argument('--bekle', type=float, metavar='SN',
                   help='arac DURURKEN SN saniye drift olcer')
    a = ap.parse_args()
    if a.mesafe or a.donus or a.bekle:
        return olcum_yap(a)

    rclpy.init(args=args)
    node = OdometriTest()

    def enter_dinle():
        for _ in sys.stdin:
            node.sifirla()

    t = threading.Thread(target=enter_dinle, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
