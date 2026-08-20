#!/usr/bin/env python3
"""120° koni + koridor filtresi testleri (donanımsız).

Asıl soru: geniş açı yandaki nesneleri GÖRÜR ama onlara TEPKİ VERMEZ mi?
"""
import importlib.util
import math
import os

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import LaserScan

BURASI = os.path.dirname(os.path.abspath(__file__))
YOL = os.path.join(BURASI, '..', 'EngelTespit', 'engel-tespit.py')

N = 3240                      # RPLIDAR S2 tam turda ~3240 nokta


@pytest.fixture(scope='module')
def mod():
    if not rclpy.ok():
        rclpy.init()
    spec = importlib.util.spec_from_file_location('engel', YOL)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    yield m
    rclpy.shutdown()


def tarama(engeller, menzil=30.0):
    """engeller: [(aci_derece, mesafe_m), ...] -> LaserScan"""
    m = LaserScan()
    m.angle_min = -math.pi
    m.angle_max = math.pi
    m.angle_increment = 2 * math.pi / N
    m.range_min = 0.05
    m.range_max = menzil
    r = [float('inf')] * N
    for aci, mes in engeller:
        i = int(round((math.radians(aci) - m.angle_min) / m.angle_increment)) % N
        r[i] = mes
    m.ranges = r
    return m


def calistir(mod, scan, **params):
    """Testler geometriyi sinar; uretim varsayilanlarini MIRAS ALMAZ.

    forward_angle_deg araca gore ayarlanan bir montaj parametresi (su an
    180: LiDAR ters monte). Testler onu devralirsa, montaj degistiginde
    gecmesi gereken testler duser. Bu yuzden acikca 0 sabitleniyor;
    yonun kendisi test_lidar_dondurulmus_montaj ile ayrica sinaniyor."""
    n = mod.LidarObstacleDetector()
    n.FORWARD_ANGLE_DEG = 0.0
    # LiDAR->tampon payi da montaj parametresi: geometri testleri SENSOR
    # cercevesinde olcer. Payin kendisi ayri testlerde sinaniyor.
    n.LIDAR_ON_OFSET_M = 0.0
    for k, v in params.items():
        setattr(n, k, v)
    sonuc = {}
    n.publish_obstacle_status = lambda d, dist: sonuc.update(tespit=d, mesafe=dist)
    n.scan_callback(scan)
    n.destroy_node()
    return sonuc


# ---------------- varsayilanlar ----------------

def test_varsayilan_120_derece(mod):
    n = mod.LidarObstacleDetector()
    assert n.SECTOR_WIDTH_DEG == 120.0
    assert n.CORRIDOR_WIDTH_M > 0
    n.destroy_node()


def test_montaj_yonu_180(mod):
    """Bu araçta LiDAR ters monte (0° işareti arkaya bakıyor).
    20 Ağustos'ta araç üzerinde canlı görüntüyle doğrulandı."""
    n = mod.LidarObstacleDetector()
    assert n.FORWARD_ANGLE_DEG == 180.0
    n.destroy_node()


# ---------------- koridor: asil mesele ----------------

def test_tam_onde_engel_tespit_edilir(mod):
    s = calistir(mod, tarama([(0.0, 3.0)]))
    assert s['tespit']
    assert s['mesafe'] == pytest.approx(3.0, abs=0.05)


def test_yandaki_nesne_koni_icinde_ama_koridor_disinda(mod):
    """+50°, 3 m => yanal 2.30 m. Koni içinde GÖRÜLÜR ama koridorda değil,
    tepki VERİLMEMELİ. Koridor filtresinin varlık sebebi bu."""
    s = calistir(mod, tarama([(50.0, 3.0)]), CORRIDOR_WIDTH_M=1.2)
    assert not s['tespit']


def test_ayni_nesne_koridor_kapaliyken_yanlis_alarm(mod):
    """Koridor kapatılırsa aynı yan nesne engel sayılır - filtrenin
    gerçekten iş yaptığının kanıtı."""
    s = calistir(mod, tarama([(50.0, 3.0)]), CORRIDOR_WIDTH_M=0.0)
    assert s['tespit']


def test_koridor_sinirinda_iceride(mod):
    """Yanal 0.5 m, koridor 1.2 m (yarisi 0.6) => İÇERİDE."""
    aci = math.degrees(math.asin(0.5 / 3.0))
    s = calistir(mod, tarama([(aci, 3.0)]), CORRIDOR_WIDTH_M=1.2)
    assert s['tespit']


def test_koridor_sinirinda_disarida(mod):
    """Yanal 0.9 m => koridor yarısı 0.6'nın dışında."""
    aci = math.degrees(math.asin(0.9 / 3.0))
    s = calistir(mod, tarama([(aci, 3.0)]), CORRIDOR_WIDTH_M=1.2)
    assert not s['tespit']


def test_genis_koridor_daha_cok_yakalar(mod):
    aci = math.degrees(math.asin(0.9 / 3.0))
    dar = calistir(mod, tarama([(aci, 3.0)]), CORRIDOR_WIDTH_M=1.2)
    genis = calistir(mod, tarama([(aci, 3.0)]), CORRIDOR_WIDTH_M=2.4)
    assert not dar['tespit'] and genis['tespit']


# ---------------- EGIK MENZIL DEGIL, ILERI MESAFE ----------------

def test_ileri_mesafe_bildirilir_egik_menzil_degil(mod):
    """45°'de 2.0 m eğik menzil => ileri mesafe 1.41 m.
    Eğik menzili bildirmek 'daha uzakta' sanıp GEÇ fren demektir."""
    s = calistir(mod, tarama([(45.0, 2.0)]), CORRIDOR_WIDTH_M=4.0)
    assert s['tespit']
    assert s['mesafe'] == pytest.approx(2.0 * math.cos(math.radians(45)), abs=0.05)
    assert s['mesafe'] < 2.0


def test_en_yakin_ileri_mesafe_secilir(mod):
    """İki engel: biri eğik menzilde daha yakın ama ileri mesafede daha uzak."""
    s = calistir(mod, tarama([(0.0, 2.0), (55.0, 1.9)]), CORRIDOR_WIDTH_M=6.0)
    # 55°'de 1.9 m -> ileri 1.09 m, 0°'de 2.0 m -> ileri 2.0 m
    assert s['mesafe'] == pytest.approx(1.9 * math.cos(math.radians(55)), abs=0.05)


# ---------------- yon parametresi korunuyor ----------------

def test_lidar_dondurulmus_montaj(mod):
    """forward_angle_deg=90 iken koridor da 90° etrafında olmalı."""
    s = calistir(mod, tarama([(90.0, 3.0)]),
                 FORWARD_ANGLE_DEG=90.0, CORRIDOR_WIDTH_M=1.2)
    assert s['tespit']
    ileri = calistir(mod, tarama([(0.0, 3.0)]),
                     FORWARD_ANGLE_DEG=90.0, CORRIDOR_WIDTH_M=1.2)
    assert not ileri['tespit']


# ---------------- gecersiz veri ----------------

def test_bos_koridorda_engel_yok(mod):
    s = calistir(mod, tarama([(50.0, 3.0), (-50.0, 3.0)]), CORRIDOR_WIDTH_M=1.2)
    assert not s['tespit']


def test_cok_yakin_gurultu_elenir(mod):
    """min_valid_distance altındaki okumalar aracın kendi gövdesidir."""
    s = calistir(mod, tarama([(0.0, 0.03)]), CORRIDOR_WIDTH_M=1.2)
    assert not s['tespit']


def test_esik_disindaki_engel_tespit_degil(mod):
    s = calistir(mod, tarama([(0.0, 12.0)]),
                 CORRIDOR_WIDTH_M=1.2, OBSTACLE_THRESHOLD=5.0)
    assert not s['tespit']


def test_genis_acida_govde_yansimasi_elenir(mod):
    """REGRESYON: min_valid_distance İLERİ MESAFEYE de uygulanmalı.

    60°'de 0.15 m eğik menzil = 0.075 m ileri mesafe. Alt sınır yalnızca
    eğik menzile uygulanırsa bu okuma geçer ve aracın kendi tamponundan
    gelen yansıma 'engel' olarak raporlanır - geniş açıda sürekli yanlış
    acil duruş demektir."""
    s = calistir(mod, tarama([(60.0, 0.15)]),
                 CORRIDOR_WIDTH_M=1.2, MIN_VALID_DISTANCE=0.10)
    assert not s['tespit'], 'gövde yansıması engel sayılmamalı'


def test_bildirilen_mesafe_alt_sinirin_altina_inmez(mod):
    """Koridor modunda raporlanan ileri mesafe her zaman alt sınırın
    üstünde olmalı; aksi halde eşik mantığı anlamsızlaşır."""
    s = calistir(mod, tarama([(0.0, 3.0), (55.0, 0.2), (-58.0, 0.18)]),
                 CORRIDOR_WIDTH_M=1.2, MIN_VALID_DISTANCE=0.10)
    assert s['mesafe'] > 0.10


# ---------------- LiDAR -> tampon payi ----------------

def test_mesafe_tampondan_bildirilir(mod):
    """Sensör 3.0 m ölçüyor, LiDAR tampondan 0.5 m geride
    -> tampona kalan 2.5 m bildirilmeli.

    Bu düşüm yapılmazsa acil duruş eşiği 0.5 m GEÇ devreye girer."""
    s = calistir(mod, tarama([(0.0, 3.0)]),
                 CORRIDOR_WIDTH_M=1.5, LIDAR_ON_OFSET_M=0.5)
    assert s['mesafe'] == pytest.approx(2.5, abs=0.05)


def test_ofset_sifirken_sensor_mesafesi(mod):
    s = calistir(mod, tarama([(0.0, 3.0)]),
                 CORRIDOR_WIDTH_M=1.5, LIDAR_ON_OFSET_M=0.0)
    assert s['mesafe'] == pytest.approx(3.0, abs=0.05)


def test_ofset_esigi_erkene_ceker(mod):
    """Sensörden 5.2 m = tampondan 4.7 m. Eşik 5.0 m ise:
    ofsetsiz tespit YOK, ofsetli tespit VAR. Payın bütün mesele olduğu yer."""
    ofsetsiz = calistir(mod, tarama([(0.0, 5.2)]), CORRIDOR_WIDTH_M=1.5,
                        LIDAR_ON_OFSET_M=0.0, OBSTACLE_THRESHOLD=5.0)
    ofsetli = calistir(mod, tarama([(0.0, 5.2)]), CORRIDOR_WIDTH_M=1.5,
                       LIDAR_ON_OFSET_M=0.5, OBSTACLE_THRESHOLD=5.0)
    assert not ofsetsiz['tespit']
    assert ofsetli['tespit']


def test_temas_negatife_dusmez(mod):
    """Engel LiDAR'dan 0.3 m, ofset 0.5 m -> negatif olamaz.

    0.0'a DEGIL kucuk bir POZITIF degere kirpilir: karar dugumu
    /obstacle_distance'ta pozitif olmayan degeri "veri yok" sayip mesafeyi
    SONSUZ yapiyor. 0.0 yayinlanirsa tampona DEGMIS engel "sonsuz uzakta"
    gorunup acil durusu hic tetiklemez - 20 Agustos'ta canli gozlendi."""
    s = calistir(mod, tarama([(0.0, 0.3)]),
                 CORRIDOR_WIDTH_M=1.5, LIDAR_ON_OFSET_M=0.5,
                 MIN_VALID_DISTANCE=0.10)
    assert s['mesafe'] > 0.0, "sifir yayinlanirsa karar dugumu SONSUZ okur"
    assert s['mesafe'] < 0.05, "temas mesafesi cok kucuk olmali"
    assert s['tespit']


def test_varsayilan_koridor_arac_genisligine_uygun(mod):
    """Araç 1.30 m; koridor bundan dar olursa yolun ortasındaki engeli kaçırır."""
    n = mod.LidarObstacleDetector()
    assert n.CORRIDOR_WIDTH_M >= 1.30
    assert n.LIDAR_ON_OFSET_M > 0.0
    n.destroy_node()
