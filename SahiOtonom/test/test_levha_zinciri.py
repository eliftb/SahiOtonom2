#!/usr/bin/env python3
"""LEVHA -> DAVRANIS ZINCIRI TESTI - ARAC/KAMERA GEREKTIRMEZ (rclpy yeter).

SORU: levhalar tespit ediliyor, peki ARAC ONA GORE HAREKET EDIYOR MU?

Zincir uc halkadan olusuyor ve her halkada sessizce kopabilir:

    1) MODEL ADI      run_tracker.py -> model.names[...] -> 'cls' alani
    2) KARAR TABLOSU  basic-decision-making-node.py -> CLS_*/TURN_SIGNS/DIRECTION_SIGNS
    3) TUKETICI       UART (donus) / serit dugumu (koridor tercihi)

Halka 1-2 arasindaki kopma HICBIR HATA VERMEZ: sign_callback taniyamadigi
sinifi sessizce atlar, ekranda kutu ve etiket gorunmeye devam eder. Yani
"levhayi goruyor" ile "levhaya uyuyor" ayni sey degil - bu test farki olcer.

    python3 SahiOtonom/test/test_levha_zinciri.py
"""
import importlib.util
import json
import os
import re
import sys
import time

KOK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KARAR_SRC = os.path.join(KOK, 'KararAlg', 'basic-decision-making-node.py')
SERIT_SRC = os.path.join(KOK, 'SeritTespit', 'serit-tespitcopy.py')
UART_SRC = os.path.join(KOK, 'Haberlesme', 'uart_sender_node3.py')
MODEL_PT = os.path.join(KOK, 'GoruntuIsleme',
                        'UltraConservative_BEST_mAP0.9248_20250801_115028.pt')

sonuclar = []


def bildir(gecti, baslik, aciklama=''):
    sonuclar.append((gecti, baslik))
    print(f"  {'PASS' if gecti else 'FAIL'}  {baslik}")
    if aciklama:
        for satir in aciklama.strip().splitlines():
            print(f"        {satir}")


def karar_modulu():
    """Karar dugumunu ROS baglamadan modul olarak yukler."""
    spec = importlib.util.spec_from_file_location('karar_dugumu', KARAR_SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def model_sinif_adlari():
    """Egitilmis modelin GERCEK sinif adlari. Okunamazsa None."""
    try:
        import torch
        d = torch.load(MODEL_PT, map_location='cpu', weights_only=False)
        return set(getattr(d['model'], 'names', {}).values())
    except Exception as e:
        print(f'  (model okunamadi: {e})')
        return None


# --- 1. HALKA: model adlari karar tablolariyla ortusuyor mu ------------------

def test_sinif_adlari(mod):
    print('\n1) MODEL SINIF ADI  ->  KARAR TABLOSU')
    adlar = model_sinif_adlari()
    if adlar is None:
        bildir(False, 'model sinif adlari okunamadi (torch yok?)')
        return

    tanimli = set(mod.TURN_SIGNS) | set(mod.DIRECTION_SIGNS) | {
        mod.CLS_RED, mod.CLS_YELLOW, mod.CLS_GREEN, mod.CLS_STOP}
    # Karsilastirma sign_callback'in GORDUGU hale gore yapilir: orada da ad
    # once sadelestiriliyor ('ileriden-saga...' ile 'ileriden-sağa...' ayni ada
    # iner). Ham adlarla kiyaslamak testi gercekte olmayan bir hataya bagirtir.
    sade = {mod.sinif_adini_sadelestir(a) for a in adlar}

    # Karar tablosunda yazip modelde OLMAYAN ad: o kural hic tetiklenemez.
    hayalet = sorted(tanimli - sade)
    bildir(not hayalet,
           'karar tablosundaki her ad modelde var',
           'Modelde OLMAYAN adlar (bu kurallar asla tetiklenmez):\n  '
           + '\n  '.join(hayalet) if hayalet else '')

    # Modelde olup tabloda olmayan MANEVRA levhalari: goruluyor ama etkisiz.
    manevra_kokleri = ('mecburi-yon', 'gidiniz', 'donulmez', 'serit-duzenlemesi')
    etkisiz = sorted(a for a in adlar
                     if any(k in mod.sinif_adini_sadelestir(a) for k in manevra_kokleri)
                     and mod.sinif_adini_sadelestir(a) not in tanimli)
    bildir(not etkisiz,
           'modeldeki her manevra levhasinin bir karsiligi var',
           'Tespit edilip YOK SAYILAN manevra levhalari:\n  '
           + '\n  '.join(etkisiz) if etkisiz else '')


# --- 2. HALKA: karar dugumu gercekten davranis uretiyor mu -------------------

class Yakalayici:
    """Dugumun yayinlarini gercek DDS'e gitmeden yakalar."""
    def __init__(self, node):
        self.hiz = None
        self.taraf = None
        self.donus = None
        node.speed_pub.publish = lambda m: setattr(self, 'hiz', m.data)
        node.side_pub.publish = lambda m: setattr(self, 'taraf', m.data)
        node.turn_pub.publish = lambda m: setattr(self, 'donus', m.data)


def sifirla(node, mod):
    """Dugumu temiz duruma alir.

    ZORUNLU: durum makinesi ZAMANA bagli (sign_memory_sec, red_release_sec).
    Senaryolari arka arkaya calistirmak onceki isigi tazeleyip sonrakini
    kirletiyor - testin kendi kurgusundan dogan sahte sonuc olur.
    """
    node.state = mod.DriveState.SURUYOR
    node.state_since = time.time()
    node.last_red = node.last_green = node.last_stop_sign = None
    node.stop_sign_released_at = 0.0
    node.preferred_side = 0
    node.bekleyen_donus = 0


def levha_ver(node, cls, mesafe_m, track_id=1):
    """Verilen mesafeye denk gelen kutu yuksekligiyle bir tespit besler."""
    yukseklik = node.ref_distance_m * node.ref_box_height_px / mesafe_m
    node.sign_callback(type('M', (), {'data': json.dumps({'boxes': [
        {'x1': 0, 'y1': 0, 'x2': 40, 'y2': yukseklik,
         'cls': cls, 'id': track_id}]})}))


def test_karar_davranisi(mod):
    print('\n2) KARAR DUGUMU: levha -> hiz / donus / taraf')
    import rclpy
    rclpy.init()
    try:
        node = mod.DecisionMakingNode()
        y = Yakalayici(node)

        # 2a. Kirmizi isik -> tam durus
        sifirla(node, mod)
        levha_ver(node, mod.CLS_RED, 3.0)
        node.decision_loop()
        bildir(y.hiz == 0.0 and node.state.name == 'KIRMIZI_BEKLIYOR',
               f'kirmizi isik 3 m -> hiz {y.hiz} (beklenen 0.0)')

        # 2b. Yesil isik -> kalkis (2a'nin kirmizisi hala hafizada, dogru kurgu)
        levha_ver(node, mod.CLS_GREEN, 3.0)
        node.decision_loop()
        bildir(y.hiz == node.manual_speed and node.state.name == 'SURUYOR',
               f'yesil isik -> hiz {y.hiz} (beklenen {node.manual_speed})')

        # 2c. Sari isik -> tabloda yok, hicbir sey olmamali (bilgi amacli)
        sifirla(node, mod)
        levha_ver(node, mod.CLS_YELLOW, 3.0)
        node.decision_loop()
        bildir(y.hiz == node.manual_speed,
               'sari isik -> davranis DEGISMIYOR (bilerek: kural yazilmamis)')

        # 2d. Dur levhasi -> stop_sign_wait_sec boyunca 0
        sifirla(node, mod)
        levha_ver(node, mod.CLS_STOP, 3.0, track_id=7)
        node.decision_loop()
        bildir(y.hiz == 0.0 and node.state.name == 'DUR_BEKLIYOR',
               f'dur levhasi 3 m -> hiz {y.hiz}, '
               f'{node.stop_sign_wait_sec:.0f} sn bekleme')
        node.state_since -= node.stop_sign_wait_sec + 0.1     # sureyi ileri sar
        node.decision_loop()
        bildir(y.hiz == node.manual_speed, 'dur beklemesi bitince kalkiyor')

        # 2e. Ayni 'dur' levhasi (ayni takip id) tekrar durdurmamali.
        # sifirla() handled_stop_ids'e DOKUNMAZ - korunmasi gereken tam da o.
        sifirla(node, mod)
        levha_ver(node, mod.CLS_STOP, 3.0, track_id=7)
        node.decision_loop()
        bildir(y.hiz == node.manual_speed, 'ayni dur levhasinda ikinci kez durmuyor')

        # 2f. Mecburi donus levhalari -> /route/turn
        sifirla(node, mod)
        for cls, beklenen in [('saga-mecburi-yon', +1), ('sola-mecburi-yon', -1),
                              ('ileriden-sola-mecburi-yon', -1),
                              ('ileriden-saga-mecburi-yon', +1),
                              # Modelin GERCEK yazimi - Turkce 'g' ile:
                              ('ileriden-sağa-mecburi-yon', +1),
                              ('ileri-ve-saga-mecburi-yon', +1),
                              ('ileri-ve-sola-mecburi-yon', -1)]:
            node.bekleyen_donus = 0
            levha_ver(node, cls, 5.0)
            node.decision_loop()
            bildir(y.donus == beklenen,
                   f"'{cls}' -> /route/turn = {y.donus} (beklenen {beklenen})")

        # 2g. Yon levhasi -> /route/preferred_side
        sifirla(node, mod)
        levha_ver(node, 'sagdan-gidiniz', 5.0)
        node.decision_loop()
        bildir(y.taraf == +1,
               f"'sagdan-gidiniz' -> /route/preferred_side = {y.taraf}")

        # 2h. UZAK levha durdurmamali (mesafe esigi calisiyor mu)
        sifirla(node, mod)
        levha_ver(node, mod.CLS_RED, node.stop_distance_m * 3)
        node.decision_loop()
        bildir(y.hiz == node.manual_speed,
               f'{node.stop_distance_m * 3:.0f} m uzaktaki kirmizi durdurmuyor')

        node.destroy_node()
    finally:
        rclpy.shutdown()


# --- 3. HALKA: yayinlanan emri TUKETEN var mi -------------------------------

def test_tuketiciler():
    print('\n3) EMIR YAYINLANIYOR -> ONU TUKETEN KOD CALISIYOR MU')
    serit = open(SERIT_SRC, encoding='utf-8').read()
    uart = open(UART_SRC, encoding='utf-8').read()

    # preferred_side yalnizca _corridor_centers_at_rows icinde kullaniliyor; o da
    # yalnizca _route_centers'tan cagriliyor. compute_lateral_deviation ise
    # route_source == 'mesafe' iken DAHA ONCE return ediyor.
    m = re.search(r"declare_parameter\('route_source',\s*'(\w+)'\)", serit)
    kaynak = m.group(1) if m else '?'
    kullanim = re.findall(r'self\.preferred_side\b', serit)
    metrik_erken_return = "if self.route_source == 'mesafe':" in serit and \
                          'return self._mesafe_sapmasi(lane_mask)' in serit
    olu = kaynak == 'mesafe' and metrik_erken_return
    bildir(not olu,
           f"koridor tercihi (/route/preferred_side) surus yolunda tuketiliyor "
           f"[route_source='{kaynak}', {len(kullanim)} kullanim]",
           "compute_lateral_deviation, route_source='mesafe' iken\n"
           "_mesafe_sapmasi ile ERKEN return ediyor; preferred_side'i okuyan\n"
           "_corridor_centers_at_rows bu modda HIC CAGRILMIYOR.\n"
           "Yani 'sagdan/soldan gidiniz' levhalari direksiyonu etkilemiyor."
           if olu else '')

    # Mecburi donus: yalnizca /lane/valid False'a DUSTUGU AN uygulaniyor.
    sadece_kavsakta = 'kayma = math.radians(self.turn_angle_deg) * self.bekleyen_donus' in uart
    bildir(sadece_kavsakta,
           'mecburi donus (/route/turn) UART tarafinda uygulaniyor',
           'NOT: yalnizca /lane/valid True->False gecisinde. Serit hic\n'
           'kaybolmazsa donus yapilmaz ve 12 sn sonra zaman asimiyla silinir.')

    bildir('self.tuketilen_donus' in uart,
           'tuketilen donus mandali var (ayni levhayla ikinci donus engelli)')

    # Kademeli hiz UART'ta ikili sinyale iniyor.
    ikili = 'if speed_ms > 0.1:' in uart
    bildir(not ikili,
           'karar dugumunun kademeli hizi araca kademeli gidiyor',
           "speed_to_digital_signal hizi 0/1'e indiriyor: engel/levha kaynakli\n"
           "0.2-0.8 carpanlarinin hepsi 'h,1' olarak gidiyor. Fiilen yalnizca\n"
           "TAM DUR ile TAM GIT var." if ikili else '')


def uart_dugumu():
    """UART dugumunu __init__ calistirmadan, sadece bu testin ihtiyaci kadar kurar."""
    sys.path.insert(0, os.path.join(KOK, 'test'))
    from test_direksiyon_byte import load_uart_module
    mod = load_uart_module()

    class Log:
        def info(self, m): pass
        def warn(self, m): pass

    n = object.__new__(mod.UartSenderNode)
    n.get_logger = lambda: Log()
    n.bekleyen_donus = 0
    n.tuketilen_donus = 0
    n.serit_gecerli = True
    n.guncel_yaw = 0.0
    n.hedef_yaw = None
    n.turn_angle_deg = 90.0
    n.yon_tutma_basladi = None
    n.yon_tutma_uyarildi = False
    n.integral = 0.0
    n.d_filtered = 0.0
    n.prev_error = 0.0
    n.current_lateral_deviation = 0.0
    n.last_pid_time = None
    return n


def test_donus_mandali():
    print('\n4) DONUS MANDALI: bir levha -> BIR donus')
    import math
    n = uart_dugumu()
    Msg = lambda d: type('M', (), {'data': d})

    # Kavsak: karar dugumu 'saga don' yayinliyor, serit kayboluyor.
    n.donus_callback(Msg(+1))
    n.serit_gecerli_callback(Msg(False))
    ilk_hedef = math.degrees(n.hedef_yaw)
    bildir(abs(ilk_hedef + 90.0) < 1e-6,
           f'1. kavsak: hedef yon {ilk_hedef:+.0f}deg (saga 90)')

    # Serit geri geliyor -> donus tuketildi.
    n.serit_gecerli_callback(Msg(True))
    bildir(n.bekleyen_donus == 0 and n.tuketilen_donus == +1,
           'donus tuketildi, mandal kuruldu')

    # Karar dugumu ayni emri yayinlamaya DEVAM ediyor (12 sn hafiza).
    n.donus_callback(Msg(+1))
    bildir(n.bekleyen_donus == 0, 'ayni emir tekrar gelince yeniden kurulmuyor')

    # Hemen ardindan serit yine kayboluyor (viraj/golge) -> DUZ gecmeli.
    n.guncel_yaw = math.radians(-90.0)      # ilk donus yapildi, arac artik saga bakiyor
    n.serit_gecerli_callback(Msg(False))
    ikinci_hedef = math.degrees(n.hedef_yaw)
    bildir(abs(ikinci_hedef + 90.0) < 1e-6,
           f'2. kayipta ikinci donus YOK, yon korunuyor ({ikinci_hedef:+.0f}deg)')

    # Emir 0'a dusunce mandal serbest: sonraki kavsak yine donebilir.
    n.serit_gecerli_callback(Msg(True))
    n.donus_callback(Msg(0))
    n.donus_callback(Msg(+1))
    bildir(n.bekleyen_donus == +1, 'emir 0a dusunce mandal serbest kaliyor')


def main():
    print(__doc__.splitlines()[0])
    mod = karar_modulu()
    test_sinif_adlari(mod)
    test_karar_davranisi(mod)
    test_tuketiciler()
    test_donus_mandali()

    gecen = sum(1 for g, _ in sonuclar if g)
    print(f'\n{gecen}/{len(sonuclar)} kontrol gecti')
    for gecti, baslik in sonuclar:
        if not gecti:
            print(f'  ! {baslik}')
    return 0 if gecen == len(sonuclar) else 1


if __name__ == '__main__':
    sys.exit(main())
