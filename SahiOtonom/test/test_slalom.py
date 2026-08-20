#!/usr/bin/env python3
"""Slalom durum makinesi testleri — donanımsız, ROS'suz.

En kritik üç kontrol:
  * İLK engelde durur, SONRAKİLERDE durmaz
  * Her engelde şerit DEĞİŞİR (sağ→sol→sağ→sol...)
  * Kaydırma İŞARETİ doğru (yanlış işaret aracı ters yöne kırar)
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Slalom'))
from slalom_durum import SlalomMakinesi, Durum, Serit  # noqa: E402


@pytest.fixture
def m():
    return SlalomMakinesi(manevra_tetik_m=3.0, durma_suresi_s=2.0,
                          gecis_suresi_s=3.0, gecis_sapmasi=0.6,
                          gecis_sonrasi_bekleme_s=1.0)


def kos(m, t, engel=False, mes=None, hiz=0.0):
    return m.adim(t, engel, mes, hiz)


def _engeli_gec(m, t0, dur=False):
    """Bir engeli baştan sona geçir; geçişin bittiği anı döndür."""
    if dur:
        kos(m, t0, engel=True, mes=2.5, hiz=0.0)          # DURDU
        kos(m, t0 + 0.1, engel=True, mes=2.5, hiz=0.0)    # sayaç başlar
        kos(m, t0 + 2.2, engel=True, mes=2.5, hiz=0.0)    # GECIS
        gecis_bas = t0 + 2.2
    else:
        kos(m, t0, engel=True, mes=2.5, hiz=1.0)          # GECIS (durmadan)
        gecis_bas = t0
    return kos(m, gecis_bas + 3.1, engel=False, hiz=1.0)  # geçiş bitti


# ---------------- temel ----------------

def test_baslangic(m):
    c = kos(m, 0.0)
    assert c.durum is Durum.TAKIP and c.serit is Serit.SAG
    assert c.kaydirma == 0.0


def test_uzak_engel_tetiklemez(m):
    c = kos(m, 0.0, engel=True, mes=8.0)
    assert c.durum is Durum.TAKIP and c.kaydirma == 0.0


def test_tetik_acil_durustan_uzakta(m):
    """Manevra acil duruş eşiğinden (1.5 m) ÖNCE başlamalı, yoksa araç
    geçişe başlayamadan fren yapar."""
    assert m.tetik > 1.5


# ---------------- 1. engel: DURUR ----------------

def test_ilk_engel_durdurur(m):
    assert kos(m, 0.0, engel=True, mes=2.5).durum is Durum.DURDU


def test_arac_durmadan_bekleme_baslamaz(m):
    kos(m, 0.0, engel=True, mes=2.5, hiz=1.0)
    for t in (1.0, 3.0, 5.0):
        assert kos(m, t, engel=True, mes=2.5, hiz=1.0).durum is Durum.DURDU


def test_durunca_sure_dolar_ve_gecer(m):
    kos(m, 0.0, engel=True, mes=2.5, hiz=1.0)
    kos(m, 1.0, engel=True, mes=2.5, hiz=0.0)      # durdu, sayaç t=1.0
    assert kos(m, 2.5, engel=True, mes=2.5, hiz=0.0).durum is Durum.DURDU
    assert kos(m, 3.1, engel=True, mes=2.5, hiz=0.0).durum is Durum.GECIS


# ---------------- 2. ve SONRAKİ engeller: DURMAZ ----------------

def test_ikinci_engelde_durmaz(m):
    _engeli_gec(m, 0.0, dur=True)
    c = kos(m, 20.0, engel=True, mes=2.5, hiz=1.0)
    assert c.durum is Durum.GECIS, "ikinci engelde DURMAMALI"


def test_ucuncu_ve_dorduncu_engel_de_calisir(m):
    """SINIRSIZ koni: önceki sürüm ikiden sonra hiçbir şey yapmıyordu."""
    _engeli_gec(m, 0.0, dur=True)
    for i, t in enumerate((20.0, 40.0, 60.0, 80.0), start=2):
        c = kos(m, t, engel=True, mes=2.5, hiz=1.0)
        assert c.durum is Durum.GECIS, f"{i}. engelde geçiş olmalı"
        c = _engeli_gec(m, t, dur=False) if False else kos(m, t + 3.1, engel=False, hiz=1.0)
        assert c.durum is Durum.TAKIP
    assert c.gecilen_engel == 5


# ---------------- şerit sırası ----------------

def test_seritler_sirayla_degisir(m):
    assert kos(m, 0.0).serit is Serit.SAG
    c = _engeli_gec(m, 0.0, dur=True)
    assert c.serit is Serit.SOL, "1. engelden sonra SOL"
    c = _engeli_gec(m, 20.0)
    assert c.serit is Serit.SAG, "2. engelden sonra SAĞ"
    c = _engeli_gec(m, 40.0)
    assert c.serit is Serit.SOL, "3. engelden sonra SOL"


# ---------------- kaydırma işareti ----------------

def test_sagdan_sola_kaydirma_negatif(m):
    """PID sözleşmesi: POZİTİF = sağa kır. Sola gitmek NEGATİF olmalı."""
    kos(m, 0.0, engel=True, mes=2.5, hiz=0.0)
    kos(m, 0.1, engel=True, mes=2.5, hiz=0.0)
    c = kos(m, 2.2, engel=True, mes=2.5, hiz=0.0)
    assert c.durum is Durum.GECIS
    assert c.kaydirma == pytest.approx(-0.6)


def test_soldan_saga_kaydirma_pozitif(m):
    _engeli_gec(m, 0.0, dur=True)                  # artık SOL şeritte
    c = kos(m, 20.0, engel=True, mes=2.5, hiz=1.0)
    assert c.kaydirma == pytest.approx(+0.6)


def test_takipte_kaydirma_yok(m):
    c = _engeli_gec(m, 0.0, dur=True)
    assert c.durum is Durum.TAKIP and c.kaydirma == 0.0


# ---------------- geçiş sonrası soğuma ----------------

def test_gecis_sonrasi_ayni_koni_zikzak_yaptirmaz(m):
    """Geçiş biter bitmez aynı koni hâlâ görünüyorsa araç anında geri
    dönmeye kalkmamalı - zikzak yapardı."""
    c = _engeli_gec(m, 0.0, dur=True)              # geçiş t=5.3'te bitti
    c = kos(m, 5.5, engel=True, mes=2.0, hiz=1.0)  # soğuma içinde
    assert c.durum is Durum.TAKIP, "soğuma süresinde yeni geçiş başlamamalı"


def test_sogumadan_sonra_tekrar_tetiklenir(m):
    _engeli_gec(m, 0.0, dur=True)
    c = kos(m, 8.0, engel=True, mes=2.0, hiz=1.0)  # soğuma bitti
    assert c.durum is Durum.GECIS


# ---------------- sıfırlama / seçenekler ----------------

def test_sifirla(m):
    _engeli_gec(m, 0.0, dur=True)
    m.sifirla()
    c = kos(m, 50.0)
    assert c.durum is Durum.TAKIP and c.serit is Serit.SAG
    assert c.gecilen_engel == 0 and c.kaydirma == 0.0


def test_ilk_engelde_durmama_secenegi():
    mm = SlalomMakinesi(ilk_engelde_dur=False, manevra_tetik_m=3.0)
    assert mm.adim(0.0, True, 2.5, 1.0).durum is Durum.GECIS


def test_soldan_baslama_secenegi():
    mm = SlalomMakinesi(baslangic_serit=Serit.SOL, ilk_engelde_dur=False)
    c = mm.adim(0.0, True, 2.5, 1.0)
    assert c.kaydirma > 0, "sol şeritten başlayınca ilk geçiş SAĞA olmalı"


def test_durum_degisimi_bir_kez_bildirilir(m):
    assert kos(m, 0.0, engel=True, mes=2.5, hiz=0.0).degisti
    assert not kos(m, 0.1, engel=True, mes=2.5, hiz=0.0).degisti
