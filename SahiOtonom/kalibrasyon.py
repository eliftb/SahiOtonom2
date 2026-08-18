#!/usr/bin/env python3
"""Kalıcı kalibrasyon değerleri (kalibrasyon.yaml) için okuma/yazma yardımcısı.

NEDEN: 'ros2 param set' ile girilen değerler düğüm kapanınca kaybolur. Kamera
kayması, kaput sınırı, LiDAR montaj açısı gibi değerler ise FİZİKSEL - bir kez
ölçülür ve donanım değişmedikçe aynı kalır. Bunlar kodun içinde varsayılan
olarak durursa her ölçümde kaynak dosya düzenlemek gerekir; burada durursa
her açılışta kendiliğinden yüklenir.

KULLANIM (düğüm içinde):
    KAL = kalibrasyon('lidar_obstacle_detector')
    self.declare_parameter('forward_angle_deg', KAL('forward_angle_deg', 0.0))
"""
import os

try:
    import yaml
except ImportError:      # yaml yoksa sistem yine de çalışsın, varsayılanlar kullanılır
    yaml = None

DOSYA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'kalibrasyon.yaml')

_onbellek = None
_uyarildi = False


def _yukle():
    """Dosyayı bir kez okuyup önbelleğe alır."""
    global _onbellek, _uyarildi
    if _onbellek is not None:
        return _onbellek
    _onbellek = {}
    if yaml is None:
        if not _uyarildi:
            print('[kalibrasyon] PyYAML yok, varsayılan değerler kullanılıyor.')
            _uyarildi = True
        return _onbellek
    try:
        with open(DOSYA, 'r', encoding='utf-8') as f:
            veri = yaml.safe_load(f) or {}
        if isinstance(veri, dict):
            _onbellek = veri
    except FileNotFoundError:
        if not _uyarildi:
            print(f'[kalibrasyon] {DOSYA} yok, varsayılan değerler kullanılıyor.')
            _uyarildi = True
    except Exception as e:
        print(f'[kalibrasyon] {DOSYA} okunamadı ({e}), varsayılanlar kullanılıyor.')
    return _onbellek


def kalibrasyon(dugum_adi):
    """Bir düğüm için değer okuyucu döndürür.

    Dosyada değer yoksa verilen varsayılan kullanılır, yani kalibrasyon.yaml
    silinse bile sistem çalışmaya devam eder.
    """
    veriler = _yukle()
    bolum = veriler.get(dugum_adi, {}) or {}

    def oku(ad, varsayilan):
        if ad not in bolum:
            return varsayilan
        deger = bolum[ad]

        # TÜRKÇE ONDALIK: "-69,0" gibi virgüllü değerler YAML tarafından METİN
        # olarak okunuyor ve sayıya çevrilemiyordu. Değer sessizce varsayılana
        # düşüyor, yani yapılan kalibrasyon HİÇ UYGULANMIYORDU. Virgülü noktaya
        # çevirip kabul ediyoruz - klavye alışkanlığı yüzünden ölçüm kaybolmasın.
        if isinstance(deger, str) and ',' in deger:
            duzeltilmis = deger.replace(',', '.')
            try:
                float(duzeltilmis)
                print(f'[kalibrasyon] {dugum_adi}.{ad}: "{deger}" virgüllü yazılmış, '
                      f'"{duzeltilmis}" olarak okundu.')
                deger = duzeltilmis
            except ValueError:
                pass

        try:
            # Varsayılanın tipini koru: int parametreye float yazılırsa
            # ROS parametre tipi uyuşmazlığı hatası veriyor.
            return type(varsayilan)(deger)
        except (TypeError, ValueError):
            # Bu satır GÖRÜNÜR olmalı: sessizce varsayılana düşmek, kalibrasyon
            # yapıldı sanıp yapılmamış olmaktan çok daha kötü.
            print(f'\n{"!" * 64}')
            print(f'!! KALIBRASYON OKUNAMADI: {dugum_adi}.{ad} = {deger!r}')
            print(f'!! Varsayilan {varsayilan!r} kullanilacak - OLCUMUNUZ UYGULANMIYOR.')
            print(f'!! kalibrasyon.yaml icindeki bu satiri duzeltin.')
            print(f'{"!" * 64}\n')
            return varsayilan

    return oku


def kaydet(dugum_adi, ad, deger):
    """Tek bir kalibrasyon değerini dosyaya yazar (diğerlerini korur)."""
    global _onbellek
    if yaml is None:
        print('[kalibrasyon] PyYAML yok, kaydedilemiyor.')
        return False
    veri = {}
    try:
        with open(DOSYA, 'r', encoding='utf-8') as f:
            veri = yaml.safe_load(f) or {}
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f'[kalibrasyon] Mevcut dosya okunamadı ({e}), üzerine yazılmayacak.')
        return False

    veri.setdefault(dugum_adi, {})[ad] = deger
    try:
        with open(DOSYA, 'w', encoding='utf-8') as f:
            f.write('# SAHI OTONOM - kalici kalibrasyon degerleri\n')
            f.write('# Bu dosya kalibrasyon araclari tarafindan guncellenir.\n')
            f.write('# Elle de duzenlenebilir; dugum adi -> parametre adi -> deger.\n\n')
            yaml.safe_dump(veri, f, allow_unicode=True, sort_keys=True,
                           default_flow_style=False)
        _onbellek = None       # bir sonraki okuma dosyadan gelsin
        print(f'[kalibrasyon] Kaydedildi: {dugum_adi}.{ad} = {deger}')
        return True
    except Exception as e:
        print(f'[kalibrasyon] Yazılamadı: {e}')
        return False
