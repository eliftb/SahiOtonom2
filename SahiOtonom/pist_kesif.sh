#!/bin/bash
# PIST KESIF - ARAC OLMADAN veri toplama.
#
# Sadece ALGI dugumlerini calistirir: kamera, serit tespiti, levha tespiti,
# gorsellestirme ve KAYIT. Arac dugumleri (UART, karar alma) baslatilmaz -
# arac yokken onlar sadece hata basar ve logu kirletir.
#
# AMAC: pistin gercek goruntusunu kaydetmek. Kamerayi aracin monte edilecegi
# YUKSEKLIKTE ve ACIDA elde tutup parkuru yavas yuruyun. Sonra bu kayit
# uzerinden laboratuvarda calisiriz.
#
# KULLANIM:  ./pist_kesif.sh
#            CTRL+C ile bitir

set +u
source /opt/ros/humble/setup.bash 2>/dev/null
set -u

KOK="$(cd "$(dirname "$0")" && pwd)"
KLASOR="$KOK/kayitlar"
mkdir -p "$KLASOR"
AD="kesif_$(date +%Y%m%d_%H%M%S)"

PIDLER=()
temizle() {
    echo
    echo "Kapatiliyor... (kayit sikistiriliyor, BEKLEYIN)"
    for p in "${PIDLER[@]}"; do
        kill -INT "$p" 2>/dev/null
    done

    # Kayit kapanirken parcayi SIKISTIRIYOR; buyuk dosyada bu uzun surer.
    # Erken oldururseniz metadata.yaml yazilmaz ve kayit ACILAMAZ hale gelir
    # (kurtarmak icin 'ros2 bag reindex' gerekir). O yuzden 90 sn'ye kadar
    # sabirla bekliyoruz.
    for i in $(seq 1 90); do
        kalan=0
        for p in "${PIDLER[@]}"; do
            kill -0 "$p" 2>/dev/null && kalan=$((kalan+1))
        done
        [ "$kalan" -eq 0 ] && break
        [ $((i % 10)) -eq 0 ] && echo "  hala kapaniyor ($kalan surec, ${i} sn)..."
        sleep 1
    done

    for p in "${PIDLER[@]}"; do
        kill -9 "$p" 2>/dev/null
    done

    echo
    if [ -f "$KLASOR/$AD/metadata.yaml" ]; then
        echo "  KAYIT TAMAM: $KLASOR/$AD"
        du -sh "$KLASOR/$AD" 2>/dev/null | sed 's/^/  boyut: /'
    else
        echo "  UYARI: metadata.yaml yok - kayit duzgun kapanmamis."
        echo "  Kurtarmak icin:  ros2 bag reindex $KLASOR/$AD"
    fi
}
trap temizle INT TERM

echo "=============================================="
echo "  PIST KESIF (arac yok)"
echo "=============================================="
echo "  kayit: $KLASOR/$AD"
echo

echo "-> Kamera..."
( cd "$KOK/Kamera" && python3 zedi2connect_port.py ) &
PIDLER+=($!)
sleep 8

echo "-> Serit tespiti..."
( cd "$KOK/SeritTespit" && python3 serit-tespitcopy.py ) &
PIDLER+=($!)
sleep 3

echo "-> Levha tespiti..."
( cd "$KOK/GoruntuIsleme" && python3 run_tracker.py ) &
PIDLER+=($!)
sleep 3

echo "-> Birlesik goruntu..."
( cd "$KOK/GoruntuIsleme" && python3 combined_view.py ) &
PIDLER+=($!)
sleep 2

echo "-> Kayit basliyor..."
ros2 bag record -o "$KLASOR/$AD" \
    --max-bag-size 2000000000 \
    --compression-mode file --compression-format zstd \
    /zed2i_rgb/image_raw /zed2i/odom /zed2i/depth /zed2i/camera_info &
PIDLER+=($!)

echo
echo "=============================================="
echo "  HAZIR - kayit aliniyor"
echo "=============================================="
echo "  Kamerayi aracin monte edilecegi YUKSEKLIKTE tutun."
echo
echo "  YAPILACAKLAR:"
echo "   1. Her LEVHANIN onunde 5 sn durun (ekranda taniyor mu bakin)"
echo "   2. Kirmizi isiktan TAM 4 m uzakta 10 sn durun"
echo "   3. Yesil isikta da 5 sn durun"
echo "   4. Seridin ortasinda durup boyunca yavas yuruyun"
echo "   5. Virajlari ve kavsaklari yuruyerek gecin"
echo
echo "  Bitirmek icin CTRL+C"
echo

wait
