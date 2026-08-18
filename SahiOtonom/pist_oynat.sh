#!/bin/bash
# PIST KAYDINI TEKRAR OYNAT - kamera/LiDAR olmadan, laboratuvarda.
#
# Kayit ham sensor verisi oldugu icin, tekrar oynatirken serit/levha/engel
# tespiti YENIDEN calisir. Yani parametreleri degistirip ayni sahnede sonucu
# gorebilirsiniz - pistte tur atmadan.
#
# KULLANIM:
#   ./pist_oynat.sh                  -> kayitlari listele
#   ./pist_oynat.sh pist_2026...     -> oynat (dongusel)
#   ./pist_oynat.sh <ad> 0.5         -> yarim hizda oynat (detayli inceleme)
#
# AYRI BIR TERMINALDE isleme dugumlerini baslatin (kamera dugumu OLMADAN):
#   cd SeritTespit && python3 serit-tespitcopy.py
#   cd GoruntuIsleme && python3 combined_view.py
# Boylece kayittaki goruntu, canli kameraymis gibi islenir.

set -u
KOK="$(cd "$(dirname "$0")" && pwd)"
KLASOR="$KOK/kayitlar"

# ROS setup betikleri tanimsiz degisken kullaniyor; 'set -u' ile birlikte betik
# sessizce oluyor. Sadece bu satir icin kapatiyoruz.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
set -u

if [ $# -eq 0 ]; then
    echo "Mevcut kayitlar:"
    if [ -d "$KLASOR" ] && [ -n "$(ls -A "$KLASOR" 2>/dev/null)" ]; then
        for d in "$KLASOR"/*/; do
            ad=$(basename "$d")
            boyut=$(du -sh "$d" 2>/dev/null | cut -f1)
            printf "  %-40s %s\n" "$ad" "$boyut"
        done
    else
        echo "  (kayit yok - once ./pist_kayit.sh calistirin)"
    fi
    exit 0
fi

AD="$1"
HIZ="${2:-1.0}"
HEDEF="$KLASOR/$AD"

if [ ! -d "$HEDEF" ]; then
    echo "HATA: kayit bulunamadi: $HEDEF"
    echo "Mevcutlari gormek icin argumansiz calistirin."
    exit 1
fi

echo "Oynatiliyor: $AD   (hiz ${HIZ}x, dongusel)"
echo "Isleme dugumlerini AYRI terminalde baslatmayi unutmayin."
echo "Durdurmak icin CTRL+C."
echo

# --loop: ayni sahneyi tekrar tekrar gorursunuz, parametre degisikliginin
# etkisini ayni goruntude karsilastirmak icin.
ros2 bag play "$HEDEF" --rate "$HIZ" --loop
