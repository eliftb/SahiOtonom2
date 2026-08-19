#!/bin/bash
# PIST KAYDI - sinirli pist suresini olcum yaparak degil, KAYIT ALARAK gecirin.
#
# NEDEN: Pistte parametre denemek pahali (sure sinirli, her deneme bir tur).
# Ham sensor verisini kaydederseniz ayni sahneyi laboratuvarda istediginiz kadar
# tekrar oynatip parametreleri orada ayarlarsiniz. Kayit, gercek pistin
# goruntusu/LiDAR'i oldugu icin ayar gercege gore yapilmis olur.
#
# KULLANIM:
#   ./pist_kayit.sh                 -> kayit basla (CTRL+C ile bitir)
#   ./pist_kayit.sh dosya_adi       -> belirli isimle kaydet
#
# Kayitlar: SahiOtonom/kayitlar/  altina yazilir.

set -u
KOK="$(cd "$(dirname "$0")" && pwd)"
KLASOR="$KOK/kayitlar"
mkdir -p "$KLASOR"

AD="${1:-pist_$(date +%Y%m%d_%H%M%S)}"
HEDEF="$KLASOR/$AD"

# ROS setup betikleri tanimsiz degisken kullaniyor; 'set -u' ile birlikte betik
# SESSIZCE oluyordu. Sadece bu satir icin kapatiyoruz.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
set -u

# SADECE HAM SENSOR verileri kaydedilir. Islenmis topicler (lane_detection_output,
# /lane/lateral_deviation ...) BILEREK disarida: onlar mevcut parametrelerle
# uretildi, tekrar oynatirken yeni parametrelerle YENIDEN uretilecekler.
TOPICLER=(
  /zed2i_rgb/image_raw     # kamera - serit ve levha tespitinin girdisi
  /scan                    # LiDAR - engel tespitinin girdisi
  /zed2i/odom              # ZED odometrisi (aciksa)
  # Serit dugumu METRIK modda calisiyor: mesafe olcumu ve viraj tespiti
  # derinlige dayali. Bu ikisi kayitta yoksa tekrar oynatmada viraj HIC
  # olculmez, ayar yapilamaz. Derinlik ~100 MB/sn - kaydi kisa tutun.
  /zed2i/depth
  /zed2i/camera_info
)

echo "=============================================="
echo "  PIST KAYDI"
echo "=============================================="
echo "  hedef : $HEDEF"
echo "  disk  : $(df -h "$KLASOR" | tail -1 | awk '{print $4}') bos"
echo
echo "  Kaydedilecek topicler:"
mevcut=()
for t in "${TOPICLER[@]}"; do
    if ros2 topic list 2>/dev/null | grep -qx "$t"; then
        echo "    ✓ $t"
        mevcut+=("$t")
    else
        echo "    ✗ $t  (yayinlanmiyor, atlanacak)"
    fi
done

if [ ${#mevcut[@]} -eq 0 ]; then
    echo
    echo "  HATA: kaydedilecek topic yok. Sistem calisiyor mu?"
    echo "  Once: python3 launch_all_nodes.py"
    exit 1
fi

echo
echo "  RGB ~83 MB/sn + derinlik ~110 MB/sn = ~190 MB/sn ham (zstd ile ~1/3)."
echo "  Viraj ayari icin TEK VIRAJ, 30-60 sn yeter - tur atmayin."
echo "  Bitirmek icin CTRL+C."
echo "=============================================="
echo

# zstd sikistirma: goruntu icin diski ~3 kat idareli kullanir, CPU maliyeti kabul
# edilebilir. Kayit sirasinda sistem zaten surusle mesgul degil.
ros2 bag record -o "$HEDEF" \
    --compression-mode file --compression-format zstd \
    "${mevcut[@]}"

echo
echo "Kayit bitti: $HEDEF"
echo "Tekrar oynatmak icin:  ./pist_oynat.sh $AD"
