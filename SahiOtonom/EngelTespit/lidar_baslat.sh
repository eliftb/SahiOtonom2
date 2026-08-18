#!/bin/bash
# LIDAR SURUCUSUNU RESET ILE BASLAT.
#
# NEDEN: RPLIDAR besleme sinirinda calisiyor; motor yuku artinca cihaz kendini
# koruma moduna aliyor ve 'health status 2' MANDALLI kaliyor. Bu mandali surucu
# temizlemiyor - yeniden baslatmak ise yaramiyor, cihaza RESET komutu gitmesi
# gerekiyor. Bu betik once reset atar, sonra surucuyu baslatir. Boylece
# launcher'in yeniden baslatma dongusu cihazi kendi kendine toparlayabilir.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null

KOK="$(cd "$(dirname "$0")" && pwd)"
PORT="${1:-}"
BAUD="${2:-1000000}"

# Python tarafi port bulamayinca argumani "None" METNI olarak gonderebiliyor;
# bos string gibi degerlendirilmeli, yoksa surucu var olmayan bir porta
# baglanmaya calisip 'buffer overflow' ile cokuyor.
if [ "$PORT" = "None" ] || [ "$PORT" = "none" ]; then
    PORT=""
fi
if [ -z "$PORT" ]; then
    PORT=$(ls /dev/sahi_lidar /dev/serial/by-id/*CP2102* 2>/dev/null | head -1)
fi
if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    echo "[lidar] Port bulunamadi - LiDAR takili mi?"
    echo "[lidar] Kontrol: ls /dev/serial/by-id/ | grep -i cp2102"
    exit 1
fi

# Reset: hata mandalliysa temizler, degilse zararsizdir.
python3 "$KOK/lidar_sifirla.py" "$PORT" 2>&1 | sed 's/^/[lidar-reset] /'

exec ros2 launch rplidar_ros rplidar_s2_launch.py \
    serial_port:="$PORT" serial_baudrate:="$BAUD"
