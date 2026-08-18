#!/bin/bash
# ARACI TEKRAR HAREKETE GECIR (dur.sh'in tersi).
# Hiz verilmezse 1.0 m/s kullanilir:   ./devam.sh        ./devam.sh 0.5
set +u
source /opt/ros/humble/setup.bash 2>/dev/null

HIZ="${1:-1.0}"
CIKTI=$(ros2 param set /decision_making_node base_speed "$HIZ" 2>&1)
echo "$CIKTI"
echo
if ! echo "$CIKTI" | grep -qi "successful"; then
    echo "  UYGULANAMADI - sistem calisiyor mu? (python3 launch_all_nodes.py)"
    exit 1
fi
echo "  ARAC HAREKETE HAZIR - hiz $HIZ m/s"
