#!/bin/bash
# ARACI DURDUR - sistem ve kayit calismaya devam eder.
# Olcum sahnesi cekerken (kutu koyarken, araci yerlestirirken) kullanin.
set +u
source /opt/ros/humble/setup.bash 2>/dev/null

CIKTI=$(ros2 param set /decision_making_node base_speed 0.0 2>&1)
echo "$CIKTI"
echo

# ros2 param set, dugum bulunamasa bile 0 donebiliyor. Bu bir DURDURMA komutu:
# gercekten uygulandigini dogrulamadan "durduruldu" demek tehlikeli - araci
# durdu sanip onune girersiniz.
if ! echo "$CIKTI" | grep -qi "successful"; then
    echo "  ############################################################"
    echo "  #  DURDURULAMADI - ARAC HALA HAREKET EDEBILIR              #"
    echo "  ############################################################"
    echo
    echo "  Olasi sebep:"
    echo "    - sistem calismiyor  ->  python3 launch_all_nodes.py"
    echo "    - karar dugumu acilmadi (loglara bakin)"
    echo
    echo "  ARACIN ONUNE GIRMEDEN ONCE MOTOR GUCUNU KESIN."
    exit 1
fi

echo "  ARAC DURDURULDU. Kayit ve algilama calismaya devam ediyor."
echo "  Devam etmek icin:  ./devam.sh"
echo
echo "  UYARI: bu YAZILIMSAL durdurmadir. Aracin onune gireceksiniz,"
echo "         motor gucunu de kesin / acil durdurmayi kullanin."
