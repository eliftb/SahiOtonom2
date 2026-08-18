#!/bin/bash
# SAHI OTONOM - USB baglanti kontrolu
#
# ZED icin "cihaz gorunuyor mu" YETERLI DEGIL: kamera USB 2.0 hattina takilinca
# sadece HID arayuzu aciliyor, video (UVC) arayuzu hic olusmuyor. Cihaz lsusb'de
# gorunur ama SDK "CAMERA NOT DETECTED" der. Bu yuzden burada VIDEO ARAYUZU
# aranir, sadece vendor id degil.

zed_dir=""
for d in /sys/bus/usb/devices/*/; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = "2b03" ] && zed_dir="$d"
done

echo "=== CIHAZLAR ==="
for id in "1a86:Arduino (CH340)" "10c4:LiDAR (CP2102)"; do
    vid="${id%%:*}"; ad="${id#*:}"
    bulundu="YOK"
    for d in /sys/bus/usb/devices/*/; do
        [ -f "$d/idVendor" ] || continue
        if [ "$(cat "$d/idVendor")" = "$vid" ]; then
            bulundu="BAGLI ($(cat "$d/speed") Mbps)"
        fi
    done
    printf "  %-20s %s\n" "$ad" "$bulundu"
done

if [ -n "$zed_dir" ]; then
    printf "  %-20s BAGLI (%s Mbps)\n" "ZED 2i" "$(cat "$zed_dir/speed")"
else
    printf "  %-20s YOK\n" "ZED 2i"
fi

echo
echo "=== ZED KARARI ==="
if [ -z "$zed_dir" ]; then
    echo "  ZED USB'de hic gorunmuyor. Kabloyu/portu kontrol et."
    exit 1
fi

# Video arayuzu var mi? (UVC surucusu baglanmis bir alt-arayuz)
video_var=0
for v in /sys/class/video4linux/*/; do
    [ -e "$v/name" ] || continue
    # ZED'in video dugumu, ZED usb cihazinin altinda olmali
    if readlink -f "$v" | grep -q "$(basename "$zed_dir")"; then
        video_var=1
        echo "  video dugumu: /dev/$(basename "$v") ($(cat "$v/name"))"
    fi
done

hiz=$(cat "$zed_dir/speed")
if [ "$video_var" = "1" ] && [ "$hiz" -ge 5000 ] 2>/dev/null; then
    echo "  ✅ $hiz Mbps + video arayuzu var. ZED calisir."
elif [ "$video_var" = "1" ]; then
    echo "  ⚠️  Video arayuzu var ama hiz $hiz Mbps (USB 3.0 degil)."
    echo "     HD720@30 kopabilir. USB 3.0 portuna almak gerekir."
else
    echo "  ❌ SADECE HID ARAYUZU ACILMIS, VIDEO ARAYUZU YOK ($hiz Mbps)."
    echo "     Kamera bu baglantida ACILMAZ - SDK 'CAMERA NOT DETECTED' der."
    echo "     Sebep: ZED USB 2.0 hattina bagli (hub ya da port USB 2.0)."
    echo "     COZUM: ZED'i dogrudan laptopun USB 3.0 (USB-A) portuna tak."
fi

echo
echo "=== USB YOLU (ZED neyin arkasinda?) ==="
ust=$(basename "$zed_dir" | sed 's/\.[0-9]*$//')
if [ -d "/sys/bus/usb/devices/$ust" ] && [ "$ust" != "$(basename "$zed_dir")" ]; then
    echo "  ZED bir HUB arkasinda: $ust  $(cat /sys/bus/usb/devices/$ust/idVendor):$(cat /sys/bus/usb/devices/$ust/idProduct) @ $(cat /sys/bus/usb/devices/$ust/speed) Mbps"
    echo "  (hub 480 Mbps ise USB 2.0'dir; ZED buradan calismaz)"
else
    echo "  ZED dogrudan anakarta bagli (hub yok)."
fi
