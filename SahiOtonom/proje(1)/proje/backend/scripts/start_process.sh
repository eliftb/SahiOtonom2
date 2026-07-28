#!/bin/bash

echo "🚀 Şahi Otonom sistemi başlatılıyor..."
echo "Zaman: $(date)"
echo "Process ID: $$"

# Sürekli çalışan bir döngü (örnek)
counter=1
while true; do
    echo "[$counter] Sistem çalışıyor... ($(date))"
    sleep 5
    counter=$((counter + 1))
    
    # 100 iterasyondan sonra bitir (isteğe bağlı)
    if [ $counter -gt 10000 ]; then
        echo "✅ İşlem tamamlandı!"
        break
    fi
done

echo "🔚 Script sonlandı."
