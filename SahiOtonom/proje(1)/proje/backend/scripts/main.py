#!/usr/bin/env python3
import time
import os
import signal
import sys
from datetime import datetime

def signal_handler(sig, frame):
    """Graceful shutdown handler"""
    print(f"\n🛑 Signal {sig} alındı, script sonlandırılıyor...")
    print(f"⏰ Sonlandırma zamanı: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    sys.exit(0)

def main():
    # Signal handler'ları ayarla
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    print("🚀 Şahi Otonom Python Script Başlatıldı!")
    print(f"📍 Process ID: {os.getpid()}")
    print(f"⏰ Başlama zamanı: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 50)
    
    counter = 1
    
    try:
        while True:
            current_time = datetime.now().strftime('%H:%M:%S')
            print(f"[{counter:03d}] 🔄 Sistem çalışıyor... | Zaman: {current_time} | PID: {os.getpid()}")
            
            # Burada gerçek işlemlerinizi yapabilirsiniz
            # Örnek: veri işleme, API çağrıları, dosya operasyonları vb.
            
            time.sleep(5)  # 5 saniye bekle
            counter += 1
            
            # Örnek: 100 iterasyondan sonra otomatik dur (isteğe bağlı)
            if counter > 100:
                print("✅ 100 iterasyon tamamlandı, script sonlanıyor...")
                break
                
    except KeyboardInterrupt:
        print("\n🛑 KeyboardInterrupt alındı!")
    except Exception as e:
        print(f"❌ Hata oluştu: {e}")
    finally:
        print("🔚 Python script sonlandı.")

if __name__ == "__main__":
    main()
