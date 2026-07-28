# Dosya Adı: launch_all_nodes.py 

import multiprocessing
import time
import os
import sys
import importlib.util

# --- DÜĞÜM BAŞLATMA SIRASI VE DOSYA YOLLARI ---
# Yollar, bu launch dosyasının bulunduğu klasöre (SahiOtonom) göre otomatik hesaplanır.
# Böylece projeyi başka bir yere taşısan bi
# 
# le yolları elle güncellemene gerek kalmaz.
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

NODE_LAUNCH_ORDER = [
    {
        "name": "KAMERA YAYINI (ZED)",
        "file_path": os.path.join(BASE_DIR, "Kamera/zedi2connect_port.py"),
        "delay_after": 2
    },
        {
        "name": "ŞERİT TESPİT",
        "file_path": os.path.join(BASE_DIR, "SeritTespit/serit-tespitcopy.py"),
        "delay_after": 2
    },
    {
        "name": "LEVHA TESPİT (Görüntü İşleme)",
        "file_path": os.path.join(BASE_DIR, "GoruntuIsleme/run_tracker.py"),
        "delay_after": 3
    },
    {
        # Şerit + levha görüntülerini TEK pencerede yan yana gösterir
        "name": "BİRLEŞİK GÖRÜNTÜ (Yol + Levhalar)",
        "file_path": os.path.join(BASE_DIR, "GoruntuIsleme/combined_view.py"),
        "delay_after": 1
    },
    {
        # NOT: Bu düğüm /scan topic'ini dinler. Lidar donanım sürücüsü ayrıca
        # başlatılmalıdır: ros2 launch sllidar_ros2 sllidar_XX_launch.py serial_port:=/dev/ttyUSB0
        "name": "LIDAR ENGEL TESPİTİ",
        "file_path": os.path.join(BASE_DIR, "EngelTespit/engel-tespit.py"),
        "delay_after": 3
    },
    {
        "name": "KARAR ALMA ALGORİTMASI",
        "file_path": os.path.join(BASE_DIR, "KararAlg/basic-decision-making-node.py"),
        "delay_after": 5
    },
    {
        "name": "HABERLEŞME (UART)",
        "file_path": os.path.join(BASE_DIR, "Haberlesme/uart_sender_node3.py"),
        "delay_after": 2
    },
]

def run_script(script_path):
    """Verilen tam yoldaki Python script'ini çalıştırır."""
    try:
        # Script'in bulunduğu dizini Python'un arama yoluna ekle
        # Bu, script'in kendi içindeki 'utils' gibi yerel importları bulabilmesi için KRİTİK!
        script_dir = os.path.dirname(script_path)
        sys.path.insert(0, script_dir)
        
        print(f"[{os.getpid()}] Proses başlatıldı: {os.path.basename(script_path)}")
        
        module_name = os.path.splitext(os.path.basename(script_path))[0]
        spec = importlib.util.spec_from_file_location(module_name, script_path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        
        if hasattr(module, 'main'):
            module.main()
        else:
            print(f"HATA: {script_path} içinde 'main' fonksiyonu bulunamadı.", file=sys.stderr)
            
        # Eklenen yolu temizle
        sys.path.pop(0)

    except Exception as e:
        print(f"[{os.getpid()}] HATA ({os.path.basename(script_path)}): {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    processes = []
    
    print("--- OTONOM SİSTEM SIRAYLA BAŞLATILIYOR (Farklı Klasörlerden) ---")

    try:
        for node_info in NODE_LAUNCH_ORDER:
            node_name = node_info["name"]
            path = node_info["file_path"]
            delay = node_info["delay_after"]
            
            if not os.path.exists(path):
                print(f"\n‼️ UYARI: '{node_name}' için dosya bulunamadı, atlanıyor: {path}")
                continue
            
            print(f"\n▶️  '{node_name}' başlatılıyor...")
            process = multiprocessing.Process(target=run_script, args=(path,))
            processes.append(process)
            process.start()
            
            print(f"    PID {process.pid} ile başlatıldı. Sonraki düğüm için {delay} saniye bekleniyor...")
            time.sleep(delay)

        print("\n✅ Tüm düğümler başarıyla ve sırayla başlatıldı.")
        print("Kapatmak için terminalde CTRL+C tuşlarına basın.")

        for process in processes:
            process.join()

    except KeyboardInterrupt:
        print("\n\n--- KAPATMA SİNYALİ ALINDI (CTRL+C) ---")
        print("Tüm düğümler sonlandırılıyor...")
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(timeout=2)
        
        for process in processes:
            if process.is_alive():
                print(f"PID {process.pid} zorla kapatılıyor...")
                process.kill()
                process.join()

        print("🛑 Tüm işlemler güvenli bir şekilde sonlandırıldı.")