# Dosya Adı: launch_all_nodes.py
# Konum: /home/sahi/SahiOtonom/launch_all_nodes.py

import multiprocessing
import time
import os
import sys
import importlib.util

# --- DÜĞÜM BAŞLATMA SIRASI VE DOSYA YOLLARI ---
# Her düğümün tam dosya yolunu buraya yazıyoruz.
# Projeyi başka bir yere taşırsan, sadece bu yolları güncellemen yeterli olacaktır.
NODE_LAUNCH_ORDER = [
    {
        "name": "KAMERA YAYINI (ZED)",
        "file_path": "/home/sahi/SahiOtonom/Kamera/zedi2connect_port.py", # <-- TAM DOSYA YOLU
        "delay_after": 5
    },
    {
        "name": "HABERLEŞME (UART) - AVOIDANCE",
        "file_path": "/home/sahi/SahiOtonom/Haberlesme/uart_sender_node3.py", # <-- TEK UART DOSYASI (avoidance surumu kaldirildi)
        "delay_after": 2
    },
    {
        "name": "ŞERİT TESPİT",
        "file_path": "/home/sahi/SahiOtonom/SeritTespit/serit-tespitcopy.py", # <-- TAM DOSYA YOLU
        "delay_after": 3
    },
    {
        "name": "LEVHA TESPİT (Görüntü İşleme)",
        "file_path": "/home/sahi/SahiOtonom/GoruntuIsleme/run_tracker.py", # <-- TAM DOSYA YOLU
        "delay_after": 3
    },
    {
        "name": "LIDAR ENGEL TESPİTİ",
        "file_path": "/home/sahi/SahiOtonom/EngelTespit/engel-tespit.py", # <-- TAM DOSYA YOLU
        "delay_after": 2
    },
    {
        "name": "KARAR ALMA ALGORİTMASI - AVOIDANCE",
        "file_path": "/home/sahi/SahiOtonom/KararAlg/decision-making-node-avoidance.py", # <-- GÜNCEL AVOIDANCE VERSIYONU
        "delay_after": 1
    }
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
    
    print("--- OTONOM SİSTEM SIRAYLA BAŞLATILIYOR (Avoidance Versiyonu) ---")

    try:
        for node_info in NODE_LAUNCH_ORDER:
            node_name = node_info["name"]
            # 'file' yerine 'file_path' anahtarını kullanıyoruz
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