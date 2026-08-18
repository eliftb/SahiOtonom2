# Dosya Adı: launch_all_nodes.py 

import multiprocessing
import time
import os
import sys
import signal
import glob
import subprocess
import importlib.util


def lidar_portu_bul():
    """LiDAR'ın seri portunu otomatik bulur.

    /dev/ttyUSB0, ttyUSB1 ... numaraları TAKMA SIRASINA göre dağıtılır: Arduino'yu
    LiDAR'dan önce takınca LiDAR ttyUSB1'e kayıyor ve sabit yazılmış bir port
    yüzünden sürücü açılmıyordu. by-id yolu ise cihazın kendi kimliğinden türer,
    sıradan etkilenmez. (udev kuralı kuruluysa /dev/sahi_lidar tercih edilir -
    bkz. udev/99-sahi-otonom.rules)
    """
    for kalip in ('/dev/sahi_lidar',
                  '/dev/serial/by-id/*CP2102*',        # RPLIDAR S2'nin USB köprü çipi
                  '/dev/serial/by-id/*Silicon_Labs*'):
        bulunan = sorted(glob.glob(kalip))
        if bulunan:
            return bulunan[0]
    return None


LIDAR_PORT = lidar_portu_bul()

# --- DÜĞÜM BAŞLATMA SIRASI VE DOSYA YOLLARI ---
# Yollar, bu launch dosyasının bulunduğu klasöre (SahiOtonom) göre otomatik hesaplanır.
# Böylece projeyi başka bir yere taşısan bi
#
# le yolları elle güncellemene gerek kalmaz.
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# --- PIST KAYDI --------------------------------------------------------------
# kalibrasyon.yaml icinde  kayit: true  yapilirsa sistem calisirken ham sensor
# verisi otomatik kaydedilir. Pistte ayri terminal acip komut yazmak gerekmesin
# diye burada: sure kisitliyken en az adim en az hata demek.
sys.path.insert(0, BASE_DIR)
try:
    from kalibrasyon import kalibrasyon as _kal
    KAYIT_ACIK = bool(_kal('launcher')('kayit', False))
except Exception as _e:
    print(f"[kayit] kalibrasyon.yaml okunamadi ({_e}), kayit kapali.")
    KAYIT_ACIK = False

KAYIT_KLASORU = os.path.join(BASE_DIR, 'kayitlar')
KAYIT_ADI = time.strftime('pist_%Y%m%d_%H%M%S')
KAYIT_TOPICLERI = [
    '/zed2i_rgb/image_raw',   # serit + levha tespitinin girdisi
    '/scan',                  # engel tespitinin girdisi
    '/zed2i/odom',            # ZED odometrisi (aciksa)
]

NODE_LAUNCH_ORDER = [
    {
        # ZED'in open() çağrısı ölçümde ~4.5 sn sürüyor (USB + firmware init).
        # 2 sn beklersek şerit/levha düğümleri kamera daha yayına başlamadan açılır.
        # critical: bu düğüm ayağa kalkmazsa diğerlerini başlatmanın anlamı yok,
        # hepsi görüntü bekleyerek boşta kalır.
        "name": "KAMERA YAYINI (ZED)",
        "file_path": os.path.join(BASE_DIR, "Kamera/zedi2connect_port.py"),
        "delay_after": 6,
        "critical": True
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
        # LiDAR DONANIM SÜRÜCÜSÜ. Eskiden bu ayrı terminalde elle başlatılıyordu
        # ve unutulduğunda /scan hiç yayınlanmıyordu: engel-tespit.py sessizce
        # ayakta kalıyor ("initialized" yazıyor) ama tek bir ölçüm bile almıyor,
        # yani engel tespiti çalışıyor sanılıp aslında hiç çalışmıyordu.
        # RPLIDAR S2M1-R2: baud 1000000 (A1'in 115200'ü DEĞİL).
        "name": "LIDAR SÜRÜCÜSÜ (RPLIDAR S2)",
        # Doğrudan 'ros2 launch' değil, RESET atan sarmalayıcı: RPLIDAR besleme
        # sınırında çalıştığı için motor yükü artınca kendini koruma moduna alıp
        # 'health status 2' mandalına düşüyor. Bu mandal sürücü yeniden
        # başlatılınca temizlenmiyor, cihaza RESET komutu gitmesi gerekiyor.
        "command": [os.path.join(BASE_DIR, "EngelTespit/lidar_baslat.sh"),
                    str(LIDAR_PORT), "1000000"],
        # LiDAR araç bataryasından besleniyorsa acil durdurmada USB'den düşer.
        # Batarya geri gelince sürücü kendiliğinden ayağa kalksın.
        "restart": True,
        "delay_after": 4
    },
    {
        # Yukarıdaki sürücünün yayınladığı /scan topic'ini dinler.
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

if KAYIT_ACIK:
    # En sona eklenir: diger dugumler yayina baslamis olur, kaydin ilk saniyesi
    # bos gecmez. CTRL+C'de run_command proses grubunu kapattigi icin kayit
    # duzgun sonlanir ve dosya bozuk kalmaz.
    os.makedirs(KAYIT_KLASORU, exist_ok=True)
    NODE_LAUNCH_ORDER.append({
        "name": f"PİST KAYDI ({KAYIT_ADI})",
        "command": ["ros2", "bag", "record",
                    "-o", os.path.join(KAYIT_KLASORU, KAYIT_ADI),
                    # 2 GB'lik parcalara bol. Tek parca kaydedilirse sikistirma
                    # SADECE kapanista yapiliyor: 30 dakikalik kayitta bu dakikalar
                    # surer ve sabirsizlanip kapatilirsa metadata.yaml yazilmadan
                    # kalir (kayit acilamaz hale gelir). Parcali kayitta her parca
                    # dolunca sikistirilir, kapanis aninda sadece son parca kalir.
                    # Ayrica kayit ortasinda bir sorun olursa onceki parcalar saglam.
                    "--max-bag-size", "2000000000",
                    "--compression-mode", "file",
                    "--compression-format", "zstd"] + KAYIT_TOPICLERI,
        "delay_after": 2
    })

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

def run_command(command, yeniden_baslat=False):
    """Harici bir komutu (ör. 'ros2 launch') çalıştırır.

    Komut KENDİ PROSES GRUBUNDA başlatılır ve bu sarmalayıcıya SIGTERM gelince
    tüm gruba SIGINT gönderilir. Aksi halde launcher kapandığında 'ros2 launch'
    çocukları hayatta kalıyor: LiDAR motoru dönmeye devam ediyor ve seri portu
    tutmaya devam ettiği için bir sonraki başlatma 'Device or resource busy'
    hatası veriyor.

    yeniden_baslat=True ise komut ölünce tekrar başlatılır. LiDAR araç
    bataryasından besleniyorsa acil durdurmada USB'den düşer ve sürücü ölür;
    batarya geri geldiğinde kendiliğinden ayağa kalksın diye. (UART düğümü aynı
    sorunu kendi içinde çözüyor, ama sürücü harici bir proses.)
    """
    proc = None
    kapaniyor = {'evet': False}

    def _kapat(signum, frame):
        kapaniyor['evet'] = True
        if proc and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        sys.exit(0)

    signal.signal(signal.SIGTERM, _kapat)
    signal.signal(signal.SIGINT, _kapat)

    deneme = 0
    while True:
        try:
            if deneme == 0:
                print(f"[{os.getpid()}] Komut başlatıldı: {' '.join(command)}")
            proc = subprocess.Popen(command, start_new_session=True)
            proc.wait()
        except FileNotFoundError:
            print(f"HATA: '{command[0]}' bulunamadı. ROS ortamı source edilmiş mi? "
                  f"(source /opt/ros/humble/setup.bash)", file=sys.stderr)
            return
        except Exception as e:
            print(f"[{os.getpid()}] HATA (komut): {e}", file=sys.stderr)
            return

        if kapaniyor['evet'] or not yeniden_baslat:
            return

        deneme += 1
        # ARTAN BEKLEME. Sabit 4 sn ile denemek iki durumu ayırt edemiyordu:
        #  - batarya kesintisi: cihaz birkaç saniyede geri gelir, hızlı denemek doğru
        #  - donanım hatası (ör. RPLIDAR 'health status 2'): tekrar denemek ASLA
        #    düzeltmez, cihazın fişini çekip takmak gerekir. Sabit aralıkta
        #    denemek terminali aynı hatayla dolduruyor ve gerçek sorunu gizliyor.
        # Bekleme 4 sn'den 30 sn'ye çıkar: ilki hâlâ hızlı toparlanır, ikincisi
        # sessizleşir.
        bekleme = min(4 * deneme, 30)
        if deneme <= 3 or deneme % 10 == 0:
            print(f"[{os.getpid()}] '{' '.join(command[:3])}' kapandı "
                  f"(deneme {deneme}). {bekleme} sn sonra yeniden denenecek.")
            if deneme == 3:
                print(f"[{os.getpid()}] NOT: üst üste açılamıyorsa cihazın kendi "
                      f"hatası olabilir (ör. RPLIDAR 'health status 2'). "
                      f"USB'den ÇIKARIP TEKRAR TAKIN - yazılım bunu düzeltemez.")
        time.sleep(bekleme)


def kapat(processes):
    """Başlatılmış tüm düğümleri önce nazikçe, sonra zorla sonlandırır."""
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


if __name__ == '__main__':
    processes = []
    kritik_prosesler = []
    kritik_hata = False

    print("--- OTONOM SİSTEM SIRAYLA BAŞLATILIYOR (Farklı Klasörlerden) ---")

    try:
        for node_info in NODE_LAUNCH_ORDER:
            node_name = node_info["name"]
            path = node_info.get("file_path")
            command = node_info.get("command")
            delay = node_info["delay_after"]

            if path is not None:
                if not os.path.exists(path):
                    print(f"\n‼️ UYARI: '{node_name}' için dosya bulunamadı, atlanıyor: {path}")
                    continue
                hedef, argumanlar = run_script, (path,)
            else:
                # Harici komut (ör. LiDAR sürücüsü). Cihaz takılı değilse portu
                # bulamayız; sessizce yanlış porta bağlanmak yerine atlıyoruz.
                # NOT: eskiden komut metninde 'serial_port:=None' aranıyordu ama
                # sürücü RESET sarmalayıcısına taşınınca komut biçimi değişti ve
                # kontrol tutmaz oldu: port 'None' STRING olarak geçiyor, sürücü
                # açılmaya çalışıp 'buffer overflow' ile çöküyordu.
                if LIDAR_PORT is None and 'lidar' in ' '.join(command).lower():
                    print(f"\n‼️ UYARI: '{node_name}' atlanıyor - LiDAR USB'de bulunamadı.")
                    print("   Kontrol: ls /dev/serial/by-id/ | grep -i cp2102")
                    continue
                hedef = run_command
                argumanlar = (command, node_info.get("restart", False))

            print(f"\n▶️  '{node_name}' başlatılıyor...")
            process = multiprocessing.Process(target=hedef, args=argumanlar)
            processes.append(process)
            kritik_prosesler.append((node_name, process, node_info.get("critical", False)))
            process.start()

            print(f"    PID {process.pid} ile başlatıldı. Sonraki düğüm için {delay} saniye bekleniyor...")
            time.sleep(delay)

            # Bekleme bitince hâlâ ayakta mı? Kamera gibi kritik bir düğüm
            # açılışta ölmüşse (ör. ZED'i başka bir proses tutuyorsa) hata
            # mesajı yukarıda kaybolur ve sistem sessizce görüntü bekler.
            if not process.is_alive():
                print(f"\n‼️ '{node_name}' başlatıldıktan sonra kapandı "
                      f"(çıkış kodu {process.exitcode}). Yukarıdaki hata mesajına bak.")
                if node_info.get("critical"):
                    print("   Bu düğüm kritik, sistem başlatılmıyor.")
                    kritik_hata = True
                    break

        # ZED düğümü açılamazsa 5 kez tekrar dener (~20 sn); yukarıdaki tek
        # seferlik kontrol o sırada onu "ayakta" görüp geçebiliyor. Bu yüzden
        # hepsi başladıktan sonra kritik düğümleri bir kez daha doğrula.
        if not kritik_hata:
            for ad, proc, kritik in kritik_prosesler:
                if kritik and not proc.is_alive():
                    print(f"\n‼️ '{ad}' açılış sırasında kapandı "
                          f"(çıkış kodu {proc.exitcode}). Bu düğüm kritik.")
                    kritik_hata = True

        if kritik_hata:
            print("\n--- KRİTİK DÜĞÜM AÇILAMADI, BAŞLATILANLAR KAPATILIYOR ---")
            kapat(processes)
            sys.exit(1)

        print("\n✅ Tüm düğümler başarıyla ve sırayla başlatıldı.")
        if KAYIT_ACIK:
            print(f"🔴 KAYIT ALINIYOR -> kayitlar/{KAYIT_ADI}")
            print("   Turu attıktan sonra CTRL+C ile bitirin, kayıt kapanır.")
        print("Kapatmak için terminalde CTRL+C tuşlarına basın.")

        for process in processes:
            process.join()

    except KeyboardInterrupt:
        print("\n\n--- KAPATMA SİNYALİ ALINDI (CTRL+C) ---")
        print("Tüm düğümler sonlandırılıyor...")
        kapat(processes)