#!/usr/bin/env python3

import math
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Int32, Bool
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import collections

from utils.utils import (
    select_device,
    driving_area_mask, lane_line_mask
)

# NOT: Bu düğümün ayarları AŞAĞIDA, declare_parameter satırlarında sabittir.
# (kalibrasyon.yaml 2026-08-18'de kaldırıldı.) Pistte yeniden başlatmadan
# denemek için 'ros2 param set /lane_detection_node <ad> <deger>', beğendiğin
# değeri buraya yaz - yoksa sistem kapanınca kaybolur.

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.br = CvBridge()

        # Model, bu dosyanın yanındaki models/ klasöründen otomatik bulunur
        self.weights = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models', 'tusimple_18.pt')
        self.device = select_device('0')
        self.half = self.device.type != 'cpu'
        self.img_size = 640

        # CLAHE nesnesi bir kez kurulur. Her karede createCLAHE çağırmak ölçümde
        # kare başına ~4 ms yiyordu.
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Kuyruk derinliği 1: bu bir KONTROL döngüsü, bayat kare işlemenin değeri
        # yok. Derinlik 10 ile düğüm kameradan yavaş kaldığında 10 kare birikiyor
        # ve direksiyon 300 ms öncesinin görüntüsüne göre kırıyordu.
        self.subscription = self.create_subscription(
            Image, '/zed2i_rgb/image_raw', self.image_callback, 1)

        # KAVŞAK TERCİHİ. Yol ikiye ayrıldığında koridor seçici, takip merkezine
        # en yakın parçayı alıyordu - yani araç hangi kola yakınsa oraya gidiyor,
        # kavşaktan kavşağa değişiyordu. Karar alma düğümü mecburi yön levhasını
        # görünce burayı -1 (sol) / +1 (sağ) yapar.
        self.preferred_side = 0
        self.create_subscription(
            Int32, '/route/preferred_side', self.preferred_side_callback, 10)
        self.publisher = self.create_publisher(Image, 'lane_detection_output', 10)
        self.lateral_pub = self.create_publisher(Float32, '/lane/lateral_deviation', 10)
        self.intersection_pub = self.create_publisher(Int32, '/lane/intersection_direction', 10)
        self.center_px_pub = self.create_publisher(Float32, '/lane/center_px', 10)
        # ŞERİT GEÇERLİ Mİ. Kavşakta şerit çizgileri biter; rota o birkaç metre
        # boyunca ölçüme değil tahmine dayanır ve araç savrulur. UART düğümü bu
        # bayrağı görünce şerit takibini bırakıp ODOMETRİYLE yön tutmaya geçer.
        self.lane_valid_pub = self.create_publisher(Bool, '/lane/valid', 10)
        # KALİBRASYON İÇİN: en yakın satırda (sample_rows[0]) ölçülen, dik
        # genişliğe çevrilmiş şerit genişliği. _expected_lane_width ile AYNI
        # birimde - lane_width_frac kalibrasyonu bunu kullanır (bkz.
        # kalibrasyon_lane_width.py). Sadece yayın, kontrol yolunu etkilemez.
        self.width_px_pub = self.create_publisher(Float32, '/lane/width_px', 10)
        # KALİBRASYON İÇİN: viraj ileri besleme terimi (uzak satır - yakın satır).
        # Kamera ileri baktığı için "şerit dönüyor" ile "araç şeride göre AÇILI"
        # ayırt edilemez; ikisi de bunu sıfırdan uzaklaştırır. Offset kalibrasyonu
        # aracın şeride PARALEL olmasını şart koştuğu için ölçüm buna bakıp
        # hizasız duruşu reddeder (bkz. kalibrasyon_kamera.py).
        self.curve_pub = self.create_publisher(Float32, '/lane/curve', 10)

        # --- METRİK SAĞ ÇİZGİ TAKİBİ (route_source='mesafe') ---------------
        # Piksel geometrisi yerine ZED'in ölçtüğü GERÇEK mesafeyi kullanır.
        # Gerekçe: gerçek kayıtta satırların %85'inde model tek çizgi görüyor;
        # iki çizgiye dayanan şerit-ortası hesabı bu yüzden kararsızdı ve
        # merkez bir şerit öteye sıçrayabiliyordu. Tek bir çizgiye sabit
        # METRİK mesafede gitmek bu varsayıma hiç ihtiyaç duymuyor.
        self.depth_image = None
        self.fx = None
        self.cx = None
        self.create_subscription(Image, '/zed2i/depth', self.depth_callback, 1)
        self.create_subscription(CameraInfo, '/zed2i/camera_info',
                                 self.camera_info_callback, 1)

        # Aracın orta çizgisi ile SAĞDAKİ en yakın çizgi arasında korunacak
        # mesafe (metre). Ekrandaki mavi çizgi = aracın ortası.
        #
        # ÖLÇÜLDÜ (2026-08-19): pist şeridi 3.0 m -> şeridin ortasında gitmek
        # için sağ çizgiye 1.5 m. Kayıt bunu BAĞIMSIZ olarak doğruluyor:
        # pist_20260819_104914'ün düz kesitinde ölçülen mesafe medyanı 1.57 m.
        # Bir ara 1.1/1.2 denendi; o bir TAHMİNDİ ve iki yönden zararlıydı:
        # aracı şerit ortasının 30-40 cm sağına oturtuyor, ayrıca
        # hata = ölçülen - hedef büyüdüğü için SOL virajda "sağa kır" komutunu
        # güçlendiriyordu (bkz. aşağıdaki viraj notu).
        self.declare_parameter('hedef_sag_mesafe_m', 1.5)
        # Hatanın -1..+1 sapmaya ölçeklenmesi: bu kadar metre hata = tam sapma.
        # HATA -> SAPMA ÖLÇEĞİ. Kaç metre hata 'tam sapma' (1.0) sayılsın.
        # DİKKAT: metrik moda geçerken sapmanın BİRİMİ değişti - eskiden 1.0
        # görüntü yarı genişliği demekti, şimdi bu kadar METRE demek. Ölçek
        # 1.0 iken kp=1.0 ile 0.5 m'lik hata direksiyonu TAVANA dayıyordu
        # (0.5 rad ~ 29°); araç sert kırıp aşıyor ve şeritten çıkıyordu.
        # 2.5 ile aynı hata ~11° veriyor - düzeltir ama savurmaz.
        # 2.5 -> 4.0 (2026-08-19). 2.5 ile 0.9 m hata sapmayı 0.37'ye
        # çıkarıyor ve kp ile birlikte direksiyonu tavana dayıyordu. 4.0 ile
        # aynı hata 0.22 sapma verir; kp 0.3 ile ≈4° direksiyon demek - pistin
        # gerçek virajının istediği açı bu.
        # KANITLANMIŞ DEĞER: kp 0.3 ile birlikte sapmanın düzgün yakınsadığı
        # tur bu ölçekle koşuyordu. 2.5 ile aynı hata direksiyonu tavana
        # dayıyor ve araç şeridi bir hamlede geçiyordu.
        # SABAHKİ DEĞERE DÖNÜLDÜ (2026-08-19 akşam). 4.0 tek başına makuldü
        # ama kp 0.8->0.3 ile AYNI ANDA değişti ve ikisi aynı yönde çarpıştı:
        # etkin döngü kazancı 0.320 -> 0.075, yani 4.3 KAT zayıfladı. 0.6 m
        # hatada sabah 23° istenirken şimdi 5° isteniyordu - araç şeridi
        # düzeltemeyip dışarı akıyordu.
        self.declare_parameter('mesafe_hata_olcegi_m', 2.5)
        # Derinlik gürültülüdür; çizgi çevresinden bu yarıçapta medyan alınır.
        self.declare_parameter('derinlik_pencere_px', 7)

        # ŞERİT DEĞİŞTİRMEME KİLİDİ (metrik mod). Takip edilen sağ çizgi
        # kaybolursa 'en yakın sağ çizgi' bir sonraki şeridin çizgisi olur ve
        # ölçülen mesafe bir anda sıçrar; araç o çizgiyi 1.5 m'de tutmaya
        # çalışıp ŞERİT DEĞİŞTİRİR. Gerçek bir yanal hareket bu kadar ani
        # olamayacağı için, sıçrayan ölçüm kabul edilmez: ölçüm yok sayılır ve
        # araç önceki şeridini korur.
        # --- ÖLÇÜM GEÇERLİLİK BANDI (3 m'lik şeride göre) -------------------
        # Araç KENDİ ŞERİDİNDE ise sağdaki çizgi bu aralıkta olmak zorunda:
        #     şeridin ortası      -> 1.5 m   (hedef)
        #     sağ çizgiye yapışık -> ~0.3 m
        #     sol çizgiye yapışık -> ~2.5 m  (şeridin sınırı)
        #     yan şeridin çizgisi -> ~4.5 m  (bizim çizgimiz DEĞİL)
        # Bandın dışındaki okuma ya yanlış çizgiye kilitlenmedir ya da araç
        # zaten şeridin dışındadır; ikisinde de o sayıyı hedefe kovalamak
        # aracı daha da uzağa götürür. Pistte 2.78 m okundu ve araç o çizgiye
        # doğru sürüklendi - şeritten çıkmasının sebebi buydu.
        # Bandın dışı ÖLÇÜM YOK sayılır: son komut sönümlenir, araç düz gider.
        # --- MERKEZDE YUMUŞAK, KENARDA SERT ---------------------------------
        # Tek eğimli kontrol iki isteği aynı anda karşılayamıyor: zikzak
        # yapmayacak kadar yumuşak olursa kenara kaçmayı geç yakalıyor, kenarı
        # yakalayacak kadar sert olursa ortada salınıyor.
        #
        # merkez_bandi_m: şerit ortasından bu kadar sapma NORMAL sayılır,
        #   tepki doğrusal ve yumuşaktır (ortada gereksiz düzeltme yapmaz).
        # kenar_kazanci : bandın dışında hata bu katsayıyla büyütülür, yani
        #   araç çizgiye yaklaştıkça geri çekiş sertleşir.
        #
        # 3 m'lik şeritte: hata 0.5 m = araç ortadan yarım metre kaymış (hâlâ
        # şeritte); hata 1.4 m = çizginin dibinde. İkisine aynı tepkiyi vermek
        # yanlış - ikincisinde şerit ihlali an meselesidir.
        # ŞERİT GENİŞLİĞİ (m) - ÖLÇÜLDÜ: 3.0. Referansın yan şeridin
        # çizgisine kaymasını yakalamak için gerekli (bkz. _mesafe_sapmasi).
        self.declare_parameter('serit_genisligi_m', 3.0)   # 0 = serit gecis tespiti KAPALI
        self.declare_parameter('merkez_bandi_m', 0.5)
        self.declare_parameter('kenar_kazanci', 1.0)   # 1.0 = KAPALI (dogrusal, sabahki gibi)
        # MAKULLÜK BANDI AÇILDI (2026-08-19, "1.5 metreyi referans alsın").
        # Band kapalıyken (0/99) sağdaki EN YAKIN çizgi olarak yan şeridin
        # çizgisi ya da bir bariyer ölçülse bile geçerli sayılıyordu. hedef
        # 1.5 m olduğu için 3 m'lik bir okuma 'çok soldayım' diye yorumlanıp
        # aracı sağa, yani YAN ŞERİDE sürüyordu; oraya varınca hata sıfırlanıp
        # kontrolcüye göre her şey yolunda görünüyordu. Şerit değiştirmesinin
        # sebebi buydu.
        # 1.5 ± 1.0 m: kendi şeridinde makul her yeri kapsar, komşu şeridin
        # çizgisini (≈3 m) kapsamaz.
        self.declare_parameter('mesafe_alt_sinir_m', 0.5)
        self.declare_parameter('mesafe_ust_sinir_m', 2.5)
        self.declare_parameter('mesafe_sicrama_esigi_m', 0.8)
        # AYNI ÇİZGİ Mİ? Örnekleme satırları BİRBİRİNDEN BAĞIMSIZ ölçüyor: biri
        # gerçek şerit çizgisini, diğeri saha kenarını/bariyeri yakalayabiliyor.
        # En yakın adaydan bu kadar UZAKTAKİ adaylar başka bir cisim sayılır ve
        # ölçüme hiç karıştırılmaz. Gerçek bir şerit çizgisinin yanal mesafesi
        # satırdan satıra bu kadar oynamaz; virajdaki gerçek değişimi zaten
        # aşağıdaki doğru uydurma modelliyor.
        # Şerit genişliğinin (3.0 m) YARISINDAN küçük tutun: komşu şeridin
        # çizgisi tam bir şerit genişliği ötede, tolerans oraya ulaşmamalı.
        self.declare_parameter('ayni_cizgi_tol_m', 1.0)
        # KİLİT KAÇ KARE SONRA AÇILIR. Sıçrama koruması reddettiği karede
        # son_mesafe_m'i KORUYOR; araç gerçekten yer değiştirdiyse referans
        # DONUYOR ve ondan sonraki her ölçüm o bayat değere göre reddediliyor.
        # Kilit bir kez kapanınca bir daha açılmıyordu: kayıtta 110 karenin
        # 75'i böyle elendi ve /lane/valid False'a düşüp araç 'şerit yok'
        # moduna geçti. Bu kadar kare üst üste aynı yönde ölçüm geliyorsa
        # sıçrama değil GERÇEK hareket demektir; referans yenilenir.
        # 9999 -> 8 (2026-08-19): 9999 bu valfi TAMAMEN kapatıyordu, yani
        # yukarıda anlatılan arıza hâlâ aktifti. Pist koşusunda gerçekleşti:
        # 'kaynak mesafe-sicrama | kayip kare 156 | serit gecerli False' -
        # kilit 156 kare (~10 sn) boyunca kapalı kaldı, /lane/valid False'ta
        # takıldı ve uart düğümü bütün koşu boyunca 'YÖN TUTMA (şerit yok)'
        # yaptı. Oysa yolda şerit VARDI ve ölçüm (2.34 m) makul bandın
        # (0.5-2.5 m) içindeydi; sadece donmuş referanstan 0.8 m'den fazla
        # uzak olduğu için her karede reddediliyordu.
        # 8 SEÇİLDİ, gecerlilik_kayip_kare'den (12) KÜÇÜK olsun diye: kilit,
        # /lane/valid False'a düşmeden ÖNCE açılır, yani bu arıza bir daha
        # aracı 'şerit yok' moduna sokamaza yetiyor.
        # Büyütmek koruma süresini uzatır ama 12'yi geçers. ~17 FPS'te 8 kare ≈ 0.5 sn:
        # gerçek bir yanlış-çizgi sıçramasını savuşturmaye arıza geri gelir.
        self.declare_parameter('mesafe_sicrama_kabul_kare', 8)

        # /lane/valid NE ZAMAN False OLMALI. UART düğümü bu bayrağı görünce
        # şerit takibini TAMAMEN bırakıp odometriyle düz gidiyor (kavşak
        # davranışı). Metrik modda çizgi kısa süreli kaybolmak NORMAL - gerçek
        # kayıtta satırların %85'inde tek çizgi var - ve her kısa kayıpta
        # kontrolü bırakmak, hesaplanan 1.5 m komutunun araca hiç ulaşmaması
        # demekti. Şerit ancak bu kadar kare üst üste ölçülemezse geçersiz
        # sayılır; kavşakta çizgi gerçekten bittiği için eşik yine aşılır.
        self.declare_parameter('gecerlilik_kayip_kare', 12)

        # ÖLÇÜM SABİT İLERİ MESAFEDEN ALINIR. Eskiden 'veri veren ilk satır'
        # kullanılıyordu; satırlar farklı ileri mesafelere denk geldiği ve
        # virajda yanal mesafe ileri mesafeyle değiştiği için, hangi satırın
        # veri verdiği değişince ÖLÇÜLEN MESAFE de zıplıyordu. Araç sabit
        # olmayan bir hedefi kovalıyor, bu da şeritten kaymaya yol açıyordu.
        # Derinlik sayesinde her noktanın Z'si bilindiği için, hep bu uzaklığa
        # en yakın noktadan ölçmek karşılaştırılabilir bir değer verir.
        self.declare_parameter('olcum_ileri_mesafe_m', 3.0)

        # --- ŞERİT TAKİBİ AYARLARI ---
        # Kamera aracın tam ortasında/tam ileri bakacak şekilde değilse şerit
        # ortası görüntü ortasına denk düşmez. Bu sabit kayma düz yolda sürekli
        # bir sapma üretip aracı kenara çekiyordu. Kalibrasyon: aracı şeridin
        # ortasına koy, ekrandaki "Merkez" değeri ile "ref" farkını buraya yaz.
        # 0.0: kayıt analizi de ~0 diyor. TEMİZ BİR ÖLÇÜM YAPILMADI - aracı
        # şeridin ortasına düz koyup kalibrasyon_kamera.py ile ölçün, çıkan
        # değeri buraya yazın.
        self.declare_parameter('camera_center_offset_px', 0.0)
        # Sapmanın okunduğu satır (1.0 = görüntünün en altı). Yukarı çekmek daha
        # ileriye baktırır (yumuşak direksiyon), aşağı çekmek tepkiselleştirir.
        # 0.80 iken kaputun hemen üstüne, yani birkaç metre öteye bakıyordu ve
        # viraj ancak araç içine girdiğinde fark ediliyordu.
        self.declare_parameter('look_ahead_frac', 0.68)
        # Bu eşiğin altındaki sapma 0 sayılır: düz yolda sürekli minik düzeltme
        # yapıp zikzak çizmesini engeller.
        self.declare_parameter('deviation_deadband', 0.02)
        # SANİYE başına izin verilen en büyük sapma değişimi. Tek karelik yanlış
        # eşleşme aracı bir anda parkurdan çıkaramasın. Kare başına sınır,
        # model yavaşladığında (düşük FPS) viraja girişi geciktiriyordu.
        self.declare_parameter('max_deviation_rate', 2.5)
        # (Pistte 0.5 yerine 0.6 ile sürüldü; sabitlenen değer o.)
        self.declare_parameter('curve_feedforward', 0.6)
        
        # öteye fırlıyordu - aracın şerit değiştirmesinin sebebi buydu.
        self.declare_parameter('horizon_frac', 0.512)
        # ✓ kayıttan doğrulandı (0.395)
        self.declare_parameter('lane_width_frac', 0.40)
        self.declare_parameter('paint_inside_only', True)
        self.declare_parameter('corridor_widen_tol', 1.35)
        # Koridor parçası referanstan bu kadar uzaktaysa (görüntü genişliğinin
        # oranı) o satır ölçülmez. Yolun yanındaki alakasız sürülebilir alana
        # sıçramayı engeller - bkz. _corridor_centers_at_rows.
        self.declare_parameter('corridor_max_jump_frac', 0.20)
        # ROTA KARARLILIĞI. Satır merkezleri her karede sıfırdan ölçülüyor ve
        # eğri onlara uyduruluyordu - hiçbir zamansal yumuşatma yoktu. Bir satır
        # bir karede iki çizgiyi, sonraki karede tek çizgiyi görünce merkez
        # (ölçülen orta) ile (çizgi ± genişlik/2) arasında sıçrıyor ve hedef
        # rota sürekli değişiyordu. 0 = yumuşatma yok (eski davranış),
        # 0.5 = yeni ölçüm yarı ağırlıkla karışır. Yükseltmek kararlılığı
        # artırır ama viraja tepkiyi geciktirir.
        self.declare_parameter('route_smoothing', 0.5)
        self.declare_parameter('max_line_jump_frac', 0.40)
        self.declare_parameter('auto_lane_width', True)
        self.declare_parameter('auto_lane_width_min_samples', 15)
        self.declare_parameter('lane_width_min_frac', 0.10)
        self.declare_parameter('lane_width_max_frac', 0.70)
        self.declare_parameter('debug_every_n', 3)
        self.declare_parameter('debug_scale', 0.5)
        self.declare_parameter('route_source', 'mesafe')
        # 'auto' için: kaç satırda İKİ çizgi de sürülebilir alanın içinde olmalı
        # (bkz. _looks_like_paint - bariyerler alanın dışında kalır)
        self.declare_parameter('min_paint_rows', 3)
        # ARACIN KAPUTU bu satırın altında kalır. Model kaputu "sürülebilir alan"
        # sayıyor (ekranda kaput da yeşil), ayrıca eski örnekleme satırlarının en
        # alt üçü (0.95/0.90/0.85) doğrudan gövdenin üstüne düşüyordu.
        # Kalibrasyon: debug karesinde kaputun üst kenarı görüntünün yüzde kaçında?
        # ✓ kayıttan doğrulandı (0.832)



        
        self.declare_parameter('hood_frac', 0.82)
        # TANI: her örnekleme satırındaki koridor kenarlarını ve merkezini loglar.
        # Rota yamuk çıktığında hangi satırın bozulduğu ancak böyle görülüyor -
        # ekrandaki tek "Merkez" sayısı sorunun nerede başladığını söylemiyor.
        self.declare_parameter('debug_rows_log', True)

        self.camera_center_offset_px = float(self.get_parameter('camera_center_offset_px').value)
        self.look_ahead_frac = float(self.get_parameter('look_ahead_frac').value)
        self.deviation_deadband = float(self.get_parameter('deviation_deadband').value)
        self.max_deviation_rate = float(self.get_parameter('max_deviation_rate').value)
        self.curve_feedforward = float(self.get_parameter('curve_feedforward').value)
        self.horizon_frac = float(self.get_parameter('horizon_frac').value)
        self.lane_width_frac = float(self.get_parameter('lane_width_frac').value)
        self.paint_inside_only = bool(self.get_parameter('paint_inside_only').value)
        self.corridor_widen_tol = float(self.get_parameter('corridor_widen_tol').value)
        self.corridor_max_jump_frac = float(self.get_parameter('corridor_max_jump_frac').value)
        self.route_smoothing = float(self.get_parameter('route_smoothing').value)
        self.hedef_sag_mesafe_m = float(self.get_parameter('hedef_sag_mesafe_m').value)
        self.mesafe_hata_olcegi_m = float(self.get_parameter('mesafe_hata_olcegi_m').value)
        self.derinlik_pencere_px = int(self.get_parameter('derinlik_pencere_px').value)
        self.serit_genisligi_m = float(self.get_parameter('serit_genisligi_m').value)
        self.merkez_bandi_m = float(self.get_parameter('merkez_bandi_m').value)
        self.kenar_kazanci = float(self.get_parameter('kenar_kazanci').value)
        self.mesafe_alt_sinir_m = float(self.get_parameter('mesafe_alt_sinir_m').value)
        self.mesafe_ust_sinir_m = float(self.get_parameter('mesafe_ust_sinir_m').value)
        self.mesafe_sicrama_esigi_m = float(self.get_parameter('mesafe_sicrama_esigi_m').value)
        self.ayni_cizgi_tol_m = float(self.get_parameter('ayni_cizgi_tol_m').value)
        self.mesafe_sicrama_kabul_kare = int(
            self.get_parameter('mesafe_sicrama_kabul_kare').value)
        self._sicrama_sayaci = 0
        self.gecerlilik_kayip_kare = int(self.get_parameter('gecerlilik_kayip_kare').value)
        self.olcum_ileri_mesafe_m = float(self.get_parameter('olcum_ileri_mesafe_m').value)
        self.max_line_jump_frac = float(self.get_parameter('max_line_jump_frac').value)
        self.auto_lane_width = bool(self.get_parameter('auto_lane_width').value)
        self.auto_lane_width_min = int(self.get_parameter('auto_lane_width_min_samples').value)
        self.lane_width_min_frac = float(self.get_parameter('lane_width_min_frac').value)
        self.lane_width_max_frac = float(self.get_parameter('lane_width_max_frac').value)
        self.debug_every_n = int(self.get_parameter('debug_every_n').value)
        self.debug_scale = float(self.get_parameter('debug_scale').value)
        self.route_source = str(self.get_parameter('route_source').value)
        self.min_paint_rows = int(self.get_parameter('min_paint_rows').value)
        self.hood_frac = float(self.get_parameter('hood_frac').value)
        self.debug_rows_log = bool(self.get_parameter('debug_rows_log').value)

        # Şerit çizgileri artık kontur ağırlık merkeziyle değil, SABİT satırlarda
        # örneklenerek bulunuyor. Kontur merkezi, çizginin görüntüde nereye kadar
        # uzandığına göre kayıyor: kesikli şeritte sol parça yukarı, sağ parça
        # aşağı denk gelince iki merkezin ortası gerçek şerit ortası olmuyor ve
        # araç düz yolda yana kaçıyordu. Aynı satırdan örneklenen iki nokta ise
        # her zaman aynı yükseklikte, yani ortası gerçekten şeridin ortası.
        #
        # En üstteki satırlar virajı ERKEN görmek için var: alt %30'a bakan eski
        # ROI'de viraj ancak aracın önüne geldiğinde fark ediliyordu, o noktada
        # dönmek için geç kalınmış oluyordu.
        #
        # 2026-08-17: satırlar KAPUTUN ÜSTÜNE taşındı. Eski dizide 0.95/0.90/0.85
        # aracın kendi gövdesine düşüyordu ve sol/sağ ayrımı zinciri oradan
        # başladığı için (bkz. _lane_centers_at_rows) gövdedeki gürültü tüm üst
        # satırlara yayılıyordu. İleri bakış da 0.80 ile kaputun hemen üstündeydi,
        # yani birkaç metre - o mesafeden viraj görünmez.
        self.sample_rows = (0.80, 0.76, 0.72, 0.68, 0.64, 0.60)
        self.lane_center_track = None                      # satır başına takip edilen şerit merkezi (px)
        # Satır başına öğrenilen KORİDOR genişliği (sürülebilir alan kaynağı için).
        # Koridorun bir kenarı kadraj dışına çıktığında orta noktayı bundan kurar.
        self.corridor_width_track = [None] * len(self.sample_rows)
        # Satır başına ÖĞRENİLEN şerit genişliği. Şeride DİK genişlik saklanır:
        # virajda şerit eğik durduğu için aynı satırdaki iki çizginin yatay
        # mesafesi gerçek genişlikten büyüktür (bkz. _slant_factor).
        self.lane_width_track = [None] * len(self.sample_rows)
        # Satır başına KAREDEN KAREYE takip edilen sol/sağ çizgi konumu. Sadece
        # lane_center_track (kayan ORTA nokta) yeterli değil: iki şerit yan
        # yanayken merkez, iki koridor arasındaki sınırın etrafında dolanabilir
        # ve o anda hangi çizgilerin gerçekten TAKİP EDİLEN şeride ait olduğunu
        # söylemez - sistem kare kare farklı şeride "kayabiliyordu". Bu iz,
        # önceki karede kilitlenen fiziksel çizgiye yakınlığı tercih ettirip
        # araç zaten hangi şeritteyse onda kalmasını sağlar.
        self.left_line_track = [None] * len(self.sample_rows)
        self.right_line_track = [None] * len(self.sample_rows)
        # Satır merkezinin ZAMANSAL izi + o satırın en son hangi karede
        # ölçüldüğü. Bayat bir izle karıştırmak (satır birkaç kare hiç
        # görülmediyse) sıçramayı azaltmaz, geciktirir - o yüzden tazelik
        # şartı var (bkz. _smooth_row_center).
        self.row_meas_track = [None] * len(self.sample_rows)
        self.row_meas_frame = [-999] * len(self.sample_rows)
        # KALİBRASYON İÇİN ham genişlik: en yakın satırda bulunan çift, MANTIKLILIK
        # KONTROLÜNDEN ÖNCEKİ hâliyle. Öğrenilen genişlik (lane_width_track) ancak
        # çift kabul edilirse doluyor; lane_width_frac yanlışken çift zaten
        # reddedildiği için kalibrasyon hiç veri göremiyordu (kısırdöngü).
        self.raw_width_meas = None
        self.debug_points = []
        self.son_mesafe_m = None
        # Ölçülen şerit genişliği örnekleri, 0.85 satırına NORMALİZE edilmiş
        # (perspektifte genişlik ufka uzaklıkla değişir; normalize etmeden
        # farklı satırların ölçümleri kıyaslanamaz). Medyanı beklenen
        # genişliği verir - bkz. _expected_lane_width.
        self.width_samples = collections.deque(maxlen=120)
        # 'auto' kaynak seçiminin oy sayacı (bkz. _route_centers histerezisi).
        # Negatif = koridor ('yol'), pozitif = şerit çizgisi ('serit').
        self.paint_votes = 0
        self.source_switch_frames = 5
        self.lost_frames = 0
        self.lane_valid = False
        self.lane_slope = None   # önceki karenin şerit eğimi (dx/dy), eğiklik düzeltmesi için

        self.deviation_history = []
        self.max_history_size = 5
        self.last_update_time = None

        # Letterbox dolgusunu maskeden geri sökmek için (bkz. unletterbox_mask)
        self.letterbox_pad = (0.0, 0.0)
        self.letterbox_shape = (self.img_size, self.img_size)

        # Görselleştirme durumu
        self.debug_rows = []
        self.debug_fit = None
        self.debug_center = None
        self.debug_curve = 0.0
        self.debug_source = '-'      # rotanın hangi kaynaktan geldiği (yol / serit)
        self.fps = 0.0
        self.last_frame_time = None
        self.frame_count = 0
        self.last_logged_direction = None

        # Parametreleri CANLI değiştirebilmek için. Düğümü yeniden başlatmak
        # modeli tekrar yüklüyor (~10 sn) ve pistte her denemeyi pahalılaştırıyordu.
        #   ros2 param set /lane_detection_node curve_feedforward 1.4
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.intersection_history = []
        self.intersection_history_size = 3
        self.stable_intersection_count = 0
        self.min_stable_frames = 2

        self.load_model()
        self.get_logger().info('🚦 Lane Detection Node başlatıldı.')

    # Pistte canlı ayarlanabilen parametreler (öznitelik adlarıyla birebir aynı)
    LIVE_PARAMS = ('camera_center_offset_px', 'look_ahead_frac', 'deviation_deadband',
                   'max_deviation_rate', 'curve_feedforward', 'horizon_frac',
                   'debug_every_n', 'debug_scale', 'min_paint_rows', 'hood_frac',
                   'route_source', 'debug_rows_log', 'lane_width_frac',
                   'paint_inside_only', 'corridor_widen_tol',
                   'corridor_max_jump_frac', 'route_smoothing',
                   'max_line_jump_frac', 'auto_lane_width',
                   'lane_width_min_frac', 'lane_width_max_frac',
                   'hedef_sag_mesafe_m', 'mesafe_hata_olcegi_m',
                   'mesafe_sicrama_esigi_m', 'mesafe_sicrama_kabul_kare',
                   'ayni_cizgi_tol_m',
                   'mesafe_alt_sinir_m', 'mesafe_ust_sinir_m',
                   'merkez_bandi_m', 'kenar_kazanci', 'serit_genisligi_m',
                   'derinlik_pencere_px',
                   'mesafe_sicrama_esigi_m', 'gecerlilik_kayip_kare',
                   'olcum_ileri_mesafe_m')

    def preferred_side_callback(self, msg):
        """Kavşakta tercih edilen kol (-1 sol, 0 yok, +1 sağ)."""
        if msg.data != self.preferred_side:
            yon = {-1: 'SOL', 0: 'yok', 1: 'SAĞ'}.get(int(msg.data), '?')
            self.get_logger().info(f'🧭 Kavşak tercihi: {yon}')
        self.preferred_side = int(msg.data)

    def _on_parameter_update(self, params):
        for p in params:
            if p.name in self.LIVE_PARAMS:
                # Mevcut değerin tipini koru: debug_every_n int olmalı, yoksa
                # modülo işlemi float'la çalışır.
                setattr(self, p.name, type(getattr(self, p.name))(p.value))
                self.get_logger().info(f'⚙️  {p.name} = {getattr(self, p.name)}')
        return SetParametersResult(successful=True)

    def load_model(self):
        self.model = torch.jit.load(self.weights).to(self.device)
        if self.half:
            self.model.half()
        self.model.eval()
        self.get_logger().info('✅ Model yüklendi.')

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        shape = img.shape[:2]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:
            r = min(r, 1.0)
        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        if auto:
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)
        elif scaleFill:
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
        dw /= 2
        dh /= 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, ratio, (dw, dh)

    def preprocess_image(self, im0):
        # CLAHE'yi MODEL çözünürlüğünde uygula, 1280x720'de değil: letterbox zaten
        # hemen ardından aynı boyuta küçültüyordu, yani sonuç neredeyse aynı ama
        # maliyet ~8 ms'den ~4 ms'ye iniyor. (Küçültme letterbox'ın kendi
        # hesabıyla birebir aynı: r = img_size / max(h, w).)
        h, w = im0.shape[:2]
        r = self.img_size / max(h, w)
        if r < 1.0:
            im0 = cv2.resize(im0, (int(round(w * r)), int(round(h * r))),
                             interpolation=cv2.INTER_LINEAR)

        lab = cv2.cvtColor(im0, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        img, _, pad = self.letterbox(enhanced, new_shape=self.img_size, stride=32)
        # Dolgu miktarını sakla: maske görüntüye hizalanırken geri sökülecek
        self.letterbox_pad = pad
        self.letterbox_shape = img.shape[:2]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        return img

    def unletterbox_mask(self, mask, im_shape):
        """Letterbox dolgusunu (gri kenarlık) atıp maskeyi görüntüye hizalar.

        Model 1280x720 kareyi 640x384'e letterbox'lıyor; alt ve üstte 12'şer
        satır dolgu var. Dolgu atılmadan yapılan resize maskeyi dikeyde ~%6
        esnetiyor, en alttaki ~22 satır ise tamamen boş dolguya denk geliyordu -
        yani araca en yakın, en güvenilir şerit bilgisi kayboluyordu.
        """
        mh, mw = mask.shape[:2]
        lb_h, lb_w = self.letterbox_shape
        dw, dh = self.letterbox_pad
        top = int(round(dh * mh / lb_h))
        left = int(round(dw * mw / lb_w))
        if mh - 2 * top > 1 and mw - 2 * left > 1:
            mask = mask[top:mh - top, left:mw - left]
        return cv2.resize(mask, (im_shape[1], im_shape[0]), interpolation=cv2.INTER_NEAREST)

    def _row_lane_points(self, lane_mask, y, band, merge_gap, max_cluster_px):
        """Tek bir satır bandındaki şerit çizgilerinin x konumları."""
        height, width = lane_mask.shape
        top = max(0, y - band)
        bottom = min(height, y + band + 1)
        # Bandı dikeyde OR'la: kesikli şeritte tek satır boşluğa denk gelebilir
        column_hit = (lane_mask[top:bottom, :] > 0).any(axis=0)
        xs = np.flatnonzero(column_hit)
        if xs.size == 0:
            return []

        points = []
        for group in np.split(xs, np.flatnonzero(np.diff(xs) > merge_gap) + 1):
            # tek piksel = gürültü, aşırı geniş = yatay çizgi / parlama
            if 2 <= group.size <= max_cluster_px:
                points.append(float(group.mean()))
        return points

    def _expected_lane_width(self, index, width, height):
        """Verilen örnekleme satırında beklenen şerit genişliği (px).

        Şerit genişliği perspektifte ufka olan uzaklıkla neredeyse doğrusal
        azalır; öğrenilmiş bir satır varsa oradan ölçekleyerek türetilir.
        """
        horizon = self.horizon_frac * height
        di = max(self.sample_rows[index] * height - horizon, 1.0)

        learned = [(abs(index - j), j) for j, val in enumerate(self.lane_width_track) if val is not None]
        if learned:
            j = min(learned)[1]
            dj = max(self.sample_rows[j] * height - horizon, 1.0)
            return self.lane_width_track[j] * di / dj

        # Öğrenilmiş satır yoksa: ÖLÇÜLEN genişliklerin medyanı (auto). Elle
        # girilen lane_width_frac yalnızca henüz yeterli örnek yokken kullanılır.
        ref_di = max(0.85 * height - horizon, 1.0)
        if self.auto_lane_width and len(self.width_samples) >= self.auto_lane_width_min:
            return float(np.median(self.width_samples)) * di / ref_di
        return width * self.lane_width_frac * di / ref_di

    def _slant_factor(self, y):
        """Şerit eğikken aynı satırdaki yatay genişlik, dik genişlikten büyüktür.

        Virajda tek çizgi görüldüğünde düz yolda öğrenilen genişlik kullanılırsa
        şerit merkezi olduğundan yakın tahmin ediliyor, yani araç virajı geniş
        alıyordu. Önceki karenin şerit eğimiyle (dx/dy) düzeltilir.
        """
        if self.lane_slope is None:
            return 1.0
        slope = float(self.lane_slope(y))
        # Aşırı eğimde katsayı patlamasın
        return float(min(np.sqrt(1.0 + slope * slope), 2.5))

    @staticmethod
    def _nearest(points, predicted, tolerance):
        """Tahmin edilen konuma en yakın nokta (tolerans dışındaysa None)."""
        if predicted is None or not points:
            return None
        best = min(points, key=lambda x: abs(x - predicted))
        return best if abs(best - predicted) <= tolerance else None

    def _bariyerleri_ele(self, points, y, da_mask, tol):
        """Sürülebilir alanın DIŞINDA kalan çizgi noktalarını atar.

        Model, boyalı şeridi bulmakta iyi ama bariyer/korkuluk gibi uzun düz
        kenarları da 'şerit çizgisi' sayıyor. Ayırt edici konum: yol boyası
        sürülebilir alanın içindedir, bariyer dışında kalır. Bu filtre olmadan
        bariyer çifti geçerli bir 'şerit' gibi görünüyor ve rota oradan
        kuruluyordu.
        """
        if da_mask is None or not points:
            return points
        h, w = da_mask.shape
        y = int(np.clip(y, 0, h - 1))
        kalan = []
        for x in points:
            x0 = int(np.clip(x - tol, 0, w - 1))
            x1 = int(np.clip(x + tol, 0, w - 1)) + 1
            if (da_mask[y, x0:x1] > 0).any():
                kalan.append(x)
        return kalan

    @staticmethod
    def _kirp(onceki, yeni, sinir):
        """İzi en fazla 'sinir' kadar kaydırır (ilk karede serbest)."""
        if onceki is None:
            return yeni
        fark = yeni - onceki
        if abs(fark) > sinir:
            return onceki + (sinir if fark > 0 else -sinir)
        return yeni

    def _smooth_row_center(self, index, center):
        """Satır merkezini zamansal olarak yumuşatır (rota kararlılığı).

        Ölçüm gürültüsü doğrudan eğriye, oradan da direksiyona geçiyordu.
        Özellikle bir satır bir karede iki çizgiyi, sonrakinde tek çizgiyi
        gördüğünde merkez iki farklı tahmin arasında sıçrıyor.

        BAYAT İZLE KARIŞTIRMA: satır birkaç karedir hiç ölçülmediyse eski
        değer artık o satırı temsil etmiyor; karıştırmak sıçramayı önlemez,
        sadece geciktirir. O yüzden iz yalnızca TAZEyse kullanılır.
        """
        onceki = self.row_meas_track[index]
        taze = (self.frame_count - self.row_meas_frame[index]) <= 2
        if onceki is not None and taze and self.route_smoothing > 0.0:
            center = (self.route_smoothing * onceki
                      + (1.0 - self.route_smoothing) * center)
        self.row_meas_track[index] = center
        self.row_meas_frame[index] = self.frame_count
        return center

    def _lane_centers_at_rows(self, lane_mask, da_mask=None):
        """Örnekleme satırlarının her birinde ego şeridin merkezini bulur.

        Satırlar ALTTAN YUKARI taranır ve çizgiler bir alt satırdaki
        konumlarından takip edilir. Her satırı ekran ortasına/şerit merkezine
        göre bağımsız sınıflandırmak keskin virajda çöküyordu: dış çizgi üst
        satırlarda ekran ortasını geçtiği için "karşı taraftaki çizgi" sanılıyor,
        viraj düzleşiyor ve araç geniş dönüyordu. Alttaki satırda sol/sağ ayrımı
        güvenilir (araç şeridin içinde), yukarısı oradan takip edilir.
        """
        height, width = lane_mask.shape
        band = max(2, int(height * 0.006))
        merge_gap = max(4, int(width * 0.012))
        max_cluster_px = int(width * 0.25)
        min_half = width * 0.04   # takip merkezine bu kadar yakın çizgi şerit sınırı sayılmaz
        window = width * 0.08     # çizgi bir üst satırda tahminden bu kadar sapabilir

        ys, centers = [], []
        self.debug_rows = []
        self.debug_points = []
        self.raw_width_meas = None

        prev_left = prev_right = prev_y = None
        slope_left = slope_right = 0.0    # satırlar arası dx/dy tahmini

        for i, frac in enumerate(self.sample_rows):
            y = int(height * frac)
            points = self._row_lane_points(lane_mask, y, band, merge_gap, max_cluster_px)
            # Bariyerleri en BAŞTA ele: sol/sağ seçimine hiç girmesinler.
            if self.paint_inside_only:
                points = self._bariyerleri_ele(points, y, da_mask,
                                               max(3, int(width * 0.006)))
            if not points:
                continue
            self.debug_points.append((y, list(points)))

            # Şerit eğikse yatay ölçümü dik genişliğe çevirmek için katsayı
            slant = self._slant_factor(y)
            expected = self._expected_lane_width(i, width, height)

            if prev_y is None:
                # ARACA EN YAKIN GEÇERLİ SATIR: burada takip merkezi güvenilir
                reference = self.lane_center_track[i]
                tracked_left, tracked_right = self.left_line_track[i], self.right_line_track[i]
                left = right = None
                if tracked_left is not None and tracked_right is not None:
                    # ÖNCE önceki karede kilitlenen FİZİKSEL çizgilere yakınlığa
                    # bak: iki şerit yan yanayken kayan merkez (reference) iki
                    # koridor arasındaki sınırın etrafında dolanabilir ve hangi
                    # çizgilerin TAKİP EDİLEN şeride ait olduğunu söylemez - bu
                    # da kare kare farklı şeride kaymaya yol açıyordu.
                    left = self._nearest(points, tracked_left, window)
                    right = self._nearest(points, tracked_right, window)
                if left is None:
                    left = max([x for x in points if x < reference - min_half], default=None)
                if right is None:
                    right = min([x for x in points if x > reference + min_half], default=None)
            else:
                # ÜST SATIRLAR: çizgileri alttaki konumlarından + eğimden tahmin et
                dy = y - prev_y
                left = self._nearest(points, prev_left + slope_left * dy, window)
                right = self._nearest(points, prev_right + slope_right * dy, window)
                if left is not None and right is not None and left >= right:
                    # İkisi de aynı çizgiye oturdu: tahminine daha yakın olanı tut
                    if abs(left - (prev_left + slope_left * dy)) <= abs(right - (prev_right + slope_right * dy)):
                        right = None
                    else:
                        left = None
                reference = self.lane_center_track[i]

            center = None
            if left is not None and right is not None:
                measured = (right - left) / slant   # dik genişliğe çevir
                if i == 0 or self.raw_width_meas is None:
                    # Kalibrasyon bunu okur; kontrol yolunu etkilemez.
                    self.raw_width_meas = measured
                # 0.85 satırına normalize edip örnek havuzuna at (auto genişlik).
                # FİZİKSEL SINIR: bu aralığın dışı bir şerit olamaz (en soldaki
                # çizgiyi en sağdakiyle eşleştirmiş demektir); öğrenmeye sokmak
                # beklenen genişliği bozar ve merkezi kadraj dışına atar.
                horizon = self.horizon_frac * height
                di = max(self.sample_rows[i] * height - horizon, 1.0)
                ref_di = max(0.85 * height - horizon, 1.0)
                norm = measured * ref_di / di
                fiziksel = (self.lane_width_min_frac * width <= norm
                            <= self.lane_width_max_frac * width)
                if fiziksel:
                    self.width_samples.append(norm)
                # ÖNYÜKLEME: genişlik daha ölçülmediyse mantıklılık kapısı
                # DOĞRULANMAMIŞ bir tahmine dayanır ve doğru çifti eleyebilir
                # (ölçümde: gerçek 122 px çift, 426 px beklendiği için
                # reddediliyor; merkez tek çizgiden 792 px'e kuruluyordu).
                # Araç başlangıçta şeridinin ortasında olduğu için en yakın
                # sol/sağ çift zaten DOĞRU çifttir - önce onu ölçelim, kapıyı
                # ondan sonra uygulayalım.
                onyukleme = (self.auto_lane_width
                             and len(self.width_samples) < self.auto_lane_width_min)
                if (onyukleme and fiziksel) or 0.5 * expected < measured < 1.8 * expected:
                    center = (left + right) / 2.0
                    previous = self.lane_width_track[i]
                    self.lane_width_track[i] = (
                        measured if previous is None else 0.85 * previous + 0.15 * measured
                    )
                else:
                    # Aralık mantıksız: çizgilerden biri komşu şeride ait.
                    # Referansa yakın olanı tek çizgi kabul et.
                    if (reference - left) <= (right - reference):
                        right = None
                    else:
                        left = None

            known = self.lane_width_track[i]
            if known is None:
                known = expected
            known *= slant   # dik genişlikten o satırdaki yatay genişliğe

            if center is None:
                if left is not None:
                    center = left + known / 2.0
                elif right is not None:
                    center = right - known / 2.0

            if center is None:
                continue

            center = self._smooth_row_center(i, center)

            # Görülmeyen çizginin yerini de doldur ki takip penceresi kesikli
            # çizgide/kadraj dışına çıkan çizgide yukarı doğru kaymaya devam etsin
            filled_left = left if left is not None else center - known / 2.0
            filled_right = right if right is not None else center + known / 2.0
            if prev_y is not None:
                dy = y - prev_y
                if dy != 0:
                    slope_left = 0.5 * slope_left + 0.5 * (filled_left - prev_left) / dy
                    slope_right = 0.5 * slope_right + 0.5 * (filled_right - prev_right) / dy
            prev_left, prev_right, prev_y = filled_left, filled_right, y
            # İzi kare başına sınırlı kaydır: yan şeridin çizgisine yürümesin.
            # 'known' o satırdaki şerit genişliği (öğrenilmiş ya da beklenen).
            sinir = self.max_line_jump_frac * known
            self.left_line_track[i] = self._kirp(self.left_line_track[i], filled_left, sinir)
            self.right_line_track[i] = self._kirp(self.right_line_track[i], filled_right, sinir)

            ys.append(float(y))
            centers.append(center)
            self.debug_rows.append((y, left, right, center))

        return ys, centers

    def _corridor_centers_at_rows(self, da_mask):
        """Örnekleme satırlarında SÜRÜLEBİLİR ALAN koridorunun orta noktasını bulur.

        Boyalı şerit olmayan pistte şerit çizgisi modeli bariyer kenarlarını çizgi
        sanıyor ve rota bariyerden türetiliyordu. Sürülebilir alan maskesi ise
        yolun kendisini veriyor: her satırda aracın içinde bulunduğu bitişik yol
        parçasının (koridorun) iki kenarının ortası hedef rotadır.

        Kaputa denk gelen satırlar atlanır: model aracın gövdesini de sürülebilir
        alan sayıyor (debug karesinde kaput da yeşil).
        """
        height, width = da_mask.shape
        band = max(2, int(height * 0.006))
        gap = max(2, int(width * 0.004))     # koridor içindeki küçük delikler kapatılır
        min_width = width * 0.08             # bundan dar parça yol değil, gürültü

        # Normalde compute_lateral_deviation ilkler; burada da korunuyor ki bu
        # metot tek başına çağrıldığında kontrol döngüsü istisnayla düşmesin.
        if self.lane_center_track is None:
            self.lane_center_track = [width / 2.0 + self.camera_center_offset_px] * len(self.sample_rows)

        ys, centers = [], []
        self.debug_rows = []
        prev_center = None

        for i, frac in enumerate(self.sample_rows):
            if frac > self.hood_frac:
                continue
            y = int(height * frac)
            top = max(0, y - band)
            bottom = min(height, y + band + 1)
            row_hit = (da_mask[top:bottom, :] > 0).any(axis=0)
            xs = np.flatnonzero(row_hit)
            if xs.size == 0:
                continue

            groups = [g for g in np.split(xs, np.flatnonzero(np.diff(xs) > gap) + 1)
                      if g.size >= min_width]
            if not groups:
                continue

            # EGO KORİDORU: aracın izlediği merkezi içeren parça. Yol ikiye
            # ayrılmış görünürse (bariyer, gölge) yanlış parçaya atlamamak için
            # önce bir alt satırın merkezi, o yoksa takip merkezi referans alınır.
            reference = prev_center if prev_center is not None else self.lane_center_track[i]

            if self.preferred_side and len(groups) > 1:
                # KAVŞAK: mecburi yön levhası görülmüşse yola en yakın olanı
                # değil, levhanın gösterdiği taraftaki kolu seç.
                chosen = (min if self.preferred_side < 0 else max)(
                    groups, key=lambda g: (g[0] + g[-1]) / 2.0)
            else:
                chosen = next((g for g in groups if g[0] <= reference <= g[-1]), None)
                if chosen is None:
                    # Hiçbir parça referansı içermiyor. En yakınına atlamak
                    # SINIRSIZ olamaz: yolun yanındaki bambaşka bir sürülebilir
                    # parça (servis yolu, açık alan) referanstan çok uzakta da
                    # olsa "en yakın" olabiliyor ve rota şerit bile olmayan bir
                    # yere kuruluyordu. Uzaksa bu satırı ÖLÇME - eksik satır,
                    # yanlış satırdan iyidir (kalan satırlar rotayı yine kurar).
                    chosen = min(groups, key=lambda g: min(abs(g[0] - reference),
                                                           abs(g[-1] - reference)))
                    uzaklik = min(abs(chosen[0] - reference), abs(chosen[-1] - reference))
                    if uzaklik > self.corridor_max_jump_frac * width:
                        continue

            left, right = float(chosen[0]), float(chosen[-1])
            # Kadraj dışına çıkan kenar: koridor gerçekte daha geniş, orta nokta
            # görünen tarafa kayar. Öğrenilmiş genişlikle diğer kenardan kurulur.
            left_clipped = chosen[0] <= 1
            right_clipped = chosen[-1] >= width - 2
            known = self.corridor_width_track[i]

            if left_clipped and right_clipped:
                if known is None:
                    continue                      # iki kenar da yok, ölçüm anlamsız
                center = (left + right) / 2.0     # elde bundan iyisi yok
            elif left_clipped and known is not None:
                center = right - known / 2.0
            elif right_clipped and known is not None:
                center = left + known / 2.0
            elif (known is not None and not self.preferred_side
                  and (right - left) > self.corridor_widen_tol * known):
                # YAN KOL AÇILDI. Sürülebilir alan yandaki şeride bitişik
                # olduğu için groups bunu ayıramaz (aralarında boşluk yok);
                # orta noktayı almak aracı o kola çeker. Takip edilen şeridin
                # ÖĞRENİLMİŞ genişliğini koru ve SÜREKLİ kalan kenara sabitle:
                # yan kol hangi taraftaysa o kenar referanstan uzağa sıçrar,
                # diğeri yerinde kalır. Böylece araç düz şeridinde kalır.
                # (Mecburi dönüş varsa bu dal atlanır - bkz. preferred_side.)
                if abs(left - (reference - known / 2.0)) <= abs(right - (reference + known / 2.0)):
                    center = left + known / 2.0
                else:
                    center = right - known / 2.0
                # Genişliği ÖĞRENME: bu ölçüm şeridin değil, birleşik alanın.
            else:
                # Buraya iki kenar da kadraj içindeyken gelinir; ya da bir kenar
                # kesikken henüz öğrenilmiş genişlik yokken (o durumda orta nokta
                # kaymış olur ama elde başka bir şey yok, genişliği ÖĞRENMEYİZ).
                center = (left + right) / 2.0
                if not (left_clipped or right_clipped):
                    measured = right - left
                    self.corridor_width_track[i] = (
                        measured if known is None else 0.85 * known + 0.15 * measured
                    )

            center = self._smooth_row_center(i, center)
            prev_center = center
            ys.append(float(y))
            centers.append(center)
            self.debug_rows.append((y, left, right, center))

        return ys, centers

    def _looks_like_paint(self, lane_mask, da_mask):
        """Bulunan çizgiler gerçekten YOL BOYASI mı, yoksa bariyer/korkuluk mu?

        İlk denemede ölçüt "kaç satırda sol+sağ çifti bulundu" idi ama bu işe
        yaramıyor: yolun iki yanındaki bariyerler de düzgün bir çift üretiyor -
        rotanın bariyerden kurulmasının sebebi tam olarak buydu.

        Ayırt edici şu: yol boyası sürülebilir alanın İÇİNDEdir, bariyer ise
        DIŞINDA kalır. Her iki çizgi noktası da sürülebilir alanla örtüşen
        satırları sayarız.
        """
        height, width = da_mask.shape
        tol = max(3, int(width * 0.006))     # boya tam sınırda olabilir
        drivable = da_mask > 0
        inside_rows = 0

        for y, left, right, _ in self.debug_rows:
            if left is None or right is None:
                continue
            y = int(np.clip(y, 0, height - 1))
            ok = True
            for x in (left, right):
                x0 = int(np.clip(x - tol, 0, width - 1))
                x1 = int(np.clip(x + tol, 0, width - 1)) + 1
                if not drivable[y, x0:x1].any():
                    ok = False
                    break
            if ok:
                inside_rows += 1

        return inside_rows >= self.min_paint_rows

    def _mesafe_sapmasi(self, lane_mask, da_mask):
        """Sağ çizgiye olan metrik mesafeyi hedefle kıyaslayıp sapma üretir."""
        height, width = lane_mask.shape
        olcum = self._sag_cizgi_mesafesi(lane_mask, da_mask)

        # BAND: ELEME DEĞİL KIRPMA.
        # Eskiden bandın dışını atıyordum ama bu, aracın şeritten KAÇTIĞI anı
        # da eliyordu: okuma 2.5 m'yi aşınca ölçüm yok sayılıyor, sapma
        # sönümleniyor ve araç tam düzeltmesi gereken anda kör kalıyordu.
        # Doğrusu yönü korumak: 2.5 m üstü okuma 'çok soldayım, sağa git'
        # bilgisini taşır; büyüklüğü sınırlanır ama bilgi atılmaz.
        band_disi = False
        ham_mesafe = None
        if olcum is not None:
            ham = olcum[0]
            # HAM değer saklanır: aşağıdaki ŞERİT GEÇİŞ testi bunu ister.
            # Kırpılmış değerle ('mesafe') test edilemez, çünkü geçiş anındaki
            # okuma tanımı gereği bandın DIŞINDADIR (bir şerit genişliği kadar
            # sıçramıştır) ve kırpma tam o bilgiyi siler.
            ham_mesafe = ham
            kirpik = float(np.clip(ham, self.mesafe_alt_sinir_m,
                                   self.mesafe_ust_sinir_m))
            if kirpik != ham:
                band_disi = True
                olcum = (kirpik, olcum[1], olcum[2])

        # İLK REFERANS BAND İÇİNDEN KURULUR.
        # Sıçrama kilidi ancak elde bir referans varken koruyor (onceki is not
        # None). İlk karede referans YOKTU, yani ilk ölçüm ne gelirse takip
        # edilecek çizgi o oluyordu - yanlış çizgiyle başlarsak kilit de o
        # yanlışı savunuyor. Band dışı bir okumayla referans KURMUYORUZ;
        # ölçüm yokmuş gibi davranıp makul bir kare bekliyoruz.
        if olcum is not None and self.son_mesafe_m is None and band_disi:
            if not getattr(self, '_ilk_referans_uyarildi', False):
                self._ilk_referans_uyarildi = True
                self.get_logger().warn(
                    f'⏳ İlk referans BEKLETİLDİ: ölçüm {ham:.2f} m, makul band '
                    f'{self.mesafe_alt_sinir_m:.1f}-{self.mesafe_ust_sinir_m:.1f} m '
                    f'dışında (hedef {self.hedef_sag_mesafe_m:.1f} m). Yanlış '
                    f'çizgiye kilitlenmemek için bu kare atlandı.')
            olcum = None

        if olcum is None:
            # ÇİZGİ KAYBOLDU. Viraj algılama KALDIRILDI (2026-08-19): tek kural
            # 'sağdaki çizgiyle hedef mesafeyi koru'. Çizgi yokken uydurulacak
            # bir komut da yok - son komut sönümlenerek sürer.
            self.lost_frames += 1
            self.debug_fit = None
            self.debug_center = None
            self.debug_curve = 0.0
            self.son_mesafe_m = None

            # Kısa kayıpta kontrolü BIRAKMA: eski komut sönümlenerek sürer.
            self.lane_valid = self.lost_frames <= self.gecerlilik_kayip_kare

            self.debug_source = 'mesafe-disi' if band_disi else 'mesafe-yok'
            son = self.deviation_history[-1] if self.deviation_history else 0.0
            sonuc = son * 0.85
            self._push_history(sonuc)
            return float(sonuc)

        mesafe, u, v = olcum

        # ŞERİT DEĞİŞTİRMEME KİLİDİ: takip edilen çizgi kaybolduğunda 'en yakın
        # sağ çizgi' yan şeridin çizgisi olur ve mesafe bir anda sıçrar. Gerçek
        # yanal hareket bu kadar ani olamaz; sıçrayan ölçümü KABUL ETME, araç
        # kendi şeridinde kalsın. (Ölçüm yokmuş gibi davranılır: sapma sönümlenir.)
        onceki = self.son_mesafe_m
        if (onceki is not None
                and abs(mesafe - onceki) > self.mesafe_sicrama_esigi_m):
            # ŞERİT DEĞİŞTİRME Mİ? İKİ YÖNDE DE olabilir; ikisi de aynı şeyi
            # anlatır: takip edilen çizgi değişti, okumayı ESKİ çizgiye geri
            # çevirmek gerekir.
            #
            #   SOL çizgi geçildi -> o çizgi artık SAĞIMIZDA kalır ve 'en yakın
            #     sağ çizgi' bir anda o olur: okuma bir ŞERİT GENİŞLİĞİ kadar
            #     DÜŞER. Geri çevirmek için +W. Araç 'çok soldayım' bilgisini
            #     alır ve sağa döner.
            #
            #   SAĞ çizgi geçildi -> takip ettiğimiz çizgi SOLUMUZDA kaldığı
            #     için artık hiç sayılmıyor (x > u_arac şartı) ve 'en yakın sağ
            #     çizgi' KOMŞU şeridin çizgisi oluyor: okuma bir şerit genişliği
            #     kadar ARTAR. Geri çevirmek için -W. Sonuç NEGATİF çıkabilir ve
            #     ÇIKMALIDIR: 'kendi çizgimin 10 cm sağındayım' bilgisi budur,
            #     hata büyük negatif olur ve araç sola çekilir.
            #
            # Eskiden yalnız +W denetleniyordu. Sağa taşma bu yüzden hiç
            # yakalanmıyordu: sıçrama kilidi mesafe_sicrama_kabul_kare kadar
            # bekleyip KOMŞU şeridin çizgisini referans kabul ediyor, hata
            # yeniden sıfıra oturuyor ve araç 'sağa kır' komutunu sürdürerek
            # komşu şeride yerleşiyordu - kontrolcüye göre her şey yolunda.
            #
            # Aday HAM okumadan türetilir (ham_mesafe): geçiş anındaki okuma
            # tanımı gereği makul bandın dışındadır, kırpılmış değerle bu test
            # hiçbir zaman tutmaz.
            W = self.serit_genisligi_m
            duzeltilmis = None
            gecilen = None
            if W > 0 and ham_mesafe is not None:
                for aday, yon in ((ham_mesafe + W, 'SOL'), (ham_mesafe - W, 'SAĞ')):
                    if (abs(aday - onceki) < abs(ham_mesafe - onceki)
                            and abs(aday - onceki) <= self.mesafe_sicrama_esigi_m):
                        duzeltilmis, gecilen = aday, yon
                        break
            if duzeltilmis is not None:
                if self._sicrama_sayaci == 0:
                    self.get_logger().warn(
                        f'🚧 {gecilen} ŞERİT ÇİZGİSİ GEÇİLDİ: ölçüm '
                        f'{ham_mesafe:.2f} m (referans {onceki:.2f} m). Bir '
                        f'şerit genişliği '
                        f'{"eklenip" if gecilen == "SOL" else "çıkarılıp"} '
                        f'{duzeltilmis:.2f} m sayıldı - araç kendi şeridine '
                        f'geri çekiliyor.')
                self._sicrama_sayaci = 0
                self.lost_frames = 0
                self.lane_valid = True
                self.debug_source = 'serit-gecildi'
                self.son_mesafe_m = duzeltilmis
                hata = duzeltilmis - self.hedef_sag_mesafe_m
                band = max(self.merkez_bandi_m, 0.0)
                etkin = (hata if abs(hata) <= band else math.copysign(
                    band + (abs(hata) - band) * max(self.kenar_kazanci, 1.0), hata))
                return self._yumusat_ve_sinirla(float(np.clip(
                    etkin / max(self.mesafe_hata_olcegi_m, 1e-3), -1.0, 1.0)))

            # KİLİT TAKILI KALMASIN: üst üste bu kadar kare aynı şeyi
            # ölçüyorsak sıçrama değil gerçek hareket; referansı yenile.
            self._sicrama_sayaci += 1
            if self._sicrama_sayaci >= max(self.mesafe_sicrama_kabul_kare, 1):
                self.get_logger().warn(
                    f'↔️  Sıçrama kilidi {self._sicrama_sayaci} karedir açık '
                    f'(referans {onceki:.2f} m, ölçüm {mesafe:.2f} m) - '
                    f'gerçek hareket sayılıp referans yenilendi.')
                self._sicrama_sayaci = 0
                self.son_mesafe_m = mesafe
                self.lost_frames = 0
            else:
                self.lost_frames += 1
                self.debug_source = 'mesafe-sicrama'
                # Bilerek eski şeridi koruyoruz: bu bir KARAR, kontrol kaybı değil.
                self.lane_valid = self.lost_frames <= self.gecerlilik_kayip_kare
                # son_mesafe_m KORUNUR: hedefimiz hâlâ eski çizgi.
                son = self.deviation_history[-1] if self.deviation_history else 0.0
                sonuc = son * 0.85
                self._push_history(sonuc)
                return float(sonuc)

        self._sicrama_sayaci = 0

        self.lost_frames = 0
        self.lane_valid = True
        self.debug_source = 'mesafe'
        self.son_mesafe_m = mesafe
        self.debug_center = (int(u), int(v))
        self.debug_fit = None
        self.debug_curve = 0.0

        # Hedeften SAPMA. mesafe > hedef ise çizgi olması gerekenden uzakta,
        # yani araç fazla SOLDA -> sağa kırmalı (pozitif sapma sağa kırdırır).
        # TEK KURAL: sağdaki en yakın çizgiyle aramızda hep hedef mesafe kalsın.
        # Viraj için AYRI bir yanlılık EKLENMEZ - çizgi virajda kavis yaptığı
        # için 1.5 m'yi korumak aracı zaten virajdan geçirir. Üstüne yanlılık
        # eklemek bu kuralla çekişir ve şeritten kaydırırdı.
        hata = mesafe - self.hedef_sag_mesafe_m

        # KADEMELİ TEPKİ: merkez bandı içinde doğrusal ve yumuşak, dışında
        # kenar_kazanci ile büyütülmüş. Amaç şeridin ORTASINDA kalmak ve
        # çizgiye yaklaşıldığında kararlı biçimde geri çekilmek.
        band = max(self.merkez_bandi_m, 0.0)
        if abs(hata) <= band:
            etkin = hata
        else:
            etkin = math.copysign(
                band + (abs(hata) - band) * max(self.kenar_kazanci, 1.0), hata)
        sapma = etkin / max(self.mesafe_hata_olcegi_m, 1e-3)
        return self._yumusat_ve_sinirla(float(np.clip(sapma, -1.0, 1.0)))

    def depth_callback(self, msg):
        self.depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def camera_info_callback(self, msg):
        self.fx = float(msg.k[0])
        self.cx = float(msg.k[2])

    def _derinlik_oku(self, u, v):
        """(u,v) çevresindeki geçerli derinliklerin medyanı (metre), yoksa None.

        Tek piksel okumak güvenilmez: stereo derinlikte delikler (nan/inf) ve
        kenar gürültüsü var. Çizginin kendisi yüksek kontrastlı olduğu için
        çevresindeki küçük pencerede genelde yeterli geçerli ölçüm bulunur.
        """
        d = self.depth_image
        if d is None:
            return None
        r = self.derinlik_pencere_px
        y0, y1 = max(0, v - r), min(d.shape[0], v + r + 1)
        x0, x1 = max(0, u - r), min(d.shape[1], u + r + 1)
        parca = d[y0:y1, x0:x1]
        gecerli = parca[np.isfinite(parca) & (parca > 0.1) & (parca < 40.0)]
        if gecerli.size < 3:
            return None
        return float(np.median(gecerli))

    def _sag_cizgi_mesafesi(self, lane_mask, da_mask):
        """Araç orta çizgisinin SAĞINDAKİ en yakın çizgiye METRİK uzaklık.

        Dönen: (mesafe_m, u, v) ya da None.

        Yanal uzaklık pinhole modelinden: X = (u - u_arac) * Z / fx.
        u_arac aracın orta çizgisi (ekrandaki mavi çizgi), Z o pikseldeki
        ZED derinliği. Perspektif/ufuk varsayımı YOK - ölçülen mesafe bu.

        BARİYER ELEMESİ (da_mask): model bariyer/korkuluk kenarlarını da
        'şerit çizgisi' sayıyor. Bu fonksiyon SADECE 'en yakın sağ nokta'ya
        baktığı için bariyer, gerçek çizgiden daha yakınsa onun yerine
        ölçülüyordu - araç var olmayan bir çizgiyi 1.5 m'de tutmaya çalışıp
        şeritten çıkıyor. Piksel modundaki _lane_centers_at_rows bu elemeyi
        zaten yapıyordu (bkz. _bariyerleri_ele); metrik mod atlıyordu.
        """
        if self.fx is None or self.depth_image is None:
            return None
        height, width = lane_mask.shape
        band = max(2, int(height * 0.006))
        merge_gap = max(4, int(width * 0.012))
        max_cluster_px = int(width * 0.25)
        # _lane_centers_at_rows ile AYNI tolerans: iki yol da aynı bariyeri
        # aynı şekilde elesin, yoksa mod değişince davranış sessizce kayar.
        bariyer_tol = max(3, int(width * 0.006))
        u_arac = width / 2.0 + self.camera_center_offset_px

        # Tüm satırlardan ölç, sonra HEDEF İLERİ MESAFEYE en yakın olanı seç.
        # 'İlk veri veren satır' yaklaşımı, satır değiştikçe ölçümü zıplatıyordu.
        adaylar = []
        for frac in self.sample_rows:
            v = int(height * frac)
            noktalar = self._row_lane_points(lane_mask, v, band, merge_gap,
                                             max_cluster_px)
            # Bariyerleri EN BAŞTA ele: 'en yakın sağ nokta' seçimine hiç
            # girmesinler. Sonradan elemek işe yaramaz - seçim çoktan yapılmış olur.
            if self.paint_inside_only:
                noktalar = self._bariyerleri_ele(noktalar, v, da_mask, bariyer_tol)
            sagdakiler = [x for x in noktalar if x > u_arac]
            if not sagdakiler:
                continue
            u = int(min(sagdakiler))          # en yakın SAĞ çizgi
            z = self._derinlik_oku(u, v)
            if z is None:
                continue
            adaylar.append(((u - u_arac) * z / self.fx, z, u, v))

        if not adaylar:
            return None

        # ÖNCE EN YAKIN SAĞ ÇİZGİYE KİLİTLEN, SONRA ÖLÇ.
        # Her satır kendi 'en yakın sağ noktasını' veriyor ama satırlar
        # birbirinden BAĞIMSIZ: alttaki satır gerçek şerit çizgisini, üstteki
        # satır saha kenarını/bariyeri yakalayabiliyor. Eskiden bütün adaylara
        # TEK bir doğru uyduruluyordu; farklı cisimler aynı fite girince sonuç
        # ikisinin arasında bir yere düşüyor ve aracın kendi çizgisiyle ilgisi
        # kalmıyordu. Pistte görülen buydu: bariyerden gelen 3.5-6 m'lik okuma
        # makul banda KIRPILIP 2.50 m diye geçiyor, hata sabit +1.0 m kalıyor
        # ve direksiyon sağa dayanmış halde kilitleniyordu.
        #
        # Aracın KENDİ sağ çizgisi, tanımı gereği sağdaki EN YAKIN çizgidir.
        # O yüzden önce en küçük yanal mesafeli aday bulunur, sonra yalnızca
        # onunla AYNI ÇİZGİYE ait olabilecek adaylar tutulur; bariyer ve komşu
        # şeridin çizgisi ölçüme hiç girmez.
        en_kucuk = min(p[0] for p in adaylar)
        adaylar = [p for p in adaylar
                   if p[0] - en_kucuk <= self.ayni_cizgi_tol_m]

        # HEDEF MESAFEDEKİ DEĞERİ ARA DEĞERLEMEYLE bul. 'En yakın noktayı seç'
        # yetmiyor: o mesafede hiç nokta yoksa ölçüm yine zıplıyor. Noktalara
        # doğru uydurup hedef Z'de değerlendirmek, hangi satırların veri
        # verdiğinden BAĞIMSIZ ve karşılaştırılabilir bir mesafe üretir.
        # (Artık fit YALNIZCA tek bir çizginin noktalarına çekiliyor.)
        hedef_z = self.olcum_ileri_mesafe_m
        en_yakin = min(adaylar, key=lambda p: abs(p[1] - hedef_z))
        if len(adaylar) >= 2:
            X = np.array([p[0] for p in adaylar])
            Z = np.array([p[1] for p in adaylar])
            if Z.max() - Z.min() > 0.3:
                m, c = np.polyfit(Z, X, 1)
                # Ölçüm aralığının dışına taşma: uzağa uzatmak güvenilmez
                z_kirp = float(np.clip(hedef_z, Z.min(), Z.max()))
                deger = float(m * z_kirp + c)
                # UYDURULAN DEĞER ÖLÇÜLENLERİN DIŞINA ÇIKAMAZ.
                # 2-3 noktaya çekilen doğru, Z aralığının İÇİNDE bile ölçülen
                # en küçük X'in altına inebiliyor. Kayıtta (2026-08-19)
                # -0.16 m okundu; negatif mesafe fiziksel olarak imkânsız
                # (yalnızca aracın SAĞINDAKİ noktalar alınıyor) ama sapmayı
                # -0.42'ye götürüp direksiyonu ters tarafa kırdırıyor.
                return (float(np.clip(deger, X.min(), X.max())),
                        en_yakin[2], en_yakin[3])
        return en_yakin[0], en_yakin[2], en_yakin[3]

    def _route_centers(self, lane_mask, da_mask):
        """Rota noktalarını seçilen kaynaktan üretir (bkz. route_source parametresi)."""
        if self.route_source in ('serit', 'auto'):
            ys, centers = self._lane_centers_at_rows(lane_mask, da_mask)
            if self.route_source == 'serit':
                return ys, centers, 'serit'
            # HİSTEREZİS: 'auto'da kaynak kare kare değişirse rota merkezi iki
            # farklı yerden kurulur, sapma işaret değiştirir ve direksiyon
            # sağa-sola savrulur - yani kararsızlığı çözmek yerine üretir.
            # Karar ancak ARKA ARKAYA aynı sonucu veren kareler sonrası döner.
            if self._looks_like_paint(lane_mask, da_mask):
                self.paint_votes = min(self.paint_votes + 1, self.source_switch_frames)
            else:
                self.paint_votes = max(self.paint_votes - 1, -self.source_switch_frames)
            if self.paint_votes >= self.source_switch_frames:
                return ys, centers, 'serit'

        ys, centers = self._corridor_centers_at_rows(da_mask)
        if ys:
            return ys, centers, 'yol'

        # Sürülebilir alan da yoksa şerit tahminine geri dön (hiç veri yok demek
        # değil: tek çizgiden kurulmuş merkez bile düz gitmekten iyidir)
        if self.route_source != 'serit':
            ys, centers = self._lane_centers_at_rows(lane_mask, da_mask)
            return ys, centers, 'serit*'
        return [], [], '-'

    def compute_lateral_deviation(self, lane_mask, da_mask):
        """Rota merkezine göre aracın yanal sapmasını hesaplar (-1..+1).

        Pozitif = rota merkezi aracın sağında, yani araç rotanın solunda.
        """
        height, width = lane_mask.shape

        # METRİK MOD: aracın orta çizgisi ile sağdaki çizgi arasındaki GERÇEK
        # mesafeyi hedefte tutar. Şerit ortası hesabına, şerit genişliğine,
        # ufuk kalibrasyonuna ihtiyaç duymaz - gerçek kayıtta bunların hepsi
        # kararsızdı (satırların %85'inde tek çizgi görülüyor).
        if self.route_source == 'mesafe':
            return self._mesafe_sapmasi(lane_mask, da_mask)

        center_ref = width / 2.0 + self.camera_center_offset_px

        if self.lane_center_track is None:
            self.lane_center_track = [center_ref] * len(self.sample_rows)

        ys, centers, self.debug_source = self._route_centers(lane_mask, da_mask)

        if not ys:
            # Şerit hiç görülmedi. Eski sapmayı olduğu gibi sürdürmek aracı
            # dönmeye devam ettiriyordu; kademeli olarak düze getir.
            self.lost_frames += 1
            self.lane_valid = False
            if self.lost_frames > 15:
                # Uzun süre kayıpsa takip merkezini sıfırla, yoksa şerit geri
                # geldiğinde bayat merkez yanlış sol/sağ eşleşmesine yol açıyor.
                self.lane_center_track = [center_ref] * len(self.sample_rows)
                self.lane_width_track = [None] * len(self.sample_rows)
                self.corridor_width_track = [None] * len(self.sample_rows)
                self.left_line_track = [None] * len(self.sample_rows)
                self.right_line_track = [None] * len(self.sample_rows)
                self.row_meas_track = [None] * len(self.sample_rows)
                self.row_meas_frame = [-999] * len(self.sample_rows)
            last = self.deviation_history[-1] if self.deviation_history else 0.0
            decayed = last * 0.85
            self._push_history(decayed)
            self.lane_slope = None
            self.debug_fit = None
            self.debug_center = None
            self.debug_curve = 0.0
            return float(decayed)

        self.lost_frames = 0
        self.lane_valid = True

        # Satır merkezlerine eğri uydur. Viraj İKİNCİ dereceden bir eğridir;
        # doğru uydurmak virajı düzleştirip ileri bakış noktasındaki kaymayı
        # olduğundan küçük gösteriyor, yani aracı geniş döndürüyordu.
        y_span = max(ys) - min(ys)
        if len(ys) >= 4 and y_span > height * 0.15:
            poly = np.poly1d(np.polyfit(ys, centers, 2))
        elif len(ys) >= 2:
            poly = np.poly1d(np.polyfit(ys, centers, 1))
        else:
            poly = np.poly1d([centers[0]])
        self.lane_slope = poly.deriv()

        # Eğri SADECE ölçüm yapılan satır aralığında kullanılır. Uzak satırlar
        # görülmediğinde ikinci dereceden eğriyi dışarı uzatmak çılgın değerler
        # üretebiliyor; görülmeyen mesafe için viraj iddia etmemek doğrusu.
        y_min, y_max = min(ys), max(ys)

        def fit(y):
            return float(poly(float(np.clip(y, y_min, y_max))))

        y_look = height * self.look_ahead_frac
        lane_center = fit(y_look)

        # Bir sonraki karenin sol/sağ ayrımı için satır takipçilerini güncelle
        for i, frac in enumerate(self.sample_rows):
            target = float(fit(height * frac))
            self.lane_center_track[i] = 0.6 * self.lane_center_track[i] + 0.4 * target

        # Eğri artık doğrusal olmayabilir; satır satır örnekleyip çiz
        self.debug_fit = np.array(
            [[int(fit(height * frac)), int(height * frac)] for frac in self.sample_rows],
            dtype=np.int32)
        self.debug_center = (int(lane_center), int(y_look))

        # 1) Yanal sapma: araç şeridin neresinde (gecikmeli ama kararlı terim)
        cross_track = (lane_center - center_ref) / (width / 2.0)

        # 2) Viraj ileri beslemesi: şerit ileride nereye gidiyor. Uzak satırdaki
        # şerit merkezi ile yakın satırdakinin farkı. Düz yolda araç şeridin
        # kenarında olsa bile bu fark ~0'dır; sadece viraj varken büyür, o yüzden
        # düz gidişi bozmadan direksiyonu erken çevirir.
        near = float(fit(height * self.sample_rows[0]))
        far = float(fit(height * self.sample_rows[-1]))
        curve_term = (far - near) / (width / 2.0)
        self.debug_curve = curve_term

        deviation = float(np.clip(cross_track + self.curve_feedforward * curve_term, -1.0, 1.0))

        return self._yumusat_ve_sinirla(deviation)

    def _yumusat_ve_sinirla(self, deviation):
        """Hız sınırı + ağırlıklı ortalama + ölü bant. Her rota kaynağı kullanır."""
        # Tek karelik yanlış eşleşme direksiyonu bir anda kilitlemesin. Sınır
        # saniye bazlı: düşük FPS'te kare başına sınır viraja girişi geciktiriyordu.
        now = time.monotonic()
        dt = 0.1 if self.last_update_time is None else min(now - self.last_update_time, 0.5)
        self.last_update_time = now
        max_step = self.max_deviation_rate * dt

        previous = self.deviation_history[-1] if self.deviation_history else 0.0
        deviation = float(np.clip(deviation, previous - max_step, previous + max_step))
        self._push_history(deviation)

        if len(self.deviation_history) >= 3:
            # Son 3 değerin ağırlıklı ortalaması (en son değer daha ağırlıklı)
            weights = np.array([0.2, 0.3, 0.5])
            smooth_deviation = float(np.average(self.deviation_history[-3:], weights=weights))
        else:
            smooth_deviation = deviation

        # Ölü bant: düz yolda piksel gürültüsü yüzünden sürekli düzeltme yapıp
        # zikzak çizmesini engeller.
        if abs(smooth_deviation) < self.deviation_deadband:
            smooth_deviation = 0.0

        return float(smooth_deviation)

    def _satir_tanisi(self, width):
        """Satır satır koridor kenarlarını basar (debug_rows_log açıkken).

        Rota yamuk çıktığında sebep tek bir satırda başlar: koridor orada yandaki
        açık zeminle birleşir, genişler ve merkezi kayar. Ekrandaki tek 'Merkez'
        sayısı bunu göstermiyor, bu tablo gösteriyor.
        """
        # METRİK MOD KENDİ TANISINI BASAR. debug_rows yalnızca PİKSEL modunun
        # fonksiyonlarında doldurulur; metrik modda o fonksiyonlar hiç
        # çağrılmadığı için liste hep boş kalıyor ve bu satır her karede
        # 'hiç satır ölçülemedi' diye bağırıyordu - ölçüm gayet çalışırken.
        # Yanlış alarm, gerçek arızayı gizliyordu.
        if self.route_source == 'mesafe':
            m = self.son_mesafe_m
            self.get_logger().info(
                f'TANI (metrik): mesafe='
                f'{"YOK" if m is None else f"{m:.2f} m"} | hedef '
                f'{self.hedef_sag_mesafe_m:.2f} m | kaynak {self.debug_source} | '
                f'kayip kare {self.lost_frames} | serit gecerli {self.lane_valid} | '
                f'derinlik {"var" if self.depth_image is not None else "YOK"} | '
                f'fx {"var" if self.fx is not None else "YOK"}')
            return

        if not self.debug_rows:
            self.get_logger().info('TANI: hiç satır ölçülemedi')
            return
        ref = width / 2.0 + self.camera_center_offset_px
        satirlar = []
        for y, sol, sag, merkez in self.debug_rows:
            genislik = (sag - sol) if (sol is not None and sag is not None) else float('nan')
            satirlar.append(
                f'y={y:3d} sol={sol if sol is None else int(sol):>4} '
                f'sag={sag if sag is None else int(sag):>4} '
                f'gen={genislik:5.0f} mrk={int(merkez):>4} '
                f'({merkez - ref:+5.0f})')
        adaylar = {y: pts for y, pts in getattr(self, 'debug_points', [])}
        for k, (y, sol, sag, merkez) in enumerate(self.debug_rows):
            pts = adaylar.get(y)
            if pts is not None:
                satirlar[k] += '  adaylar=[' + ' '.join(f'{int(x)}' for x in pts) + ']'
        genislikler = list(self.width_samples)
        ozet = ('-' if not genislikler
                else f'{float(np.median(genislikler)):.0f}px (n={len(genislikler)})')
        self.get_logger().info(
            f'TANI (ref={ref:.0f}, kaynak={self.debug_source}, '
            f'viraj={self.debug_curve:+.3f}, ogrenilen_genislik={ozet})\n  '
            + '\n  '.join(satirlar))

    def _push_history(self, value):
        self.deviation_history.append(value)
        if len(self.deviation_history) > self.max_history_size:
            self.deviation_history.pop(0)

    def _blend_masks(self, img, da_mask, ll_mask):
        """Maskeleri kare üstüne yarı saydam basar: yeşil = yol, kırmızı = şerit.

        utils.show_seg_result yerine bu kullanılıyor. O fonksiyon her karede
        720x1280x3 float64 ara diziler üretiyor (np.mean) ve maliyeti maskenin
        kapladığı alanla büyüyordu: ölçümde yol kareyi doldurduğunda ~145 ms,
        yani kontrol döngüsünü 3 FPS'e düşüren asıl darboğaz buydu. Bu sürüm
        aynı görüntüyü üretiyor (ortalama piksel farkı 0.05) ama tamsayı
        aritmetiğiyle tek geçişte.
        """
        ll_b = ll_mask.astype(bool, copy=False)
        da_b = da_mask.astype(bool, copy=False) & ~ll_b
        img[da_b] = img[da_b] // 2 + np.uint8([0, 127, 0])    # yeşil (BGR)
        img[ll_b] = img[ll_b] // 2 + np.uint8([0, 0, 127])    # kırmızı (BGR)

    def draw_lane_debug(self, im0, scale=1.0):
        """Tespit edilen çizgileri, şerit merkezini ve referans merkezi çizer.

        Ölçümler TAM çözünürlük koordinatında tutulur; scale, küçültülmüş debug
        karesine çizerken uygulanır.
        """
        height, width = im0.shape[:2]
        full_width = width / scale
        center_ref = int((full_width / 2 + self.camera_center_offset_px) * scale)

        # Mavi = aracın referans merkezi (kamera kayması dahil)
        cv2.line(im0, (center_ref, int(height * 0.6)), (center_ref, height), (255, 0, 0), 2)

        # METRİK MODDA sadece iki şey çizilir: aracın orta çizgisi (mavi) ve
        # ölçüm alınan sağ çizgi noktası. Şerit ortası/eğri/satır işaretçileri
        # bu modda kullanılmıyor; ekranda tutmak yanıltıcı olur.
        if self.route_source == 'mesafe':
            if self.debug_center is not None:
                u = int(self.debug_center[0] * scale)
                v = int(self.debug_center[1] * scale)
                cv2.drawMarker(im0, (u, v), (0, 255, 255), cv2.MARKER_TILTED_CROSS,
                               max(12, int(24 * scale)), 2)
                cv2.line(im0, (center_ref, v), (u, v), (0, 255, 255), 2)
            return

        # Sol çizgi işaretçisi MOR: eskiden kırmızıydı, ama şerit maskesi de
        # kırmızı olduğu için işaretçi maskenin içinde kayboluyordu - ekranda
        # "sol çizgi hiç bulunamıyor" gibi görünüyordu, oysa bulunuyordu.
        radius = max(2, int(round(5 * scale)))
        for y, left, right, center in self.debug_rows:
            y_s = int(y * scale)
            if left is not None:
                cv2.circle(im0, (int(left * scale), y_s), radius, (255, 0, 255), -1)   # mor = sol çizgi
            if right is not None:
                cv2.circle(im0, (int(right * scale), y_s), radius, (0, 255, 255), -1)  # sarı = sağ çizgi
            cv2.circle(im0, (int(center * scale), y_s), radius, (0, 255, 0), -1)       # yeşil = satır merkezi

        if self.debug_fit is not None:
            cv2.polylines(im0, [(self.debug_fit * scale).astype(np.int32)],
                          False, (0, 255, 0), 2)
        if self.debug_center is not None:
            cv2.drawMarker(im0, (int(self.debug_center[0] * scale), int(self.debug_center[1] * scale)),
                           (0, 165, 255), cv2.MARKER_TILTED_CROSS,
                           max(10, int(20 * scale)), 2)

    def publish_debug_view(self, im0s, da_mask, ll_mask, lateral_deviation, intersection_direction):
        """Debug görüntüsünü üretip yayınlar. KONTROL YOLUNDA DEĞİL: sadece insan
        için, o yüzden düşük çözünürlükte (debug_scale) ve her karede değil
        (debug_every_n). 'Merkez / ref' sayıları kalibrasyon için TAM çözünürlük
        pikselinde kalır, küçültmeden etkilenmez.
        """
        s = self.debug_scale
        height, width = im0s.shape[:2]

        if s != 1.0:
            dw, dh = int(width * s), int(height * s)
            view = cv2.resize(im0s, (dw, dh), interpolation=cv2.INTER_LINEAR)
            da_mask = cv2.resize(da_mask.astype(np.uint8), (dw, dh), interpolation=cv2.INTER_NEAREST)
            ll_mask = cv2.resize(ll_mask, (dw, dh), interpolation=cv2.INTER_NEAREST)
        else:
            view = im0s.copy()

        self._blend_masks(view, da_mask, ll_mask)
        self.draw_lane_debug(view, s)

        center_ref = int(width / 2 + self.camera_center_offset_px)
        lane_center_px = self.debug_center[0] if self.debug_center else -1
        if self.route_source == 'mesafe':
            m = getattr(self, 'son_mesafe_m', None)
            debug_text = (f'LatDev: {lateral_deviation:+.3f} | '
                          f'Sag cizgi: {"--" if m is None else f"{m:.2f}"} m '
                          f'(hedef {self.hedef_sag_mesafe_m:.2f} m) | '
                          f'Kaynak: {self.debug_source} | '
                          f'FPS: {self.fps:.1f}')
        else:
            debug_text = (f'LatDev: {lateral_deviation:+.3f} | '
                          f'Viraj: {self.debug_curve:+.3f} | '
                          f'Merkez: {lane_center_px} / ref {center_ref} | '
                          f'Kaynak: {self.debug_source} | '
                          f'FPS: {self.fps:.1f} | '
                          f'Int: {intersection_direction}')
        cv2.putText(view, debug_text, (10, max(20, int(40 * s))), cv2.FONT_HERSHEY_SIMPLEX,
                    max(0.45, 0.8 * s), (0, 255, 0), max(1, int(round(2 * s))))

        self.publisher.publish(self.br.cv2_to_imgmsg(view, encoding="bgr8"))

    def detect_brightness_level(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        return brightness

    def detect_horizontal_lines(self, lane_mask):
        height, width = lane_mask.shape

        roi_start = int(0.5 * height)
        roi_end = int(0.85 * height)
        # Maske 0/1 değerli; Canny'nin 50/150 eşikleri bu ölçekte HİÇBİR kenar
        # bulamıyordu (yani kavşak tespiti hiç çalışmıyordu). 0-255'e ölçekle.
        # Ayrıca kopya al: aşağıdaki çizimler asıl maskeyi bozmasın.
        roi = (lane_mask[roi_start:roi_end, :] > 0).astype(np.uint8) * 255

        blurred = cv2.GaussianBlur(roi, (5,5), 0)
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                                minLineLength=width // 4, maxLineGap=20)

        has_horizontal = False
        strongest = 0

        if lines is not None:
            horizontal_lines = []
            for line in lines:
                x1,y1,x2,y2 = line[0]
                angle = np.arctan2((y2 - y1), (x2 - x1)) * 180 / np.pi
                if abs(angle) < 15:
                    horizontal_lines.append(line)

            strongest = len(horizontal_lines)
            has_horizontal = strongest > 0

        return has_horizontal, strongest

    def detect_intersection_direction(self, lane_mask, original_image):
        height, width = lane_mask.shape

        has_horizontal_line, horizontal_strength = self.detect_horizontal_lines(lane_mask)

        if not has_horizontal_line:
            intersection_flags = 0
            self.stable_intersection_count = 0
        else:
            left_band = lane_mask[:, :width // 3]
            right_band = lane_mask[:, -width // 3:]

            left_sum = np.sum(left_band)
            right_sum = np.sum(right_band)

            direction_threshold = 1.3

            if left_sum > right_sum * direction_threshold:
                intersection_flags = 1
            elif right_sum > left_sum * direction_threshold:
                intersection_flags = 2
            else:
                intersection_flags = 4

        self.intersection_history.append(intersection_flags)
        if len(self.intersection_history) > self.intersection_history_size:
            self.intersection_history.pop(0)

        if len(self.intersection_history) >= self.min_stable_frames:
            if all(flag == intersection_flags for flag in self.intersection_history[-self.min_stable_frames:]):
                self.stable_intersection_count += 1
            else:
                self.stable_intersection_count = 0

        if self.stable_intersection_count >= self.min_stable_frames:
            final_direction = intersection_flags
        else:
            final_direction = 0

        # Bu log her karede yazılıyordu: üç düğüm birlikte terminale saniyede
        # onlarca satır basıp hem okunmaz hale getiriyor hem de konsol I/O'suyla
        # döngüden zaman yiyordu. Sadece yön değişince ya da ~30 karede bir yaz.
        # (Parlaklık da yalnızca yazılacaksa hesaplanır: cvtColor + mean ~2 ms.)
        if final_direction != self.last_logged_direction or self.frame_count % 30 == 0:
            self.last_logged_direction = final_direction
            brightness = self.detect_brightness_level(original_image)
            self.get_logger().info(
                f'Brightness: {brightness:.1f} | Horizontal: {has_horizontal_line} ({horizontal_strength}) | '
                f'Direction: {final_direction} | Stable: {self.stable_intersection_count} | '
                f'FPS: {self.fps:.1f}'
            )
        return final_direction

    def image_callback(self, msg):
        try:
            # İşlem hızı: tepki süresi ve max_deviation_rate buna bağlı,
            # ayar yapmadan önce bu sayının kaç olduğunu bilmek gerekiyor.
            now = time.monotonic()
            if self.last_frame_time is not None:
                dt = now - self.last_frame_time
                if dt > 0:
                    self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt) if self.fps else 1.0 / dt
            self.last_frame_time = now

            im0s = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = self.preprocess_image(im0s)
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()
            img /= 255.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            with torch.no_grad():
                [pred, anchor_grid], seg, ll = self.model(img)

            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)
            ll_seg_mask = self.unletterbox_mask(ll_seg_mask, im0s.shape[:2])

            self.frame_count += 1

            # --- KONTROL YOLU: her karede, hızlı tutulmalı ---------------------
            # Direksiyonun gecikmesi doğrudan buraya bağlı, o yüzden bu blokta
            # görselleştirme işi YAPILMAZ.
            lateral_deviation = self.compute_lateral_deviation(ll_seg_mask, da_seg_mask)
            self.lateral_pub.publish(Float32(data=lateral_deviation))
            if self.debug_center is not None:
                self.center_px_pub.publish(Float32(data=float(self.debug_center[0])))
            self.lane_valid_pub.publish(Bool(data=bool(self.lane_valid)))
            self.curve_pub.publish(Float32(data=float(self.debug_curve)))
            ham = self.raw_width_meas
            if ham is None and self.lane_width_track[0] is not None:
                ham = self.lane_width_track[0]
            if ham is not None:
                self.width_px_pub.publish(Float32(data=float(ham)))

            if self.debug_rows_log and self.frame_count % 15 == 0:
                self._satir_tanisi(ll_seg_mask.shape[1])

            intersection_direction = self.detect_intersection_direction(ll_seg_mask, im0s)
            self.intersection_pub.publish(Int32(data=intersection_direction))

            # --- GÖRSELLEŞTİRME: seyrek ve düşük çözünürlüklü ------------------
            # Görüntü ayrı pencerede gösterilmiyor; birleşik izleme düğümü
            # (combined_view.py) bu topic'i dinleyip tek pencerede gösterir.
            if self.debug_every_n > 0 and self.frame_count % self.debug_every_n == 0:
                self.publish_debug_view(im0s, da_seg_mask, ll_seg_mask,
                                        lateral_deviation, intersection_direction)

        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException: launcher, kritik bir düğüm (kamera) ölünce
        # hepsini kapatıyor. Bu NORMAL bir kapanış, traceback basmaya gerek yok.
        pass
    finally:
        node.destroy_node()
        # Context zaten kapatılmışsa shutdown() ikinci kez RCLError fırlatıyor ve
        # asıl hata mesajı (ör. "ZED açılamadı") bu gürültünün içinde kayboluyordu.
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()