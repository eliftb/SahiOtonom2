#!/usr/bin/env python3

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, Bool
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np

from utils.utils import (
    select_device,
    driving_area_mask, lane_line_mask
)

# Kalıcı kalibrasyon değerleri (bkz. kalibrasyon.yaml). Bu düğüm alt klasörde
# olduğu için proje kökü arama yoluna ekleniyor.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kalibrasyon import kalibrasyon

KAL = kalibrasyon('lane_detection_node')

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
        # KALİBRASYON İÇİN: rota merkezinin HAM piksel konumu. Ekrandaki "Merkez"
        # sayısıyla aynı şey ama okunabilir/ölçülebilir hâlde - camera_center_offset_px
        # kalibrasyonu bunu kullanır (bkz. kalibrasyon_kamera.py). Sadece yayın,
        # kontrol yolunu etkilemez.
        self.center_px_pub = self.create_publisher(Float32, '/lane/center_px', 10)
        # ŞERİT GEÇERLİ Mİ. Kavşakta şerit çizgileri biter; rota o birkaç metre
        # boyunca ölçüme değil tahmine dayanır ve araç savrulur. UART düğümü bu
        # bayrağı görünce şerit takibini bırakıp ODOMETRİYLE yön tutmaya geçer.
        self.lane_valid_pub = self.create_publisher(Bool, '/lane/valid', 10)

        # --- ŞERİT TAKİBİ AYARLARI ---
        # Kamera aracın tam ortasında/tam ileri bakacak şekilde değilse şerit
        # ortası görüntü ortasına denk düşmez. Bu sabit kayma düz yolda sürekli
        # bir sapma üretip aracı kenara çekiyordu. Kalibrasyon: aracı şeridin
        # ortasına koy, ekrandaki "Merkez" değeri ile "ref" farkını buraya yaz.
        self.declare_parameter('camera_center_offset_px',
                               KAL('camera_center_offset_px', 0.0))
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
        # VİRAJ İLERİ BESLEMESİ. Yanal sapma tek başına bir GECİKMELİ ölçüdür:
        # araç viraja girip şeridin dışına çıkmaya başlamadan büyümez, o yüzden
        # araç virajı geniş alır. Bu terim şeridin ileride NEREYE gittiğine bakar
        # (uzak satır ile yakın satırın farkı) ve direksiyonu erken çevirir.
        # Düz yolda bu fark ~0'dır, yani düz gidişi bozmaz.
        #
        # 1.0'dan 0.5'e indirildi (2026-08-17): ileri bakış 0.80'den 0.68'e
        # çıkınca cross_track virajın çok daha büyük bölümünü kendisi yakalıyor
        # (aynı virajda ölçüm: -0.093 -> -0.247), ikisi üst üste binince orta bir
        # virajda direksiyon tavana dayanıyordu. Toplam agresiflik eskisiyle
        # aynı kaldı, ama artık ağırlık sezgisel terimde değil GERÇEK geometride.
        self.declare_parameter('curve_feedforward', 0.5)
        # Yaklaşık ufuk çizgisi (şerit genişliğinin perspektifle küçülme modeli)
        self.declare_parameter('horizon_frac', 0.55)
        # BEKLENEN ŞERİT GENİŞLİĞİ: 0.85 satırında görüntü genişliğinin kaçta kaçı.
        # Bulunan iki çizgi arasındaki mesafe bununla karşılaştırılıp mantıksızsa
        # reddediliyor - BARİYERLERİ ELEYEN TEK MEKANİZMA bu. Yanlışsa ya bariyer
        # çifti "şerit" sayılır, ya da gerçek şerit reddedilir.
        # ÖLÇÜM: debug_rows_log açıkken tablodaki 'gen' sütunu, y=0.85*yükseklik
        # satırında kaç piksel? Onu görüntü genişliğine bölün.
        self.declare_parameter('lane_width_frac', KAL('lane_width_frac', 0.40))
        # Şerit çizgisi noktalarını SÜRÜLEBİLİR ALANIN İÇİNDE olanlarla sınırla.
        # Bariyer/korkuluk alanın dışında kaldığı için bu filtre onları eler.
        # Kapatmak, modelin bulduğu her çizgiyi kabul etmek demektir.
        self.declare_parameter('paint_inside_only', True)
        # GÖRSELLEŞTİRME BÜTÇESİ. Debug karesi sadece insan için; kontrol yolunda
        # değil. Maskeleri tam çözünürlükte her karede basmak ölçümde ~145 ms
        # tutuyordu (maske ne kadar çok pikseli kaplarsa o kadar yavaş - yol
        # kareyi doldurunca FPS 9'dan 3.3'e düşmesinin sebebi buydu).
        # 0 = debug görüntüsü hiç üretilmesin (en hızlı).
        self.declare_parameter('debug_every_n', 3)
        self.declare_parameter('debug_scale', 0.5)
        # ROTA KAYNAĞI. Bu pistte boyalı şerit yok; şerit çizgisi modeli bariyer
        # kenarlarını çizgi sanıyor ve rota bariyerden türetiliyordu. Sürülebilir
        # alan maskesi ise yolu ve virajı net veriyor - zaten hesaplanıyordu ama
        # sadece görselleştirmede kullanılıyordu.
        #   'yol'   = sürülebilir alanın koridor ortası (boyasız pist için)
        #   'serit' = şerit çizgileri (boyalı yol için, eski davranış)
        #   'auto'  = çizgiler sürülebilir alanın içindeyse 'serit', değilse 'yol'
        # Varsayılan 'auto': pistte asfalta boya ÇİZİLMİŞ ve kenarda bariyer var.
        # Boya sürülebilir alanın içinde, bariyer dışında kalır; ayırt edici bu.
        # (Önceki varsayılan 'yol' idi, "boya yok" varsayımına dayanıyordu.)
        self.declare_parameter('route_source', KAL('route_source', 'auto'))
        # 'auto' için: kaç satırda İKİ çizgi de sürülebilir alanın içinde olmalı
        # (bkz. _looks_like_paint - bariyerler alanın dışında kalır)
        self.declare_parameter('min_paint_rows', 3)
        # ARACIN KAPUTU bu satırın altında kalır. Model kaputu "sürülebilir alan"
        # sayıyor (ekranda kaput da yeşil), ayrıca eski örnekleme satırlarının en
        # alt üçü (0.95/0.90/0.85) doğrudan gövdenin üstüne düşüyordu.
        # Kalibrasyon: debug karesinde kaputun üst kenarı görüntünün yüzde kaçında?
        self.declare_parameter('hood_frac', KAL('hood_frac', 0.82))
        # TANI: her örnekleme satırındaki koridor kenarlarını ve merkezini loglar.
        # Rota yamuk çıktığında hangi satırın bozulduğu ancak böyle görülüyor -
        # ekrandaki tek "Merkez" sayısı sorunun nerede başladığını söylemiyor.
        self.declare_parameter('debug_rows_log', False)

        self.camera_center_offset_px = float(self.get_parameter('camera_center_offset_px').value)
        self.look_ahead_frac = float(self.get_parameter('look_ahead_frac').value)
        self.deviation_deadband = float(self.get_parameter('deviation_deadband').value)
        self.max_deviation_rate = float(self.get_parameter('max_deviation_rate').value)
        self.curve_feedforward = float(self.get_parameter('curve_feedforward').value)
        self.horizon_frac = float(self.get_parameter('horizon_frac').value)
        self.lane_width_frac = float(self.get_parameter('lane_width_frac').value)
        self.paint_inside_only = bool(self.get_parameter('paint_inside_only').value)
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
                   'paint_inside_only')

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

        # Hiç ölçüm yoksa: 0.85 satırında görüntünün lane_width_frac kadarı
        return width * self.lane_width_frac * di / max(0.85 * height - horizon, 1.0)

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

            # Şerit eğikse yatay ölçümü dik genişliğe çevirmek için katsayı
            slant = self._slant_factor(y)
            expected = self._expected_lane_width(i, width, height)

            if prev_y is None:
                # ARACA EN YAKIN GEÇERLİ SATIR: burada takip merkezi güvenilir
                reference = self.lane_center_track[i]
                left = max([x for x in points if x < reference - min_half], default=None)
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
                if 0.5 * expected < measured < 1.8 * expected:
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
                    chosen = min(groups, key=lambda g: min(abs(g[0] - reference),
                                                           abs(g[-1] - reference)))

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

    def _route_centers(self, lane_mask, da_mask):
        """Rota noktalarını seçilen kaynaktan üretir (bkz. route_source parametresi)."""
        if self.route_source in ('serit', 'auto'):
            ys, centers = self._lane_centers_at_rows(lane_mask, da_mask)
            if self.route_source == 'serit':
                return ys, centers, 'serit'
            if self._looks_like_paint(lane_mask, da_mask):
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
        self.get_logger().info(
            f'TANI (ref={ref:.0f}, kaynak={self.debug_source}, viraj={self.debug_curve:+.3f})\n  '
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