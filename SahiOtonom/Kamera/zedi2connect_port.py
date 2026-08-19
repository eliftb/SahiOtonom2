import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2

# Araçtaki ZED 2i'nin seri numarası. /dev/videoN numarası takılma sırasına göre
# değişir (dizüstünün dahili kamerası video0/video1'i kapıyor), seri numarası
# değişmez. 0 verilirse SDK bulduğu ilk ZED'i açar.
DEFAULT_CAMERA_SERIAL = 36258172


class ZEDCameraError(RuntimeError):
    """ZED kamera açılamadığında fırlatılır."""


class ZEDPublisherNode(Node):
    def __init__(self):
        super().__init__('zed_publisher_node')

        self.declare_parameter('camera_serial', DEFAULT_CAMERA_SERIAL)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('open_retries', 5)
        self.declare_parameter('open_retry_delay', 2.0)
        # ODOMETRİ (görsel-eylemsel konum takibi). VARSAYILAN KAPALI.
        # Kapalıyken bu düğüm bugüne kadar ne yapıyorsa aynısını yapar: derinlik
        # kapalı kalır, ek yayın olmaz, timer_callback'te ek iş yapılmaz.
        # Açmak GPU'ya derinlik hesabı ekler ve şerit/levha tespitiyle aynı kartı
        # paylaşır - açtıktan sonra FPS'i ölçmeden pistte kullanmayın.
        # AÇIK: kavşakta yön tutma (uart_sender_node) ve virajın metrik
        # takibi (serit-tespitcopy) odometriye dayanıyor; kapatırsan araç
        # kavşakta açık döngüye düşer. Kapatmak için burayı False yap ve
        # sistemi yeniden başlat.
        self.declare_parameter('enable_odometry', True)

        serial = self.get_parameter('camera_serial').value
        fps = self.get_parameter('camera_fps').value
        retries = self.get_parameter('open_retries').value
        retry_delay = self.get_parameter('open_retry_delay').value
        self.odometri_acik = bool(self.get_parameter('enable_odometry').value)

        # SDK 0'ı "varsayılan fps" sayar ama biz timer periyodunu 1/fps ile
        # hesapladığımız için 0 gelirse sıfıra bölme olur.
        if fps <= 0:
            self.get_logger().warning(f"Geçersiz camera_fps={fps}, 30 kullanılıyor.")
            fps = 30

        self.publisher_ = self.create_publisher(Image, '/zed2i_rgb/image_raw', 10)
        # DERİNLİK. Şerit takibi piksel geometrisiyle metre tahmin etmek yerine
        # ZED'in ölçtüğü GERÇEK mesafeyi kullanır: aracın orta çizgisi ile
        # sağdaki kırmızı çizgi arasındaki uzaklık doğrudan metre cinsinden
        # bilinir. Bu, ufuk/şerit-genişliği kalibrasyonuna olan bağımlılığı
        # tamamen kaldırır (o varsayımlar bu pistte tutmuyordu).
        self.depth_publisher = self.create_publisher(Image, '/zed2i/depth', 1)
        # İç parametreler (fx, cx...) olmadan piksel -> metre çevrilemez.
        self.info_publisher = self.create_publisher(CameraInfo, '/zed2i/camera_info', 1)
        self.bridge = CvBridge()
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.camera_info_msg = None
        self.get_logger().info("ZEDPublisherNode başlatılıyor...")

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = fps
        if self.odometri_acik:
            # Konum takibi derinlik olmadan çalışmaz. En hafif mod seçiliyor:
            # NEURAL doğruluk için daha iyi ama GPU'yu şerit/levha tespitiyle
            # paylaştığımız için burada maliyet doğruluktan önemli.
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
            init_params.coordinate_units = sl.UNIT.METER
            # ROS uyumu: x ileri, y sola, z yukarı
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        else:
            # Odometri kapalı olsa bile derinlik GEREKLİ: şerit takibi artık
            # metrik mesafe ölçüyor (bkz. /zed2i/depth). PERFORMANCE en hafif
            # mod - NEURAL doğruluk için daha iyi ama GPU'yu şerit/levha
            # tespitiyle paylaşıyoruz.
            init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
            init_params.coordinate_units = sl.UNIT.METER
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        if serial:
            # Doğru kameraya bağlan; başka bir kamera takılıysa onu açmasın.
            init_params.set_from_serial_number(serial)

        self._open_camera(init_params, serial, retries, retry_delay)

        # Odometri SADECE istendiyse kurulur ve HATA FIRLATMAZ: bu düğüm kritik,
        # burada bir istisna tüm sistemi kapatır. Kurulamazsa kendini kapatıp
        # görüntü yayınına devam eder.
        self.odom_publisher = None
        self.zed_pose = None
        if self.odometri_acik:
            try:
                tracking_params = sl.PositionalTrackingParameters()
                hata = self.zed.enable_positional_tracking(tracking_params)
                if hata != sl.ERROR_CODE.SUCCESS:
                    raise ZEDCameraError(str(hata))
                self.zed_pose = sl.Pose()
                self.odom_publisher = self.create_publisher(Odometry, '/zed2i/odom', 10)
                self.get_logger().info('🧭 ZED odometrisi açık -> /zed2i/odom')
            except Exception as e:
                self.odometri_acik = False
                self.odom_publisher = None
                self.get_logger().error(
                    f'Odometri açılamadı ({e}). Görüntü yayını normal sürüyor.')

        # Kamera hazır olmadan timer'ı kurma, yoksa grab() boşa çağrılır.
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def _open_camera(self, init_params, serial, retries, retry_delay):
        """Kamerayı açar; USB yeni enumerate olduysa ilk deneme başarısız olabilir.

        Açılışta 'CAMERA NOT DETECTED' dönmesi, cihaz USB'de daha oturmadan
        open() çağrıldığında görülüyor. Bu yüzden birkaç kez tekrar deniyoruz.
        """
        hedef = f"seri {serial}" if serial else "bulunan ilk ZED"
        for deneme in range(1, retries + 1):
            err = self.zed.open(init_params)
            if err == sl.ERROR_CODE.SUCCESS:
                bilgi = self.zed.get_camera_information()
                cfg = bilgi.camera_configuration
                self.get_logger().info(
                    f"ZED kamera açıldı: {bilgi.camera_model} "
                    f"(seri {bilgi.serial_number}) "
                    f"{cfg.resolution.width}x{cfg.resolution.height} @ {cfg.fps} fps"
                )
                return

            self.get_logger().warning(
                f"ZED açılamadı ({hedef}), deneme {deneme}/{retries}: {err}"
            )
            if deneme < retries:
                time.sleep(retry_delay)

        # SDK'nın gördüğü cihazları da yaz: kamera listede AVAILABLE değilse
        # başka bir proses tutuyordur, listede hiç yoksa USB sorunudur.
        try:
            cihazlar = sl.Camera.get_device_list()
            if cihazlar:
                durum = ", ".join(
                    f"seri {d.serial_number} ({d.camera_state})" for d in cihazlar
                )
            else:
                durum = "SDK hiç ZED görmüyor"
        except Exception as e:  # get_device_list da patlayabilir, teşhisi kaybetme
            durum = f"cihaz listesi alınamadı: {e}"

        raise ZEDCameraError(
            f"ZED kamera {retries} denemede açılamadı ({hedef}). "
            f"SDK'nın gördüğü cihazlar: {durum}. "
            f"Kontrol et: USB 3.0 portunda mı, 'lsusb | grep 2b03' cihazı görüyor mu, "
            f"başka bir proses kamerayı tutuyor mu."
        )

    def timer_callback(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            frame = self.image.get_data()

            # ZED BGRA döndürür, RGB'ye çevir
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

            # ROS Image mesajı olarak yayınla
            msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "zed2i_left_camera"
            self.publisher_.publish(msg)
            # 30 FPS'te her kareyi loglamak terminali boğuyor, seyrelt.
            self.get_logger().debug("Görüntü yayınlandı.")

            # Derinlik ve iç parametreler. Görüntüden SONRA ve ayrı korumada:
            # buradaki bir hata görüntü yayınını durdurmasın.
            try:
                self._yayinla_derinlik(msg.header.stamp)
            except Exception as e:
                self.get_logger().warning(f'Derinlik yayınlanamadı: {e}',
                                          throttle_duration_sec=5.0)

            # Odometri GÖRÜNTÜDEN SONRA ve ayrı korumada: buradaki bir hata
            # görüntü yayınını hiçbir şekilde etkilemesin.
            if self.odometri_acik:
                self._yayinla_odometri(msg.header.stamp)
        else:
            self.get_logger().warning("ZED'den görüntü alınamadı.",
                                      throttle_duration_sec=2.0)

    def _yayinla_derinlik(self, stamp):
        """Derinlik haritasını ve kamera iç parametrelerini yayınlar.

        Şerit takibi bunlarla piksel konumunu METREYE çevirir: bir pikselin
        yanal uzaklığı (u - cx) * derinlik / fx. Böylece 'araç orta çizgisi ile
        sağdaki çizgi arası 1.5 m' gibi gerçek bir hedef tanımlanabiliyor.
        """
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
        derinlik = self.depth.get_data()          # float32, metre, gecersiz = nan/inf
        d_msg = self.bridge.cv2_to_imgmsg(derinlik, encoding='32FC1')
        d_msg.header.stamp = stamp
        d_msg.header.frame_id = 'zed2i_left_camera'
        self.depth_publisher.publish(d_msg)

        if self.camera_info_msg is None:
            # Bir kez kurulur: kamera açıkken iç parametreler değişmez.
            cal = self.zed.get_camera_information().camera_configuration.calibration_parameters
            sol = cal.left_cam
            bilgi = CameraInfo()
            bilgi.width = int(derinlik.shape[1])
            bilgi.height = int(derinlik.shape[0])
            bilgi.k = [sol.fx, 0.0, sol.cx,
                       0.0, sol.fy, sol.cy,
                       0.0, 0.0, 1.0]
            bilgi.p = [sol.fx, 0.0, sol.cx, 0.0,
                       0.0, sol.fy, sol.cy, 0.0,
                       0.0, 0.0, 1.0, 0.0]
            self.camera_info_msg = bilgi
            self.get_logger().info(
                f'📷 Kamera ic parametreleri: fx={sol.fx:.1f} cx={sol.cx:.1f}')
        self.camera_info_msg.header.stamp = stamp
        self.camera_info_msg.header.frame_id = 'zed2i_left_camera'
        self.info_publisher.publish(self.camera_info_msg)

    def _yayinla_odometri(self, stamp):
        """ZED'in konum takibinden Odometry mesajı üretir.

        Hata durumunda SESSİZCE kendini kapatır: odometri yardımcı bir veri,
        yokluğu aracı durdurmamalı ama her karede hata basıp logu boğmamalı.
        """
        try:
            durum = self.zed.get_position(self.zed_pose, sl.REFERENCE_FRAME.WORLD)
            if durum == sl.POSITIONAL_TRACKING_STATE.OFF:
                return

            t = self.zed_pose.get_translation().get()
            q = self.zed_pose.get_orientation().get()

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = float(t[0])
            odom.pose.pose.position.y = float(t[1])
            odom.pose.pose.position.z = float(t[2])
            odom.pose.pose.orientation = Quaternion(
                x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
            self.odom_publisher.publish(odom)
        except Exception as e:
            self.odometri_acik = False
            self.get_logger().error(
                f'Odometri okunamadı, kapatıldı ({e}). Görüntü yayını sürüyor.')

    def destroy_node(self):
        # rclpy.ok() kontrolü: kapanış sırasında context zaten geçersizleşmiş
        # olabiliyor ve logger "Failed to publish log message to rosout" basıyordu.
        if rclpy.ok():
            self.get_logger().info("ZED kamera kapatılıyor...")
        try:
            if self.odometri_acik:
                self.zed.disable_positional_tracking()
        except Exception:
            pass
        self.zed.close()
        super().destroy_node()


def _sigterm_handler(signum, frame):
    """SIGTERM'i temiz kapanmaya çevirir.

    Varsayılanda SIGTERM süreci anında öldürür: finally bloğu çalışmaz,
    zed.close() hiç çağrılmaz ve kamera yarı-açık kalır. O durumda lsusb
    cihazı görmeye devam eder ama SDK 'CAMERA STREAM FAILED TO START' der
    ve kurtarmak için USB reset gerekir. Launcher düğümleri kapatırken
    process.terminate() (SIGTERM) kullandığı için bu her kapanışta oluyordu.

    SADECE İLK sinyalde iş görür: launcher kapanışta ikinci bir SIGTERM
    gönderdiğinde bu zed.close()'un ORTASINDA tekrar KeyboardInterrupt fırlatıp
    traceback basıyordu. İlk sinyalden sonra varsayılan davranışa dönüyoruz -
    temizlik zaten başlamış durumda.
    """
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    raise KeyboardInterrupt


def main(args=None):
    signal.signal(signal.SIGTERM, _sigterm_handler)
    rclpy.init(args=args)
    node = None
    hata = False
    try:
        node = ZEDPublisherNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ZEDCameraError as e:
        print(f"HATA: {e}", file=sys.stderr)
        hata = True
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if hata:
        sys.exit(1)


if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import pyzed.sl as sl 
# import cv2
# class ZEDPublisherNode(Node):
#     def __init__(self):
#         super().__init__('zed_publisher_node')
#         self.publisher_ = self.create_publisher(Image, '/zed2i_rgb/image_raw', 10)
#         self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS
#         self.bridge = CvBridge()

#         self.get_logger().info("ZEDPublisherNode başlatılıyor...")

#         # ZED kamera başlat
#         self.zed = sl.Camera()
#         init_params = sl.InitParameters()
#         init_params.camera_resolution = sl.RESOLUTION.HD720
#         init_params.camera_fps = 30

#         err = self.zed.open(init_params)
#         if err != sl.ERROR_CODE.SUCCESS:
#             self.get_logger().error(f"ZED kamerayı açamadı: {err}")
#             rclpy.shutdown()
#             return

#         self.image = sl.Mat()
#         self.get_logger().info("ZED kamera başarıyla başlatıldı.")

#     def timer_callback(self):
#         if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
#             self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
#             frame = self.image.get_data()
#             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)

#             # ROS Image mesajı olarak yayınla
#             msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
#             self.publisher_.publish(msg)
#             self.get_logger().info("Görüntü yayınlandı.")
#         else:
#             self.get_logger().warning("ZED'den görüntü alınamadı.")

#     def destroy_node(self):
#         self.get_logger().info("ZED kamera kapatılıyor...")
#         self.zed.close()
#         super().destroy_node()

# def main(args=None):
#     rclpy.init(args=args)
#     node = ZEDPublisherNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import zmq
# import numpy as np
# import cv2

# class ZedRgbSubscriber(Node):
#     def __init__(self):
#         super().__init__('zed_rgb_subscriber')
        
#         # RGB görüntü için topic
#         self.publisher_ = self.create_publisher(Image, '/zed2i_rgb/image_raw', 10)
#         self.bridge = CvBridge()
        
#         # ZeroMQ subscriber
#         context = zmq.Context()
#         self.socket = context.socket(zmq.SUB)
#         self.socket.connect("tcp://172.30.64.1:5555")  # Windows IP adresi
#         self.socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Tüm mesajları al
        
#         # Timer - 30Hz
#         self.timer = self.create_timer(0.03, self.timer_callback)
        
#         self.get_logger().info("ZED RGB Subscriber başlatıldı")
#         self.get_logger().info(f"ZeroMQ bağlantısı: tcp://172.30.64.1:5555")
#         self.get_logger().info(f"Yayın topic'i: /zed2i_rgb/image_raw")
    
#     def timer_callback(self):
#         try:
#             msg = self.socket.recv(flags=zmq.NOBLOCK)
#             self.get_logger().info(f"RGB veri alındı! Boyut: {len(msg)} bytes")
#         except zmq.Again:
#             return  # Veri yoksa hemen çık
        
#         # JPEG verisini RGB olarak decode et
#         np_arr = np.frombuffer(msg, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR formatında
        
#         if cv_image is None:
#             self.get_logger().warning("RGB görüntü decode edilemedi!")
#             return
        
#         self.get_logger().info(f"RGB görüntü işlendi: {cv_image.shape}, dtype: {cv_image.dtype}")
        
#         # ROS mesajına çevir (BGR8 encoding)
#         ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
#         ros_image.header.stamp = self.get_clock().now().to_msg()
#         ros_image.header.frame_id = "zed2i_rgb_frame"
        
#         # Yayınla
#         self.publisher_.publish(ros_image)
#         self.get_logger().info("RGB görüntü topic'te yayınlandı!")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ZedRgbSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()