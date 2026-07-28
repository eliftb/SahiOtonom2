# Sahi Otonom Projesi
Bu proje, Sahi Otonom araç simülasyonunu içerir. Proje, çeşitli bileşenlerden oluşur ve aşağıdaki adımları izleyerek çalıştırılabilir:

## Kurulum

### Bağımlılıkları Kurma

1. ROS 2 ve gerekli paketlerin yüklü olduğundan emin olun.
2. Proje bağımlılıklarını yüklemek için aşağıdaki komutu çalıştırın:

    ```bash
    cd src/sahi_otonom/sahi_otonom/serit-tespit/
    pip3 install -r requirements.txt
    ```

### Proje Dosyalarının Yapısı

- `src/sahi_otonom/worlds`: Gazebo parkur dosyalarını içerir.
- `src/yolov8/src/yolov8_object_detection/scripts`: Nesne tespiti ve derinlik hesaplama scriptlerini içerir.

- `src/yolov12/src/yolov12_object_detection/scripts`: Nesne tespiti ve derinlik hesaplama scriptlerini içerir.

- `src/sahi_otonom/sahi_otonom/serit-tespit`: Şerit tespit scriptlerini içerir.

### 1. Parkuru Simülasyona Ekleme
Gazebo simülasyonunu başlatmak için aşağıdaki komutu çalıştırın:

```bash
gazebo src/sahi_otonom/worlds/parkurcopy.world
```

### 2. YOLOv8'i Çalıştırma
YOLOv8 nesne tespiti algoritmasını çalıştırmak için aşağıdaki ROS 2 komutunu kullanın:

```bash
ros2 launch yolov8_object_detection launch_yolov8.launch.py 
```

### 2. YOLOv12'i Çalıştırma
YOLOv12 nesne tespiti algoritmasını çalıştırmak için aşağıdaki ROS 2 komutunu kullanın:

```bash
ros2 launch yolov12_object_detection launch_yolov12.launch.py 
```

### 3. Tespit Edilen Nesnenin Uzaklığını Hesaplama
Tespit edilen nesnelerin uzaklığını hesaplamak için aşağıdaki adımları izleyin:

```bash
cd /home/cantaskin/sahi_otonom-main/src/yolov8/src/yolov8_object_detection/scripts
python3 depth.py
```

### 4. Şerit Tespit Entegrasyonu 
Şerit tespit entegrasyonunu çalıştırmak için aşağıdaki adımları izleyin:

```bash
cd /home/sahi_otonom-main/src/sahi_otonom/sahi_otonom/serit-tespit
python3 serit-tespit.py
```
### 4. Engel Tespit Entegrasyonu 
Engel tespit entegrasyonunu çalıştırmak için aşağıdaki adımları izleyin:

```bash
cd /home/cantaskin/sahi_otonom-main/src/sahi_otonom/sahi_otonom/engel-tespit
python3 engel-tespit.py
```

### 5. Karar Tespit Entegrasyonu
Karar tespit entegrasyonunu çalıştırmak için aşağıdaki adımları izleyin:

```bash
cd /home/cantaskin/sahi_otonom-main/src/sahi_otonom/sahi_otonom
python3 basic-decision-making-node.py

lateral deviation kontrolü yap

/home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/uart_sender/uart_sender_node3.py BURADA




Lidar C1 için rvizli
ros2 launch sllidar_ros2 view_sllidar_s1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=256000

rvizsiz
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0 serial_baudrate:=256000

python3 /home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/engel-tespit/engel-tespit.py

ZED için

python3 /home/sahi/sahi_otonom-main/zedi2impl/zedi2connect_port.py

python3 /home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/serit-tespit/serit-tespit.py

yukarıdakileri de uygula eğer levha test edeceksen 


direksiyon döndürmeyi test etmek istiyorsan
python3 /home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/uart_sender/uart_sender.py


eğer direkt test edecekseniz de,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
python3 /home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/basic-decision-making-node.py
python3 /home/sahi/sahi_otonom-main/src/sahi_otonom/sahi_otonom/uart_sender/uart_sender_node3.py

ls /dev/ttyACM* 
ls /dev/ttyACM 
bunlardan usb port bunlardan tak çıkart yaparak bulabilirsin


ttyACM0 - FREN
ttyACM3 - DIREKSIYON



basic-decision-making'te itki ve fren mantığı yapılması gerekiyor buradan bir topic açarak veriler uart_sender_node3.py'a publish edilmeli.uart_sender_node3.py burada topiclere subscribe olup elektroniğe veri yollayayacağız.