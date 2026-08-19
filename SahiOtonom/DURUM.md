# SahiOtonom — Durum Raporu (2026-08-18)

Otonom araç yarışma projesi. Bu belge, projeyi devralan için mevcut durumu ve
sıradaki işleri özetler.

## Donanım

| cihaz | bağlantı | durum |
|---|---|---|
| ZED 2i | USB 3.0 **doğrudan laptopa** (hub'dan çalışmaz) | çalışıyor, odometri açık |
| RPLIDAR S2 | USB (CP2102), 1000000 baud | **uzatma kablosuyla çalışmıyor** — aşağıya bak |
| Arduino | USB (CH340), 38400 baud | protokol: `d,0-360` `h,0|1` `f,0|1` |
| GPS (NEO-M8N) | alındı, bağlanmadı | — |

## Mimari

```
zedi2connect_port.py  ──▶ /zed2i_rgb/image_raw, /zed2i/odom
serit-tespitcopy.py   ──▶ /lane/lateral_deviation, /lane/valid, /lane/center_px
run_tracker.py        ──▶ /sign_detection/boxes  (cls + id alanları var)
engel-tespit.py       ──▶ /obstacle_detected, /obstacle_distance   (girdi: /scan)
basic-decision-...py  ──▶ /speed, /ackermann_cmd, /route/turn, /route/preferred_side
uart_sender_node3.py  ──▶ Arduino (direksiyonun TEK sahibi)
```

`launch_all_nodes.py` hepsini sırayla başlatır; LiDAR sürücüsünü de (reset atarak) o
başlatır. Kapanış temiz (CTRL+C).

## Bugün yapılanlar (commit `bc86e35`)

**Performans.** Görselleştirme kontrol yolundan ayrıldı: `show_seg_result` her karede
tam çözünürlükte çalışıyordu ve maliyeti maskenin kapladığı alanla büyüyordu
(~145 ms). Ölçüm: **5.5 → 34.8 FPS**. Artık kamera sınırında (30).

**PID zaman tabanlı yapıldı.** Eskiden `dt` kullanmıyordu; FPS sahneye göre 3-30
arasında değiştiği için kazançların etkisi sürekli kayıyordu (türev genliği 3 Hz'de
0.0164, 30 Hz'de 0.0025). `ki` [1/s], `kd` [s] birimli oldu — **eski sayılar geçersiz**.
Anti-windup eklendi.

**Levha/ışık algısı kontrolcüye bağlandı.** Önceden `/sign_detection/boxes` sadece
ekrana gidiyordu, hiçbir levha davranışı etkilemiyordu. Artık durum makinesi var:
kırmızıda dur (4 m), yeşilde kalk, `dur` levhasında 10 sn bekle.

**Engel tespiti yeniden yazıldı.** Eskiden aracın **ARKASINA** bakıyordu ve sadece açı
aralığı kontrol ediyordu (5 m'de ±15° = ±1.3 m, yan bariyerler sürekli fren
yaptırıyordu). Artık kartezyen koridor: ileri x, yanal |y| ≤ 0.5 m. 5 m'den itibaren
kademeli yavaşlama, 1.5 m'de duruş.

**Kavşak.** Şerit kaybolunca (`/lane/valid` False) odometriden yön tutma; mecburi yön
levhası varsa hedef yön ±90° kaydırılır. Eski davranış açık döngüydü, araç savruluyordu.

**Dayanıklılık.** UART kopunca otomatik yeniden bağlanma (acil durdurma bataryayı
kesiyor → Arduino USB'den düşüyor; eskiden tüm sistemi yeniden başlatmak gerekiyordu).
LiDAR koruma moduna düşerse sürücü reset atıp yeniden başlıyor.

**Altyapı.** Kalibrasyon araçları, pist kayıt/oynatma betikleri, `dur.sh`/`devam.sh`
(araç durdurma).

**Kalibrasyon dosyası kaldırıldı (2026-08-18).** `kalibrasyon.py` + `kalibrasyon.yaml`
silindi; her ayar artık ilgili düğümün `declare_parameter` satırında sabit. Ölçüm
araçları (`kalibrasyon_*.py`) çalışmaya devam ediyor ama artık kaydetmiyor: ölçtükleri
değeri hangi dosyaya yazacağınızı ekrana basıyorlar. Kayıt açma/kapama artık
`launch_all_nodes.py` içindeki `KAYIT_ACIK`.

## Ayarlar ve durumları

Tek doğru kaynak **kodun kendisi**: `declare_parameter(...)` satırları. Pistte
yeniden başlatmadan denemek için `ros2 param set`, beğendiğiniz değeri koda yazın —
yoksa sistem kapanınca kaybolur.

| değer | ayar | nerede | durum |
|---|---|---|---|
| `lane_width_frac` | 0.40 | `SeritTespit/serit-tespitcopy.py` | ✓ **kayıttan doğrulandı** (0.395) |
| `hood_frac` | 0.82 | `SeritTespit/serit-tespitcopy.py` | ✓ **kayıttan doğrulandı** (0.832) |
| `route_source` | `mesafe` | `SeritTespit/serit-tespitcopy.py` | ✓ kayıtta %100 şerit |
| `camera_center_offset_px` | 0.0 | `SeritTespit/serit-tespitcopy.py` | ✗ temiz ölçüm gerek (veri ~0 diyor) |
| `kp / ki / kd` | 0.8 / 0.0 / 0.3 | `Haberlesme/uart_sender_node3.py` | ✗ **pistte doğrulanmadı** (aşağıya bak) |
| `steering_trim` | 50 | `Haberlesme/uart_sender_node3.py` | ✓ **ölçüldü 2026-08-18** (sehpada, merkez byte 230) |
| `max_steering_angle` | 0.5 rad | `Haberlesme/uart_sender_node3.py` | ✗ ölçülmedi — merkez ölçümüyle KARIŞTIRMAYIN, ayrı adım (`--kilit`) |
| `enable_odometry` | True | `Kamera/zedi2connect_port.py` | ✓ **ölçüldü 2026-08-19** (aşağıya bak) |
| `forward_angle_deg` | 0.0 | `EngelTespit/engel-tespit.py` | ✗ LiDAR monte değil, ölçülmedi |
| `min_check_distance_m` | 0.15 | `EngelTespit/engel-tespit.py` | ✗ montajdan sonra ayarlanmalı |
| `ref_box_height_px` | 60.0 | `KararAlg/basic-decision-making-node.py` | ✗ tahmin — "4 metre" gerçek 4 m değil |

Kayıt analizi (7.6 dk, 1184 kare): rota kaynağı **%100 `serit`**, şerit kaybı **%1.3**.
Yani şerit tespiti pistte tutarlı çalışıyor, bariyere kaçmıyor.

### Odometri ölçümü (2026-08-19)

`Kamera/odometri_test.py` ile ölçüldü, **düzeltme gerekmedi**:

| ölçüm | sonuç | yorum |
|---|---|---|
| drift (30 sn, araç dururken) | 0.001 m / 0.0° , yol birikimi 0.101 m | gürültü tabanı ~3.4 mm/sn — sağlıklı |
| mesafe ölçeği (2 m düz) | **0.995** (yol integrali 1.989 m) | %0.5 hata, parametre değişmedi |
| açı ölçeği (4 × 90°) | sağ 1.008 (−92.6°, −88.8°), sol 1.090 (+97.9°, +98.3°) | işaret DOĞRU, ~%5 saçılma |

**İşaret doğrulandı:** sağa dönüş yaw'ı azaltıyor (eksi). `uart_sender_node3.py`
içindeki `ham = guncel_yaw - kayma` hesabı bu varsayıma dayanıyor; artık ölçümle
sabit. `turn_angle_deg = 90` ve `viraj_donus_acisi_deg = 90` olduğu gibi kalıyor.

**Sol dönüşler ~%9 fazla okundu.** Muhtemelen elle döndürürken gerçekten 90°'den
fazla dönülmesi (referans çizgi yoktu), sensör asimetrisi değil. Önemsemek
gerekirse yere referans çizgi çekip tek bir kontrollü ölçüm yeterli. 90°'de 4-5°
hata kavşak çıkışında şerit yakalanınca kapanıyor.

**Kamera dönme merkezinin ~30 cm önünde.** Yerinde 90° dönüşte odometri 0.40-0.51 m
yer değiştirme okuyor (kordon uzunluğundan r ≈ 0.3 m). Beklenen davranış, ama şunu
getiriyor: dönüş sırasında `viraj_mesafe_m` aracın ilerlemesinden değil kameranın
yay boyundan düşüyor. 90°'lik dönüş başına ~0.5 m; `viraj_donus_yolu_m = 6.0`
emniyetinin yanında zararsız.

## Sıradaki işler

1. **Araç mekaniği + LiDAR kablosu** — bunlar olmadan hiçbir ölçüm yapılamıyor
2. **Kalibrasyonlar** (yarım gün): `camera_center_offset_px` → `forward_angle_deg` →
   `min_check_distance_m` → `ref_box_height_px`. Ölçüm araçları değeri artık
   kaydetmiyor — çıkan sayıyı ilgili düğümdeki `declare_parameter` satırına yazın.
3. **Levha modelini pistte doğrula** — *hiç denenmedi*. Tanımıyorsa kırmızı ışık, dur,
   mecburi yön hepsi çalışmaz. 5 dakikalık iş, sonucuna göre öncelikler değişir.
4. **Düşük hızda tam tur** (`./devam.sh 0.5`) — kavşak yön tutmayı gerçek pistte sına
5. **Park manevrası** — hiç başlanmadı. Cepler **çizgili**, yani kameranın işi:
   levha ✓ + çizgi tespiti ✓ + boşluk kontrolü (LiDAR) ✓ + odometri ✓ hazır; eksik
   olan birleştiren mantık. **Cebin dik mi paralel mi olduğu ve fotoğrafı gerekiyor.**
6. **Arduino firmware** → kademeli hız (aşağıya bak)
7. GPS, durak görevi, kavşak tespiti (`/lane/intersection_direction` kimse dinlemiyor)

## Bilinmesi gerekenler (zaman kaybettirmesin)

**Arduino sadece git/dur anlıyor.** Karar alma "engel 3 m → %60 hız" hesaplıyor ama
`speed_to_digital_signal` bunu `h,1`'e düşürüyor. Kademeli yavaşlama **hesaplanıyor ama
uygulanmıyor**. Çözüm firmware'de: `h` komutunun sayısal değer kabul etmesi. `.ino`
dosyası görülmeden yapılmamalı.

**LiDAR uzatma kablosu çalışmıyor.** 1.5 m ince uzatmada gerilim düşüyor, motor devri
tutmuyor, cihaz `health status 2` mandalına düşüyor. Uzatmasız kesintisiz çalışıyor —
**teşhis kesin**. Çözüm: beslemeli USB hub ya da aktif uzatma. Mandal kalıcıdır,
sürücüyü yeniden başlatmak temizlemez; `EngelTespit/lidar_sifirla.py` reset komutu
gönderir (`0xA5 0x40`) ve düzeltir. `lidar_baslat.sh` bunu her açılışta otomatik yapar.

**"Hız komutu gidiyor ama araç gitmiyor" — sebep bulundu, sehpada doğrulanmayı
bekliyor (2026-08-18).** Loglar kesintisiz `h,1 f,0` ve `d,230` basıyordu, karar
düğümü 1.00 m/s veriyordu, araç kımıldamıyordu. Sebep `send_command`'ın komutları
**sonlandırıcısız** yazmasıydı: porta kesintisiz `d,230d,230h,1f,0d,230...` akıyor.
Firmware `Serial.parseInt()` kullanıyorsa sayıyı bitiren karakteri okuyup **atar** —
kesintisiz akışta 230'u bitiren karakter bir sonraki komutun **harfidir**. `d`
sayısını okuyan parseInt arkasından gelen `h`yi yutuyor, geride kalan `,1` harfsiz
kaldığı için yok sayılıyordu: **hız komutu Arduino'ya hiç ulaşmıyordu.**

Kalibrasyon modunda gizli kalmasının sebebi akış yoğunluğu: orada komutlar tuş tuş,
aralarında saniyeler geçerek gidiyor, akış durunca parseInt kendi zaman aşımıyla
sayıyı bitiriyor ve sonraki harfi yutmuyor. Arıza sadece sürüşteki 20 Hz kesintisiz
akışta çıkıyor — bu yüzden "sehpada çalışıyor, pistte çalışmıyor" görünüyordu.

Düzeltme: `send_command` artık her komuta `\n` ekliyor ve `flush()` yapıyor.
Kalibrasyon modu da aynı biçimde yazıyor (ölçüm ile sürüş ayrışmasın).
Firmware `\n`'i sevmezse: `ros2 param set /uart_sender_node satir_sonu false`.

**Sehpada doğrulama (yapılmadı):** `python3 test/test_hiz_akisi.py` — sürüş akışını
önce ayraçsız, sonra `\n`'li gönderir. Beklenen: ilkinde tahrik dönmez, ikincisinde
döner. İkisinde de dönmüyorsa arıza yazılımda değil (besleme / ESC / acil durdurma).

**Direksiyon dönmüyor şüphesi — kısmen çözüldü, kalanı ölçüm bekliyor.** Aynı
oturumda teker de dönmüyordu. İki sebep vardı, ikisi de bulundu:

1. **Merkez 180 değil, 230.** ÖLÇÜLDÜ (sehpada): `steering_trim` 0 → **50**. Yazılım
   "hafif sola kırıyorum" derken gerçek merkezin sağında kalıyordu. Bedeli: merkez
   230 olunca kullanılabilir aralık simetri için ±130 birime düşüyor.
2. **Yukarıdaki sonlandırıcı arızası** `d` komutlarının bir kısmını da düşürüyordu
   (`f,0`dan sonra gelen `d` harfi yutuluyor). Artık düzeltildi.
3. **Genlik küçük (AÇIK).** Ölçülen anda şerit hiç görülmüyordu (`hiç satır
   ölçülemedi`), yani sapma gerçek değil, `*0.85` ile sönümlenen hayalet. Üstelik
   sönümlenirken türev terimi P'yi yarıya kesiyor (p 0.112 rad → çıkış 0.061). Bu
   direksiyondan bağımsız İKİNCİ bir arıza: teker düzelse bile kalırsa araç savrulur.

Sıra: `test_hiz_akisi.py` (tahrik) → kilit ölçümü (`--kalibrasyon --kilit`) → sapmayı
elle yayınlayıp (`test/test_lateral_deviatiton.py -0.5`) zinciri sehpada doğrula →
sonra şerit tespiti. `max_steering_angle` (0.5 rad) hâlâ ölçülmemiş bir varsayım;
ölçülene kadar logdaki "rad" değerleri fiziksel açı değildir.

**Test iskeleti.** Düğümler ROS olmadan test edilebiliyor: `rclpy` taklit edilip modül
`importlib` ile yükleniyor. Bugün yazılan her özellik böyle test edildi
(engel senaryoları, PID hız bağımsızlığı, kavşak dönüşü, kamera modları).

**rosbag2 tuzağı.** Sıkıştırılmış kayıtlar `SequentialCompressionReader` ister
(`SequentialReader` değil) ve `set_filter` bu sürümde her şeyi eliyor — topic filtresini
Python tarafında yapın. Ayrıca `rclpy` taklit edilmişken `deserialize_message` bozulur;
kare çıkarma ile analiz **ayrı süreçlerde** yapılmalı.

**Kayıtlar git'e girmemeli.** `.gitignore`'da `SahiOtonom/kayitlar/` var. Ham kayıt
~30 MB/sn yer kaplar.

## Faydalı komutlar

```bash
./usb_kontrol.sh                              # üç cihaz bağlı mı, ZED gerçekten çalışır mı
python3 launch_all_nodes.py                   # tüm sistem (kayıt: KAYIT_ACIK sabiti)
./dur.sh   /  ./devam.sh [hız]                # aracı durdur / devam ettir
python3 test/test_hiz_akisi.py                         # tahrik: hız komutu ulaşıyor mu (SEHPADA)

python3 Haberlesme/uart_sender_node3.py --kalibrasyon  # direksiyon merkezi + gerçek kilit
python3 SeritTespit/kalibrasyon_kamera.py --kaydet     # kamera merkez kayması
python3 EngelTespit/kalibrasyon_lidar.py --kaydet      # LiDAR montaj açısı
python3 EngelTespit/lidar_sifirla.py                   # LiDAR koruma modundan çıkar
python3 Kamera/odometri_test.py                        # odometri canlı izleme
python3 Kamera/odometri_test.py --mesafe 2.0           # mesafe ölçeği (2 m düz it)
python3 Kamera/odometri_test.py --donus -90            # açı ölçeği (sağa 90°, sağ = eksi)
python3 Kamera/odometri_test.py --bekle 30             # dururken drift
./pist_kayit.sh  /  ./pist_oynat.sh <ad>               # kayıt al / tekrar oynat
```

Canlı ayar (yeniden başlatma gerekmez):
```bash
ros2 param set /lane_detection_node debug_rows_log true    # satır satır tanı
ros2 param set /uart_sender_node kp 1.0                    # PID (kalıcı için koda yaz)
ros2 param set /lidar_obstacle_detector min_check_distance_m 0.5
```
