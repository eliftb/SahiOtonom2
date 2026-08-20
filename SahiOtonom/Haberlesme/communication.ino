/*
 * communication.ino - SahiOtonom MIX PORT FIRMWARE
 * ============================================================================
 * Jetson/PC tarafındaki karşılığı TEK dosya:
 *     Haberlesme/uart_sender_node3.py   (ROS düğümü, porta yazan tek yer)
 * Bu dosya ONUN KAYNAĞIDIR: buradaki sabitler değişirse Python tarafı sessizce
 * eski protokolü konuşmaya başlar. Değişiklikten sonra mutlaka:
 *     python3 SahiOtonom/test/test_direksiyon_byte.py
 * (o testin son bölümü bu dosyanın kaynağını okuyup sabitleri karşılaştırır)
 *
 * --- PROTOKOL --------------------------------------------------------------
 * 38400 baud, 8N1.  Komut biçimi kesin:  <harf> ',' <sayı>
 *
 *     h,1   gaz AÇ      -> analogWrite(PIN_GAZ, 134)
 *     h,X   gaz KAPA    -> X != 1 olan HER değer (0, 50, 134, 255 dâhil)
 *     f,1   fren BAS    -> digitalWrite(PIN_FREN, LOW)
 *     f,0   fren BIRAK  -> digitalWrite(PIN_FREN, HIGH)
 *     d,N   direksiyon  -> N: 0..420, 210 = merkez, birim DERECE
 *                          aralık dışı N SESSİZCE yok sayılır
 *
 * Komut ŞU ÜÇ durumda uygulanır:
 *   (a) bir sonraki komut harfi (h/d/f) gelince
 *   (b) '\n' ya da '\r' gelince           <- Python her komuta bunu ekliyor
 *   (c) akış BEKLEYEN_KOMUT_MS kadar durunca
 * (b) olmadan fren komutu bir sonraki komut gelene kadar bekler; gecikmenin
 * tek sebebi buydu, o yüzden Python tarafında satır sonu ZORUNLU.
 *
 * --- İKİ TANE SESSİZ TUZAK (bilerek böyle) ---------------------------------
 * 1) ZAMAN AŞIMI: KOMUT_ZAMAN_ASIMI_MS boyunca GEÇERLİ komut işlenmezse gaz
 *    kesilir VE fren basılır. Bilgisayar çökerse, USB çıkarsa ya da düğüm
 *    susarsa araç kendi kendine durur. Python tarafı bu yüzden 10 Hz kalp
 *    atışı gönderir (gaz_tekrar_hz).
 * 2) BOZUK KOMUT KÖPEK BEKÇİSİNİ BESLEMEZ: aralık dışı bir 'd' ya da tanınmayan
 *    bir harf sonKomutMs'i GÜNCELLEMEZ. Yani hatalı komut yağdırarak aracı
 *    "canlı" tutmak mümkün değil.
 *
 * --- stepAt() BLOKLAR (Python'daki kademe sınırının sebebi) ----------------
 * stepAt() bloklayan bir döngü; adım başına 2 x delayMicroseconds(stepdelays)
 * = 2 ms, derece başına 3200/360 = 8.89 adım -> DERECE BAŞINA ~17.8 ms.
 * O süre boyunca while (Serial.available()) döngüsüne HİÇ dönülmez ve gelen
 * baytlar 64 baytlık donanım tamponunda birikir. Tampon taşarsa akıştan bayt
 * düşer, 'h,1' bozulup 'h,0' okunur ve GAZ KESİLİR. Python tarafı bu yüzden
 * tek komutta en fazla 12 birim gönderir (bkz. direksiyon_max_adim_byte).
 * BURADAKİ SAYILARI DEĞİŞTİRİRSEN o sınırı da yeniden hesapla.
 *
 * --- TAM SAYI BÖLMESİ KAYBI ------------------------------------------------
 * adim = fark * 3200 / 360 tam sayı bölmesidir ve sıfıra doğru KIRPAR:
 * 1 birimlik komut 8.89 yerine 8 adım gönderir. Firmware yine de "vardım"
 * sayar (stepangle = hedef), yani komut başına 0.11 dereceye kadar kayıp açık
 * döngüde BİRİKİR.
 *
 * ŞU AN TELAFİ EDİLMİYOR - bilinçli. Salınımlı şerit takibinde kayıplar iki
 * yöne de düştüğü için büyük ölçüde birbirini götürüyor; uzun ve tek yönlü
 * dönüşte ise birikiyor. Kapatmak istenirse yol şu: Python tarafında hedef
 * byte'ları 'merkez + 9k' ızgarasına oturtmak, çünkü 9*3200/360 = 80 TAM
 * bölünür - o zaman her komut tam 80 adım gönderir ve kayıp sıfırlanır.
 * Bedeli çözünürlük (9 birim ~= 1.35 derece teker açısı).
 */

// --- PİNLER -----------------------------------------------------------------
// GAZ ve FREN pinleri Python tarafındaki yorumlarla EŞLEŞMEK ZORUNDA (9 ve 10).
#define PIN_GAZ    9    // PWM. analogWrite(9, 134) = "ilerle"
#define PIN_FREN  10    // LOW = fren BASILI, HIGH = serbest (ters mantık!)

// STEP MOTOR PİNLERİ - ORİJİNAL FIRMWARE'DEN DOĞRULANDI (2026-08-20).
// Kartın üstündeki etiketle: 2 = "Step Dir", 3 = "Step Pull".
// DİKKAT, İKİSİ TERS YAZILIRSA hiçbir test yakalamaz (Python tarafı bu iki
// pini görmüyor) ve tek belirtisi direksiyonun dönmemesi/titremesidir.
#define PIN_DIR    2    // yön
#define PIN_STEP   3    // adım darbesi

// Adım darbesi yarı periyodu (mikrosaniye). Adım = 2 x bu süre.
// DÜŞÜRÜRSEN direksiyon hızlanır ama sürücü/motor adım kaçırmaya başlar;
// kaçan adım açık döngüde ASLA geri gelmez.
#define stepdelays 1000

// Sürücünün mikroadım ayarıyla birlikte: bir tam turdaki adım sayısı.
// Hesapta 3200L / 360L olarak SABİT yazılı (aşağıda), burası belge amaçlı.
#define TUR_ADIM   3200
#define TUR_DERECE 360

// Bu kadar süre GEÇERLİ komut işlenmezse gaz kesilir + fren basılır.
const unsigned long KOMUT_ZAMAN_ASIMI_MS = 1000;
// Akış bu kadar sessiz kalırsa tamponda bekleyen komut uygulanır.
const unsigned long BEKLEYEN_KOMUT_MS = 20;

// Komut tamponu. 16 = harf + ',' + en fazla 13 hane + '\0'.
// Python tarafı bu sınırı biliyor ve taşacak komutu porta hiç yazmıyor.
char buf[16];
byte buflen = 0;
bool gecerli = false;          // tampondaki komut biçimsel olarak sağlam mı

// FIRMWARE'İN İNANDIĞI DİREKSİYON KONUMU (derece, merkeze göre).
// Açılışta 0, yani "ben d,210 konumundayım". Step motor AÇIK DÖNGÜ: gerçek
// teker açısını bilmez, sadece ardışık iki komut ARASINDAKİ farkı döner.
// Port açmak Arduino'ya DTR reset attırdığı için her bağlantıda buraya dönülür;
// Python tarafı da o an kendi sayacını 210'a çeker (bkz. _portu_ac).
int stepangle = 0;

unsigned long sonKomutMs = 0;  // en son GEÇERLİ komut ne zaman işlendi
unsigned long sonBaytMs = 0;   // porttan en son ne zaman bayt geldi
bool zamanAsimindaydi = false; // aynı durumu tekrar tekrar uygulamamak için


void setup() {
  pinMode(PIN_GAZ, OUTPUT);
  pinMode(PIN_FREN, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  // AÇILIŞ = GÜVENLİ DURUM: gaz yok, fren basılı. Bilgisayar bağlanana kadar
  // araç frende bekler. (sonKomutMs = 0 olduğu için zaman aşımı da baştan
  // etkin sayılır - ilk geçerli komut gelene kadar bu durum korunur.)
  analogWrite(PIN_GAZ, 0);
  digitalWrite(PIN_FREN, LOW);
  digitalWrite(PIN_STEP, LOW);

  Serial.begin(38400);
}


/* Direksiyonu 'hedef' derecesine götürür. BLOKLAR (bkz. dosya başı). */
void stepAt(int hedef) {
  long adim = (long)(hedef - stepangle) * 3200L / 360L;

  stepangle = hedef;

  if (adim == 0) {
    return;
  }

  digitalWrite(PIN_DIR, adim > 0 ? HIGH : LOW);
  delayMicroseconds(stepdelays);        // sürücünün DIR kurulum süresi

  long sayi = adim > 0 ? adim : -adim;
  for (long i = 0; i < sayi; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(stepdelays);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(stepdelays);
  }
}

void komutUygula() {
  char harf = buf[0];
  bool tamamdi = gecerli && buflen >= 3 && buf[1] == ',';

  buf[buflen] = '\0';
  int deger = atoi(&buf[2]);

  buflen = 0;
  gecerli = false;

  if (!tamamdi) {
    return;
  }

  if (harf == 'h') {
    analogWrite(PIN_GAZ, deger == 1 ? 134 : 0);
  } else if (harf == 'f') {
        digitalWrite(PIN_FREN, deger == 1 ? LOW : HIGH);
  } else if (harf == 'd') {
    if (deger < 0 || deger > 420) return;
    int hedef = deger - 210;
    stepAt(hedef);
  } else {
    return;                             // tanınmayan harf
  }

  sonKomutMs = millis();
  zamanAsimindaydi = false;
}


void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    sonBaytMs = millis();

    if (c == 'h' || c == 'd' || c == 'f') {
      if (buflen > 0) komutUygula();
      buf[0] = c;
      buflen = 1;
      gecerli = true;
    } else if (c == '\n' || c == '\r') {
      if (buflen > 0) komutUygula();
    } else if (buflen > 0) {
      
      bool uygun = (buflen == 1) ? (c == ',')
                                 : (c >= '0' && c <= '9') || (buflen == 2 && c == '-');
      if (!uygun || buflen >= sizeof(buf) - 1) {
        gecerli = false;                // taşma ya da bozuk karakter
      }
      if (buflen < sizeof(buf) - 1) {
        buf[buflen++] = c;
      }
    }
  }

  // (c) AKIŞ DURDU: sonlandırıcı gelmemiş olsa bile bekleyen komutu uygula.
  if (buflen > 0 && (millis() - sonBaytMs) >= BEKLEYEN_KOMUT_MS) {
    komutUygula();
  }

  // KÖPEK BEKÇİSİ. Bağlantı koparsa araç kendi kendine durur.
  if ((millis() - sonKomutMs) > KOMUT_ZAMAN_ASIMI_MS) {
    if (!zamanAsimindaydi) {
      zamanAsimindaydi = true;
      analogWrite(PIN_GAZ, 0);
      digitalWrite(PIN_FREN, LOW);
    }
  }
}
