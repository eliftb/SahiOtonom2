import os
import shutil

# Klasör yolunu ve işlem yapılacak klasörü belirtin
klasor_yolu = '/home/enbiyagoral/Desktop/sahi_ws/src/foto'

# Klasör adını tanımlayın
eslesmeyenler_klasoru = os.path.join(klasor_yolu, 'eslesmeyenler')

# Eğer klasör yoksa oluşturun
os.makedirs(eslesmeyenler_klasoru, exist_ok=True)

# Klasördeki dosyaları listele
dosyalar = os.listdir(klasor_yolu)

# Dosya isimlerini depolamak için bir küme oluştur
dosya_isimleri = set()

# .png ve .txt dosyalarını işlemek için boş listeler oluştur
png_dosyalar = []
txt_dosyalar = []

# Dosyaları türlerine göre ayır ve isimleri depola
for dosya in dosyalar:
    dosya_adi, dosya_uzanti = os.path.splitext(dosya)
    dosya_isimleri.add(dosya_adi)
    if dosya.endswith('.png'):
        png_dosyalar.append(dosya)
    elif dosya.endswith('.txt'):
        txt_dosyalar.append(dosya)

# Eşleşen dosyaları bul ve taşı
for dosya_adi in dosya_isimleri:
    if f'{dosya_adi}.png' in png_dosyalar and f'{dosya_adi}.txt' in txt_dosyalar:
        kaynak_png = os.path.join(klasor_yolu, f'{dosya_adi}.png')
        kaynak_txt = os.path.join(klasor_yolu, f'{dosya_adi}.txt')
        hedef_yolu = os.path.join(klasor_yolu, 'eslesmeyenler')
        shutil.move(kaynak_png, hedef_yolu)
        shutil.move(kaynak_txt, hedef_yolu)
        print(f'{dosya_adi}.png ve {dosya_adi}.txt taşındı.')

# Eşleşmeyen dosyaları sil
for dosya in dosyalar:
    dosya_adi, dosya_uzanti = os.path.splitext(dosya)
    if dosya_uzanti in ['.png', '.txt'] and dosya_adi not in dosya_isimleri:
        dosya_yolu = os.path.join(klasor_yolu, dosya)
        os.remove(dosya_yolu)
        print(f'{dosya} silindi.')

print('İşlem tamamlandı.')
