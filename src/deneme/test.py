import sys, serial, time

if len(sys.argv) != 3:
    print(f"Kullanım: python3 {sys.argv[0]} <PORT> <DEĞER>")
    sys.exit(1)

PORT, VALUE = sys.argv[1], int(sys.argv[2])
BYTE_TO_SEND = VALUE.to_bytes(1, 'little')
BAUD_RATE = 38400

ser = None
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"-> Port: {PORT} | Değer: {VALUE} gönderiliyor... (Durdurmak için CTRL+C)")
    time.sleep(2) # Cihazın kendine gelmesi için bekle
    while True:
        ser.write(BYTE_TO_SEND)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n-> Durduruldu.")
except Exception as e:
    print(f"\n-> HATA: {e}")
finally:
    if ser and ser.is_open:
        ser.write((127).to_bytes(1, 'little')) # Nötr pozisyona al
        ser.close()
        print("-> Port kapatıldı.")