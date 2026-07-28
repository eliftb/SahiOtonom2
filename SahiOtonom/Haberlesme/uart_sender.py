import serial
import time
# itki 1 direksiyon 0 fren 2
PORT = '/dev/ttyACM1'  # Gerekirse /dev/ttyACM0 veya /dev/ttyACM1 olarak değiştir
BAUD = 38400

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)  

    while True:
        try:
            value = 1


            ser.write(value.to_bytes(1, byteorder='little'))
            print(f"Gönderildi: {value}")
        except Exception as e:
            print(f"Hata: {e}")

if __name__ == "__main__":
    main()