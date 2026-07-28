#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import time

def main():
    if len(sys.argv) != 2:
        print("Kullanım: python3 send_lateral.py <değer>")
        print("Örnek: python3 send_lateral.py -0.5")
        return
    
    try:
        value = float(sys.argv[1])
    except ValueError:
        print("Hata: Geçerli bir sayı girin")
        return
    
    rclpy.init()
    node = Node('lateral_sender')
    pub = node.create_publisher(Float32, '/lane/lateral_deviation', 10)
    
    msg = Float32()
    msg.data = value
    
    print(f"Sürekli gönderiliyor: {value} (Ctrl+C ile durdurun)")
    
    try:
        while rclpy.ok():
            pub.publish(msg)
            time.sleep(0.1)  # 10 Hz
    except KeyboardInterrupt:
        print("\nDurduruldu.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()