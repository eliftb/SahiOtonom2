# Bu dosya ESKİ launch dosyasıydı (eski proje kopyasının yollarını içeriyordu).
# Karışıklık olmasın diye artık doğrudan GÜNCEL launch dosyasını çalıştırır:
#   SahiOtonom/launch_all_nodes.py
import os
import sys

GUNCEL_LAUNCH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "SahiOtonom", "launch_all_nodes.py")

print(f"ℹ️  Bu eski launch dosyası, güncel olana yönlendiriyor:\n   {GUNCEL_LAUNCH}\n")
os.execv(sys.executable, [sys.executable, GUNCEL_LAUNCH])
