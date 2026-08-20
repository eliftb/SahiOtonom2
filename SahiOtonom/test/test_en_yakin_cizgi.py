#!/usr/bin/env python3
"""EN YAKIN SAG CIZGIYE KILITLENME - gercek ZED kalibrasyonuyla."""
import importlib.util, os, sys, types
import numpy as np
KOK = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
for n in ['rclpy','rclpy.node','rclpy.executors','sensor_msgs','sensor_msgs.msg',
          'std_msgs','std_msgs.msg','rcl_interfaces','rcl_interfaces.msg',
          'cv_bridge','torch','utils','utils.utils']:
    sys.modules.setdefault(n, types.ModuleType(n))
sys.modules['rclpy.node'].Node=type('Node',(),{})
sys.modules['rclpy.executors'].ExternalShutdownException=Exception
class _M:
    def __init__(s,data=None): s.data=data
for a in ('Float32','Int32','Bool'): setattr(sys.modules['std_msgs.msg'],a,_M)
sys.modules['sensor_msgs.msg'].Image=object; sys.modules['sensor_msgs.msg'].CameraInfo=object
sys.modules['rcl_interfaces.msg'].SetParametersResult=object
sys.modules['cv_bridge'].CvBridge=object
for f in ('select_device','driving_area_mask','lane_line_mask'):
    setattr(sys.modules['utils.utils'], f, lambda *a, **k: None)
spec=importlib.util.spec_from_file_location('serit', os.path.join(KOK,'SeritTespit/serit-tespitcopy.py'))
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)

H,W=720,1280
FX,CX = 535.23, 641.69      # ZED 2i SN36258172, LEFT_CAM_HD
ROWS=(0.80,0.76,0.72,0.68,0.64,0.60)

class Log:
    def info(s,m): pass
    def warn(s,m): pass
    def error(s,m): pass

def dugum(tol=1.0):
    n=object.__new__(mod.LaneDetectionNode)
    n.get_logger=lambda: Log()
    n.sample_rows=ROWS; n.camera_center_offset_px=0.0
    n.paint_inside_only=True; n.derinlik_pencere_px=7
    n.fx=FX; n.cx=CX; n.olcum_ileri_mesafe_m=3.0
    n.ayni_cizgi_tol_m=tol
    # Derinlik: alt satir yakin (2 m), ust satir uzak (6 m)
    d=np.zeros((H,W),np.float32)
    for v in range(H):
        d[v,:] = 2.0 + max(0.0,(576-v))/144.0*4.0
    n.depth_image=d
    return n

def maske(parcalar):
    m=np.zeros((H,W),np.uint8)
    for x0,x1,fraclar in parcalar:
        for f in fraclar:
            v=int(H*f); m[v-6:v+7, x0:x1]=1
    return m

def X(u,z): return (u-640.0)*z/FX
def z_of(f): return 2.0 + max(0.0,(576-int(H*f)))/144.0*4.0

print('='*78)
print('SENARYO: alt 3 satirda GERCEK sag cizgi (x=900), TUM satirlarda')
print('         saha kenari/bariyer (x=1250). Ekrandaki durumun aynisi.')
print('='*78)
for f in ROWS:
    print(f'   satir {f:.2f}: z={z_of(f):.2f} m | gercek cizgi X={X(900,z_of(f)):.2f} m'
          f' | bariyer X={X(1250,z_of(f)):.2f} m')
print()

lane = maske([(895,905,(0.80,0.76,0.72)), (1245,1255,ROWS)])
da   = np.ones((H,W),np.uint8)          # model her yeri 'surulebilir' isaretlemis

# ESKI davranis: tolerans cok buyuk -> butun adaylar ayni fite girer
eski = dugum(tol=999.0)._sag_cizgi_mesafesi(lane, da)
yeni = dugum(tol=1.0)._sag_cizgi_mesafesi(lane, da)

print(f'  ESKI (tum adaylar tek fite) : {eski[0]:.2f} m   isaretci u={eski[1]}')
print(f'  YENI (en yakin cizgiye kilit): {yeni[0]:.2f} m   isaretci u={yeni[1]}')
print()
ok=True
def kontrol(ad,kosul,detay):
    global ok
    print(f'  {"OK  " if kosul else "HATA"}  {ad:<50} {detay}')
    if not kosul: ok=False

gercek = X(899, 3.0)     # _row_lane_points kume ORTALAMASINI alir -> 899
kontrol('isaretci GERCEK cizgide, bariyerde (u=1250) degil',
        yeni[1] < 1000, f'u={yeni[1]}')
kontrol('yeni olcum gercek cizgiyle ayni',
        abs(yeni[0]-gercek) < 0.10, f'{yeni[0]:.2f} m (gercek {gercek:.2f})')
kontrol('eski olcum bariyerle KIRLENMIS',
        abs(eski[0]-gercek) > 0.30, f'{eski[0]:.2f} m, sapma {eski[0]-gercek:+.2f} m')

# Kontrolcuye ne gidiyor: hata = mesafe - 1.5, sapma = hata / 2.5
for ad, m in (('ESKI', eski[0]), ('YENI', yeni[0])):
    h = m - 1.5; sp = max(-1.0, min(h/2.5, 1.0))
    print(f'       {ad}: mesafe {m:.2f} m -> hata {h:+.2f} m -> Dev {sp:+.3f}'
          f'  ({"SAGA kir" if sp>0.02 else "SOLA kir" if sp<-0.02 else "MERKEZ"})')
kontrol('eski SAGA kirdiriyordu, yeni merkezde tutuyor',
        (eski[0]-1.5)/2.5 > 0.15 and abs((yeni[0]-1.5)/2.5) < 0.05,
        f'{(eski[0]-1.5)/2.5:+.3f} -> {(yeni[0]-1.5)/2.5:+.3f}')

print()
print('  --- gercek cizgi YALNIZ en alt satirda gorunuyor (agir kirlenme) ---')
lane_a = maske([(895,905,(0.80,)), (1245,1255,ROWS)])
e_a = dugum(tol=999.0)._sag_cizgi_mesafesi(lane_a, da)
y_a = dugum(tol=1.0)._sag_cizgi_mesafesi(lane_a, da)
print(f'       ESKI {e_a[0]:.2f} m (banda kirpilir -> 2.50)   YENI {y_a[0]:.2f} m  u={y_a[1]}')
kontrol('agir kirlenmede bile en yakin cizgide kaliyor',
        y_a[1] < 1000 and y_a[0] < 2.5, f'{y_a[0]:.2f} m, u={y_a[1]}')
kontrol('eski bu durumda bandi asiyordu', e_a[0] > 2.5, f'{e_a[0]:.2f} m')

print()
print('  --- gercek cizgi HIC gorunmuyorsa (sadece bariyer) ---')
lane2 = maske([(1245,1255,ROWS)])
s2 = dugum(tol=1.0)._sag_cizgi_mesafesi(lane2, da)
kontrol('kilitlenecek baska sey yok: bariyer donuyor (KALAN RISK)',
        s2[0] > 2.5, f'{s2[0]:.2f} m -> hala 2.50\'ye kirpilir')

print()
print('  --- regresyon: tek cizgi, tum satirlarda ---')
lane3 = maske([(895,905,ROWS)])
s3 = dugum(tol=1.0)._sag_cizgi_mesafesi(lane3, da)
bek = X(900, 3.0)
kontrol('normal durum bozulmadi (hedef z=3 m\'de degerlendiriliyor)',
        abs(s3[0]-bek) < 0.15, f'{s3[0]:.2f} m (beklenen ~{bek:.2f})')

print()
print('='*78); print('TUMU GECTI' if ok else 'BASARISIZ')
sys.exit(0 if ok else 1)
