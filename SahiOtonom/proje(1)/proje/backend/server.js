// backend/index.js
const express = require('express');
const cors = require('cors');
const { spawn } = require('child_process');
const os = require('os');

const app = express();
app.use(cors());
app.use(express.json());

const PORT = 3001;

// Ağ IP adresini bul
function getLocalIp() {
  const interfaces = os.networkInterfaces();
  for (let iface of Object.values(interfaces)) {
    for (let alias of iface) {
      if (alias.family === 'IPv4' && !alias.internal) {
        return alias.address;
      }
    }
  }
  return 'localhost';
}

// Çalıştırılacak komut listesi
const NODE_LAUNCH_ORDER = [
  {
    name: "KAMERA YAYINI (ZED)",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/Kamera/zedi2connect_port.py"],
    delay_after: 5000
  },
  {
    name: "HABERLEŞME (UART)",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/Haberlesme/uart_sender_node3.py"],
    delay_after: 2000
  },
  {
    name: "ŞERİT TESPİT",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/SeritTespit/serit-tespitcopy.py"],
    delay_after: 3000
  },
  {
    name: "LEVHA TESPİT (Görüntü İşleme)",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/GoruntuIsleme/run_tracker.py"],
    delay_after: 3000
  },
  {
    name: "LIDAR ENGEL TESPİTİ",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/EngelTespit/engel-tespit.py"],
    delay_after: 2000
  },
  {
    name: "KARAR ALMA ALGORİTMASI",
    cmd: "python3",
    args: ["/home/sahi/SahiOtonom/KararAlg/basic-decision-making-node.py"],
    delay_after: 1000
  }
];

let isRunning = false;
let runningProcesses = [];

function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

app.get('/start', async (req, res) => {
  if (isRunning) {
    return res.status(400).json({ success: false, message: 'Zaten çalışıyor' });
  }

  isRunning = true;
  runningProcesses = [];

  try {
    for (const step of NODE_LAUNCH_ORDER) {
      console.log(`🚀 Başlatılıyor: ${step.cmd} ${step.args.join(' ')}`);

      const child = spawn(step.cmd, step.args, { stdio: 'inherit' });
      runningProcesses.push(child);

      console.log(`PID: ${child.pid}`);

      if (step.delay_after) {
        await delay(step.delay_after);
      }
    }

    res.json({ success: true, message: 'Tüm komutlar başlatıldı', pids: runningProcesses.map(p => p.pid) });
  } catch (err) {
    console.error('❌ Başlatma hatası:', err);
    res.status(500).json({ success: false, message: 'Başlatma sırasında hata oluştu' });
  }
});

app.get('/stop', (req, res) => {
  if (!isRunning) {
    return res.status(400).json({ success: false, message: 'Zaten çalışmıyor' });
  }

  runningProcesses.forEach(proc => {
    try {
      process.kill(proc.pid);
      console.log(`🛑 PID ${proc.pid} sonlandırıldı`);
    } catch (err) {
      console.error(`PID ${proc.pid} kapatılamadı:`, err);
    }
  });

  runningProcesses = [];
  isRunning = false;

  res.json({ success: true, message: 'Tüm processler durduruldu' });
});

// Burada 0.0.0.0 ile dinleme yapıyoruz
app.listen(PORT, '0.0.0.0', () => {
  console.log(`✅ Backend http://${getLocalIp()}:${PORT} üzerinden erişilebilir`);
});
