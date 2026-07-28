# 🚀 Start App - Full Stack

Frontend'den tetiklenen `/start` endpoint'i olan basit full-stack uygulama.

## 📁 Proje Yapısı

```
proje/
├── backend/          # Express.js API
│   ├── package.json
│   └── server.js
├── frontend/         # React App
│   ├── package.json
│   ├── webpack.config.js
│   ├── public/
│   │   └── index.html
│   └── src/
│       ├── index.js
│       ├── App.js
│       └── App.css
└── README.md
```

## 🛠️ Kurulum ve Çalıştırma

### Backend (Port 3001)
```bash
cd backend
npm install
npm start
```

### Frontend (Port 3000)
```bash
cd frontend
npm install
npm start
```

## 🎯 API Endpoints

### POST /start
- **Açıklama**: Start komutunu alır ve işler
- **URL**: `http://localhost:3001/start`
- **Method**: POST
- **Response**: 
  ```json
  {
    "success": true,
    "message": "Start komutu başarıyla alındı!",
    "timestamp": "2024-01-01T12:00:00.000Z",
    "status": "started"
  }
  ```

### GET /health
- **Açıklama**: Backend sağlık kontrolü
- **URL**: `http://localhost:3001/health`

## ✨ Özellikler

- ✅ Express.js backend API
- ✅ React frontend uygulaması
- ✅ CORS desteği
- ✅ Modern UI tasarımı
- ✅ Loading states
- ✅ Error handling
- ✅ Responsive tasarım

## 🔧 Teknik Detaylar

- **Backend**: Express.js + CORS
- **Frontend**: React 18 + Webpack
- **API Communication**: Fetch API
- **Styling**: Vanilla CSS (gradients, animations)

## 🎮 Kullanım

1. Backend'i başlatın (`cd backend && npm start`)
2. Frontend'i başlatın (`cd frontend && npm start`)
3. Tarayıcıda `http://localhost:3000` adresini açın
4. "START" butonuna tıklayın
5. Backend yanıtını görün! 🎉


