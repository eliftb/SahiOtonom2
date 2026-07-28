import React, { useState, useEffect } from 'react';
import './App.css';

const API_URL = 'http://127.20.10.2:3001';

function App() {
  const [isStartLoading, setIsStartLoading] = useState(false);
  const [isStopLoading, setIsStopLoading] = useState(false);
  const [response, setResponse] = useState(null);
  const [error, setError] = useState(null);
  const [processStatus, setProcessStatus] = useState({
    isRunning: false,
    processId: null,
    lastChecked: null
  });

  // Status kontrolü için fonksiyon
  const checkProcessStatus = async () => {
    try {
      const res = await fetch(`${API_URL}/status`);
      if (res.ok) {
        const data = await res.json();
        setProcessStatus({
          isRunning: data.isRunning,
          processId: data.processId,
          lastChecked: new Date().toLocaleString('tr-TR')
        });
      }
    } catch (err) {
      console.log('Status kontrolü başarısız:', err);
    }
  };

  // Component mount olduğunda ve her 3 saniyede bir status kontrolü
  useEffect(() => {
    checkProcessStatus(); // İlk kontrol
    const interval = setInterval(checkProcessStatus, 3000); // 3 saniyede bir
    
    return () => clearInterval(interval); // Cleanup
  }, []);

  const handleStart = async () => {
    setIsStartLoading(true);
    setError(null);
    setResponse(null);

    try {
      const res = await fetch(`${API_URL}/start`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      const data = await res.json();

      if (res.ok) {
        setResponse(data);
        console.log('✅ Backend yanıtı:', data);
        // Start başarılı olduğunda hemen status kontrol et
        setTimeout(checkProcessStatus, 1000);
      } else {
        setError(data.message || 'Bir hata oluştu');
      }
    } catch (err) {
      setError('Backend ile bağlantı kurulamadı. Backend çalışıyor mu?');
      console.error('❌ Bağlantı hatası:', err);
    } finally {
      setIsStartLoading(false);
    }
  };

  const handleStop = async () => {
    setIsStopLoading(true);
    setError(null);
    setResponse(null);

    try {
      const res = await fetch(`${API_URL}/stop`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      const data = await res.json();

      if (res.ok) {
        setResponse(data);
        console.log('🛑 Backend yanıtı:', data);
        // Stop başarılı olduğunda hemen status kontrol et
        setTimeout(checkProcessStatus, 1000);
      } else {
        setError(data.message || 'Bir hata oluştu');
      }
    } catch (err) {
      setError('Backend ile bağlantı kurulamadı. Backend çalışıyor mu?');
      console.error('❌ Bağlantı hatası:', err);
    } finally {
      setIsStopLoading(false);
    }
  };

  return (
    <div className="App">
      <div className="container">
        <h1>🚀 Şahi Otonom</h1>
        <p className="subtitle">Şahi Otonom</p>
        
        {/* Process Status Göstergesi */}
        <div className={`status-indicator ${processStatus.isRunning ? 'running' : 'stopped'}`}>
          <div className="status-dot"></div>
          <span className="status-text">
            {processStatus.isRunning ? 'Script Çalışıyor' : 'Script Durdu'}
          </span>
          {processStatus.processId && (
            <span className="process-id">PID: {processStatus.processId}</span>
          )}
        </div>
        
        <div className="button-container">
          <button 
            className={`start-button ${isStartLoading ? 'loading' : ''}`}
            onClick={handleStart}
            disabled={isStartLoading || isStopLoading}
          >
            {isStartLoading ? 'Başlatılıyor...' : 'START'}
          </button>
          
          <button 
            className={`stop-button ${isStopLoading ? 'loading' : ''}`}
            onClick={handleStop}
            disabled={isStartLoading || isStopLoading}
          >
            {isStopLoading ? 'Durduruluyor...' : 'STOP'}
          </button>
        </div>

        {response && (
          <div className="response success">
            <h3>✅ Başarılı!</h3>
            <p><strong>Mesaj:</strong> {response.message}</p>
            <p><strong>Durum:</strong> {response.status}</p>
            <p><strong>Zaman:</strong> {new Date(response.timestamp).toLocaleString('tr-TR')}</p>
          </div>
        )}

        {error && (
          <div className="response error">
            <h3>❌ Hata!</h3>
            <p>{error}</p>
          </div>
        )}

        <div className="info">
          <p>Backend adresi: <code>{API_URL}</code></p>
          <p>Backend'in çalıştığından emin olun!</p>
          {processStatus.lastChecked && (
            <p className="last-checked">Son kontrol: {processStatus.lastChecked}</p>
          )}
        </div>
      </div>
    </div>
  );
}

export default App;