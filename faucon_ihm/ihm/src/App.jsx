import { useState, useEffect, useRef, useCallback } from "react";

// ─── ROSBRIDGE CONNECTION HOOK ────────────────────────────────────────────────
function useRosBridge(url) {
  const wsRef = useRef(null);
  const [connected, setConnected] = useState(false);
  const [latency, setLatency] = useState(null);
  const subscribers = useRef({});
  const topicTypes = useRef({});
  const pingInterval = useRef(null);

  const connect = useCallback(() => {
    if (wsRef.current) wsRef.current.close();
    const ws = new WebSocket(url);
    wsRef.current = ws;

    ws.onopen = () => {
      setConnected(true);
      // Re-subscribe all topics after reconnect
      Object.keys(subscribers.current).forEach(topic => {
        if (subscribers.current[topic]?.length > 0 && topicTypes.current[topic]) {
          ws.send(JSON.stringify({ op: "subscribe", topic, type: topicTypes.current[topic] }));
        }
      });
      // Ping : on publie sur un topic /rosout (toujours présent) et on mesure
      // le RTT via le callback onmessage — méthode universelle sans service
      pingInterval.current = setInterval(() => {
        ws._pingTime = Date.now();
        ws.send(JSON.stringify({
          op: "advertise",
          topic: "/__ihm_ping__",
          type: "std_msgs/Bool"
        }));
        ws.send(JSON.stringify({
          op: "publish",
          topic: "/__ihm_ping__",
          msg: { data: true }
        }));
        ws.send(JSON.stringify({
          op: "unadvertise",
          topic: "/__ihm_ping__"
        }));
        // RTT estimé = temps entre envoi et traitement de la réponse WebSocket suivante
        const sent = Date.now();
        const orig = ws.onmessage;
        const once = (evt) => {
          setLatency(Date.now() - sent);
          ws.onmessage = orig;
        };
        ws._pingOnce = once;
      }, 3000);
    };

    ws.onclose = () => {
      setConnected(false);
      clearInterval(pingInterval.current);
      setTimeout(connect, 3000);
    };

    ws.onmessage = (evt) => {
      // RTT ping : on mesure sur le premier message reçu après envoi
      if (ws._pingOnce) {
        ws._pingOnce(evt);
        ws._pingOnce = null;
      }
      try {
        const msg = JSON.parse(evt.data);
        if (msg.op === "publish" && subscribers.current[msg.topic]) {
          subscribers.current[msg.topic].forEach(cb => cb(msg.msg));
        }
      } catch {}
    };
  }, [url]);

  useEffect(() => { connect(); return () => { wsRef.current?.close(); clearInterval(pingInterval.current); }; }, [connect]);

  const subscribe = useCallback((topic, type, cb) => {
    if (!subscribers.current[topic]) subscribers.current[topic] = [];
    subscribers.current[topic].push(cb);
    topicTypes.current[topic] = type;
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ op: "subscribe", topic, type }));
    }
    return () => {
      subscribers.current[topic] = (subscribers.current[topic] || []).filter(f => f !== cb);
      if (wsRef.current?.readyState === WebSocket.OPEN && subscribers.current[topic].length === 0) {
        wsRef.current.send(JSON.stringify({ op: "unsubscribe", topic }));
      }
    };
  }, []);

  const publish = useCallback((topic, type, msg) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ op: "publish", topic, type, msg }));
    }
  }, []);

  return { connected, latency, subscribe, publish };
}

// ─── STYLES ───────────────────────────────────────────────────────────────────
const css = `
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700;900&display=swap');

  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg: #080c0f;
    --bg2: #0d1317;
    --bg3: #111820;
    --border: #1a2a2a;
    --border-hi: #00ff9d33;
    --green: #00ff9d;
    --green-dim: #00ff9d66;
    --green-lo: #00ff9d1a;
    --amber: #ffb347;
    --red: #ff4455;
    --blue: #00aaff;
    --text: #c8ddd8;
    --text-dim: #5a7a72;
    --font-mono: 'Share Tech Mono', monospace;
    --font-display: 'Orbitron', monospace;
    --glow: 0 0 12px #00ff9d55;
    --glow-strong: 0 0 24px #00ff9d88;
  }

  body { background: var(--bg); color: var(--text); font-family: var(--font-mono); overflow: hidden; }

  /* Scanlines overlay */
  body::before {
    content: '';
    position: fixed; inset: 0; z-index: 9999; pointer-events: none;
    background: repeating-linear-gradient(0deg, transparent, transparent 2px, #00000022 2px, #00000022 4px);
  }

  #root { height: 100vh; display: flex; flex-direction: column; }

  /* TOPBAR */
  .topbar {
    display: flex; align-items: center; justify-content: space-between;
    padding: 0 20px; height: 52px; flex-shrink: 0;
    background: var(--bg2);
    border-bottom: 1px solid var(--border);
    box-shadow: 0 1px 0 var(--border-hi);
  }
  .topbar-brand { font-family: var(--font-display); font-size: 14px; font-weight: 900; color: var(--green); letter-spacing: 4px; text-shadow: var(--glow); }
  .topbar-brand span { color: var(--text-dim); font-weight: 400; font-size: 11px; }
  .topbar-center { display: flex; gap: 24px; }
  .status-pill {
    display: flex; align-items: center; gap: 6px; font-size: 11px; letter-spacing: 1px; color: var(--text-dim);
  }
  .status-dot { width: 7px; height: 7px; border-radius: 50%; animation: pulse 2s infinite; }
  .status-dot.on { background: var(--green); box-shadow: var(--glow); }
  .status-dot.off { background: var(--red); animation: none; }
  @keyframes pulse { 0%,100% { opacity:1 } 50% { opacity:.3 } }
  .topbar-right { display: flex; align-items: center; gap: 16px; font-size: 10px; color: var(--text-dim); }
  .latency { color: var(--green); font-size: 11px; }

  /* GRID */
  .dashboard {
    flex: 1; display: grid; overflow: hidden;
    grid-template-columns: 1fr 1fr 1fr;
    grid-template-rows: 1fr 1fr;
    gap: 1px; background: var(--border);
    padding: 1px;
  }

  /* PANEL */
  .panel {
    background: var(--bg2); position: relative; overflow: hidden;
    display: flex; flex-direction: column;
  }
  .panel::before {
    content: ''; position: absolute; inset: 0; pointer-events: none;
    background: linear-gradient(135deg, var(--green-lo) 0%, transparent 50%);
    opacity: 0;
    transition: opacity 0.3s;
  }
  .panel:hover::before { opacity: 1; }
  .panel-header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 8px 14px; border-bottom: 1px solid var(--border);
    background: var(--bg3); flex-shrink: 0;
  }
  .panel-title { font-family: var(--font-display); font-size: 9px; letter-spacing: 3px; color: var(--green); text-transform: uppercase; }
  .panel-badge { font-size: 9px; color: var(--text-dim); letter-spacing: 1px; }
  .panel-body { flex: 1; display: flex; align-items: center; justify-content: center; overflow: hidden; padding: 12px; }

  /* GPS PANEL - full left column */
  .panel-gps { grid-row: 1 / 3; }
  .map-container { width: 100%; height: 100%; position: relative; background: #0a1a1a; border: 1px solid var(--border); }
  .map-placeholder {
    width: 100%; height: 100%; display: flex; flex-direction: column;
    align-items: center; justify-content: center; gap: 12px;
    font-size: 11px; color: var(--text-dim);
  }
  .map-grid {
    position: absolute; inset: 0; pointer-events: none;
    background-image:
      linear-gradient(var(--border) 1px, transparent 1px),
      linear-gradient(90deg, var(--border) 1px, transparent 1px);
    background-size: 40px 40px;
  }
  .map-robot {
    position: absolute; transform: translate(-50%, -50%);
    transition: left 0.5s, top 0.5s;
  }
  .robot-dot { width: 14px; height: 14px; background: var(--green); border-radius: 50%; box-shadow: var(--glow-strong); animation: pulse 1.5s infinite; }
  .robot-ring {
    position: absolute; top: 50%; left: 50%; transform: translate(-50%,-50%);
    border: 1px solid var(--green-dim); border-radius: 50%;
    animation: radar-ring 2s infinite;
  }
  @keyframes radar-ring { 0% { width:14px;height:14px;opacity:1 } 100% { width:60px;height:60px;opacity:0 } }
  .gps-coords {
    position: absolute; bottom: 12px; left: 12px; right: 12px;
    background: #0008; backdrop-filter: blur(4px);
    border: 1px solid var(--border-hi); padding: 8px 12px;
    font-size: 10px; display: flex; gap: 16px;
  }
  .coord-item { display: flex; flex-direction: column; gap: 2px; }
  .coord-label { color: var(--text-dim); font-size: 9px; letter-spacing: 1px; }
  .coord-value { color: var(--green); font-size: 12px; }
  .map-trail { position: absolute; inset: 0; pointer-events: none; }

  /* IMU COCKPIT */
  .adi-container { position: relative; width: 180px; height: 180px; }
  .adi-outer {
    width: 180px; height: 180px; border-radius: 50%;
    border: 2px solid var(--green-dim);
    box-shadow: 0 0 30px #00ff9d22, inset 0 0 30px #00000066;
    overflow: hidden; position: relative;
    background: #0a1a14;
  }
  .adi-horizon {
    position: absolute; width: 100%; transform-origin: center;
    transition: transform 0.15s ease-out;
  }
  .adi-sky { height: 90px; background: linear-gradient(180deg, #001a33 0%, #003a66 100%); }
  .adi-ground { height: 90px; background: linear-gradient(180deg, #2a1400 0%, #1a0d00 100%); }
  .adi-line { position: absolute; width: 100%; height: 2px; background: var(--amber); top: 90px; margin-top: -1px; box-shadow: 0 0 8px var(--amber); }
  .adi-grid-lines { position: absolute; inset: 0; }
  .adi-pitch-line {
    position: absolute; width: 60%; left: 20%;
    height: 1px; background: rgba(255,255,255,0.3); font-size: 8px; color: rgba(255,255,255,0.5);
    display: flex; align-items: center; justify-content: space-between; padding: 0 4px;
  }
  .adi-overlay { position: absolute; inset: 0; pointer-events: none; }
  .adi-center-line { position: absolute; width: 100%; height: 1px; background: var(--amber); top: 50%; box-shadow: 0 0 6px var(--amber); }
  .adi-wings {
    position: absolute; top: 50%; left: 50%; transform: translate(-50%,-50%);
    width: 120px; height: 2px;
  }
  .adi-wing-left, .adi-wing-right {
    position: absolute; width: 35px; height: 3px; background: var(--amber);
    box-shadow: 0 0 6px var(--amber); top: -1px;
  }
  .adi-wing-left { left: 0; }
  .adi-wing-right { right: 0; }
  .adi-center-dot { position: absolute; top: 50%; left: 50%; transform: translate(-50%,-50%); width: 6px; height: 6px; background: var(--amber); border-radius: 50%; box-shadow: 0 0 8px var(--amber); }
  .adi-roll-arc {
    position: absolute; top: -2px; left: 50%; transform: translateX(-50%);
    width: 160px; height: 80px; border: 1px solid var(--green-dim);
    border-bottom: none; border-radius: 80px 80px 0 0;
  }
  .adi-roll-marker {
    position: absolute; bottom: 0; left: 50%; transform-origin: bottom center;
    width: 2px; height: 8px; background: var(--green);
  }
  .imu-readouts {
    display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; margin-top: 16px; width: 100%;
  }
  .imu-value-box { text-align: center; background: var(--bg3); border: 1px solid var(--border); padding: 6px 4px; }
  .imu-val-label { font-size: 8px; color: var(--text-dim); letter-spacing: 1px; }
  .imu-val { font-family: var(--font-display); font-size: 13px; color: var(--amber); }
  .imu-wrapper { display: flex; flex-direction: column; align-items: center; width: 100%; }

  /* CAMERA */
  .camera-wrapper { width: 100%; height: 100%; display: flex; flex-direction: column; gap: 8px; }
  .camera-feed {
    flex: 1; background: #000; position: relative; overflow: hidden;
    border: 1px solid var(--border);
    display: flex; align-items: center; justify-content: center;
  }
  .camera-feed img { width: 100%; height: 100%; object-fit: cover; display: block; }
  .camera-feed video { width: 100%; height: 100%; object-fit: cover; display: block; }
  .camera-no-feed { display: flex; flex-direction: column; align-items: center; gap: 8px; color: var(--text-dim); font-size: 11px; }
  .camera-hud {
    position: absolute; inset: 0; pointer-events: none;
    background: linear-gradient(180deg, rgba(0,255,157,0.08) 0%, transparent 30%, transparent 70%, rgba(0,255,157,0.08) 100%);
  }
  .camera-corners::before, .camera-corners::after {
    content: ''; position: absolute; width: 20px; height: 20px;
    border-color: var(--green); border-style: solid;
  }
  .camera-corners::before { top: 8px; left: 8px; border-width: 2px 0 0 2px; }
  .camera-corners::after { top: 8px; right: 8px; border-width: 2px 2px 0 0; }
  .camera-crosshair {
    position: absolute; top: 50%; left: 50%; transform: translate(-50%,-50%);
    width: 30px; height: 30px; pointer-events: none;
  }
  .camera-crosshair::before, .camera-crosshair::after {
    content: ''; position: absolute; background: var(--green-dim);
  }
  .camera-crosshair::before { width: 100%; height: 1px; top: 50%; }
  .camera-crosshair::after { width: 1px; height: 100%; left: 50%; }
  .camera-url-input { background: var(--bg3); border: 1px solid var(--border); color: var(--text); font-family: var(--font-mono); font-size: 10px; padding: 4px 8px; width: 100%; outline: none; }
  .camera-url-input:focus { border-color: var(--green-dim); }
  .rec-badge { position: absolute; top: 10px; right: 10px; background: var(--red); color: #fff; font-size: 9px; letter-spacing: 2px; padding: 2px 6px; display: flex; align-items: center; gap: 4px; }
  .rec-dot { width: 5px; height: 5px; border-radius: 50%; background: #fff; animation: pulse 1s infinite; }

  /* JOYSTICK */
  .joystick-panel { display: flex; flex-direction: column; align-items: center; gap: 16px; width: 100%; }
  .joystick-area { position: relative; display: flex; gap: 20px; align-items: center; }
  .joystick-wrap { display: flex; flex-direction: column; align-items: center; gap: 6px; }
  .joystick-label { font-size: 9px; color: var(--text-dim); letter-spacing: 2px; }
  .joystick-base {
    width: 120px; height: 120px; border-radius: 50%;
    background: radial-gradient(circle at 30% 30%, #1a2a22, #060e0a);
    border: 2px solid var(--border);
    box-shadow: 0 0 20px #00000088, inset 0 0 20px #00000088;
    position: relative; cursor: pointer; touch-action: none;
    display: flex; align-items: center; justify-content: center;
  }
  .joystick-base::before {
    content: ''; position: absolute; inset: 8px; border-radius: 50%;
    border: 1px solid var(--border-hi);
  }
  .joystick-thumb {
    position: absolute; width: 36px; height: 36px; border-radius: 50%;
    background: radial-gradient(circle at 35% 35%, #00ff9d88, #006644);
    border: 2px solid var(--green);
    box-shadow: var(--glow);
    transform: translate(-50%, -50%);
    left: 50%; top: 50%;
    transition: box-shadow 0.1s;
    pointer-events: none;
  }
  .joystick-thumb.active { box-shadow: var(--glow-strong); }
  .joystick-crosshair { position: absolute; inset: 0; pointer-events: none; opacity: 0.2; }
  .cmd-readout { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; width: 100%; }
  .cmd-item { background: var(--bg3); border: 1px solid var(--border); padding: 6px 10px; }
  .cmd-label { font-size: 9px; color: var(--text-dim); letter-spacing: 1px; }
  .cmd-val { font-family: var(--font-display); font-size: 14px; color: var(--green); }
  .cmd-bar-wrap { width: 100%; height: 3px; background: var(--bg3); margin-top: 4px; }
  .cmd-bar { height: 100%; background: var(--green); box-shadow: var(--glow); transition: width 0.1s; }
  .emergency-stop {
    background: transparent; border: 1px solid var(--red); color: var(--red);
    font-family: var(--font-display); font-size: 9px; letter-spacing: 3px;
    padding: 6px 20px; cursor: pointer;
    transition: all 0.15s;
  }
  .emergency-stop:hover { background: var(--red); color: #fff; box-shadow: 0 0 20px #ff445588; }
  .keyboard-hint { font-size: 9px; color: var(--text-dim); display: flex; gap: 12px; }
  .key-badge { background: var(--bg3); border: 1px solid var(--border); padding: 1px 5px; color: var(--text); }

  /* TELEMETRY PANEL */
  .telemetry-grid { width: 100%; display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .telem-card { background: var(--bg3); border: 1px solid var(--border); padding: 8px 10px; }
  .telem-card-label { font-size: 8px; color: var(--text-dim); letter-spacing: 2px; margin-bottom: 4px; }
  .telem-card-value { font-family: var(--font-display); font-size: 16px; color: var(--green); }
  .telem-card-unit { font-size: 9px; color: var(--text-dim); }
  .telem-card.warn .telem-card-value { color: var(--amber); }
  .telem-card.crit .telem-card-value { color: var(--red); animation: pulse 0.5s infinite; }
  .sparkline { width: 100%; height: 24px; margin-top: 4px; }

  /* CONFIG MODAL */
  .modal-overlay { position: fixed; inset: 0; background: #000a; z-index: 100; display: flex; align-items: center; justify-content: center; }
  .modal { background: var(--bg2); border: 1px solid var(--border-hi); padding: 24px; width: 420px; box-shadow: var(--glow-strong); }
  .modal-title { font-family: var(--font-display); font-size: 12px; letter-spacing: 3px; color: var(--green); margin-bottom: 16px; }
  .modal-field { margin-bottom: 12px; }
  .modal-field label { display: block; font-size: 10px; color: var(--text-dim); letter-spacing: 1px; margin-bottom: 4px; }
  .modal-field input { width: 100%; background: var(--bg3); border: 1px solid var(--border); color: var(--text); font-family: var(--font-mono); font-size: 12px; padding: 6px 10px; outline: none; }
  .modal-field input:focus { border-color: var(--green-dim); }
  .modal-actions { display: flex; gap: 8px; margin-top: 16px; }
  .btn { background: transparent; border: 1px solid; font-family: var(--font-mono); font-size: 10px; letter-spacing: 2px; padding: 6px 16px; cursor: pointer; transition: all 0.15s; }
  .btn-green { border-color: var(--green); color: var(--green); }
  .btn-green:hover { background: var(--green); color: #000; }
  .btn-dim { border-color: var(--border); color: var(--text-dim); }
  .btn-dim:hover { border-color: var(--text-dim); color: var(--text); }

  /* SCROLLBAR */
  ::-webkit-scrollbar { width: 4px; }
  ::-webkit-scrollbar-track { background: var(--bg); }
  ::-webkit-scrollbar-thumb { background: var(--border); }

  /* MISSION WIDGET */
  .mission-wrapper { width: 100%; display: flex; flex-direction: column; gap: 10px; }
  .mission-state-row { display: flex; justify-content: space-between; align-items: center; }
  .mission-state-dot {
    width: 8px; height: 8px; border-radius: 50%;
    flex-shrink: 0;
  }
  .mission-state-label { font-family: var(--font-display); font-size: 11px; letter-spacing: 2px; }
  .mission-id { font-size: 9px; color: var(--text-dim); }

  .mission-progress-wrap { width: 100%; }
  .mission-progress-header { display: flex; justify-content: space-between; font-size: 9px; color: var(--text-dim); margin-bottom: 4px; }
  .mission-progress-bar { width: 100%; height: 4px; background: var(--bg3); border: 1px solid var(--border); }
  .mission-progress-fill { height: 100%; transition: width 0.5s; }
  .mission-progress-pct { text-align: right; font-size: 9px; margin-top: 2px; }

  .mission-btns { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 6px; }
  .btn-amber { border-color: var(--amber); color: var(--amber); }
  .btn-amber:hover:not(:disabled) { background: var(--amber); color: #000; }
  .btn-red { border-color: var(--red); color: var(--red); }
  .btn-red:hover:not(:disabled) { background: var(--red); color: #fff; }
  .btn:disabled { opacity: 0.3; cursor: not-allowed; }

  .mission-resume-btn { width: 100%; font-size: 9px; padding: 6px 4px; }

  .mission-load-row { display: flex; gap: 6px; align-items: center; }
  .mission-filename { font-size: 9px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; flex: 1; }

  .mission-error { background: #ff445511; border: 1px solid var(--red); padding: 6px 8px; font-size: 9px; color: var(--red); }

  .mission-telem { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-top: 2px; }
  .mission-telem-card { background: var(--bg3); border: 1px solid var(--border); padding: 6px 8px; }
  .mission-telem-label { font-size: 8px; color: var(--text-dim); letter-spacing: 2px; margin-bottom: 2px; }
  .mission-telem-value { font-family: var(--font-display); font-size: 13px; }
`;

// ─── GPS MAP WIDGET ───────────────────────────────────────────────────────────
// ─── LEAFLET CSS (injected once) ─────────────────────────────────────────────
const leafletCSS = `
  .leaflet-container { background: #0a1a14 !important; font-family: var(--font-mono) !important; }
  .leaflet-tile { filter: brightness(0.7) saturate(0.6) hue-rotate(100deg); }
  .leaflet-control-zoom a { background: #0d1317 !important; color: var(--green) !important; border-color: #1a2a2a !important; font-family: var(--font-display) !important; }
  .leaflet-control-zoom a:hover { background: #1a2a22 !important; }
  .leaflet-control-attribution { background: #0008 !important; color: #5a7a72 !important; font-size: 8px !important; }
  .leaflet-control-attribution a { color: #5a7a72 !important; }
  .robot-marker-icon { position: relative; }
  .robot-pulse {
    width: 16px; height: 16px; border-radius: 50%;
    background: #00ff9d; box-shadow: 0 0 0 0 #00ff9d88;
    animation: leaflet-pulse 1.5s infinite;
  }
  @keyframes leaflet-pulse {
    0%   { box-shadow: 0 0 0 0 #00ff9d88; }
    70%  { box-shadow: 0 0 0 18px #00ff9d00; }
    100% { box-shadow: 0 0 0 0 #00ff9d00; }
  }
`;

function GPSWidget({ subscribe }) {
  const [gpsData, setGpsData] = useState(null);
  const [trail, setTrail] = useState([]);
  const mapRef = useRef(null);         // leaflet map instance
  const mapDivRef = useRef(null);      // DOM div
  const markerRef = useRef(null);      // robot marker
  const polylineRef = useRef(null);    // trail polyline
  const initializedRef = useRef(false);
  const firstFlyRef = useRef(false);

  // Inject Leaflet CSS once
  useEffect(() => {
    if (!document.getElementById("leaflet-css")) {
      const link = document.createElement("link");
      link.id = "leaflet-css";
      link.rel = "stylesheet";
      link.href = "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css";
      document.head.appendChild(link);
    }
    if (!document.getElementById("leaflet-override-css")) {
      const style = document.createElement("style");
      style.id = "leaflet-override-css";
      style.textContent = leafletCSS;
      document.head.appendChild(style);
    }
  }, []);

  // Init Leaflet map
  useEffect(() => {
    if (initializedRef.current || !mapDivRef.current) return;

    const initMap = () => {
      if (!window.L || initializedRef.current) return;
      initializedRef.current = true;

      const map = window.L.map(mapDivRef.current, {
        center: [43.9, 3.2],
        zoom: 18,
        zoomControl: true,
        attributionControl: true,
      });

      window.L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: '© <a href="https://www.openstreetmap.org/copyright">OSM</a>',
        maxZoom: 22,
        maxNativeZoom: 19,
      }).addTo(map);

      // Custom robot icon
      const robotIcon = window.L.divIcon({
        className: "robot-marker-icon",
        html: `<div class="robot-pulse"></div>`,
        iconSize: [16, 16],
        iconAnchor: [8, 8],
      });

      markerRef.current = window.L.marker([43.9, 3.2], { icon: robotIcon }).addTo(map);
      polylineRef.current = window.L.polyline([], { color: "#00ff9d", weight: 2, opacity: 0.7 }).addTo(map);
      mapRef.current = map;
    };

    // Load Leaflet JS if not present
    if (!window.L) {
      if (!document.getElementById("leaflet-js")) {
        const script = document.createElement("script");
        script.id = "leaflet-js";
        script.src = "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js";
        script.onload = initMap;
        document.head.appendChild(script);
      } else {
        const wait = setInterval(() => { if (window.L) { clearInterval(wait); initMap(); } }, 100);
      }
    } else {
      initMap();
    }

    return () => {
      if (mapRef.current) { mapRef.current.remove(); mapRef.current = null; initializedRef.current = false; }
    };
  }, []);

  // Subscribe to GNSS
  useEffect(() => {
    const unsub = subscribe("/gnss/fix", "sensor_msgs/NavSatFix", (msg) => {
      const pos = { lat: msg.latitude, lon: msg.longitude, alt: msg.altitude ?? 0, fix: msg.status?.status >= 0 };
      setGpsData(pos);
      setTrail(prev => [...prev.slice(-500), pos]);
    });
    return unsub;
  }, [subscribe]);

  // Update map when GPS data changes
  useEffect(() => {
    if (!gpsData || !mapRef.current) return;
    const latlng = [gpsData.lat, gpsData.lon];

    if (markerRef.current) markerRef.current.setLatLng(latlng);
    if (polylineRef.current) polylineRef.current.addLatLng(latlng);

    // Fly to first position, then follow with pan only
    if (!firstFlyRef.current) {
      mapRef.current.flyTo(latlng, 18, { duration: 1.5 });
      firstFlyRef.current = true;
    } else {
      mapRef.current.panTo(latlng, { animate: true, duration: 0.3 });
    }
  }, [gpsData]);

  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <div ref={mapDivRef} style={{ width: "100%", height: "100%", zIndex: 1 }} />
      <div className="gps-coords" style={{ zIndex: 10 }}>
        {[
          ["LAT", gpsData ? gpsData.lat.toFixed(6) : "WAITING..."],
          ["LON", gpsData ? gpsData.lon.toFixed(6) : "WAITING..."],
          ["ALT", gpsData ? `${gpsData.alt.toFixed(1)}m` : "---"],
          ["FIX", gpsData ? (gpsData.fix ? "OK" : "NONE") : "---"]
        ].map(([l, v]) => (
          <div key={l} className="coord-item">
            <span className="coord-label">{l}</span>
            <span className="coord-value" style={{ color: l === "FIX" ? (gpsData?.fix ? "var(--green)" : "var(--red)") : "var(--green)" }}>{v}</span>
          </div>
        ))}
      </div>
    </div>
  );
}

// ─── IMU COCKPIT (ADI) ────────────────────────────────────────────────────────
function IMUWidget({ subscribe }) {
  const [attitude, setAttitude] = useState({ roll: 0, pitch: 0, yaw: 0 });

  useEffect(() => {
    const unsub = subscribe("/imu/data", "sensor_msgs/Imu", (msg) => {
      const q = msg.orientation;
      // Quaternion to Euler
      const sinr = 2 * (q.w * q.x + q.y * q.z);
      const cosr = 1 - 2 * (q.x * q.x + q.y * q.y);
      const roll = Math.atan2(sinr, cosr) * (180 / Math.PI);
      const sinp = 2 * (q.w * q.y - q.z * q.x);
      const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * 90 : Math.asin(sinp) * (180 / Math.PI);
      const siny = 2 * (q.w * q.z + q.x * q.y);
      const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
      const yaw = Math.atan2(siny, cosy) * (180 / Math.PI);
      setAttitude({ roll, pitch, yaw });
    });
    return unsub;
  }, [subscribe]);

  // Simulate movement for demo when not connected
  useEffect(() => {
    let t = 0;
    const demo = setInterval(() => {
      t += 0.02;
      setAttitude(a => ({
        roll: Math.sin(t * 0.7) * 18,
        pitch: Math.sin(t * 0.4) * 8,
        yaw: (a.yaw + 0.3) % 360
      }));
    }, 50);
    return () => clearInterval(demo);
  }, []);

  const horizonTransform = `rotate(${-attitude.roll}deg) translateY(${attitude.pitch * 1.5}px)`;
  const pitchLines = [-20, -10, 0, 10, 20];

  return (
    <div className="imu-wrapper">
      <div className="adi-container">
        <div className="adi-outer">
          <div className="adi-horizon" style={{ transform: horizonTransform }}>
            <div className="adi-sky" />
            <div className="adi-ground" />
            <div className="adi-line" />
            {pitchLines.map(p => (
              <div key={p} className="adi-pitch-line" style={{ top: `${90 + p * 3}px` }}>
                <span>{p !== 0 ? p : ""}</span><span>{p !== 0 ? p : ""}</span>
              </div>
            ))}
          </div>
          <div className="adi-overlay">
            <div className="adi-wings">
              <div className="adi-wing-left" />
              <div className="adi-wing-right" />
            </div>
            <div className="adi-center-dot" />
          </div>
          <div className="adi-roll-arc">
            {[-30, -20, -10, 0, 10, 20, 30].map(a => (
              <div key={a} className="adi-roll-marker" style={{
                transform: `translateX(-50%) rotate(${a}deg)`,
                height: a === 0 ? "14px" : "8px",
                background: a === 0 ? "var(--green)" : "var(--text-dim)"
              }} />
            ))}
          </div>
        </div>
      </div>
      <div className="imu-readouts">
        {[["ROLL", attitude.roll, "°"], ["PITCH", attitude.pitch, "°"], ["HDG", ((attitude.yaw + 360) % 360), "°"]].map(([l, v, u]) => (
          <div key={l} className="imu-value-box">
            <div className="imu-val-label">{l}</div>
            <div className="imu-val">{v.toFixed(1)}<span style={{ fontSize: 9, color: "var(--text-dim)" }}>{u}</span></div>
          </div>
        ))}
      </div>
    </div>
  );
}

// ─── CAMERA WIDGET ────────────────────────────────────────────────────────────
function CameraWidget({ config }) {
  const [streamUrl, setStreamUrl] = useState(config.videoServer || "");
  const [inputUrl, setInputUrl] = useState(config.videoServer || "http://localhost:8080/stream?topic=/camera/image");
  const [loaded, setLoaded] = useState(false);

  return (
    <div className="camera-wrapper">
      <div className="camera-feed">
        {streamUrl ? (
          <>
            <img src={streamUrl} alt="camera" onLoad={() => setLoaded(true)} onError={() => setLoaded(false)} style={{ display: loaded ? "block" : "none" }} />
            {!loaded && <div className="camera-no-feed"><svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1"><path d="M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3l-2.5-3z"/><circle cx="12" cy="13" r="3"/></svg><span>CONNECTING...</span></div>}
          </>
        ) : (
          <div className="camera-no-feed">
            <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="#5a7a72" strokeWidth="1"><path d="M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3l-2.5-3z"/><circle cx="12" cy="13" r="3"/></svg>
            <span>NO STREAM CONFIGURED</span>
          </div>
        )}
        <div className="camera-hud" />
        <div className="camera-corners" style={{ position: "absolute", inset: 0 }} />
        <div className="camera-crosshair" />
        {streamUrl && loaded && <div className="rec-badge"><div className="rec-dot" />LIVE</div>}
        <div style={{ position: "absolute", top: 8, left: 8, fontSize: 9, color: "var(--green-dim)" }}>CAM/01</div>
      </div>
      <div style={{ display: "flex", gap: 6 }}>
        <input className="camera-url-input" style={{ flex: 1 }} value={inputUrl}
          onChange={e => setInputUrl(e.target.value)}
          placeholder="http://localhost:8080/stream?topic=/camera/image"
        />
        <button className="btn btn-green" style={{ fontSize: 9, padding: "4px 10px", whiteSpace: "nowrap" }}
          onClick={() => setStreamUrl(inputUrl)}>LOAD</button>
      </div>
    </div>
  );
}

// ─── JOYSTICK WIDGET ──────────────────────────────────────────────────────────
function JoystickWidget({ publish, connected }) {
  const [vel, setVel] = useState({ linear: 0, angular: 0 });
  const [active, setActive] = useState(false);
  const baseRef = useRef(null);
  const thumbRef = useRef(null);
  const frameRef = useRef(null);
  const velRef = useRef({ linear: 0, angular: 0 });
  const keysRef = useRef({});

  const MAX_LINEAR = 1.0;
  const MAX_ANGULAR = 1.5;

  const sendCmd = useCallback((linear, angular) => {
    if (!connected) return;
    publish("/cmd_vel", "geometry_msgs/Twist", {
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    });
  }, [publish, connected]);

  // Keyboard control
  useEffect(() => {
    const down = (e) => { keysRef.current[e.key] = true; };
    const up = (e) => { keysRef.current[e.key] = false; };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    const tick = setInterval(() => {
      const k = keysRef.current;
      let lin = 0, ang = 0;
      if (k["ArrowUp"] || k["z"] || k["w"]) lin = MAX_LINEAR;
      if (k["ArrowDown"] || k["s"]) lin = -MAX_LINEAR * 0.5;
      if (k["ArrowLeft"] || k["q"] || k["a"]) ang = MAX_ANGULAR;
      if (k["ArrowRight"] || k["d"]) ang = -MAX_ANGULAR;
      velRef.current = { linear: lin, angular: ang };
      setVel({ linear: lin, angular: ang });
      if (lin !== 0 || ang !== 0) sendCmd(lin, ang);
    }, 100);
    return () => { window.removeEventListener("keydown", down); window.removeEventListener("keyup", up); clearInterval(tick); };
  }, [sendCmd]);

  // Touch/mouse joystick
  const handlePointerDown = useCallback((e) => {
    e.preventDefault();
    setActive(true);
    const base = baseRef.current.getBoundingClientRect();
    const cx = base.left + base.width / 2, cy = base.top + base.height / 2;
    const radius = base.width / 2 - 18;

    const move = (clientX, clientY) => {
      let dx = clientX - cx, dy = clientY - cy;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist > radius) { dx = (dx / dist) * radius; dy = (dy / dist) * radius; }
      if (thumbRef.current) {
        thumbRef.current.style.left = `${50 + (dx / radius) * 50}%`;
        thumbRef.current.style.top = `${50 + (dy / radius) * 50}%`;
      }
      const lin = -dy / radius * MAX_LINEAR;
      const ang = -dx / radius * MAX_ANGULAR;
      setVel({ linear: lin, angular: ang });
      sendCmd(lin, ang);
    };

    const onMove = (ev) => move(ev.clientX ?? ev.touches?.[0]?.clientX, ev.clientY ?? ev.touches?.[0]?.clientY);
    const onEnd = () => {
      setActive(false);
      if (thumbRef.current) { thumbRef.current.style.left = "50%"; thumbRef.current.style.top = "50%"; }
      setVel({ linear: 0, angular: 0 });
      sendCmd(0, 0);
      window.removeEventListener("pointermove", onMove);
      window.removeEventListener("pointerup", onEnd);
    };
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onEnd);
    move(e.clientX, e.clientY);
  }, [sendCmd]);

  const stop = () => { sendCmd(0, 0); setVel({ linear: 0, angular: 0 }); };

  const linPct = Math.abs(vel.linear / MAX_LINEAR) * 100;
  const angPct = Math.abs(vel.angular / MAX_ANGULAR) * 100;

  return (
    <div className="joystick-panel">
      <div className="joystick-area">
        <div className="joystick-wrap">
          <div className="joystick-label">DRIVE</div>
          <div className="joystick-base" ref={baseRef} onPointerDown={handlePointerDown}>
            <svg className="joystick-crosshair" viewBox="0 0 120 120">
              <line x1="60" y1="10" x2="60" y2="110" stroke="#00ff9d" strokeWidth="0.5" />
              <line x1="10" y1="60" x2="110" y2="60" stroke="#00ff9d" strokeWidth="0.5" />
              <circle cx="60" cy="60" r="40" fill="none" stroke="#00ff9d" strokeWidth="0.5" />
              <circle cx="60" cy="60" r="20" fill="none" stroke="#00ff9d" strokeWidth="0.5" />
            </svg>
            <div ref={thumbRef} className={`joystick-thumb ${active ? "active" : ""}`} />
          </div>
        </div>
      </div>
      <div className="cmd-readout">
        {[["LINEAR", vel.linear, "m/s", linPct, vel.linear < 0], ["ANGULAR", vel.angular, "rad/s", angPct, false]].map(([l, v, u, pct, warn]) => (
          <div key={l} className="cmd-item">
            <div className="cmd-label">{l}</div>
            <div className="cmd-val" style={{ color: warn ? "var(--amber)" : "var(--green)" }}>{v.toFixed(2)}<span style={{ fontSize: 9, color: "var(--text-dim)", marginLeft: 2 }}>{u}</span></div>
            <div className="cmd-bar-wrap"><div className="cmd-bar" style={{ width: `${pct}%`, background: warn ? "var(--amber)" : "var(--green)" }} /></div>
          </div>
        ))}
      </div>
      <div className="keyboard-hint">
        {[["Z/W","FWD"], ["S","BWD"], ["Q/A","LEFT"], ["D","RIGHT"]].map(([k, l]) => (
          <span key={k}><span className="key-badge">{k}</span> {l}</span>
        ))}
      </div>
      <button className="emergency-stop" onClick={stop}>■ EMERGENCY STOP</button>
    </div>
  );
}

// ─── MISSION WIDGET ───────────────────────────────────────────────────────────
const STATE_COLOR = {
  IDLE: "var(--text-dim)", LOADING: "var(--blue)", READY: "var(--amber)",
  RUNNING: "var(--green)", PAUSED: "var(--amber)", COMPLETED: "var(--green)",
  ABORTED: "var(--red)",   ERROR: "var(--red)",
};

function MissionWidget({ subscribe, publish, connected }) {
  const [status, setStatus] = useState({
    state: "IDLE", mission_id: "", progress: 0, current_wp: 0, total_wp: 0, error: "",
  });
  const [speed, setSpeed]     = useState(0);
  const [uptime, setUptime]   = useState(0);
  const [fileName, setFileName] = useState("");
  const startRef  = useRef(Date.now());
  const fileRef   = useRef(null);

  useEffect(() => {
    const unsub = subscribe("/mission/status", "std_msgs/String", (msg) => {
      try { setStatus(JSON.parse(msg.data)); } catch {}
    });
    return unsub;
  }, [subscribe]);

  useEffect(() => {
    const unsub = subscribe("/odom", "nav_msgs/Odometry", (msg) => {
      const vx = msg.twist?.twist?.linear?.x ?? 0;
      setSpeed(Math.abs(vx));
    });
    const timer = setInterval(() => {
      setUptime(Math.floor((Date.now() - startRef.current) / 1000));
    }, 1000);
    return () => { unsub(); clearInterval(timer); };
  }, [subscribe]);

  const sendCmd = (cmd) =>
    publish("/mission/command", "std_msgs/String", { data: cmd });

  const handleFileLoad = (e) => {
    const file = e.target.files?.[0];
    if (!file) return;
    setFileName(file.name);
    const reader = new FileReader();
    reader.onload = (ev) =>
      publish("/mission/load_path", "std_msgs/String", { data: ev.target.result });
    reader.readAsText(file);
    e.target.value = "";
  };

  const { state, mission_id, progress, current_wp, total_wp, error } = status;
  const color    = STATE_COLOR[state] ?? "var(--text-dim)";
  const pct      = (progress * 100).toFixed(1);
  const isAnim   = state === "RUNNING" || state === "ERROR";
  const canStart  = state === "READY"   && connected;
  const canPause  = state === "RUNNING" && connected;
  const canResume = state === "PAUSED"  && connected;
  const canStop   = ["RUNNING","PAUSED","READY","ERROR"].includes(state) && connected;
  const canLoad   = state !== "RUNNING" && connected;
  const fmt = (s) => `${String(Math.floor(s/3600)).padStart(2,"0")}:${String(Math.floor((s%3600)/60)).padStart(2,"0")}:${String(s%60).padStart(2,"0")}`;

  return (
    <div className="mission-wrapper">
      {/* État + ID */}
      <div className="mission-state-row">
        <div style={{ display:"flex", alignItems:"center", gap:8 }}>
          <div className="mission-state-dot" style={{
            background: color,
            boxShadow: isAnim ? `0 0 8px ${color}` : "none",
            animation: isAnim ? "pulse 1.5s infinite" : "none",
          }}/>
          <span className="mission-state-label" style={{ color }}>{state}</span>
        </div>
        {mission_id && <span className="mission-id">ID: {mission_id}</span>}
      </div>

      {/* Barre de progression */}
      <div className="mission-progress-wrap">
        <div className="mission-progress-header">
          <span>PROGRESS</span>
          <span>{total_wp > 0 ? `WP ${current_wp} / ${total_wp}` : "---"}</span>
        </div>
        <div className="mission-progress-bar">
          <div className="mission-progress-fill" style={{
            width: `${pct}%`, background: color,
            boxShadow: state === "RUNNING" ? `0 0 6px ${color}` : "none",
          }}/>
        </div>
        <div className="mission-progress-pct" style={{ color }}>{pct}%</div>
      </div>

      {/* Boutons principaux */}
      <div className="mission-btns">
        <button className="btn btn-green" disabled={!canStart}
          onClick={() => sendCmd("START")} style={{ fontSize:9, padding:"6px 4px" }}>
          ▶ START
        </button>
        <button className="btn btn-amber" disabled={!canPause}
          onClick={() => sendCmd("PAUSE")} style={{ fontSize:9, padding:"6px 4px" }}>
          ⏸ PAUSE
        </button>
        <button className="btn btn-red" disabled={!canStop}
          onClick={() => sendCmd("STOP")} style={{ fontSize:9, padding:"6px 4px" }}>
          ■ STOP
        </button>
      </div>

      {/* Bouton RESUME (visible uniquement en PAUSED) */}
      {state === "PAUSED" && (
        <button className="btn btn-green mission-resume-btn" disabled={!canResume}
          onClick={() => sendCmd("RESUME")}>
          ▶▶ RESUME MISSION
        </button>
      )}

      {/* Chargement du fichier YAML */}
      <div className="mission-load-row">
        <input ref={fileRef} type="file" accept=".yaml,.yml"
          style={{ display:"none" }} onChange={handleFileLoad} />
        <button className="btn btn-dim" disabled={!canLoad}
          style={{ fontSize:9, padding:"5px 10px", whiteSpace:"nowrap" }}
          onClick={() => fileRef.current?.click()}>
          ↑ LOAD YAML
        </button>
        <span className="mission-filename" style={{ color: fileName ? "var(--text)" : "var(--text-dim)" }}>
          {fileName || "no file selected"}
        </span>
      </div>

      {/* Message d'erreur */}
      {error && <div className="mission-error">⚠ {error}</div>}

      {/* Télémétrie compacte */}
      <div className="mission-telem">
        {[
          ["SPEED",  `${speed.toFixed(2)} m/s`, speed > 0.8 ? "var(--amber)" : "var(--green)"],
          ["UPTIME", fmt(uptime),                "var(--green)"],
        ].map(([label, value, c]) => (
          <div key={label} className="mission-telem-card">
            <div className="mission-telem-label">{label}</div>
            <div className="mission-telem-value" style={{ color: c }}>{value}</div>
          </div>
        ))}
      </div>
    </div>
  );
}

// ─── CONFIG MODAL ─────────────────────────────────────────────────────────────
function ConfigModal({ config, onSave, onClose }) {
  const [cfg, setCfg] = useState(config);
  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal" onClick={e => e.stopPropagation()}>
        <div className="modal-title">⚙ SYSTEM CONFIGURATION</div>
        {[
          ["ROSBRIDGE HOST", "rosBridgeUrl", "ws://192.168.1.x:9090"],
          ["VIDEO SERVER", "videoServer", "http://192.168.1.x:8080/stream?topic=/camera/image"],
          ["GPS TOPIC", "gpsTopic", "/gps/fix"],
          ["IMU TOPIC", "imuTopic", "/imu/data"],
          ["CMD_VEL TOPIC", "cmdVelTopic", "/cmd_vel"],
        ].map(([label, key, ph]) => (
          <div key={key} className="modal-field">
            <label>{label}</label>
            <input value={cfg[key] || ""} placeholder={ph} onChange={e => setCfg(c => ({ ...c, [key]: e.target.value }))} />
          </div>
        ))}
        <div className="modal-actions">
          <button className="btn btn-green" onClick={() => onSave(cfg)}>APPLY</button>
          <button className="btn btn-dim" onClick={onClose}>CANCEL</button>
        </div>
      </div>
    </div>
  );
}

// ─── ROOT APP ─────────────────────────────────────────────────────────────────
export default function App() {
  const [showConfig, setShowConfig] = useState(false);
  const [config, setConfig] = useState({
    rosBridgeUrl: "ws://localhost:9090",
    videoServer: "",
    gpsTopic: "/gnss/fix",
    imuTopic: "/imu/data",
    cmdVelTopic: "/cmd_vel",
  });

  const { connected, latency, subscribe, publish } = useRosBridge(config.rosBridgeUrl);
  const now = new Date().toISOString().replace("T", " ").slice(0, 19);

  return (
    <>
      <style>{css}</style>
      {showConfig && <ConfigModal config={config} onSave={(c) => { setConfig(c); setShowConfig(false); }} onClose={() => setShowConfig(false)} />}
      <div id="root">
        {/* TOPBAR */}
        <div className="topbar">
          <div className="topbar-brand">ROS2<span> // GROUND CONTROL</span></div>
          <div className="topbar-center">
            <div className="status-pill">
              <div className={`status-dot ${connected ? "on" : "off"}`} />
              {connected ? "ROSBRIDGE CONNECTED" : "DISCONNECTED"}
            </div>
            {latency && <div className="status-pill latency">{latency}ms RTT</div>}
            <div className="status-pill">
              <div className="status-dot on" />
              GAZEBO SIM
            </div>
          </div>
          <div className="topbar-right">
            <span>{now} UTC</span>
            <button className="btn btn-dim" style={{ padding: "3px 10px", fontSize: 9 }} onClick={() => setShowConfig(true)}>⚙ CONFIG</button>
          </div>
        </div>

        {/* DASHBOARD GRID */}
        <div className="dashboard">
          {/* GPS — spans 2 rows */}
          <div className="panel panel-gps">
            <div className="panel-header">
              <span className="panel-title">⬡ GPS POSITION</span>
              <span className="panel-badge">NavSatFix · /gps/fix</span>
            </div>
            <div className="panel-body" style={{ padding: 0 }}>
              <GPSWidget subscribe={subscribe} />
            </div>
          </div>

          {/* IMU */}
          <div className="panel">
            <div className="panel-header">
              <span className="panel-title">✈ ATTITUDE INDICATOR</span>
              <span className="panel-badge">IMU · /imu/data</span>
            </div>
            <div className="panel-body">
              <IMUWidget subscribe={subscribe} />
            </div>
          </div>

          {/* CAMERA */}
          <div className="panel">
            <div className="panel-header">
              <span className="panel-title">◈ CAMERA FEED</span>
              <span className="panel-badge">web_video_server · MJPEG</span>
            </div>
            <div className="panel-body">
              <CameraWidget config={config} />
            </div>
          </div>

          {/* JOYSTICK */}
          <div className="panel">
            <div className="panel-header">
              <span className="panel-title">◎ MANUAL CONTROL</span>
              <span className="panel-badge">Twist · /cmd_vel</span>
            </div>
            <div className="panel-body">
              <JoystickWidget publish={publish} connected={connected} />
            </div>
          </div>

          {/* MISSION */}
          <div className="panel">
            <div className="panel-header">
              <span className="panel-title">◎ MISSION CONTROL</span>
              <span className="panel-badge">/mission/status · /mission/command</span>
            </div>
            <div className="panel-body" style={{ alignItems:"flex-start", overflowY:"auto" }}>
              <MissionWidget subscribe={subscribe} publish={publish} connected={connected} />
            </div>
          </div>
        </div>
      </div>
    </>
  );
}