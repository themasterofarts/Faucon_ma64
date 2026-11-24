
// ─── STYLES ───────────────────────────────────────────────────────────────────
export const css = `
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
  .emergency-stop.active { background: var(--red); color: #fff; box-shadow: 0 0 20px #ff445588; animation: estop-blink 1s step-end infinite; }
  .emergency-stop:disabled { opacity: 0.4; cursor: not-allowed; }
  @keyframes estop-blink { 50% { background: transparent; color: var(--red); } }
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
