import { useState, useEffect, useRef } from "react";

// ─── MISSION WIDGET ───────────────────────────────────────────────────────────
const STATE_COLOR = {
  IDLE: "var(--text-dim)", LOADING: "var(--blue)", READY: "var(--amber)",
  RUNNING: "var(--green)", PAUSED: "var(--amber)", COMPLETED: "var(--green)",
  ABORTED: "var(--red)",   ERROR: "var(--red)",
};

export default function MissionWidget({ subscribe, publish, connected }) {
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
