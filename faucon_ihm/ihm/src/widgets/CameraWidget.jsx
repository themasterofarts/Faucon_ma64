import { useState } from "react";
import { ROBOT_HOST } from "../config";

// ─── CAMERA WIDGET ────────────────────────────────────────────────────────────
export default function CameraWidget({ config }) {
  const [streamUrl, setStreamUrl] = useState(config.videoServer || "");
  const [inputUrl, setInputUrl] = useState(config.videoServer || `http://${ROBOT_HOST}:8080/stream?topic=/camera/image`);
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
