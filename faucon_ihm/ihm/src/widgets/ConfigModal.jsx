import { useState } from "react";

// ─── CONFIG MODAL ─────────────────────────────────────────────────────────────
export default function ConfigModal({ config, onSave, onClose }) {
  const [cfg, setCfg] = useState(config);
  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="modal" onClick={e => e.stopPropagation()}>
        <div className="modal-title">⚙ SYSTEM CONFIGURATION</div>
        {[
          ["ROSBRIDGE HOST", "rosBridgeUrl", "ws://192.168.1.x:9090"],
          ["VIDEO SERVER", "videoServer", "http://192.168.1.x:8080/stream?topic=/camera/image"],
          ["GPS TOPIC", "gpsTopic", "/gnss/fix"],
          ["IMU TOPIC", "imuTopic", "/imu/data"],
          ["CMD_VEL TOPIC", "cmdVelTopic", "/cmd_vel_teleop"],
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
