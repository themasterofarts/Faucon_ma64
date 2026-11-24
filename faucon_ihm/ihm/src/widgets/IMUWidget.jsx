import { useState, useEffect, useRef } from "react";

// ─── IMU COCKPIT (ADI) ────────────────────────────────────────────────────────
// Mode démo de l'horizon artificiel (attitude simulée). DÉSACTIVÉ par défaut :
// il écrasait en permanence les vraies données IMU. À n'activer que pour une
// démonstration sans robot — un bandeau "DÉMO" est alors affiché.
const IMU_DEMO = false;
// Au-delà de ce délai sans message IMU, les valeurs sont signalées comme absentes
const IMU_STALE_MS = 1500;

export default function IMUWidget({ subscribe }) {
  const [attitude, setAttitude] = useState({ roll: 0, pitch: 0, yaw: 0 });
  const lastImuRef = useRef(0);
  const [stale, setStale] = useState(true);

  useEffect(() => {
    // IMU publiée à 100 Hz : 10 Hz suffisent largement pour l'horizon artificiel
    const unsub = subscribe("/imu/data", "sensor_msgs/Imu", (msg) => {
      lastImuRef.current = Date.now();
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
    }, { throttleRate: 100 });
    return unsub;
  }, [subscribe]);

  // Détection d'absence de données IMU
  useEffect(() => {
    if (IMU_DEMO) return undefined;
    const check = setInterval(() => {
      setStale(Date.now() - lastImuRef.current > IMU_STALE_MS);
    }, 250);
    return () => clearInterval(check);
  }, []);

  // Simulation d'attitude pour démonstration (uniquement si IMU_DEMO = true)
  useEffect(() => {
    if (!IMU_DEMO) return undefined;
    setStale(false);
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
          {(stale || IMU_DEMO) && (
            <div style={{
              position: "absolute", inset: 0, display: "flex", alignItems: "center", justifyContent: "center",
              background: "#080c0fcc", color: IMU_DEMO ? "var(--amber)" : "var(--red)", zIndex: 5,
              fontSize: 10, letterSpacing: 2, textAlign: "center", pointerEvents: "none",
            }}>
              {IMU_DEMO ? "DÉMO — ATTITUDE SIMULÉE" : "PAS DE DONNÉES IMU"}
            </div>
          )}
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
            <div className="imu-val">{stale ? "--.-" : v.toFixed(1)}<span style={{ fontSize: 9, color: "var(--text-dim)" }}>{u}</span></div>
          </div>
        ))}
      </div>
    </div>
  );
}
