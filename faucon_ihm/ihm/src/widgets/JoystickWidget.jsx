import { useState, useEffect, useRef, useCallback } from "react";

// ─── JOYSTICK WIDGET ──────────────────────────────────────────────────────────
export default function JoystickWidget({ publish, subscribe, connected }) {
  const [vel, setVel] = useState({ linear: 0, angular: 0 });
  const [active, setActive] = useState(false);
  const baseRef = useRef(null);
  const thumbRef = useRef(null);
  const frameRef = useRef(null);
  const velRef = useRef({ linear: 0, angular: 0 });
  const keysRef = useRef({});

  const MAX_LINEAR = 1.0;
  const MAX_ANGULAR = 1.5;
  // Topic d'entree teleop de twist_mux (prioritaire sur Nav2), qui publie ensuite /cmd_vel
  const CMD_VEL_TELEOP_TOPIC = "/cmd_vel_teleop";

  const sendCmd = useCallback((linear, angular) => {
    if (!connected) return;
    publish(CMD_VEL_TELEOP_TOPIC, "geometry_msgs/Twist", {
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    });
  }, [publish, connected]);

  // Keyboard control
  const keyboardMovingRef = useRef(false);
  useEffect(() => {
    // Ne pas piloter le robot quand l'utilisateur tape dans un champ de saisie
    const isTyping = (e) => {
      const t = e.target;
      return t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.tagName === "SELECT" || t.isContentEditable);
    };
    const down = (e) => { if (!isTyping(e)) keysRef.current[e.key] = true; };
    const up = (e) => { keysRef.current[e.key] = false; };
    // Perte de focus : les keyup ne seront jamais reçus → on relâche toutes les touches
    const releaseAll = () => { keysRef.current = {}; };
    const onVisibility = () => { if (document.hidden) releaseAll(); };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    window.addEventListener("blur", releaseAll);
    document.addEventListener("visibilitychange", onVisibility);
    const tick = setInterval(() => {
      const k = keysRef.current;
      let lin = 0, ang = 0;
      if (k["ArrowUp"] || k["z"] || k["w"]) lin = MAX_LINEAR;
      if (k["ArrowDown"] || k["s"]) lin = -MAX_LINEAR * 0.5;
      if (k["ArrowLeft"] || k["q"] || k["a"]) ang = MAX_ANGULAR;
      if (k["ArrowRight"] || k["d"]) ang = -MAX_ANGULAR;
      const moving = lin !== 0 || ang !== 0;
      if (moving) {
        velRef.current = { linear: lin, angular: ang };
        setVel({ linear: lin, angular: ang });
        sendCmd(lin, ang);
      } else if (keyboardMovingRef.current) {
        // Transition mouvement → arrêt : on publie explicitement un zéro
        velRef.current = { linear: 0, angular: 0 };
        setVel({ linear: 0, angular: 0 });
        sendCmd(0, 0);
      }
      keyboardMovingRef.current = moving;
    }, 100);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      window.removeEventListener("blur", releaseAll);
      document.removeEventListener("visibilitychange", onVisibility);
      clearInterval(tick);
      if (keyboardMovingRef.current) { sendCmd(0, 0); keyboardMovingRef.current = false; }
    };
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
      window.removeEventListener("pointercancel", onEnd);
    };
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onEnd);
    // pointercancel (geste interrompu, perte du pointeur) doit aussi arrêter le robot
    window.addEventListener("pointercancel", onEnd);
    move(e.clientX, e.clientY);
  }, [sendCmd]);

  // ── Arrêt d'urgence ─────────────────────────────────────────────────────
  // /e_stop (std_msgs/Bool) : verrou twist_mux (bloque Nav2 et teleop), arrêt
  // immédiat des roues dans control_4ws, mise en PAUSE de la mission.
  // Interrupteur à accrochage : actif jusqu'au réarmement explicite.
  const E_STOP_TOPIC = "/e_stop";
  const [eStop, setEStop] = useState(false);

  const publishEStop = useCallback((active) => {
    publish(E_STOP_TOPIC, "std_msgs/Bool", { data: active });
  }, [publish]);

  // L'affichage suit l'état réel du topic (autre IHM, autre client…)
  useEffect(() => {
    if (!subscribe) return undefined;
    return subscribe(E_STOP_TOPIC, "std_msgs/Bool", (msg) => {
      setEStop(!!msg.data);
    });
  }, [subscribe]);

  // Tant qu'il est actif, l'état est republié à 1 Hz (robustesse : redémarrage
  // de twist_mux / control_4ws, nouveaux abonnés)
  useEffect(() => {
    if (!eStop || !connected) return undefined;
    const t = setInterval(() => publishEStop(true), 1000);
    return () => clearInterval(t);
  }, [eStop, connected, publishEStop]);

  const triggerEStop = () => {
    setEStop(true);
    publishEStop(true);
    sendCmd(0, 0);
    setVel({ linear: 0, angular: 0 });
  };

  const rearmEStop = () => {
    if (!window.confirm("Réarmer l'arrêt d'urgence ? Le robot pourra de nouveau bouger.")) return;
    setEStop(false);
    publishEStop(false);
  };

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
      {eStop ? (
        <button className="emergency-stop active" onClick={rearmEStop} disabled={!connected}
                title="Arrêt d'urgence actif — cliquer pour réarmer">
          ⚠ ARRÊT D'URGENCE ACTIF — RÉARMER
        </button>
      ) : (
        <button className="emergency-stop" onClick={triggerEStop} disabled={!connected}
                title={connected ? "Bloque Nav2 et la téléopération jusqu'au réarmement" : "rosbridge non connecté"}>
          ■ EMERGENCY STOP
        </button>
      )}
    </div>
  );
}
