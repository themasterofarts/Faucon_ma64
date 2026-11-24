import { useState } from "react";
import { useRosBridge } from "./ros/useRosBridge";
import { css } from "./styles";
import { ROBOT_HOST } from "./config";
import GPSWidget from "./widgets/GPSWidget";
import IMUWidget from "./widgets/IMUWidget";
import CameraWidget from "./widgets/CameraWidget";
import JoystickWidget from "./widgets/JoystickWidget";
import MissionWidget from "./widgets/MissionWidget";
import ConfigModal from "./widgets/ConfigModal";

// ─── ROOT APP ─────────────────────────────────────────────────────────────────
export default function App() {
  const [showConfig, setShowConfig] = useState(false);
  const [config, setConfig] = useState({
    rosBridgeUrl: `ws://${ROBOT_HOST}:9090`,
    videoServer: "",
    gpsTopic: "/gnss/fix",
    imuTopic: "/imu/data",
    cmdVelTopic: "/cmd_vel_teleop",
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
              <span className="panel-badge">NavSatFix · /gnss/fix</span>
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
              <span className="panel-badge">Twist · /cmd_vel_teleop</span>
            </div>
            <div className="panel-body">
              <JoystickWidget publish={publish} subscribe={subscribe} connected={connected} />
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
