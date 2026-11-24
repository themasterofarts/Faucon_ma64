import { useState, useEffect, useRef } from "react";

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

export default function GPSWidget({ subscribe }) {
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
