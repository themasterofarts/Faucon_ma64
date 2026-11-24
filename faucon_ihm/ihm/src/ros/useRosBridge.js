import { useState, useEffect, useRef, useCallback } from "react";

// ─── ROSBRIDGE CONNECTION HOOK ────────────────────────────────────────────────
export function useRosBridge(url) {
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
