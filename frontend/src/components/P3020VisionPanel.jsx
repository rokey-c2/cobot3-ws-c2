import { useEffect, useMemo, useState } from "react";

const RETRY_DELAY_MS = 3000;

function defaultStreamUrl() {
  if (typeof window === "undefined") return "http://localhost:8091/stream.mjpg";
  return `http://${window.location.hostname}:8091/stream.mjpg`;
}

export default function P3020VisionPanel() {
  const configuredUrl = String(
    import.meta.env.VITE_P3020_CAMERA_STREAM_URL || "",
  ).trim();
  const streamUrl = useMemo(
    () => configuredUrl || defaultStreamUrl(),
    [configuredUrl],
  );
  const [streamState, setStreamState] = useState("CONNECTING");
  const [retryToken, setRetryToken] = useState(0);

  useEffect(() => {
    if (streamState !== "OFFLINE") return undefined;
    const timer = window.setTimeout(() => {
      setStreamState("CONNECTING");
      setRetryToken((value) => value + 1);
    }, RETRY_DELAY_MS);
    return () => window.clearTimeout(timer);
  }, [streamState]);

  const imageUrl = `${streamUrl}${streamUrl.includes("?") ? "&" : "?"}retry=${retryToken}`;

  return (
    <section className="panel p3020-vision-panel">
      <div className="vision-panel-header">
        <div>
          <span className="eyebrow">P3020 VISION</span>
          <h3>Box Detection Live Feed</h3>
          <p>검출된 박스의 중심과 경계를 녹색 레이저 HUD로 표시합니다.</p>
        </div>
        <span className={`vision-live-badge ${streamState.toLowerCase()}`}>
          <i /> {streamState}
        </span>
      </div>

      <div className="vision-stream-frame">
        <img
          key={retryToken}
          src={imageUrl}
          alt="P3020 box detection live stream"
          onLoad={() => setStreamState("LIVE")}
          onError={() => setStreamState("OFFLINE")}
        />
        {streamState !== "LIVE" && (
          <div className="vision-stream-message">
            <div className="vision-scanner-icon" />
            <strong>{streamState === "OFFLINE" ? "STREAM OFFLINE" : "CONNECTING"}</strong>
            <span>Vision node의 MJPEG 포트 8091 연결을 기다리는 중입니다.</span>
          </div>
        )}
        <div className="vision-corner top-left" />
        <div className="vision-corner top-right" />
        <div className="vision-corner bottom-left" />
        <div className="vision-corner bottom-right" />
      </div>

      <div className="vision-panel-footer">
        <span><i className="legend-dot detected" /> TARGET LOCKED</span>
        <span><i className="legend-dot scanning" /> AUTO SCANNING</span>
        <code>{streamUrl}</code>
      </div>
    </section>
  );
}
