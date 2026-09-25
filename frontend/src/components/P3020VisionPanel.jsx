import { useEffect, useMemo, useState } from "react";

import "./LiveMonitorPanel.css";

const RETRY_DELAY_MS = 3000;

function defaultStreamUrl(port) {
  if (typeof window === "undefined") return `http://localhost:${port}/stream.mjpg`;
  return `http://${window.location.hostname}:${port}/stream.mjpg`;
}

export default function P3020VisionPanel() {
  const slides = useMemo(
    () => [
      {
        id: "top-view",
        tab: "TOP VIEW",
        eyebrow: "WAREHOUSE LIVE",
        title: "Warehouse Top View",
        description: "Isaac Sim 전체 물류 공정을 고정 탑뷰 카메라로 실시간 확인합니다.",
        streamUrl:
          String(import.meta.env.VITE_TOP_VIEW_STREAM_URL || "").trim() ||
          defaultStreamUrl(8092),
        kind: "top",
        waitingText: "Top View MJPEG 포트 8092 연결을 기다리는 중입니다.",
      },
      {
        id: "p3020-in",
        tab: "P3020 IN",
        eyebrow: "P3020 IN VISION",
        title: "Inbound P3020 Laser Detection",
        description: "입고 P3020 카메라에서 검출된 박스의 중심과 경계를 레이저 HUD로 표시합니다.",
        streamUrl:
          String(
            import.meta.env.VITE_P3020_IN_CAMERA_STREAM_URL ||
              import.meta.env.VITE_P3020_CAMERA_STREAM_URL ||
              "",
          ).trim() || defaultStreamUrl(8091),
        kind: "vision",
        waitingText: "P3020 IN Vision MJPEG 포트 8091 연결을 기다리는 중입니다.",
      },
      {
        id: "p3020-out",
        tab: "P3020 OUT",
        eyebrow: "P3020 OUT VISION",
        title: "Outbound P3020 Laser Detection",
        description: "출고 P3020 카메라에서 배송지 오류 박스를 탐지하는 레이저 HUD 영상입니다.",
        streamUrl:
          String(import.meta.env.VITE_P3020_OUT_CAMERA_STREAM_URL || "").trim() ||
          defaultStreamUrl(8093),
        kind: "vision",
        waitingText: "P3020 OUT 영상은 sorter가 D 박스를 검출하면 시작됩니다.",
      },
    ],
    [],
  );

  const [activeIndex, setActiveIndex] = useState(0);
  const [streamStates, setStreamStates] = useState(() =>
    Object.fromEntries(slides.map((slide) => [slide.id, "CONNECTING"])),
  );
  const [retryTokens, setRetryTokens] = useState(() =>
    Object.fromEntries(slides.map((slide) => [slide.id, 0])),
  );

  const activeSlide = slides[activeIndex];
  const streamState = streamStates[activeSlide.id] || "CONNECTING";
  const retryToken = retryTokens[activeSlide.id] || 0;

  useEffect(() => {
    if (streamState !== "OFFLINE") return undefined;

    const timer = window.setTimeout(() => {
      setStreamStates((current) => ({
        ...current,
        [activeSlide.id]: "CONNECTING",
      }));
      setRetryTokens((current) => ({
        ...current,
        [activeSlide.id]: (current[activeSlide.id] || 0) + 1,
      }));
    }, RETRY_DELAY_MS);

    return () => window.clearTimeout(timer);
  }, [activeSlide.id, streamState]);

  const imageUrl = `${activeSlide.streamUrl}${
    activeSlide.streamUrl.includes("?") ? "&" : "?"
  }retry=${retryToken}`;

  function moveSlide(direction) {
    setActiveIndex((current) => (current + direction + slides.length) % slides.length);
  }

  function setActiveState(value) {
    setStreamStates((current) => ({
      ...current,
      [activeSlide.id]: value,
    }));
  }

  return (
    <section className={`panel live-monitor-panel ${activeSlide.kind}`}>
      <div className="live-monitor-header">
        <div className="live-monitor-copy">
          <span className="eyebrow">{activeSlide.eyebrow}</span>
          <h3>{activeSlide.title}</h3>
          <p>{activeSlide.description}</p>
        </div>

        <div className="live-monitor-header-actions">
          <div className="live-monitor-tabs" role="tablist" aria-label="Live camera selection">
            {slides.map((slide, index) => (
              <button
                key={slide.id}
                type="button"
                role="tab"
                aria-selected={index === activeIndex}
                className={index === activeIndex ? "active" : ""}
                onClick={() => setActiveIndex(index)}
              >
                {slide.tab}
              </button>
            ))}
          </div>
          <span className={`live-monitor-badge ${streamState.toLowerCase()}`}>
            <i /> {streamState}
          </span>
        </div>
      </div>

      <div className="live-monitor-frame">
        <img
          key={`${activeSlide.id}-${retryToken}`}
          src={imageUrl}
          alt={`${activeSlide.title} live stream`}
          onLoad={() => setActiveState("LIVE")}
          onError={() => setActiveState("OFFLINE")}
        />

        {streamState !== "LIVE" && (
          <div className="live-monitor-message">
            <div className="live-monitor-scanner" />
            <strong>{streamState === "OFFLINE" ? "STREAM OFFLINE" : "CONNECTING"}</strong>
            <span>{activeSlide.waitingText}</span>
          </div>
        )}

        <button
          className="live-monitor-arrow previous"
          type="button"
          aria-label="Previous live camera"
          onClick={() => moveSlide(-1)}
        >
          ‹
        </button>
        <button
          className="live-monitor-arrow next"
          type="button"
          aria-label="Next live camera"
          onClick={() => moveSlide(1)}
        >
          ›
        </button>

        <div className="live-monitor-frame-label">
          <strong>{activeSlide.tab}</strong>
          <span>
            {String(activeIndex + 1).padStart(2, "0")} / {String(slides.length).padStart(2, "0")}
          </span>
        </div>

        {activeSlide.kind === "vision" && (
          <>
            <div className="live-monitor-corner top-left" />
            <div className="live-monitor-corner top-right" />
            <div className="live-monitor-corner bottom-left" />
            <div className="live-monitor-corner bottom-right" />
          </>
        )}
      </div>

      <div className="live-monitor-footer">
        {activeSlide.kind === "vision" ? (
          <>
            <span><i className="live-monitor-dot detected" /> TARGET LOCKED</span>
            <span><i className="live-monitor-dot scanning" /> AUTO SCANNING</span>
          </>
        ) : (
          <>
            <span><i className="live-monitor-dot live" /> ISAAC SIM LIVE</span>
            <span>1280 × 720 · ~10 FPS</span>
          </>
        )}
        <code>{activeSlide.streamUrl}</code>
      </div>
    </section>
  );
}
