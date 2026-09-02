import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { api } from "../api";
import StatusPill from "../components/StatusPill";

const EQUIPMENT_CODE = "AMR_IN";
const MANUAL_HEARTBEAT_MS = 180;
const CAMERA_RETRY_DELAY_MS = 3000;

function defaultCameraStreamUrl() {
  if (typeof window === "undefined") return "http://localhost:8090/stream.mjpg";
  return `http://${window.location.hostname}:8090/stream.mjpg`;
}

export default function AmrControlPage() {
  const [equipment, setEquipment] = useState([]);
  const [target, setTarget] = useState({ x: "1.30104", y: "-0.06065", yaw: "0.0" });
  const [command, setCommand] = useState(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const [manualDirection, setManualDirection] = useState("STOP");
  const [cameraStreamState, setCameraStreamState] = useState("CONNECTING");
  const [cameraRetryToken, setCameraRetryToken] = useState(0);

  const manualTimerRef = useRef(null);
  const manualDirectionRef = useRef("STOP");

  const cameraStreamUrl = useMemo(() => {
    const configuredUrl = String(
      import.meta.env.VITE_AMR_CAMERA_STREAM_URL || "",
    ).trim();
    return configuredUrl || defaultCameraStreamUrl();
  }, []);

  const cameraImageUrl = `${cameraStreamUrl}${cameraStreamUrl.includes("?") ? "&" : "?"}retry=${cameraRetryToken}`;

  const loadEquipment = useCallback(async () => {
    try {
      const data = await api.getEquipment();
      setEquipment(data.equipment || []);
      setError("");
    } catch (requestError) {
      setError(requestError.message);
    }
  }, []);

  useEffect(() => {
    loadEquipment();
    const timer = window.setInterval(loadEquipment, 1500);
    return () => window.clearInterval(timer);
  }, [loadEquipment]);

  useEffect(() => {
    if (cameraStreamState !== "OFFLINE") return undefined;

    const timer = window.setTimeout(() => {
      setCameraStreamState("CONNECTING");
      setCameraRetryToken((value) => value + 1);
    }, CAMERA_RETRY_DELAY_MS);

    return () => window.clearTimeout(timer);
  }, [cameraStreamState]);

  useEffect(() => {
    const handleWindowBlur = () => stopManual();
    window.addEventListener("blur", handleWindowBlur);

    return () => {
      window.removeEventListener("blur", handleWindowBlur);
      clearManualTimer();

      if (manualDirectionRef.current !== "STOP") {
        api.manualAmr(EQUIPMENT_CODE, "STOP").catch(() => {});
        manualDirectionRef.current = "STOP";
      }
    };
  }, []);

  const amr = useMemo(
    () => equipment.find((item) => item.code === EQUIPMENT_CODE),
    [equipment],
  );
  const poseSyncStatus = amr?.sync_status || "OFFLINE";
  const poseSynced = poseSyncStatus === "SYNCED";

  async function runAction(label, action) {
    stopManual();
    setBusy(true);
    setError("");
    try {
      const result = await action();
      setCommand({ label, status: result?.command?.status || result?.status || "PENDING", id: result?.command?.id });
      window.setTimeout(loadEquipment, 600);
    } catch (requestError) {
      setError(requestError.message);
      setCommand({ label, status: "FAILED" });
    } finally {
      setBusy(false);
    }
  }

  function navigate() {
    const parsed = {
      x: Number(target.x),
      y: Number(target.y),
      yaw: Number(target.yaw),
    };

    if (![parsed.x, parsed.y, parsed.yaw].every(Number.isFinite)) {
      setError("X, Y, Yaw에 올바른 숫자를 입력해주세요.");
      return;
    }

    runAction("NAVIGATE", () => api.navigateAmr(EQUIPMENT_CODE, parsed));
  }

  function clearManualTimer() {
    if (manualTimerRef.current !== null) {
      window.clearInterval(manualTimerRef.current);
      manualTimerRef.current = null;
    }
  }

  function sendManual(direction) {
    api.manualAmr(EQUIPMENT_CODE, direction).catch((requestError) => {
      clearManualTimer();
      manualDirectionRef.current = "STOP";
      setManualDirection("STOP");
      setCommand({ label: `MANUAL ${direction}`, status: "FAILED" });
      setError(requestError.message);

      api.manualAmr(EQUIPMENT_CODE, "STOP").catch(() => {});
    });
  }

  function startManual(direction) {
    if (busy || amr?.status !== "RUNNING") {
      return;
    }

    clearManualTimer();
    setError("");
    manualDirectionRef.current = direction;
    setManualDirection(direction);
    setCommand({ label: `MANUAL ${direction}`, status: "RUNNING" });

    sendManual(direction);
    manualTimerRef.current = window.setInterval(
      () => sendManual(direction),
      MANUAL_HEARTBEAT_MS,
    );
  }

  function stopManual() {
    clearManualTimer();

    if (manualDirectionRef.current === "STOP") {
      return;
    }

    manualDirectionRef.current = "STOP";
    setManualDirection("STOP");
    setCommand({ label: "MANUAL STOP", status: "SUCCESS" });
    api.manualAmr(EQUIPMENT_CODE, "STOP").catch((requestError) => {
      setError(requestError.message);
    });
  }

  function manualPointerDown(event, direction) {
    event.preventDefault();
    event.currentTarget.setPointerCapture?.(event.pointerId);
    startManual(direction);
  }

  const direction = yawDegrees(amr?.yaw);
  const manualDisabled = busy || amr?.status !== "RUNNING";
  const navigationDisabled = busy || amr?.status !== "RUNNING" || !poseSynced;

  return (
    <div className="stack-lg">
      <section className="page-heading-row">
        <div>
          <p className="eyebrow">Robot Operation</p>
          <h2>AMR Control</h2>
          <p className="muted">
            AMR_IN의 실제 Start/Stop, Navigate, Lift, Manual Jog 명령을 실행합니다.
          </p>
        </div>
        <div className="amr-header-state">
          <span className="dot live" />
          <div>
            <small>AMR_IN</small>
            <strong>{amr?.status || "UNKNOWN"}</strong>
          </div>
        </div>
      </section>

      {error && <div className="alert">{error}</div>}
      {amr && !poseSynced && (
        <div className="alert">
          Pose Sync {poseSyncStatus}: Auto Navigation은 사용할 수 없습니다. Manual Jog는 START AMR 후 사용할 수 있습니다.
        </div>
      )}

      <section className="amr-control-grid">
        <div className="panel camera-panel">
          <div className="panel-title">
            <div>
              <span>VISION</span>
              <h3>AMR Front Camera</h3>
            </div>
            <small>{cameraStreamState}</small>
          </div>
          <div className="camera-frame" style={{ position: "relative" }}>
            <img
              key={cameraRetryToken}
              src={cameraImageUrl}
              alt="AMR front stereo camera live stream"
              style={{ position: "absolute", inset: 0 }}
              onLoad={() => setCameraStreamState("LIVE")}
              onError={() => setCameraStreamState("OFFLINE")}
            />
            {cameraStreamState !== "LIVE" && (
              <div
                className="camera-placeholder"
                style={{ position: "absolute", inset: 0 }}
              >
                <div className="camera-reticle" />
                <strong>
                  {cameraStreamState === "OFFLINE" ? "AMR CAMERA OFFLINE" : "AMR CAMERA CONNECTING"}
                </strong>
                <span>
                  IW Hub front stereo camera → ROS2 Image → MJPEG :8090 연결을 기다리는 중입니다.
                </span>
              </div>
            )}
          </div>
          <div className="camera-footer">
            <span>
              <i className={`dot ${cameraStreamState === "LIVE" ? "live" : "waiting"}`} />
              {cameraStreamState}
            </span>
            <small title={cameraStreamUrl}>Front Stereo Left · MJPEG :8090</small>
          </div>
        </div>

        <div className="panel status-panel">
          <div className="panel-title">
            <div>
              <span>TELEMETRY</span>
              <h3>AMR Status</h3>
            </div>
            <StatusPill value={amr?.status || "UNKNOWN"} />
          </div>

          <div className="pose-card">
            <div className="compass">
              <span className="north">N</span>
              <div className="compass-ring">
                <div className="robot-arrow" style={{ transform: `rotate(${direction}deg)` }}>↑</div>
              </div>
              <strong>{direction.toFixed(1)}°</strong>
            </div>
            <div className="pose-values">
              <PoseValue label="X" value={formatNumber(amr?.position_x)} unit="m" />
              <PoseValue label="Y" value={formatNumber(amr?.position_y)} unit="m" />
              <PoseValue label="Yaw" value={formatNumber(amr?.yaw)} unit="rad" />
            </div>
          </div>

          <div className="detail-stats three-col">
            <Info label="Pose Sync" value={poseSyncStatus} />
            <Info label="Frame" value={amr?.pose_frame || "-"} />
            <Info label="Source" value={amr?.pose_source || "-"} />
          </div>

          <div className="detail-stats three-col">
            <Info label="Mode" value={manualDirection === "STOP" ? (amr?.mode || "-") : "MANUAL"} />
            <Info label="Lift" value={amr?.lift_state || "-"} />
            <Info label="Last Seen" value={formatTime(amr?.last_seen_at)} />
          </div>
          <p className="panel-help">Pose Updated: {formatDateTime(amr?.pose_updated_at)}</p>
        </div>

        <div className="panel manual-panel">
          <div className="panel-title">
            <div>
              <span>MANUAL</span>
              <h3>Jog Control</h3>
            </div>
            <small>{manualDirection === "STOP" ? "PRESS & HOLD" : manualDirection}</small>
          </div>
          <div className="dpad" aria-label="AMR manual jog control">
            <ManualButton
              className="dpad-up"
              label="↑"
              direction="FORWARD"
              activeDirection={manualDirection}
              disabled={manualDisabled}
              onPointerDown={manualPointerDown}
              onStop={stopManual}
            />
            <ManualButton
              className="dpad-left"
              label="←"
              direction="LEFT"
              activeDirection={manualDirection}
              disabled={manualDisabled}
              onPointerDown={manualPointerDown}
              onStop={stopManual}
            />
            <button
              type="button"
              className="dpad-stop"
              onClick={stopManual}
            >
              STOP
            </button>
            <ManualButton
              className="dpad-right"
              label="→"
              direction="RIGHT"
              activeDirection={manualDirection}
              disabled={manualDisabled}
              onPointerDown={manualPointerDown}
              onStop={stopManual}
            />
            <ManualButton
              className="dpad-down"
              label="↓"
              direction="BACKWARD"
              activeDirection={manualDirection}
              disabled={manualDisabled}
              onPointerDown={manualPointerDown}
              onStop={stopManual}
            />
          </div>
          <p className="panel-help">
            화살표를 누르고 있는 동안만 주행합니다. 버튼을 놓거나 창 포커스를 잃으면 STOP을 전송하며, Adapter의 deadman timeout도 자동 정지시킵니다.
          </p>
        </div>

        <div className="panel navigation-panel">
          <div className="panel-title">
            <div>
              <span>AUTO NAVIGATION</span>
              <h3>Target Pose</h3>
            </div>
            <small>Nav2</small>
          </div>
          <div className="target-form">
            <CoordinateInput label="Target X" unit="m" value={target.x} onChange={(value) => setTarget((current) => ({ ...current, x: value }))} />
            <CoordinateInput label="Target Y" unit="m" value={target.y} onChange={(value) => setTarget((current) => ({ ...current, y: value }))} />
            <CoordinateInput label="Target Yaw" unit="rad" value={target.yaw} onChange={(value) => setTarget((current) => ({ ...current, yaw: value }))} />
          </div>
          <button className="button button-primary button-wide" disabled={navigationDisabled} onClick={navigate}>
            SEND NAVIGATION GOAL
          </button>
        </div>
      </section>

      <section className="control-actions-grid">
        <div className="panel action-panel">
          <div>
            <span className="section-label">AMR POWER</span>
            <h3>Start / Stop</h3>
          </div>
          <div className="button-row">
            <button className="button button-primary" disabled={busy || amr?.status === "RUNNING"} onClick={() => runAction("START", () => api.startEquipment(EQUIPMENT_CODE))}>
              START AMR
            </button>
            <button className="button button-danger" disabled={busy || amr?.status === "STOPPED"} onClick={() => runAction("STOP", () => api.stopEquipment(EQUIPMENT_CODE))}>
              STOP AMR
            </button>
          </div>
        </div>

        <div className="panel action-panel">
          <div>
            <span className="section-label">LIFT</span>
            <h3>Lift Control</h3>
          </div>
          <div className="button-row">
            <button className="button button-secondary" disabled={busy || amr?.status !== "RUNNING" || amr?.lift_state === "UP"} onClick={() => runAction("LIFT_UP", () => api.liftAmr(EQUIPMENT_CODE, "UP"))}>
              LIFT UP
            </button>
            <button className="button button-secondary" disabled={busy || amr?.status !== "RUNNING" || amr?.lift_state === "DOWN"} onClick={() => runAction("LIFT_DOWN", () => api.liftAmr(EQUIPMENT_CODE, "DOWN"))}>
              LIFT DOWN
            </button>
          </div>
        </div>

        <div className="panel command-panel">
          <div>
            <span className="section-label">LAST COMMAND</span>
            <h3>{command?.label || "No command sent"}</h3>
          </div>
          <StatusPill value={command?.status || "READY"} />
          <small>{command?.id ? `Command #${command.id}` : "Waiting for operator input"}</small>
        </div>
      </section>
    </div>
  );
}

function ManualButton({
  className,
  label,
  direction,
  activeDirection,
  disabled,
  onPointerDown,
  onStop,
}) {
  const active = activeDirection === direction;

  return (
    <button
      type="button"
      className={className}
      disabled={disabled}
      aria-label={direction}
      aria-pressed={active}
      style={active ? { background: "#3978f6", borderColor: "#3978f6", color: "#fff" } : undefined}
      onPointerDown={(event) => onPointerDown(event, direction)}
      onPointerUp={onStop}
      onPointerCancel={onStop}
      onLostPointerCapture={onStop}
    >
      {label}
    </button>
  );
}

function CoordinateInput({ label, unit, value, onChange }) {
  return (
    <label className="coordinate-input">
      <span>{label}</span>
      <div>
        <input value={value} inputMode="decimal" onChange={(event) => onChange(event.target.value)} />
        <small>{unit}</small>
      </div>
    </label>
  );
}

function PoseValue({ label, value, unit }) {
  return (
    <div className="pose-value">
      <span>{label}</span>
      <strong>{value}</strong>
      <small>{unit}</small>
    </div>
  );
}

function Info({ label, value }) {
  return (
    <div className="info-box">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function yawDegrees(yaw) {
  const radians = Number(yaw);
  if (!Number.isFinite(radians)) return 0;
  return (radians * 180) / Math.PI;
}

function formatNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(3) : "-";
}

function formatTime(value) {
  if (!value) return "-";
  const date = new Date(value);
  return Number.isNaN(date.getTime())
    ? "-"
    : date.toLocaleTimeString("ko-KR", { hour: "2-digit", minute: "2-digit", second: "2-digit" });
}

function formatDateTime(value) {
  if (!value) return "-";
  const date = new Date(value);
  return Number.isNaN(date.getTime())
    ? "-"
    : date.toLocaleString("ko-KR");
}
