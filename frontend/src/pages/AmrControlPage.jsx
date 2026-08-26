import { useCallback, useEffect, useMemo, useState } from "react";

import { api } from "../api";
import StatusPill from "../components/StatusPill";

const EQUIPMENT_CODE = "AMR_IN";
const CAMERA_STREAM_URL = import.meta.env.VITE_AMR_CAMERA_STREAM_URL || "";

export default function AmrControlPage() {
  const [equipment, setEquipment] = useState([]);
  const [target, setTarget] = useState({ x: "1.30104", y: "-0.06065", yaw: "0.0" });
  const [command, setCommand] = useState(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);

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

  const amr = useMemo(
    () => equipment.find((item) => item.code === EQUIPMENT_CODE),
    [equipment],
  );

  async function runAction(label, action) {
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

  const direction = yawDegrees(amr?.yaw);

  return (
    <div className="stack-lg">
      <section className="page-heading-row">
        <div>
          <p className="eyebrow">Robot Operation</p>
          <h2>AMR Control</h2>
          <p className="muted">
            AMR_IN의 실제 Start/Stop, Navigate, Lift 명령을 실행합니다. Manual Jog는 Adapter 연결 전까지 잠금 상태입니다.
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

      <section className="amr-control-grid">
        <div className="panel camera-panel">
          <div className="panel-title">
            <div>
              <span>VISION</span>
              <h3>Live Camera</h3>
            </div>
            <small>{CAMERA_STREAM_URL ? "STREAM" : "NOT CONFIGURED"}</small>
          </div>
          <div className="camera-frame">
            {CAMERA_STREAM_URL ? (
              <img src={CAMERA_STREAM_URL} alt="AMR live camera" />
            ) : (
              <div className="camera-placeholder">
                <div className="camera-reticle" />
                <strong>AMR CAMERA</strong>
                <span>Set VITE_AMR_CAMERA_STREAM_URL after MJPEG server is connected.</span>
              </div>
            )}
          </div>
          <div className="camera-footer">
            <span><i className={`dot ${CAMERA_STREAM_URL ? "live" : "waiting"}`} /> {CAMERA_STREAM_URL ? "LIVE" : "STREAM WAITING"}</span>
            <small>ROS2 Camera → MJPEG → Browser</small>
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
            <Info label="Mode" value={amr?.mode || "-"} />
            <Info label="Lift" value={amr?.lift_state || "-"} />
            <Info label="Last Seen" value={formatTime(amr?.last_seen_at)} />
          </div>
        </div>

        <div className="panel manual-panel disabled-panel">
          <div className="panel-title">
            <div>
              <span>MANUAL</span>
              <h3>Jog Control</h3>
            </div>
            <small>ADAPTER REQUIRED</small>
          </div>
          <div className="dpad" aria-label="Manual control preview">
            <button disabled className="dpad-up">↑</button>
            <button disabled className="dpad-left">←</button>
            <button disabled className="dpad-stop">STOP</button>
            <button disabled className="dpad-right">→</button>
            <button disabled className="dpad-down">↓</button>
          </div>
          <p className="panel-help">
            현재 /cmd_vel은 STOP 안전 제어 용도로 사용합니다. 수동 주행 API/MQTT 규격이 확정된 후 활성화합니다.
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
          <button className="button button-primary button-wide" disabled={busy} onClick={navigate}>
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
