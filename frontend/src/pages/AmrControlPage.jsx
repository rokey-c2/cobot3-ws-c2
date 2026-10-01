import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { api } from "../api";
import StatusPill from "../components/StatusPill";
import ManualJoystick from "../components/ManualJoystick";

const EQUIPMENT_CODE = "AMR_IN";
const MANUAL_HEARTBEAT_MS = 180;
const CAMERA_RETRY_DELAY_MS = 3000;

function defaultCameraStreamUrl() {
  if (typeof window === "undefined") return "http://localhost:8090/stream.mjpg";
  return `http://${window.location.hostname}:8090/stream.mjpg`;
}

export default function AmrControlPage() {
  const [equipment, setEquipment] = useState([]);
  const [mission, setMission] = useState(null);
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
      const [equipmentData, missionData] = await Promise.all([
        api.getEquipment(),
        api.getCurrentMission(),
      ]);
      setEquipment(equipmentData.equipment || []);
      setMission(missionData.mission || null);
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
  const missionRunning = Boolean(mission && !["COMPLETE", "FAILED", "CANCELED"].includes(mission.status));
  const liftLocked = false;
  const poseSyncStatus = amr?.sync_status || "OFFLINE";
  const poseSynced = poseSyncStatus === "SYNCED";

  async function waitForCommand(commandId, label) {
    for (let attempt = 0; attempt < 30; attempt += 1) {
      await new Promise((resolve) => window.setTimeout(resolve, 500));
      const result = await api.getMissionCommand(commandId);
      const next = result?.command;
      setCommand({ label, status: next?.status || "PENDING", id: commandId });
      if (["SUCCESS", "FAILED", "CANCELED"].includes(next?.status)) {
        if (next.status === "FAILED") {
          setError(next.error_message || `${label} command failed.`);
        }
        return;
      }
    }
    setError(`Waiting for the ${label} command result. Please check again shortly.`);
  }

  async function runAction(label, action) {
    stopManual();
    setBusy(true);
    setError("");
    try {
      const result = await action();
      setCommand({ label, status: result?.command?.status || result?.status || "PENDING", id: result?.command?.id });
      if (result?.command?.id) {
        await waitForCommand(result.command.id, label);
      }
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
      setError("Please enter valid numbers for X, Y, and Yaw.");
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

  function changeManualDirection(nextDirection) {
    if (nextDirection === "STOP") stopManual();
    else if (nextDirection !== manualDirectionRef.current) startManual(nextDirection);
  }

  const direction = yawDegrees(amr?.yaw);
  const manualDisabled = busy || amr?.status !== "RUNNING";
  const navigationDisabled = busy || amr?.status !== "RUNNING" || !poseSynced;
  const arrivalDisabled = (
    busy || manualDirection !== "STOP" || amr?.status !== "RUNNING" ||
    mission?.status !== "RUNNING" || mission?.current_stage !== "AMR_NAVIGATION"
  );

  return (
    <div className="stack-lg">
      <section className="page-heading-row">
        <div>
          <p className="eyebrow">Robot Operation</p>
          <h2>AMR Control</h2>
          <p className="muted">
            Control AMR_IN with Start/Stop, Navigation, Lift, and Manual Jog commands.
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
                  Waiting for the IW Hub front stereo camera stream via ROS2 Image and MJPEG on port 8090.
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
            <small>{manualDirection === "STOP" ? "DRAG TO DRIVE" : manualDirection}</small>
          </div>
          <ManualJoystick
            disabled={manualDisabled}
            onDirectionChange={changeManualDirection}
          />
          <p className="panel-help">
            Drag the L stick up or down to drive, or left or right to turn. Return to the center to stop. Releasing the button or switching away from this window sends STOP. The adapter also stops the AMR automatically when its deadman timeout expires.
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
          {amr && !poseSynced && (
            <div className="alert pose-sync-notice" role="status">
              <div className="pose-sync-notice-heading">
                <strong>Pose Sync</strong>
                <span className="pose-sync-notice-status">{poseSyncStatus}</span>
              </div>
              <p>Auto Navigation is unavailable until pose synchronization is restored.</p>
            </div>
          )}
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
          <p className="panel-help">You can operate the lift while driving manually.</p>
        </div>

        <div className="panel action-panel">
          <div>
            <span className="section-label">MANUAL HANDOFF</span>
            <h3>P3020 Arrival Confirmation</h3>
            <small>
              {mission?.status === "RUNNING"
                ? `Current stage: ${mission.current_stage || "-"}`
                : "A running mission is required."}
            </small>
          </div>
          <button
            className="button button-primary"
            disabled={arrivalDisabled}
            onClick={() => runAction("P3020 ARRIVAL", api.confirmP3020Arrival)}
          >
            CONFIRM P3020 ARRIVAL
          </button>
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
    : date.toLocaleTimeString("en-US", { hour: "2-digit", minute: "2-digit", second: "2-digit" });
}

function formatDateTime(value) {
  if (!value) return "-";
  const date = new Date(value);
  return Number.isNaN(date.getTime())
    ? "-"
    : date.toLocaleString("en-US");
}
