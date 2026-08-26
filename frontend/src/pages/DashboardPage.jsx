import { useCallback, useEffect, useMemo, useState } from "react";
import { Link } from "react-router-dom";

import { api } from "../api";
import StatusPill from "../components/StatusPill";

const pipeline = [
  "INPUT_ZONE",
  "AMR_IN",
  "P3020_IN",
  "MAIN_CONVEYOR",
  "SORTER",
  "REGION",
  "COMPLETE",
];

export default function DashboardPage() {
  const [equipment, setEquipment] = useState([]);
  const [mission, setMission] = useState(null);
  const [packages, setPackages] = useState([]);
  const [events, setEvents] = useState([]);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);

  const loadDashboard = useCallback(async () => {
    try {
      const [equipmentData, missionData, packagesData, eventsData] =
        await Promise.all([
          api.getEquipment(),
          api.getCurrentMission(),
          api.getPackages(),
          api.getEvents(),
        ]);

      setEquipment(equipmentData.equipment || []);
      setMission(missionData.mission || null);
      setPackages(packagesData.packages || []);
      setEvents(eventsData.events || []);
      setError("");
    } catch (requestError) {
      setError(requestError.message);
    }
  }, []);

  useEffect(() => {
    loadDashboard();
    const timer = window.setInterval(loadDashboard, 3000);
    return () => window.clearInterval(timer);
  }, [loadDashboard]);

  const amr = useMemo(
    () => equipment.find((item) => item.code === "AMR_IN"),
    [equipment],
  );

  async function changeSystem(action) {
    setBusy(true);
    try {
      if (action === "START") await api.startSystem();
      else await api.stopSystem();
      await loadDashboard();
    } catch (requestError) {
      setError(requestError.message);
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className="stack-lg">
      <section className="page-heading-row">
        <div>
          <p className="eyebrow">Overview</p>
          <h2>Dashboard</h2>
          <p className="muted">
            장비 상태, 물류 진행 단계와 최근 이벤트를 한 화면에서 확인합니다.
          </p>
        </div>
        <div className="button-row">
          <button
            className="button button-primary"
            disabled={busy}
            onClick={() => changeSystem("START")}
          >
            SYSTEM START
          </button>
          <button
            className="button button-danger"
            disabled={busy}
            onClick={() => changeSystem("STOP")}
          >
            SYSTEM STOP
          </button>
        </div>
      </section>

      {error && <div className="alert">Backend: {error}</div>}

      <section className="equipment-grid">
        {equipment.length === 0 ? (
          <EmptyCard text="장비 데이터를 기다리는 중입니다." />
        ) : (
          equipment.map((item) => (
            <EquipmentCard key={item.code} equipment={item} />
          ))
        )}
      </section>

      <section className="dashboard-grid">
        <div className="panel mission-panel">
          <PanelTitle
            eyebrow="MISSION"
            title="Mission Progress"
            meta={mission?.mission_code || "No active mission"}
          />
          <div className="pipeline">
            {pipeline.map((stage, index) => {
              const current = matchesStage(mission?.current_stage, stage);
              return (
                <div className="pipeline-item" key={stage}>
                  <div className={`stage-node ${current ? "current" : ""}`}>
                    <span>{String(index + 1).padStart(2, "0")}</span>
                    <strong>{stage.replaceAll("_", " ")}</strong>
                  </div>
                  {index < pipeline.length - 1 && (
                    <span className="pipeline-arrow">→</span>
                  )}
                </div>
              );
            })}
          </div>
          <div className="panel-footnote">
            <span>Current stage</span>
            <strong>{mission?.current_stage || "Waiting for mission events"}</strong>
          </div>
        </div>

        <Link to="/amr" className="panel amr-overview interactive-panel">
          <PanelTitle eyebrow="AMR_IN" title="Live AMR State" meta="Open control →" />
          <div className="amr-pose-visual">
            <div
              className="amr-direction"
              style={{ transform: `rotate(${yawDegrees(amr?.yaw)}deg)` }}
            >
              ↑
            </div>
            <div>
              <strong>{formatNumber(amr?.position_x)} m</strong>
              <span>X position</span>
            </div>
            <div>
              <strong>{formatNumber(amr?.position_y)} m</strong>
              <span>Y position</span>
            </div>
            <div>
              <strong>{yawDegrees(amr?.yaw).toFixed(1)}°</strong>
              <span>Yaw</span>
            </div>
          </div>
          <div className="status-strip">
            <StatusPill value={amr?.status || "UNKNOWN"} />
            <span>Mode {amr?.mode || "-"}</span>
            <span>Lift {amr?.lift_state || "-"}</span>
          </div>
        </Link>
      </section>

      <section className="dashboard-grid bottom-grid">
        <div className="panel">
          <PanelTitle eyebrow="TRACKING" title="Recent Packages" meta={`${packages.length} total`} />
          <div className="compact-list">
            {packages.slice(0, 6).map((item) => (
              <Link
                key={item.package_code}
                to={`/packages/${encodeURIComponent(item.package_code)}`}
                className="compact-row"
              >
                <strong>{item.package_code}</strong>
                <span>{item.region || "-"}</span>
                <span>{item.current_zone || "NO ZONE"}</span>
                <StatusPill value={item.status} />
              </Link>
            ))}
            {packages.length === 0 && (
              <div className="empty-state">Package 데이터가 아직 없습니다.</div>
            )}
          </div>
        </div>

        <div className="panel">
          <PanelTitle eyebrow="TIMELINE" title="Event Log" meta="Latest 100" />
          <div className="event-list">
            {events.slice(0, 7).map((event) => (
              <div className="event-row" key={event.id}>
                <span className="event-time">{formatTime(event.occurred_at)}</span>
                <div>
                  <strong>{event.package_code}</strong>
                  <p>{event.event_type}</p>
                </div>
                <span>{event.zone_code || "-"}</span>
                <StatusPill value={event.result || "EVENT"} />
              </div>
            ))}
            {events.length === 0 && (
              <div className="empty-state">이벤트 연동을 기다리는 중입니다.</div>
            )}
          </div>
        </div>
      </section>
    </div>
  );
}

function EquipmentCard({ equipment }) {
  const isAmr = equipment.type === "AMR";
  const content = (
    <>
      <div className="equipment-card-head">
        <div className="equipment-icon">{equipment.type?.slice(0, 2) || "EQ"}</div>
        <StatusPill value={equipment.status || "UNKNOWN"} />
      </div>
      <strong className="equipment-name">{equipment.code}</strong>
      <span className="equipment-type">{equipment.type}</span>
      <div className="equipment-card-foot">
        <span className={`dot ${isAmr ? "live" : "waiting"}`} />
        {isAmr ? "Actual control connected" : "Adapter not connected"}
      </div>
    </>
  );

  if (isAmr) {
    return (
      <Link to="/amr" className="equipment-card equipment-link">
        {content}
      </Link>
    );
  }

  return <div className="equipment-card">{content}</div>;
}

function EmptyCard({ text }) {
  return <div className="equipment-card empty-state">{text}</div>;
}

function PanelTitle({ eyebrow, title, meta }) {
  return (
    <div className="panel-title">
      <div>
        <span>{eyebrow}</span>
        <h3>{title}</h3>
      </div>
      <small>{meta}</small>
    </div>
  );
}

function matchesStage(currentStage, pipelineStage) {
  if (!currentStage) return false;
  const current = String(currentStage).toUpperCase();
  if (pipelineStage === "SORTER") return current.startsWith("SORTER");
  if (pipelineStage === "REGION") return current.startsWith("REGION");
  return current === pipelineStage;
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
  if (!value) return "--:--";
  const date = new Date(value);
  return Number.isNaN(date.getTime())
    ? "--:--"
    : date.toLocaleTimeString("ko-KR", { hour: "2-digit", minute: "2-digit" });
}
