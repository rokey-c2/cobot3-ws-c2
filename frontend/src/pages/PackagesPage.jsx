import { useCallback, useEffect, useMemo, useState } from "react";
import { useNavigate, useParams } from "react-router-dom";

import { api } from "../api";
import StatusPill from "../components/StatusPill";
import { zoneLabel } from "../utils/missionLabels";

const plannedZones = [
  "INPUT_ZONE",
  "AMR_IN",
  "P3020_IN",
  "MAIN_CONVEYOR",
  "SORTER",
  "REGION",
  "COMPLETE",
];

export default function PackagesPage() {
  const { packageCode } = useParams();
  const navigate = useNavigate();
  const [packages, setPackages] = useState([]);
  const [detail, setDetail] = useState(null);
  const [query, setQuery] = useState("");
  const [region, setRegion] = useState("ALL");
  const [status, setStatus] = useState("ALL");
  const [error, setError] = useState("");

  const loadPackages = useCallback(async () => {
    try {
      const data = await api.getPackages();
      setPackages(data.packages || []);
      setError("");
    } catch (requestError) {
      setError(requestError.message);
    }
  }, []);

  useEffect(() => {
    loadPackages();
    const timer = window.setInterval(loadPackages, 5000);
    return () => window.clearInterval(timer);
  }, [loadPackages]);

  useEffect(() => {
    if (!packageCode) {
      setDetail(null);
      return;
    }

    let active = true;
    api
      .getPackage(packageCode)
      .then((data) => {
        if (active) setDetail(data);
      })
      .catch((requestError) => {
        if (active) setError(requestError.message);
      });

    return () => {
      active = false;
    };
  }, [packageCode]);

  const regions = useMemo(
    () => ["ALL", ...new Set(packages.map((item) => item.region).filter(Boolean))],
    [packages],
  );
  const statuses = useMemo(
    () => ["ALL", ...new Set(packages.map((item) => item.status).filter(Boolean))],
    [packages],
  );

  const filtered = useMemo(() => {
    const normalized = query.trim().toLowerCase();
    return packages.filter((item) => {
      const matchesQuery =
        !normalized ||
        item.package_code?.toLowerCase().includes(normalized) ||
        item.current_zone?.toLowerCase().includes(normalized);
      const matchesRegion = region === "ALL" || item.region === region;
      const matchesStatus = status === "ALL" || item.status === status;
      return matchesQuery && matchesRegion && matchesStatus;
    });
  }, [packages, query, region, status]);

  return (
    <div className="stack-lg">
      <section className="page-heading-row">
        <div>
          <p className="eyebrow">Tracking</p>
          <h2>Packages</h2>
          <p className="muted">
            ROS2 장비 이벤트로 자동 갱신되는 Package 현재 Zone과 이력을 조회합니다.
          </p>
        </div>
        <div className="metric-box">
          <span>Total Packages</span>
          <strong>{packages.length}</strong>
        </div>
      </section>

      {error && <div className="alert">Backend: {error}</div>}

      <section className="panel">
        <div className="filter-bar">
          <label>
            <span>Search</span>
            <input
              value={query}
              onChange={(event) => setQuery(event.target.value)}
              placeholder="PKG-001 or zone"
            />
          </label>
          <label>
            <span>Region</span>
            <select value={region} onChange={(event) => setRegion(event.target.value)}>
              {regions.map((item) => (
                <option key={item} value={item}>
                  {item}
                </option>
              ))}
            </select>
          </label>
          <label>
            <span>Status</span>
            <select value={status} onChange={(event) => setStatus(event.target.value)}>
              {statuses.map((item) => (
                <option key={item} value={item}>
                  {item}
                </option>
              ))}
            </select>
          </label>
          <button className="button button-ghost" onClick={loadPackages}>
            REFRESH
          </button>
        </div>

        <div className="table-wrap">
          <table className="data-table">
            <thead>
              <tr>
                <th>Package</th>
                <th>Region</th>
                <th>Current Zone</th>
                <th>Mission</th>
                <th>Status</th>
                <th>Updated</th>
              </tr>
            </thead>
            <tbody>
              {filtered.map((item) => (
                <tr
                  key={item.package_code}
                  className={packageCode === item.package_code ? "selected" : ""}
                  onClick={() => navigate(`/packages/${encodeURIComponent(item.package_code)}`)}
                >
                  <td><strong>{item.package_code}</strong></td>
                  <td>{item.region || "-"}</td>
                  <td>{zoneLabel(item.current_zone)}</td>
                  <td>{item.mission_code || "-"}</td>
                  <td><StatusPill value={item.status} /></td>
                  <td>{formatDateTime(item.updated_at)}</td>
                </tr>
              ))}
            </tbody>
          </table>
          {filtered.length === 0 && (
            <div className="empty-state table-empty">표시할 Package가 없습니다.</div>
          )}
        </div>
      </section>

      <section className="panel package-detail-panel">
        <div className="panel-title">
          <div>
            <span>PACKAGE DETAIL</span>
            <h3>{detail?.package?.package_code || "Select a package"}</h3>
          </div>
          {detail?.package && <StatusPill value={detail.package.status} />}
        </div>

        {detail ? (
          <div className="package-detail-grid">
            <div>
              <div className="detail-stats">
                <Info label="Region" value={detail.package.region} />
                <Info label="Current Zone" value={zoneLabel(detail.package.current_zone)} />
                <Info label="Mission" value={detail.package.mission_code} />
              </div>
              <h4 className="subheading">Route Areas</h4>
              <div className="route-strip">
                {plannedZones.map((zone, index) => (
                  <div className="route-step" key={zone}>
                    <span className={matchesCurrentZone(detail.package.current_zone, zone) ? "active" : ""}>
                      {zoneLabel(zone)}
                    </span>
                    {index < plannedZones.length - 1 && <b>→</b>}
                  </div>
                ))}
              </div>
            </div>

            <div>
              <h4 className="subheading">Event History</h4>
              <div className="timeline">
                {detail.events.map((event) => (
                  <div className="timeline-row" key={event.id}>
                    <span className="timeline-dot" />
                    <div>
                      <strong>{event.event_type}</strong>
                      <p>{zoneLabel(event.zone_code)}</p>
                    </div>
                    <small>{formatDateTime(event.occurred_at)}</small>
                  </div>
                ))}
                {detail.events.length === 0 && (
                  <div className="empty-state">아직 Package Event가 없습니다.</div>
                )}
              </div>
            </div>
          </div>
        ) : (
          <div className="empty-state large-empty">
            위 목록에서 Package를 선택하면 이동 경로와 이벤트 이력을 표시합니다.
          </div>
        )}
      </section>
    </div>
  );
}

function Info({ label, value }) {
  return (
    <div className="info-box">
      <span>{label}</span>
      <strong>{value || "-"}</strong>
    </div>
  );
}

function matchesCurrentZone(currentZone, planned) {
  if (!currentZone) return false;
  const current = String(currentZone).toUpperCase();
  if (planned === "SORTER") return current.startsWith("SORTER");
  if (planned === "REGION") return current.startsWith("REGION");
  return current === planned;
}

function formatDateTime(value) {
  if (!value) return "-";
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? "-" : date.toLocaleString("ko-KR");
}
