import { NavLink, Outlet } from "react-router-dom";

const navItems = [
  { to: "/", label: "Dashboard", icon: "D", end: true },
  { to: "/packages", label: "Packages", icon: "P" },
  { to: "/amr", label: "AMR Control", icon: "A" },
];

export default function Layout() {
  return (
    <div className="app-shell">
      <aside className="sidebar">
        <div className="brand">
          <div className="brand-mark">CT</div>
          <div>
            <strong>CONTROL TOWER</strong>
            <span>Logistics Automation</span>
          </div>
        </div>

        <nav className="nav-list" aria-label="Main navigation">
          {navItems.map((item) => (
            <NavLink
              key={item.to}
              to={item.to}
              end={item.end}
              className={({ isActive }) =>
                `nav-item ${isActive ? "active" : ""}`
              }
            >
              <span className="nav-icon">{item.icon}</span>
              {item.label}
            </NavLink>
          ))}
        </nav>

        <div className="sidebar-status">
          <p className="section-label">SYSTEM</p>
          <StatusLine label="Backend" state="live" />
          <StatusLine label="MQTT" state="live" />
          <StatusLine label="AMR_IN" state="live" />
          <StatusLine label="P3020 / Conveyor" state="waiting" />
          <StatusLine label="Sorter" state="waiting" />
        </div>

        <div className="sidebar-note">
          <span>Control scope</span>
          <strong>AMR E2E connected</strong>
          <small>Other equipment adapters are waiting for integration.</small>
        </div>
      </aside>

      <main className="main-area">
        <header className="topbar">
          <div>
            <p className="eyebrow">AMR · Robot · Conveyor orchestration</p>
            <h1>Parcel Sorting Control Tower</h1>
          </div>
          <div className="topbar-state">
            <span className="dot live" /> Backend connected
          </div>
        </header>
        <div className="page-content">
          <Outlet />
        </div>
      </main>
    </div>
  );
}

function StatusLine({ label, state }) {
  return (
    <div className="status-line">
      <span className={`dot ${state}`} />
      <span>{label}</span>
      <small>{state === "live" ? "LIVE" : "WAIT"}</small>
    </div>
  );
}
