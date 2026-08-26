export default function StatusPill({ value = "UNKNOWN" }) {
  const normalized = String(value || "UNKNOWN").toUpperCase();
  const positive = ["RUNNING", "SUCCESS", "ONLINE", "COMPLETE", "UP"].includes(
    normalized,
  );
  const negative = ["FAILED", "ERROR", "OFFLINE", "STOPPED"].includes(normalized);
  const pending = ["PENDING", "WAITING", "READY", "PAUSED", "DOWN"].includes(
    normalized,
  );

  let tone = "neutral";
  if (positive) tone = "positive";
  else if (negative) tone = "negative";
  else if (pending) tone = "pending";

  return <span className={`status-pill ${tone}`}>{normalized}</span>;
}
