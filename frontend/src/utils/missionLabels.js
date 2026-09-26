export const defaultMissionStages = [
  "INPUT_ZONE", "AMR_PICKUP", "AMR_NAVIGATION", "AMR_ARRIVAL", "MANIPULATOR_PICK",
  "MANIPULATOR_PLACE", "MAIN_CONVEYOR", "SORTER", "REGION", "COMPLETE",
];

const stages = {
  INPUT_ZONE: ["Inbound", "Package registered for sorting"],
  AMR_PICKUP: ["AMR Pickup", "AMR docks with and lifts the cargo pod"],
  AMR_NAVIGATION: ["AMR Transport", "AMR carries the cargo pod to P3020 IN"],
  AMR_ARRIVAL: ["AMR Arrival at IN", "AMR arrival and docking at P3020 IN confirmed"],
  MANIPULATOR_PICK: ["IN Pick", "Complete when the attached box is lifted at least 5 cm"],
  MANIPULATOR_PLACE: ["IN Place", "P3020 IN places the box on the conveyor"],
  MAIN_CONVEYOR: ["Conveyor", "Complete when the box enters Sorter A detection range (45 cm)"],
  SORTER: ["Sorter", "Complete after the destination sorter is passed; D must pass A, B and C"],
  REGION: ["Destination", "D: released box verified in the OUT bin. A/B/C: destination sorter exit (70 cm)"],
  EXCEPTION: ["Exception Lane", "Unclassified box travels toward P3020 OUT"],
  COMPLETE: ["Complete", "All milestones verified; D also requires OUT vertical retreat and home return"],
  AMR_IN: ["AMR Transport", "AMR cargo pickup and transport"],
  P3020_IN: ["IN Pick & Place", "P3020 IN picks and places boxes"],
};

export function progressStageCode(code) {
  const key = String(code || "").toUpperCase();
  if (/^SORTER_[A-D]$/.test(key)) return "SORTER";
  if (/^REGION_[A-D]$/.test(key) || key === "EXCEPTION") return "REGION";
  return key;
}

export function missionStageInfo(code) {
  const key = progressStageCode(code);
  const [label, description] = stages[key] || [key.replaceAll("_", " ") || "Waiting for mission", ""];
  return { label, description };
}

export function groupMissionStages(items) {
  const groups = new Map();
  for (const item of items) {
    const code = progressStageCode(item.stage_code);
    if (!groups.has(code)) groups.set(code, []);
    groups.get(code).push(item);
  }
  return [...groups].map(([stage_code, members]) => {
    const statuses = members.map((item) => item.status);
    // A completed upstream sorter does not complete the whole sorting phase.
    const status = statuses.includes("FAILED") ? "FAILED"
      : statuses.every((value) => value === "COMPLETED") ? "COMPLETED"
      : statuses.includes("RUNNING") || statuses.includes("COMPLETED") ? "RUNNING"
      : "WAITING";
    return { stage_code, status };
  });
}

export function zoneLabel(code) {
  const key = String(code || "").toUpperCase();
  const labels = {
    INPUT_ZONE: "Inbound Area", AMR_IN: "AMR Cargo Pod", P3020_IN: "P3020 IN Area",
    MAIN_CONVEYOR: "Main Conveyor", SORTER: "Sorter Area", REGION: "Destination Area",
    EXCEPTION: "Exception Lane", COMPLETE: "Complete",
  };
  if (/^SORTER_[ABC]$/.test(key)) return `Sorter ${key.at(-1)} Area`;
  if (/^REGION_[ABC]$/.test(key)) return `Destination ${key.at(-1)} Area`;
  return labels[key] || key || "Not assigned";
}
