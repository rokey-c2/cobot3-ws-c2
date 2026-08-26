const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || "";

async function request(path, options = {}) {
  const response = await fetch(`${API_BASE_URL}${path}`, {
    headers: {
      "Content-Type": "application/json",
      ...options.headers,
    },
    ...options,
  });

  let data = null;
  try {
    data = await response.json();
  } catch {
    data = null;
  }

  if (!response.ok) {
    const message = data?.detail || `Request failed (${response.status})`;
    throw new Error(message);
  }

  return data;
}

export const api = {
  health: () => request("/health"),
  getEquipment: () => request("/api/equipment"),
  getCurrentMission: () => request("/api/missions/current"),
  getPackages: () => request("/api/packages"),
  getPackage: (packageCode) =>
    request(`/api/packages/${encodeURIComponent(packageCode)}`),
  getEvents: () => request("/api/events"),

  startSystem: () => request("/api/system/start", { method: "POST" }),
  stopSystem: () => request("/api/system/stop", { method: "POST" }),
  startEquipment: (equipmentCode) =>
    request(`/api/equipment/${encodeURIComponent(equipmentCode)}/start`, {
      method: "POST",
    }),
  stopEquipment: (equipmentCode) =>
    request(`/api/equipment/${encodeURIComponent(equipmentCode)}/stop`, {
      method: "POST",
    }),
  navigateAmr: (equipmentCode, target) =>
    request(`/api/amr/${encodeURIComponent(equipmentCode)}/navigate`, {
      method: "POST",
      body: JSON.stringify(target),
    }),
  liftAmr: (equipmentCode, action) =>
    request(`/api/amr/${encodeURIComponent(equipmentCode)}/lift`, {
      method: "POST",
      body: JSON.stringify({ action }),
    }),
};
