const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

// Plan limits come from the backend (GET /api/limits), which also enforces
// them on /api/voice-plan. The Blocks page keeps no copy of its own.
export const fetchPlanLimits = async () => {
  const response = await fetch(`${API_BASE}/api/limits`, { cache: "no-store" });
  const data = await response.json().catch(() => ({}));
  if (!response.ok || !data.limits) {
    throw new Error(data.message || "Could not load plan limits from the backend");
  }
  return data.limits;
};
