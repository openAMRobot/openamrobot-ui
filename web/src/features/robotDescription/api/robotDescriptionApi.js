// Same dev/prod API base pattern as web/src/features/blocks/backendPrograms.js —
// the Flask backend runs on :5050, the CRA dev server proxies nothing, so in
// dev (port 3000) we hit Flask directly; in prod Flask serves the SPA itself.
const API_BASE =
  window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

export async function fetchRobotDescriptionManifest() {
  const res = await fetch(`${API_BASE}/api/robot-description/manifest`);
  if (!res.ok) {
    console.error(`Manifest request failed (${res.status})`);
    throw new Error("This robot's 3D model isn't available right now.");
  }
  return res.json();
}

export function resolveRobotDescriptionUrl(path) {
  return `${API_BASE}${path}`;
}
