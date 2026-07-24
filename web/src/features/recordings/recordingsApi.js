const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

const readJsonResponse = async (response) => {
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.message || data.description || "Backend request failed");
  }
  return data;
};

const postJson = async (path, body) =>
  readJsonResponse(
    await fetch(`${API_BASE}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    }),
  );

export const fetchRecordings = async () =>
  readJsonResponse(await fetch(`${API_BASE}/api/recordings`, { cache: "no-store" }));

export const fetchRecordingsStatus = async () =>
  readJsonResponse(await fetch(`${API_BASE}/api/recordings/status`, { cache: "no-store" }));

export const startRecording = ({ name, description, topics }) =>
  postJson("/api/recordings/start", { name, description, topics });

export const stopRecording = () => postJson("/api/recordings/stop");

// Triggers a browser download of the zipped bag. Not a fetch — pointing the
// browser at the URL lets it stream the (potentially large) archive straight
// to disk without buffering it in JS memory.
export const downloadRecording = (id) => {
  window.open(
    `${API_BASE}/api/recordings/${encodeURIComponent(id)}/download`,
    "_blank",
    "noopener",
  );
};

export const deleteRecording = async (id) =>
  readJsonResponse(
    await fetch(`${API_BASE}/api/recordings/${encodeURIComponent(id)}`, {
      method: "DELETE",
    }),
  );

export const startReplay = (id, rate) =>
  postJson(`/api/recordings/${encodeURIComponent(id)}/replay/start`, { rate });

export const stopReplay = () => postJson("/api/recordings/replay/stop");
export const pauseReplay = () => postJson("/api/recordings/replay/pause");
export const resumeReplay = () => postJson("/api/recordings/replay/resume");
