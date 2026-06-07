const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

const locationUrl = (name = "") =>
  `${API_BASE}/api/block-locations${
    name ? `/${encodeURIComponent(name)}` : ""
  }`;

const readJsonResponse = async (response) => {
  const data = await response.json().catch(() => ({}));

  if (!response.ok) {
    throw new Error(
      data.message || data.description || "Backend request failed",
    );
  }

  return data;
};

export const fetchBlockLocations = async () => {
  const data = await readJsonResponse(await fetch(locationUrl()));
  return data.locations || {};
};

export const saveBlockLocation = async ({ name, x, y, yaw }) =>
  readJsonResponse(
    await fetch(locationUrl(name), {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ x, y, yaw }),
    }),
  );

export const deleteBlockLocation = async (name) =>
  readJsonResponse(
    await fetch(locationUrl(name), {
      method: "DELETE",
    }),
  );
