const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

const programUrl = (name = "") =>
  `${API_BASE}/api/block-programs${name ? `/${encodeURIComponent(name)}` : ""}`;

const readJsonResponse = async (response) => {
  const data = await response.json().catch(() => ({}));

  if (!response.ok) {
    throw new Error(
      data.message || data.description || "Backend request failed",
    );
  }

  return data;
};

export const fetchBlockPrograms = async () => {
  const data = await readJsonResponse(await fetch(programUrl()));
  return data.programs || [];
};

export const fetchBlockProgram = async (name) =>
  readJsonResponse(await fetch(programUrl(name)));

export const saveBlockProgram = async ({ name, workspace, plan }) =>
  readJsonResponse(
    await fetch(programUrl(name), {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ workspace, plan }),
    }),
  );

export const deleteBlockProgram = async (name) =>
  readJsonResponse(
    await fetch(programUrl(name), {
      method: "DELETE",
    }),
  );
