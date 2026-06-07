const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

const historyUrl = () => `${API_BASE}/api/block-run-history`;

const readJsonResponse = async (response) => {
  const data = await response.json().catch(() => ({}));

  if (!response.ok) {
    throw new Error(
      data.message || data.description || "Backend request failed",
    );
  }

  return data;
};

export const fetchBlockRunHistory = async () => {
  const data = await readJsonResponse(await fetch(historyUrl()));
  return data.history || [];
};

export const saveBlockRunHistory = async (entry) =>
  readJsonResponse(
    await fetch(historyUrl(), {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(entry),
    }),
  );

export const clearBlockRunHistory = async () =>
  readJsonResponse(
    await fetch(historyUrl(), {
      method: "DELETE",
    }),
  );
