const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

const readJsonResponse = async (response) => {
  const data = await response.json().catch(() => ({}));

  if (!response.ok) {
    throw new Error(
      data.message || data.description || "Backend request failed",
    );
  }

  return data;
};

export const generatePlanFromVoice = async ({ transcript, locations }) =>
  readJsonResponse(
    await fetch(`${API_BASE}/api/voice-plan`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ transcript, locations }),
    }),
  );
