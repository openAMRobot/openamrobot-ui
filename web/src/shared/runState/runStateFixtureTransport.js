// Read-only HTTP transport for the development run-state fixture served by
// Flask (ros2/.../run_state_fixture.py). Same fetch + API_BASE convention as
// features/recordings/recordingsApi.js — no new transport. GET only.

const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

export const FIXTURE_PREFIX = "/api/dev/run-state-fixture";
const REQUEST_TIMEOUT_MS = 2500;

async function getJson(path) {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);
  try {
    const response = await fetch(`${API_BASE}${path}`, {
      method: "GET",
      cache: "no-store",
      signal: controller.signal,
    });
    const body = await response.json().catch(() => null);
    return { status: response.status, body };
  } finally {
    clearTimeout(timeoutId);
  }
}

export function createRunStateFixtureTransport() {
  return {
    fetchSnapshot: () => getJson(`${FIXTURE_PREFIX}/snapshot`),
    fetchEvents: ({ epoch, afterSeq }) =>
      getJson(
        `${FIXTURE_PREFIX}/events?epoch=${encodeURIComponent(epoch)}&after_seq=${encodeURIComponent(
          afterSeq,
        )}`,
      ),
  };
}
