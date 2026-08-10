import React, { useEffect, useState } from "react";

const API_BASE = window.location.port === "3000" ? "http://127.0.0.1:5050" : "";

/**
 * Always-mounted (in AppLayout) banner surfacing the backend's real
 * AUTH_MODE state — never inferred client-side, since hiding frontend
 * routes/buttons is not authorization; the backend (flask_app.py) is the
 * only source of truth here. Two independent things it can warn about:
 *
 *  1. A non-open AUTH_MODE was requested but isn't implemented yet, so the
 *     server fell back to open — the maintainer's config had no effect
 *     and they should know that, not discover it by assuming it worked.
 *  2. This browser reached the robot from outside the local network,
 *     which matters because AUTH_MODE=open (the only mode that currently
 *     exists) has no login standing between "on the network" and
 *     "driving the robot."
 */
const AuthModeBanner = () => {
  const [status, setStatus] = useState(null);
  const [dismissed, setDismissed] = useState(false);

  useEffect(() => {
    let cancelled = false;
    fetch(`${API_BASE}/api/auth/status`)
      .then((res) => (res.ok ? res.json() : null))
      .then((data) => {
        if (!cancelled) setStatus(data);
      })
      .catch(() => {});
    return () => {
      cancelled = true;
    };
  }, []);

  if (!status || dismissed) return null;

  const showConfigWarning = Boolean(status.warning);
  const showNetworkWarning = status.mode === "open" && status.isLocalNetwork === false;
  if (!showConfigWarning && !showNetworkWarning) return null;

  return (
    <div className="mb-3 flex items-start gap-3 rounded-xl border border-statusYellow/30 bg-statusYellow/10 px-4 py-2 font-[RobotoMono] text-xs leading-snug text-statusYellow">
      <span className="flex-1">
        {showConfigWarning && <span>{status.warning} </span>}
        {showNetworkWarning && (
          <span>
            This robot&apos;s controls are reachable from outside your local
            network with no authentication (AUTH_MODE=open) — anyone who can
            reach this address can operate the robot. Restrict network access,
            or set AUTH_MODE once a supported mode is available.
          </span>
        )}
      </span>
      <button
        onClick={() => setDismissed(true)}
        className="shrink-0 text-statusYellow/70 hover:text-statusYellow"
        aria-label="Dismiss warning"
      >
        ×
      </button>
    </div>
  );
};

export default AuthModeBanner;
