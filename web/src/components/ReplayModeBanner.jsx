import React, { useEffect, useState } from "react";
import { fetchRecordingsStatus } from "../features/recordings/recordingsApi";

const POLL_MS = 3000;

/**
 * Always-visible whenever a bag replay is active, from any page — replay
 * can be started from the Recordings page but the operator might then
 * navigate elsewhere, and every other page's telemetry would otherwise
 * look indistinguishable from a live robot. Not dismissible, same
 * reasoning as DemoModeBanner: this exists specifically to prevent
 * mistaking replayed data for live data.
 */
const ReplayModeBanner = () => {
  const [replay, setReplay] = useState(null);

  useEffect(() => {
    let cancelled = false;
    const poll = () => {
      fetchRecordingsStatus()
        .then((data) => {
          if (!cancelled) setReplay(data.replay || null);
        })
        .catch(() => {
          if (!cancelled) setReplay(null);
        });
    };
    poll();
    const id = setInterval(poll, POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, []);

  if (!replay) return null;

  return (
    <div className="mb-3 flex items-center gap-3 rounded-xl border border-statusYellow/40 bg-statusYellow/10 px-4 py-2 font-[RobotoMono] text-xs text-statusYellow">
      <span className="flex h-2 w-2 shrink-0 rounded-full bg-statusYellow" />
      <span className="flex-1">
        <span className="font-bold uppercase tracking-wider">Replay mode</span>{" "}
        — you're viewing recorded telemetry, not a live robot.
        {replay.paused ? " (paused)" : ""}
      </span>
    </div>
  );
};

export default ReplayModeBanner;
