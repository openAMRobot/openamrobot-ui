import React, { useEffect, useState } from "react";

import { useRos, useRosStatus } from "../app/App";
import { useMotionGate } from "../shared/motion/motionGate";
import { monotonicNowMs } from "../shared/motion/robotClock";
import {
  createOperatorStatusFeed,
  deriveOperatorStatus,
  DISPLAY_READY,
} from "../shared/twin/operatorStatus";

const EVALUATE_PERIOD_MS = 500;

const TONE = {
  [DISPLAY_READY]: "border-statusGreen/60 text-statusGreen",
};
const DEFAULT_TONE = "border-statusYellow/60 text-statusYellow";

const Row = ({ label, value, testId }) => (
  <div className="flex min-w-0 gap-2">
    <dt className="shrink-0 text-themeTextGray">{label}</dt>
    <dd className="min-w-0 break-words text-textWhiteHover" data-testid={testId}>
      {value}
    </dd>
  </div>
);

/**
 * Read-only operator status (readiness, stop state, current task, freshness)
 * for the robot or the digital twin. Display only: it subscribes to status
 * topics and never publishes or calls a service. See
 * shared/twin/operatorStatus.js for the sources and the display states.
 */
const OperatorStatusPanel = ({ now = monotonicNowMs }) => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const gate = useMotionGate();
  const [samples, setSamples] = useState({});
  const [tick, setTick] = useState(() => now());

  useEffect(() => {
    // A new connection starts from nothing: no sample survives a reconnect.
    setSamples({});
    if (rosStatus !== "connected" || !ros || !window.ROSLIB) return undefined;
    const feed = createOperatorStatusFeed({
      ros,
      now,
      onSample: (key, sample) => setSamples((prev) => ({ ...prev, [key]: sample })),
    });
    return () => feed.stop();
  }, [ros, rosStatus, now]);

  useEffect(() => {
    const timer = setInterval(() => setTick(now()), EVALUATE_PERIOD_MS);
    return () => clearInterval(timer);
  }, [now]);

  const status = deriveOperatorStatus(
    {
      gate: { ...gate, connection: rosStatus === "connected" ? gate.connection : "disconnected" },
      ...samples,
    },
    Math.max(tick, now()),
  );
  const tone = TONE[status.display] || DEFAULT_TONE;

  return (
    <section
      aria-label="Operator status"
      data-testid="operator-status"
      data-display={status.display}
      className={`mb-3 rounded-lg border bg-bgCard/60 px-3 py-2 font-[RobotoMono] text-xs ${tone}`}
    >
      <div className="flex flex-wrap items-baseline gap-x-3 gap-y-1">
        <strong className="text-sm" data-testid="operator-status-display">
          {status.display}
        </strong>
        <span className="text-themeTextGray" data-testid="operator-status-detail">
          {status.detail}
        </span>
      </div>
      <dl className="mt-1 grid gap-x-6 gap-y-0.5 sm:grid-cols-2">
        <Row
          label="Readiness"
          testId="operator-status-readiness"
          value={`${status.readiness.text}${
            status.readiness.reasons.length ? `: ${status.readiness.reasons.join(", ")}` : ""
          } (health ${status.readiness.health}, Nav2 ${status.readiness.stack})`}
        />
        <Row
          label="Stop state"
          testId="operator-status-stop"
          value={`${status.stop.text}${status.stop.reason ? `: ${status.stop.reason}` : ""}`}
        />
        <Row label="Task" testId="operator-status-task" value={status.task.text} />
        <Row label="Task source" testId="operator-status-task-source" value={status.task.source} />
        <Row label="Nav status" testId="operator-status-nav-age" value={status.sources.navigation} />
        <Row label="Robot pose" testId="operator-status-pose-age" value={status.sources.pose} />
      </dl>
      {status.task.phases.length > 0 && (
        <ol className="mt-1 flex flex-wrap gap-2 text-[11px]" data-testid="operator-status-phases">
          {status.task.phases.map((p) => (
            <li key={p.phase} className="rounded border border-borderSubtle px-1.5 py-0.5 text-themeTextGray">
              {p.phase}: {p.verdict}
            </li>
          ))}
        </ol>
      )}
      <p className="mt-1 text-[11px] text-themeTextGray">
        Status display only. It is not safety evidence and not the hardwired E-stop.
      </p>
    </section>
  );
};

export default OperatorStatusPanel;
