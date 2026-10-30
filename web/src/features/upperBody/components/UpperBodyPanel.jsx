import React from "react";

import { DashboardCard } from "../../../shared/ui/Dashboard";
import { JOINT_GROUPS, describeConfiguration } from "../upperBodyModel";

const Title = ({ children }) => (
  <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">{children}</p>
);

const formatMs = (ms) => (ms >= 1000 ? `${(ms / 1000).toFixed(1)} s` : `${Math.round(ms)} ms`);

// Neutral and warning tones only: freshness is never shown as a green "OK".
export function freshnessLabel(status) {
  if (!status || status.state === "not_reported") return { text: "Not reported", tone: "text-themeTextGray" };
  if (status.state === "measuring") return { text: "Measuring publish rate", tone: "text-themeTextGray" };
  if (status.state === "partial" || status.state === "stale") {
    const parts = [
      status.missing?.length ? `not reported: ${status.missing.join(", ")}` : null,
      status.invalid?.length ? `invalid: ${status.invalid.join(", ")}` : null,
      status.stale?.length ? `stale: ${status.stale.join(", ")}` : null,
    ].filter(Boolean);
    const head =
      status.state === "partial"
        ? "Partial"
        : status.ageMs === null || status.ageMs === undefined
        ? "Stale"
        : `Stale: no update for ${formatMs(status.ageMs)}`;
    return { text: `${head} (${parts.join("; ")})`, tone: "text-statusYellow" };
  }
  return { text: `Updating, measured period ${formatMs(status.periodMs)}`, tone: "text-themeBlue" };
}

function jointValue(joint, positions, status) {
  if (typeof positions?.[joint] === "number" && Number.isFinite(positions[joint])) return positions[joint].toFixed(3);
  if (status?.invalid?.includes(joint)) return "invalid";
  return "not reported";
}

const ConfigurationIdentity = ({ reportedConfiguration }) => {
  const config = describeConfiguration(reportedConfiguration);
  return (
    <div data-testid="configuration-identity">
      <p className="text-[12px] font-semibold text-textWhiteHover">Configuration identity</p>
      {config.known ? (
        <dl className="mt-1 grid grid-cols-[auto_1fr] gap-x-3 text-[12px]">
          {config.fields.map((f) => (
            <React.Fragment key={f.key}>
              <dt className="text-themeTextGray">{f.label}</dt>
              <dd className="font-[RobotoMono] text-textWhiteHover">{f.value}</dd>
            </React.Fragment>
          ))}
        </dl>
      ) : (
        <p className="mt-1 text-[12px] text-statusYellow">{config.message}</p>
      )}
      {config.flags.map((flag) => (
        <p key={flag} className="mt-1 text-[12px] text-statusYellow">
          {flag}
        </p>
      ))}
    </div>
  );
};

/**
 * Read-only upper-body view: configuration identity as reported, and the
 * reported joint positions per group. No controls of any kind.
 */
const UpperBodyPanel = ({ positions, freshness, reportedConfiguration = null }) => (
  <DashboardCard className="flex flex-col gap-3 p-4" data-testid="upper-body-panel">
    <Title>Upper body (read-only)</Title>
    <ConfigurationIdentity reportedConfiguration={reportedConfiguration} />
    <p className="text-[12px] leading-snug text-themeTextGray">
      Reported positions from <code>/joint_states</code> only. They are not readiness, controller health or
      safe-to-move status.
    </p>
    <div className="flex flex-col gap-2">
      {JOINT_GROUPS.map((group) => {
        const label = freshnessLabel(freshness?.[group.id]);
        return (
          <div key={group.id} data-testid={`joint-group-${group.id}`}>
            <div className="flex flex-wrap items-baseline justify-between gap-2">
              <span className="text-[12px] font-semibold text-textWhiteHover">{group.label}</span>
              <span className={`text-[11px] ${label.tone}`}>{label.text}</span>
            </div>
            <ul className="mt-0.5 grid grid-cols-2 gap-x-3 font-[RobotoMono] text-[11px] text-themeTextGray">
              {group.joints.map((joint) => (
                <li key={joint} className="flex justify-between gap-2">
                  <span className="truncate">{joint}</span>
                  <span className="text-textWhiteHover">
                    {jointValue(joint, positions, freshness?.[group.id])}
                  </span>
                </li>
              ))}
            </ul>
          </div>
        );
      })}
    </div>
    <p className="text-[12px] leading-snug text-themeTextGray">
      Stop (software) locks base motion only. It does not stop the arms or the lift.
    </p>
  </DashboardCard>
);

export default UpperBodyPanel;
