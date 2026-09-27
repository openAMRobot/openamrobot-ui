import React from "react";

import { DashboardCard, StatusBadge } from "../shared/ui/Dashboard";
import { CONNECTION, FRESHNESS } from "../shared/runState/runStateAdapter";
import useRunState, { getSharedRunStateAdapter } from "../shared/runState/useRunState";

/**
 * Minimal current-run view over the shared run-state adapter — development
 * fixture only (docs/proposals/i5-run-state-fixture.md). Read-only by
 * design: there are no buttons, and allowed operations are rendered as
 * text. Its state comes only from Flask via the adapter, never from Demo
 * Mode or rosbridge, so a lost fixture connection is shown even while Demo
 * Mode is simulating a healthy robot.
 */

const CONNECTION_COPY = {
  [CONNECTION.IDLE]: ["unknown", "Not polling"],
  [CONNECTION.CONNECTING]: ["unknown", "Connecting to fixture backend…"],
  [CONNECTION.CONNECTED]: ["connected", "Connected to fixture backend"],
  [CONNECTION.DISCONNECTED]: ["disconnected", "Disconnected from fixture backend"],
  [CONNECTION.INVALID]: ["error", "Invalid run-state data"],
};

const FRESHNESS_TONE = {
  [FRESHNESS.FRESH]: "success",
  [FRESHNESS.STALE]: "warning",
  [FRESHNESS.UNKNOWN]: "unknown",
};

const seconds = (ms) => (ms === null || ms === undefined ? "unknown" : `${Math.floor(ms / 1000)} s`);
const orUnknown = (v) => (v === null || v === undefined || v === "" ? "unknown" : v);
const reasonText = (reason) => (reason ? reason.code : "none reported");

const Row = ({ label, children, testId }) => (
  <div className="flex flex-wrap gap-x-3 py-1 text-sm">
    <dt className="w-40 shrink-0 text-textMuted">{label}</dt>
    <dd className="min-w-0 break-all font-[RobotoMono]" data-testid={testId}>
      {children}
    </dd>
  </div>
);

export const DevelopmentTestDataLabel = () => (
  // Deliberately unlike DemoModeBanner (solid blue bar, app-wide wording):
  // dashed amber tag scoped to this view only.
  <span
    data-testid="dev-test-data-label"
    className="inline-flex items-center gap-2 rounded-md border-2 border-dashed border-statusYellow px-2 py-0.5 font-[RobotoMono] text-xs font-bold uppercase tracking-wider text-statusYellow"
  >
    Development test data
  </span>
);

const CurrentRunFixtureView = ({ adapter = getSharedRunStateAdapter() }) => {
  const view = useRunState(adapter);

  if (view.connection === CONNECTION.DISABLED) {
    return (
      <DashboardCard data-testid="current-run-fixture-view">
        <DevelopmentTestDataLabel />
        <div data-testid="fixture-disabled" className="mt-3 text-sm">
          <p className="font-semibold">Fixture backend disabled</p>
          <p className="text-textMuted">
            The backend answered 404 for the run-state fixture routes. They are enabled only
            when the backend runs with OPENAMR_RUN_STATE_FIXTURES=1 (development only). No run
            data is shown.
          </p>
        </div>
      </DashboardCard>
    );
  }

  const [connTone, connLabel] = CONNECTION_COPY[view.connection] || ["unknown", view.connection];
  const run = view.run;
  const lastReported = view.healthy ? "" : " (last reported — not current)";
  const hasData = view.appliedSeq !== null;

  return (
    <DashboardCard data-testid="current-run-fixture-view">
      <div className="flex flex-wrap items-center gap-2">
        <DevelopmentTestDataLabel />
        <span data-testid="connection-status">
          <StatusBadge status={connTone} label={connLabel} />
        </span>
        <span data-testid="freshness-status">
          <StatusBadge
            status={FRESHNESS_TONE[view.freshness]}
            label={`Freshness: ${view.freshness}`}
          />
        </span>
      </div>
      <p className="mt-2 text-xs text-textMuted">
        Fixture output from the Flask development backend; this view only. Other panels are
        unaffected by this label. Not a live robot run and not evidence of execution.
      </p>
      {view.validity === "INVALID" ? (
        <p data-testid="invalid-reason" className="mt-2 text-sm text-statusRed">
          Rejected payload: {view.invalidReason}
        </p>
      ) : null}
      {view.lastError && view.connection === CONNECTION.DISCONNECTED ? (
        <p className="mt-2 text-sm text-statusRed">{view.lastError}</p>
      ) : null}

      {hasData ? (
        <dl className="mt-3">
          <Row label="Robot / config">
            {orUnknown(view.robot?.robot_id)} / {orUnknown(view.robot?.config_id)}
          </Row>
          <Row label="Profile">{orUnknown(view.profile)}</Row>
          <Row label="Producer epoch">{orUnknown(view.producer.epoch)}</Row>
          <Row label="Last event" testId="event-id">
            {orUnknown(view.currentEventId)}
          </Row>
          <Row label="Heartbeat age">{seconds(view.heartbeatAgeMs)}</Row>
          <Row label="Run" testId="run-id">
            {run ? run.runId : "no current run reported"}
          </Row>
          <Row label="Mission">{orUnknown(run?.missionId)}</Row>
          <Row label="Step" testId="step">
            {run?.step
              ? `${orUnknown(run.step.step_id)} (${run.step.index + 1} of ${orUnknown(run.step.count)})`
              : "unknown"}
          </Row>
          <Row label="Progress">
            {run?.progress
              ? `${run.progress.steps_completed} of ${run.progress.steps_total} steps completed`
              : "not reported"}
          </Row>
          <Row label="Operational state" testId="operational-state">
            {view.operationalState}
            {lastReported}
          </Row>
          <Row label="State reason">{reasonText(view.reason)}</Row>
          <Row label="Outcome" testId="outcome">
            {run?.outcome ? run.outcome.result : "no terminal outcome"}
          </Row>
          <Row label="Outcome reason" testId="outcome-reason">
            {run?.outcome ? reasonText(run.outcome.reason) : "—"}
          </Row>
          <Row label="Elapsed" testId="elapsed">
            {run?.elapsedMs === null || run?.elapsedMs === undefined
              ? "unknown"
              : `${seconds(run.elapsedMs)} (producer clock, ${
                  run.elapsedFinal ? "final" : "as of last producer report"
                })`}
          </Row>
          <Row label="Allowed operations" testId="allowed-operations">
            {view.allowedOperations === null
              ? "unknown (withheld unless data is fresh)"
              : view.allowedOperations.length
                ? view.allowedOperations.join(", ")
                : "none reported"}
          </Row>
        </dl>
      ) : (
        <p className="mt-3 text-sm text-textMuted" data-testid="no-data">
          No run-state data received yet.
        </p>
      )}
    </DashboardCard>
  );
};

export default CurrentRunFixtureView;
