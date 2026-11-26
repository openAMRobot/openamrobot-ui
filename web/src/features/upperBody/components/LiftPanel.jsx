import React from "react";

import { DashboardCard } from "../../../shared/ui/Dashboard";
import { LIFT_STOPS, liftReadout } from "../upperBodyModel";
import { freshnessLabel } from "./UpperBodyPanel";

/**
 * Lift height against the calibrated stops. A position readout only: the
 * view never moves the lift and being near a stop is not a calibration or
 * readiness statement.
 */
const LiftPanel = ({ liftPositionM, shoulderM, freshness }) => {
  const readout = liftReadout({ liftPositionM, shoulderM });
  const label = freshnessLabel(freshness);
  const pct = readout.fraction === null ? null : Math.min(Math.max(readout.fraction, 0), 1) * 100;
  const lo = LIFT_STOPS[0].shoulderMm;
  const span = LIFT_STOPS[LIFT_STOPS.length - 1].shoulderMm - lo;

  return (
    <DashboardCard className="flex flex-col gap-2 p-4" data-testid="lift-panel">
      <div className="flex flex-wrap items-baseline justify-between gap-2">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">Lift</p>
        <span className={`text-[11px] ${label.tone}`}>{label.text}</span>
      </div>
      <p className="text-[12px] text-themeTextGray">
        lift_joint:{" "}
        <span className="font-[RobotoMono] text-textWhiteHover">
          {readout.liftPositionM === null ? "not reported" : `${readout.liftPositionM.toFixed(3)} m`}
        </span>
      </p>
      <p className="text-[12px] text-themeTextGray">
        Shoulder height (from the candidate model):{" "}
        <span className="font-[RobotoMono] text-textWhiteHover" data-testid="shoulder-height">
          {readout.shoulderMm === null ? "unknown" : `${Math.round(readout.shoulderMm)} mm`}
        </span>
        {readout.nearest ? (
          <span data-testid="nearest-stop">
            {" "}
            ({Math.round(readout.nearest.deltaMm) === 0
              ? `at ${readout.nearest.id}`
              : `${Math.abs(Math.round(readout.nearest.deltaMm))} mm ${readout.nearest.deltaMm > 0 ? "above" : "below"} ${readout.nearest.id}`}
            )
          </span>
        ) : null}
      </p>
      <div className="relative mt-4 h-2 rounded bg-bgSurface" aria-hidden="true">
        {LIFT_STOPS.map((stop) => (
          <div
            key={stop.id}
            className="absolute -top-4 -translate-x-1/2 text-center text-[10px] text-themeTextGray"
            style={{ left: `${((stop.shoulderMm - lo) / span) * 100}%` }}
          >
            {stop.id}
            <div className="mx-auto h-4 w-px bg-themeTextGray" />
          </div>
        ))}
        {pct === null ? null : (
          <div className="absolute top-0 h-2 w-2 -translate-x-1/2 rounded-full bg-themeBlue" style={{ left: `${pct}%` }} />
        )}
      </div>
      <p className="mt-1 text-[12px] text-themeTextGray" data-testid="stop-sensors">
        Stop sensors: not reported
      </p>
      <p className="text-[11px] text-themeTextGray/70">
        No producer for the lift stop sensors exists yet (I8, openamrobot-interfaces #30).
      </p>
    </DashboardCard>
  );
};

export default LiftPanel;
