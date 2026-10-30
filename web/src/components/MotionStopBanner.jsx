import React, { useState } from "react";

import { useRos } from "../app/App";
import { useMotionGate } from "../shared/motion/motionGate";
import { resumeFromSoftwareStop } from "../shared/motion/softwareStop";
import { LOCK_LOCKED, LOCK_RELEASED } from "../shared/motion/stopLock";

/**
 * Persistent banner while the platform stop lock is locked or unknown, with
 * the Resume control. Resume is the only release path in the UI: it opens a
 * confirmation and only then calls /safety/stop_lock_release. It is enabled
 * only when rosbridge is connected and the lock state is known; otherwise it
 * says why.
 *
 * When the lock is released but the robot clock is not synced, a smaller
 * notice says manual drive is disabled.
 */
const MotionStopBanner = () => {
  const ros = useRos();
  const gate = useMotionGate();
  const [confirming, setConfirming] = useState(false);
  const [pending, setPending] = useState(false);
  const [lastError, setLastError] = useState("");

  const connected = gate.connection === "connected";
  const locked = gate.lock === LOCK_LOCKED;

  if (gate.lock === LOCK_RELEASED) {
    if (!connected || gate.clock.synced) return null;
    return (
      <div
        role="status"
        className="mb-3 rounded-lg border border-statusYellow/60 bg-statusYellow/10 px-3 py-2 font-[RobotoMono] text-xs text-statusYellow"
      >
        <strong>{gate.clock.label}</strong>
        {gate.clock.detail ? `: ${gate.clock.detail}. ` : ". "}
        Manual drive is disabled until the clock is synced.
      </div>
    );
  }

  let resumeBlockedReason = "";
  if (!connected) resumeBlockedReason = "Resume needs a rosbridge connection.";
  else if (!locked) resumeBlockedReason = "Resume needs a known lock state from /safety/stop_lock.";
  else if (pending) resumeBlockedReason = "Release request sent, waiting for the lock.";

  const detail = !connected
    ? "rosbridge is disconnected, so the lock state is unknown."
    : locked
      ? "The platform stop lock is engaged. No manual drive, program or mission can run."
      : "The lock state is unknown (no recent /safety/stop_lock sample). Treated as stopped.";

  const confirmResume = async () => {
    setConfirming(false);
    setPending(true);
    setLastError("");
    const result = await resumeFromSoftwareStop(ros);
    setPending(false);
    if (!result.success) setLastError(result.message || "Release failed");
  };

  return (
    <div
      role="alert"
      className="mb-3 flex flex-wrap items-center gap-3 rounded-lg border-2 border-statusRed bg-statusRed/10 px-3 py-2 font-[RobotoMono]"
    >
      <div className="min-w-0 flex-1">
        <p className="text-sm font-bold text-statusRed">Stopped (software)</p>
        <p className="text-xs text-themeTextGray">{detail}</p>
        <p className="text-[11px] text-themeTextGray">
          This is a software stop. It is not the hardwired E-stop.
        </p>
        {lastError && <p className="text-xs text-statusRed">Resume failed: {lastError}</p>}
      </div>
      <div className="flex flex-col items-end gap-1">
        <button
          type="button"
          onClick={() => setConfirming(true)}
          disabled={Boolean(resumeBlockedReason)}
          title={resumeBlockedReason || "Release the software stop lock"}
          className="rounded-lg border border-statusGreen px-3 py-1.5 text-xs font-semibold text-statusGreen hover:bg-statusGreen hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
        >
          Resume
        </button>
        {resumeBlockedReason && (
          <span className="text-[11px] text-themeTextGray">{resumeBlockedReason}</span>
        )}
      </div>

      {confirming && (
        <div
          role="dialog"
          aria-modal="true"
          aria-label="Confirm resume"
          className="fixed inset-0 z-[90] flex items-center justify-center bg-black/60 p-4"
        >
          <div className="max-w-md rounded-xl border border-borderSubtle bg-bgCard p-4 font-[RobotoMono]">
            <p className="mb-2 text-sm font-semibold text-textWhiteHover">Release the software stop?</p>
            <p className="mb-4 text-xs text-themeTextGray">
              Check that the area around the robot is clear. Cancelled goals, programs and
              missions do not restart on their own.
            </p>
            <div className="flex justify-end gap-2">
              <button
                type="button"
                onClick={() => setConfirming(false)}
                className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray"
              >
                Cancel
              </button>
              <button
                type="button"
                onClick={confirmResume}
                className="rounded-lg border border-statusGreen bg-statusGreen px-3 py-1.5 text-xs font-semibold text-white"
              >
                Confirm resume
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default MotionStopBanner;
