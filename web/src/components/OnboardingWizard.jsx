import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import { useRuntimeConfig } from "../app/App";

/**
 * First-run wizard, gated on localStorage (see AppLayout.jsx) so it only
 * appears automatically once — reopenable any time via HelpWidget's
 * "Replay welcome guide". Deliberately doesn't duplicate the Config page's
 * connection form inline; the "connect" path explains the three steps and
 * hands off to Config, since that form already exists and works.
 */
const OnboardingWizard = ({ open, onClose, onRequestTour }) => {
  const { updateConfig } = useRuntimeConfig();
  const navigate = useNavigate();
  const [step, setStep] = useState("welcome");
  const [connectKind, setConnectKind] = useState("robot");

  if (!open) return null;

  const finish = (path) => {
    onClose();
    setStep("welcome");
    if (path) navigate(path);
  };

  return (
    <div className="fixed inset-0 z-[300] flex items-center justify-center bg-black/70 p-4">
      <div className="w-full max-w-lg rounded-2xl border border-borderSubtle bg-bgCard p-6 shadow-2xl shadow-black/60">
        {step === "welcome" && (
          <>
            <p className="font-[RobotoMono] text-[11px] uppercase tracking-[0.14em] text-themeBlue">
              Welcome
            </p>
            <h2 className="mt-1 text-xl font-bold text-textWhiteHover">
              Welcome to OpenAMRobot
            </h2>
            <p className="mt-3 text-sm leading-relaxed text-themeTextGray">
              This is a browser-based control and monitoring interface for a
              real ROS 2 mobile robot — driving, mapping, route planning,
              visual programming, diagnostics, and more, all from here.
              Let&apos;s get you oriented.
            </p>
            <div className="mt-5 flex justify-end gap-3">
              <button
                onClick={() => finish(null)}
                className="text-xs text-themeTextGray hover:text-textWhiteHover"
              >
                Skip
              </button>
              <button
                onClick={() => setStep("choose")}
                className="rounded-lg bg-themeBlue px-4 py-2 text-sm font-semibold text-white hover:bg-themeMediumBlue"
              >
                Get started
              </button>
            </div>
          </>
        )}

        {step === "choose" && (
          <>
            <p className="font-[RobotoMono] text-[11px] uppercase tracking-[0.14em] text-themeBlue">
              How do you want to start?
            </p>
            <div className="mt-4 space-y-2">
              <button
                onClick={() => setStep("explore-confirm")}
                className="w-full rounded-xl border border-borderSubtle p-4 text-left hover:border-themeBlue"
              >
                <p className="text-sm font-semibold text-textWhiteHover">
                  Explore without a robot
                </p>
                <p className="mt-1 text-xs text-themeTextGray">
                  See every page with simulated data — no hardware or ROS
                  connection needed.
                </p>
              </button>
              <button
                onClick={() => {
                  setConnectKind("robot");
                  setStep("connect-guide");
                }}
                className="w-full rounded-xl border border-borderSubtle p-4 text-left hover:border-themeBlue"
              >
                <p className="text-sm font-semibold text-textWhiteHover">
                  Connect a robot
                </p>
                <p className="mt-1 text-xs text-themeTextGray">
                  Point this UI at a real robot&apos;s rosbridge connection.
                </p>
              </button>
              <button
                onClick={() => {
                  setConnectKind("simulation");
                  setStep("connect-guide");
                }}
                className="w-full rounded-xl border border-borderSubtle p-4 text-left hover:border-themeBlue"
              >
                <p className="text-sm font-semibold text-textWhiteHover">
                  Connect to a simulation
                </p>
                <p className="mt-1 text-xs text-themeTextGray">
                  Point this UI at a simulated ROS 2 stack (e.g. Gazebo) the
                  same way you would a real robot.
                </p>
              </button>
            </div>
          </>
        )}

        {step === "explore-confirm" && (
          <>
            <p className="font-[RobotoMono] text-[11px] uppercase tracking-[0.14em] text-themeBlue">
              Demo mode
            </p>
            <h2 className="mt-1 text-lg font-bold text-textWhiteHover">
              You&apos;re all set — try a few things
            </h2>
            <p className="mt-3 text-sm leading-relaxed text-themeTextGray">
              Demo mode is now on — every page shows believable simulated
              telemetry, always clearly labeled with a Demo mode banner.
              A few guided tasks to get a feel for it, or skip straight in.
            </p>
            <div className="mt-4 space-y-2">
              <button
                onClick={() => {
                  updateConfig({ demoMode: true });
                  onRequestTour?.();
                  finish("/");
                }}
                className="w-full rounded-xl border border-borderSubtle p-3 text-left hover:border-themeBlue"
              >
                <p className="text-sm font-semibold text-textWhiteHover">
                  1. Take the guided Map tour
                </p>
                <p className="mt-0.5 text-xs text-themeTextGray">
                  Six quick steps: navigation, connection status, map layers,
                  goals, and manual drive.
                </p>
              </button>
              <button
                onClick={() => {
                  updateConfig({ demoMode: true });
                  finish("/health");
                }}
                className="w-full rounded-xl border border-borderSubtle p-3 text-left hover:border-themeBlue"
              >
                <p className="text-sm font-semibold text-textWhiteHover">
                  2. Check the Health Centre
                </p>
                <p className="mt-0.5 text-xs text-themeTextGray">
                  See the Ready / Warnings rollup this simulated robot reports,
                  and how issues link back to where you&apos;d fix them.
                </p>
              </button>
              <button
                onClick={() => {
                  updateConfig({ demoMode: true });
                  finish("/metrics");
                }}
                className="w-full rounded-xl border border-borderSubtle p-3 text-left hover:border-themeBlue"
              >
                <p className="text-sm font-semibold text-textWhiteHover">
                  3. Watch the track record build
                </p>
                <p className="mt-0.5 text-xs text-themeTextGray">
                  Distance travelled, uptime, and goal outcomes accumulate
                  live as the simulated robot moves.
                </p>
              </button>
            </div>
            <div className="mt-4 flex justify-end">
              <button
                onClick={() => {
                  updateConfig({ demoMode: true });
                  finish("/");
                }}
                className="text-xs text-themeTextGray hover:text-textWhiteHover"
              >
                Just start exploring →
              </button>
            </div>
          </>
        )}

        {step === "connect-guide" && (
          <>
            <p className="font-[RobotoMono] text-[11px] uppercase tracking-[0.14em] text-themeBlue">
              {connectKind === "simulation"
                ? "Connect to a simulation"
                : "Connect a robot"}
            </p>
            <h2 className="mt-1 text-lg font-bold text-textWhiteHover">
              Three steps
            </h2>
            <ol className="mt-3 space-y-2.5 text-sm text-themeTextGray">
              <li>
                <span className="font-semibold text-textWhiteHover">
                  1. Configure the ROS bridge —
                </span>{" "}
                set the host and port on the Config page.
              </li>
              <li>
                <span className="font-semibold text-textWhiteHover">
                  2. Test the connection —
                </span>{" "}
                watch the status dot in the sidebar, or check the Health page
                for a full rollup.
              </li>
              <li>
                <span className="font-semibold text-textWhiteHover">
                  3. Detect available devices —
                </span>{" "}
                the Devices page can find real serial ports if you have USB
                hardware attached.
              </li>
            </ol>
            <div className="mt-5 flex justify-end">
              <button
                onClick={() => finish("/config")}
                className="rounded-lg bg-themeBlue px-4 py-2 text-sm font-semibold text-white hover:bg-themeMediumBlue"
              >
                Go to Config
              </button>
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default OnboardingWizard;
