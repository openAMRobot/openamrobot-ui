import React from "react";
import { useRuntimeConfig } from "../app/App";

/**
 * Always-visible while demo mode is on — deliberately not dismissible, the
 * same way AuthModeBanner's warnings are, but for a different reason: this
 * isn't warning about a risk, it's the one thing standing between "this is
 * simulated telemetry" and someone mistaking it for a live robot. It's also
 * the fastest way back to a real connection, per the spec's "Allow
 * switching from Demo to real connection."
 */
const DemoModeBanner = () => {
  const { config, updateConfig } = useRuntimeConfig();
  if (!config.demoMode) return null;

  return (
    <div className="mb-3 flex items-center gap-3 rounded-xl border border-themeBlue/40 bg-themeBlue/10 px-4 py-2 font-[RobotoMono] text-xs text-themeBlue">
      <span className="flex h-2 w-2 shrink-0 rounded-full bg-themeBlue" />
      <span className="flex-1">
        <span className="font-bold uppercase tracking-wider">Demo mode</span>{" "}
        — every value on screen is simulated. No robot is connected.
      </span>
      <button
        onClick={() => updateConfig({ demoMode: false })}
        className="shrink-0 rounded-lg border border-themeBlue/40 px-2.5 py-1 font-semibold hover:bg-themeBlue hover:text-white"
      >
        Exit demo mode
      </button>
    </div>
  );
};

export default DemoModeBanner;
