import React, { useEffect, useState } from "react";
import { useLocation } from "react-router-dom";

import { PAGE_HELP, DEFAULT_HELP } from "../shared/help/helpContent";
import { TOURS } from "../shared/tour/tours";
import TourOverlay from "../shared/tour/TourOverlay";

/**
 * Always-mounted floating "?" — not tied to any one page's layout (Map and
 * Control don't have a SectionHeader slot to put a help button in, so a
 * single fixed-position widget covers every page uniformly instead of 11
 * separate per-page wire-ups). Looks up the current route in PAGE_HELP/
 * TOURS via useLocation() rather than needing each page to pass anything.
 *
 * `autoStartTour`/`onTourStarted` let OnboardingWizard's "guided tasks" step
 * kick off this same tour engine after navigating here, instead of
 * duplicating a second tour implementation just for the demo-mode flow.
 */
const HelpWidget = ({ onReplayOnboarding, autoStartTour, onTourStarted }) => {
  const location = useLocation();
  const [open, setOpen] = useState(false);
  const [touring, setTouring] = useState(false);

  const content = PAGE_HELP[location.pathname] || DEFAULT_HELP;
  const tourSteps = TOURS[location.pathname];

  useEffect(() => {
    if (autoStartTour && tourSteps) {
      setTouring(true);
      onTourStarted?.();
    }
  }, [autoStartTour, tourSteps, onTourStarted]);

  return (
    <>
      <button
        onClick={() => setOpen((o) => !o)}
        className="fixed bottom-5 right-5 z-[150] flex h-12 w-12 items-center justify-center rounded-full bg-themeBlue font-[RobotoMono] text-xl font-bold text-white shadow-2xl shadow-black/40 hover:bg-themeMediumBlue"
        aria-label={open ? "Close help" : "Help"}
        aria-expanded={open}
      >
        ?
      </button>

      {open && (
        <div className="fixed bottom-20 right-5 z-[150] w-80 max-w-[calc(100vw-2.5rem)] rounded-xl border border-borderSubtle bg-bgCard p-4 shadow-2xl shadow-black/50">
          <div className="flex items-start justify-between gap-2">
            <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
              {content.title}
            </p>
            <button
              onClick={() => setOpen(false)}
              className="shrink-0 text-themeTextGray hover:text-statusRed"
              aria-label="Close help"
            >
              ×
            </button>
          </div>

          <p className="mt-2 text-xs leading-relaxed text-themeTextGray">
            {content.summary}
          </p>

          {content.tips.length > 0 && (
            <ul className="mt-3 space-y-1.5">
              {content.tips.map((tip, index) => (
                <li key={index} className="flex gap-2 text-xs text-themeTextGray">
                  <span className="shrink-0 text-themeBlue">•</span>
                  <span>{tip}</span>
                </li>
              ))}
            </ul>
          )}

          <div className="mt-4 flex flex-wrap gap-2 border-t border-borderSubtle pt-3">
            {tourSteps && (
              <button
                onClick={() => {
                  setTouring(true);
                  setOpen(false);
                }}
                className="rounded-lg border border-themeBlue px-2.5 py-1.5 text-xs font-semibold text-themeBlue hover:bg-themeBlue hover:text-white"
              >
                Take the tour
              </button>
            )}
            <button
              onClick={() => {
                onReplayOnboarding();
                setOpen(false);
              }}
              className="rounded-lg border border-borderSubtle px-2.5 py-1.5 text-xs text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            >
              Replay welcome guide
            </button>
          </div>
        </div>
      )}

      <TourOverlay steps={tourSteps} active={touring} onFinish={() => setTouring(false)} />
    </>
  );
};

export default HelpWidget;
