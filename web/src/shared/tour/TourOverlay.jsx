import React, { useEffect, useState } from "react";

/**
 * Generic guided-tour renderer: given a list of {selector, title, body}
 * steps, highlights each target element in turn (found via
 * document.querySelector) with a ring and a nearby tooltip card, no dark
 * overlay/cutout — simpler and more robust than a true spotlight mask, and
 * still reads clearly as "look here" against this app's dark theme.
 */
const TourOverlay = ({ steps, active, onFinish }) => {
  const [stepIndex, setStepIndex] = useState(0);
  const [rect, setRect] = useState(null);

  useEffect(() => {
    if (!active) return undefined;
    setStepIndex(0);
    return undefined;
  }, [active]);

  useEffect(() => {
    if (!active || !steps?.length) return undefined;

    const step = steps[stepIndex];
    const update = () => {
      const el = step ? document.querySelector(step.selector) : null;
      if (el) {
        el.scrollIntoView({ block: "center", inline: "center", behavior: "smooth" });
        setRect(el.getBoundingClientRect());
      } else {
        setRect(null);
      }
    };

    update();
    const id = setTimeout(update, 250); // after scrollIntoView settles
    window.addEventListener("resize", update);
    window.addEventListener("scroll", update, true);
    return () => {
      clearTimeout(id);
      window.removeEventListener("resize", update);
      window.removeEventListener("scroll", update, true);
    };
  }, [active, stepIndex, steps]);

  if (!active || !steps?.length) return null;

  const step = steps[stepIndex];
  const isLast = stepIndex === steps.length - 1;

  const TOOLTIP_HEIGHT = 180;
  const spaceBelow = rect ? window.innerHeight - rect.bottom : 0;
  const placeAbove = rect && spaceBelow < TOOLTIP_HEIGHT + 20;
  const tooltipTop = rect
    ? placeAbove
      ? Math.max(rect.top - TOOLTIP_HEIGHT - 12, 16)
      : Math.min(rect.bottom + 12, window.innerHeight - TOOLTIP_HEIGHT)
    : window.innerHeight / 2;
  const tooltipLeft = rect ? Math.min(Math.max(rect.left, 16), window.innerWidth - 336) : 16;

  return (
    <>
      {rect && (
        <div
          className="pointer-events-none fixed z-[200] rounded-lg ring-2 ring-themeBlue ring-offset-2 ring-offset-bgBase transition-all duration-200"
          style={{
            top: rect.top - 4,
            left: rect.left - 4,
            width: rect.width + 8,
            height: rect.height + 8,
          }}
        />
      )}

      <div
        className="fixed z-[201] w-80 rounded-xl border border-themeBlue/40 bg-bgCard p-4 shadow-2xl shadow-black/50"
        style={{ top: tooltipTop, left: tooltipLeft }}
      >
        <p className="font-[RobotoMono] text-[10px] uppercase tracking-wider text-themeBlue">
          Step {stepIndex + 1} of {steps.length}
        </p>
        <p className="mt-1 text-sm font-semibold text-textWhiteHover">{step.title}</p>
        <p className="mt-1 text-xs leading-relaxed text-themeTextGray">{step.body}</p>

        <div className="mt-3 flex items-center justify-between">
          <button
            onClick={onFinish}
            className="text-xs text-themeTextGray hover:text-statusRed"
          >
            Skip tour
          </button>
          <div className="flex gap-2">
            {stepIndex > 0 && (
              <button
                onClick={() => setStepIndex((i) => i - 1)}
                className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray hover:border-themeBlue"
              >
                Back
              </button>
            )}
            <button
              onClick={() => (isLast ? onFinish() : setStepIndex((i) => i + 1))}
              className="rounded-lg bg-themeBlue px-3 py-1.5 text-xs font-semibold text-white hover:bg-themeMediumBlue"
            >
              {isLast ? "Done" : "Next"}
            </button>
          </div>
        </div>
      </div>
    </>
  );
};

export default TourOverlay;
