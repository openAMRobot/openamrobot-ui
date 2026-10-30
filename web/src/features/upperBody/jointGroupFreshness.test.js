import { STALE_FLOOR_MS, createFreshnessTracker } from "./jointGroupFreshness";

const GROUPS = [
  { id: "lift", joints: ["lift_joint"] },
  { id: "leftArm", joints: ["openarm_left_joint1", "openarm_left_joint2"] },
];

describe("joint group freshness", () => {
  it("is not reported before any message", () => {
    expect(createFreshnessTracker(GROUPS).status("lift", 0).state).toBe("not_reported");
  });

  it("measures before it has a rate", () => {
    const t = createFreshnessTracker(GROUPS);
    t.record(["lift_joint"], 0);
    t.record(["lift_joint"], 100);
    expect(t.status("lift", 150)).toMatchObject({ state: "measuring", periodMs: null });
  });

  it("derives the stale threshold from the measured period of each group", () => {
    const t = createFreshnessTracker(GROUPS);
    // lift at 1 Hz, left arm at 10 Hz
    [0, 1000, 2000, 3000].forEach((ms) => t.record(["lift_joint"], ms));
    for (let ms = 0; ms <= 3000; ms += 100) t.record(["openarm_left_joint1"], ms);

    expect(t.status("lift", 3500)).toMatchObject({ state: "updating", periodMs: 1000, staleAfterMs: 3000 });
    expect(t.status("leftArm", 3200)).toMatchObject({ state: "updating", periodMs: 100, staleAfterMs: 300 });
    // 400 ms of silence is stale for the 10 Hz group and not for the 1 Hz one
    expect(t.status("leftArm", 3400).state).toBe("stale");
    expect(t.status("lift", 3400).state).toBe("updating");
    expect(t.status("lift", 6100).state).toBe("stale");
  });

  it("does not call a fast group stale inside the jitter floor", () => {
    const t = createFreshnessTracker(GROUPS);
    for (let ms = 0; ms <= 100; ms += 10) t.record(["lift_joint"], ms);
    expect(t.status("lift", 100 + STALE_FLOOR_MS - 1).state).toBe("updating");
    expect(t.status("lift", 100 + STALE_FLOOR_MS + 1).state).toBe("stale");
  });

  it("counts a message once per group and ignores other joints", () => {
    const t = createFreshnessTracker(GROUPS);
    [0, 50, 100].forEach((ms) => t.record(["openarm_left_joint1", "openarm_left_joint2", "wheel_joint"], ms));
    expect(t.status("leftArm", 100).periodMs).toBe(50);
    expect(t.status("lift", 100).state).toBe("not_reported");
  });
});
