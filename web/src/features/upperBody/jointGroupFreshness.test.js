import { STALE_FLOOR_MS, createFreshnessTracker, isFinitePosition, readJointState } from "./jointGroupFreshness";

const ARM = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"];
const GROUPS = [
  { id: "lift", joints: ["lift_joint"] },
  { id: "arm", joints: ARM },
];

const send = (t, names, positions, ms) => t.record(readJointState({ name: names, position: positions }), ms);
const zeros = (names) => names.map(() => 0);

describe("readJointState", () => {
  it("accepts only finite numbers", () => {
    expect([0, -1.5, NaN, Infinity, -Infinity, "1", null, undefined].map(isFinitePosition)).toEqual([
      true, true, false, false, false, false, false, false,
    ]);
    expect(readJointState({ name: ["a", "b", "c", "d"], position: [1, NaN, Infinity, -Infinity] })).toEqual({
      valid: { a: 1 },
      invalid: ["b", "c", "d"],
    });
  });

  it("treats every joint as invalid when positions do not match names", () => {
    expect(readJointState({ name: ["a", "b"], position: [1] })).toEqual({ valid: {}, invalid: ["a", "b"] });
    expect(readJointState({ name: ["a"], position: [] })).toEqual({ valid: {}, invalid: ["a"] });
  });
});

describe("per-joint freshness", () => {
  it("is not reported before any message", () => {
    expect(createFreshnessTracker(GROUPS).status("lift", 0).state).toBe("not_reported");
  });

  it("measures before it has a rate", () => {
    const t = createFreshnessTracker(GROUPS);
    send(t, ["lift_joint"], [0], 0);
    send(t, ["lift_joint"], [0], 100);
    expect(t.status("lift", 150)).toMatchObject({ state: "measuring", periodMs: null });
  });

  it("is updating only when every joint of the group is fresh", () => {
    const t = createFreshnessTracker(GROUPS);
    for (let ms = 0; ms <= 300; ms += 100) send(t, ARM, zeros(ARM), ms);
    expect(t.status("arm", 350)).toMatchObject({ state: "updating", periodMs: 100, missing: [], invalid: [], stale: [] });
  });

  it("does not let one joint refresh a whole group (partial JointState)", () => {
    const t = createFreshnessTracker(GROUPS);
    for (let ms = 0; ms <= 300; ms += 100) send(t, ["j1"], [0], ms);
    expect(t.status("arm", 350)).toMatchObject({ state: "partial", missing: ["j2", "j3", "j4", "j5", "j6", "j7"] });
    for (let ms = 400; ms <= 5000; ms += 100) send(t, ["j1"], [0], ms);
    expect(t.status("arm", 5050).state).toBe("partial");
  });

  it("names a permanently missing arm joint", () => {
    const t = createFreshnessTracker(GROUPS);
    const six = ARM.slice(0, 6);
    for (let ms = 0; ms <= 10000; ms += 100) send(t, six, zeros(six), ms);
    expect(t.status("arm", 10050)).toMatchObject({ state: "partial", missing: ["j7"], invalid: [], stale: [] });
  });

  it("marks NaN and Infinity invalid and lets them refresh nothing", () => {
    const t = createFreshnessTracker(GROUPS);
    for (let ms = 0; ms <= 300; ms += 100) send(t, ARM, zeros(ARM), ms);
    send(t, ARM, [NaN, Infinity, -Infinity, 0, 0, 0, 0], 400);
    expect(t.status("arm", 450)).toMatchObject({ state: "partial", invalid: ["j1", "j2", "j3"] });
    expect(t.jointStatus("j1", 450).state).toBe("invalid");

    const lift = createFreshnessTracker(GROUPS);
    for (let ms = 0; ms <= 1000; ms += 100) send(lift, ["lift_joint"], [NaN], ms);
    expect(lift.status("lift", 1050)).toMatchObject({ state: "stale", invalid: ["lift_joint"] });
  });

  it("recovers a joint after a valid sample follows an invalid one", () => {
    const t = createFreshnessTracker(GROUPS);
    [0, 100, 200].forEach((ms) => send(t, ["lift_joint"], [0], ms));
    send(t, ["lift_joint"], [NaN], 300);
    expect(t.status("lift", 310).state).toBe("stale");
    send(t, ["lift_joint"], [0], 400);
    expect(t.status("lift", 410).state).toBe("updating");
  });

  it("derives each joint's stale window from its own measured period", () => {
    const t = createFreshnessTracker(GROUPS);
    [0, 1000, 2000, 3000].forEach((ms) => send(t, ["lift_joint"], [0], ms));
    for (let ms = 0; ms <= 3000; ms += 100) send(t, ARM, zeros(ARM), ms);

    expect(t.status("lift", 3500)).toMatchObject({ state: "updating", periodMs: 1000 });
    expect(t.jointStatus("lift_joint", 3500).staleAfterMs).toBe(3000);
    expect(t.jointStatus("j1", 3200).staleAfterMs).toBe(300);
    expect(t.status("arm", 3400)).toMatchObject({ state: "stale", stale: ARM });
    expect(t.status("lift", 3400).state).toBe("updating");
    expect(t.status("lift", 6100).state).toBe("stale");
  });

  it("does not call a fast joint stale inside the jitter floor", () => {
    const t = createFreshnessTracker(GROUPS);
    for (let ms = 0; ms <= 100; ms += 10) send(t, ["lift_joint"], [0], ms);
    expect(t.status("lift", 100 + STALE_FLOOR_MS - 1).state).toBe("updating");
    expect(t.status("lift", 100 + STALE_FLOOR_MS + 1).state).toBe("stale");
  });

  it("ignores joints outside the groups", () => {
    const t = createFreshnessTracker(GROUPS);
    [0, 50, 100].forEach((ms) => send(t, ["wheel_joint"], [0], ms));
    expect(t.status("lift", 100).state).toBe("not_reported");
  });
});
