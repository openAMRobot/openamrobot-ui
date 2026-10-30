import { createAnnouncementMemory, nextAnnouncements, primeMemory } from "./announcements";
import { DISPLAY } from "./reportingAdapter";

const v = (display, extra = {}) => ({ display, reason: "r", warnings: [], result: null, completion: "", ...extra });

describe("speech announcements", () => {
  it("speaks each state change once", () => {
    const m = createAnnouncementMemory();
    expect(nextAnnouncements(m, v(DISPLAY.READY))).toHaveLength(1);
    expect(nextAnnouncements(m, v(DISPLAY.READY))).toHaveLength(0);
    expect(nextAnnouncements(m, v(DISPLAY.EXECUTING))).toEqual(["executing. r"]);
  });

  it("speaks a result, a warning and a completion once each", () => {
    const m = createAnnouncementMemory();
    const result = { runId: "g1", name: "Navigation goal", outcome: "SUCCEEDED", reason: "", invalid: false };
    const view = v(DISPLAY.READY, { result, completion: "Completed: Navigation goal", warnings: ["Battery low (20%)"] });
    expect(nextAnnouncements(m, view)).toEqual([
      "ready. r",
      "Navigation goal: succeeded.",
      "Warning: Battery low (20%).",
      "Completed: Navigation goal",
    ]);
    expect(nextAnnouncements(m, view)).toEqual([]);
  });

  it("never repeats on reconnect", () => {
    const m = createAnnouncementMemory();
    const result = { runId: "g1", name: "Navigation goal", outcome: "SUCCEEDED", reason: "", invalid: false };
    const ready = v(DISPLAY.READY, { result, completion: "Completed: Navigation goal", warnings: ["w"] });
    nextAnnouncements(m, ready);
    expect(nextAnnouncements(m, v(DISPLAY.DISCONNECTED))).toEqual(["disconnected. r"]);
    expect(nextAnnouncements(m, v(DISPLAY.UNKNOWN))).toEqual(["unknown. r"]);
    // Back to the same state with the same result, warning and completion: silent.
    expect(nextAnnouncements(m, ready)).toEqual([]);
  });

  it("a different state after a reconnect is spoken", () => {
    const m = createAnnouncementMemory();
    nextAnnouncements(m, v(DISPLAY.EXECUTING));
    nextAnnouncements(m, v(DISPLAY.DISCONNECTED));
    expect(nextAnnouncements(m, v(DISPLAY.READY))).toEqual(["ready. r"]);
  });

  it("switching speech on does not read out what is already on screen", () => {
    const result = { runId: "g1", name: "Navigation goal", outcome: "ABORTED", reason: "No valid path", invalid: false };
    const view = v(DISPLAY.WAITING, { result, warnings: ["w"] });
    const m = primeMemory(createAnnouncementMemory(), view);
    expect(nextAnnouncements(m, view)).toEqual([]);
  });

  it("an invalid result is spoken as result unknown", () => {
    const m = primeMemory(createAnnouncementMemory(), v(DISPLAY.EXECUTING));
    const result = { runId: "g1", name: "Navigation goal", outcome: "UNKNOWN", reason: "", invalid: true };
    expect(nextAnnouncements(m, v(DISPLAY.EXECUTING, { result }))).toEqual(["Navigation goal: result unknown."]);
  });
});
