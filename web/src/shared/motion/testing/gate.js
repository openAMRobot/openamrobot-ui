import { setClockSource, setClockStatus, setConnection, setLockState } from "../motionGate";

export const TEST_STAMP = { sec: 500, nanosec: 0 };

/** Open the motion gate: connected, lock released, clock synced. */
export const openMotionGate = () => {
  setConnection("connected");
  setLockState("released");
  setClockSource({ getStamp: () => TEST_STAMP });
  setClockStatus({ mode: "sim", synced: true, label: "Robot clock synced", detail: "" });
};

export const unsyncClock = () =>
  setClockStatus({
    mode: "real",
    synced: false,
    label: "Robot clock not synced",
    detail: "Last robot time sync is older than 6 s",
  });
