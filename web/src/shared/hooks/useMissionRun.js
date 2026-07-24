import { useEffect, useState } from "react";

import { getRun, subscribeRun } from "../missions/missionRunner";

// React binding for the module-level mission-run state MissionRunner.jsx
// drives — same shape as useEventLog/useSchedules.
export default function useMissionRun() {
  const [run, setRunState] = useState(getRun);
  useEffect(() => subscribeRun(setRunState), []);
  return run;
}
