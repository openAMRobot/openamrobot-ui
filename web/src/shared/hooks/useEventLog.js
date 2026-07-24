import { useEffect, useState } from "react";

import { getEvents, subscribeEvents } from "../events/eventLog";

// React binding for the module-level event log: re-renders the subscriber
// whenever an event is appended or the log is cleared, from anywhere.
export default function useEventLog() {
  const [events, setEvents] = useState(getEvents);
  useEffect(() => subscribeEvents(setEvents), []);
  return events;
}
