import { useEffect, useState } from "react";

import { createRunStateAdapter } from "./runStateAdapter";
import { createRunStateFixtureTransport } from "./runStateFixtureTransport";

// One adapter per app, shared by every consumer (current-run view now, a
// future full-screen Reporting route later). Subscribing is ref-counted
// inside the adapter, so extra consumers add no extra transport calls.
let sharedAdapter = null;

export function getSharedRunStateAdapter() {
  if (!sharedAdapter) {
    sharedAdapter = createRunStateAdapter({ transport: createRunStateFixtureTransport() });
  }
  return sharedAdapter;
}

const RENDER_TICK_MS = 1000;

// The 1 s interval only re-derives freshness/age from stored receipt
// timestamps for rendering; it performs no I/O and cannot make stale data
// look fresh.
export default function useRunState(adapter = getSharedRunStateAdapter()) {
  const [view, setView] = useState(() => adapter.getState());
  useEffect(() => {
    const unsubscribe = adapter.subscribe(setView);
    const id = setInterval(() => setView(adapter.getState()), RENDER_TICK_MS);
    return () => {
      clearInterval(id);
      unsubscribe();
    };
  }, [adapter]);
  return view;
}
