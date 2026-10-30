# I5 run state envelope (Proposed)

Status: **Proposed**. Owner decision pending. No shared message is added by
this proposal; it describes a candidate for the interfaces repository and
how the UI would switch to it.

## Problem

Reporting Mode (`docs/reporting-mode.md`) reports missions, programs and
routes from the browser tab that runs them. That is not robot truth: a
second device does not see the run, a closed tab loses it, and a browser
view never establishes success. Navigation goals come from Nav2 action
status, which has no run identity above a single goal.

## Proposed envelope

One robot-side producer per executor (mission executor, program executor,
route follower) publishes a run envelope. Field names are a proposal.

| Field | Meaning |
|---|---|
| `run_id` | Unique id of one run, set by the executor when the run starts |
| `epoch` | Producer start id. A new value means the producer restarted; consumers drop their cursor and take a fresh snapshot |
| `sequence` | Increments by one per change within an epoch. Consumers ignore duplicates and anything older than what they applied |
| `kind` | mission, program, route, navigation |
| `name` | Human-readable task name |
| `state` | One of READY, EXECUTING, WAITING, BLOCKED, RECOVERING, NEEDS_ASSISTANCE, SAFETY_STOPPED, FAULTED. SAFETY_STOPPED only from a physical safety-chain producer |
| `step`, `step_count` | Current step n of m, when the task has steps |
| `outcome` | Empty while running; then SUCCEEDED, FAILED or CANCELED. Latched per `run_id`: never cleared or changed once set |
| `reason` | Reason code from a shared registry, plus plain-words text |
| `progress` | Optional: distance or fraction remaining, estimated time |
| `heartbeat` | Producer time and a heartbeat counter, published at a fixed rate even without change |
| `stale_after` | The producer's own statement of when a consumer must treat it as stale (for example three heartbeat periods) |

Delivery: a latched (TRANSIENT_LOCAL) snapshot topic plus a change topic,
or one topic with depth 1 and a fixed-rate heartbeat. Freshness is judged by
the consumer from local receive time of heartbeat changes, never by
comparing producer and browser clocks.

## Consumer rules

These already hold in the browser adapter and stay the same:

- A new `run_id` clears the result panel.
- Duplicates and out-of-order changes are ignored by (`epoch`, `sequence`).
- An unknown or malformed `outcome` keeps the run open, shows "Result
  unknown (invalid data)", and logs once; a later valid outcome replaces it.
- Missing, stale or unknown data is shown as such, never as healthy.

## Adapter switch-over

1. The interfaces repository adds the message after an accepted contract
   change request. Until then nothing here changes.
2. `web/src/shared/reporting/reportingSource.js` subscribes to the new
   topic and feeds it to the same reducer as a new event type. The
   browser-side mission, program and route events stay as a fallback,
   labelled "this browser", until each executor publishes.
3. When a robot-side producer covers a run kind, its browser-side events
   are removed for that kind. `deriveReport` does not change: it already
   works on runs, results and freshness.
4. `STALE_AFTER_MS` for the new source comes from the producer's
   `stale_after`, not from a UI constant.

## Earlier prototype

An earlier fixture-only prototype of a run-state envelope and a current-run
view exists as an open draft pull request in this repository. Its rules on
epochs, sequences, latched outcomes and read-time freshness are reused in
this proposal; its fixture backend is not.

## Open questions for the owner

- Does the decision register need an entry for the Reporting Mode display
  vocabulary and this envelope?
- Which repository owns the executors that would publish it?
- Is the reason registry shared with `NavigationStatus` reason codes, or
  separate?
