# I5 run-state fixture envelope

> **Status: Proposed, fixture-only, pending owner review.**
> This was prepared on Sunday with founder authorization. Owner alignment is
> pending the Monday review. This draft is not an accepted contract or a
> production implementation. It is input to the shared-envelope discussion
> due 2 October, not the outcome of it.

- **Work item:** Moazzam WP1, a fixture-backed current-run view.
  Tracker: [openamrobot-ui#19](https://github.com/openAMRobot/openamrobot-ui/issues/19).
- **Owner:** Moazzam Ali. **Reviewer:** Mohamed Sayed.
  **Reporting counterpart:** Parth Keshari.
- **Sequencing departure:** D-03 normally requires an owner discussion
  before any execution starts. That discussion has not happened. This
  document and its code are a draft that the owner can adopt, adapt or
  reject.

## 1. What was looked for first

| Source | Result |
|---|---|
| Accepted I5 contract | None found. None of these contain one: this repository, the [openamrobot-interfaces](https://github.com/openAMRobot/openamrobot-interfaces) `main` branch (checked at `e5d654e`), or its open pull-request refs. |
| Mission-wide reason registry | None found. The only merged registry is the navigation-scoped constants block in `openamr_nav_msgs/msg/NavigationStatus.msg`: groups 1xxx to 9xxx covering stack, sensors, localization, nav task, recovery, protection, docking, parking and base. The topic names in that package's `CONTRACT.md` are themselves still marked "proposed". |
| Candidate registry | interfaces PR 12, "reject duplicate codes across the candidate reason registry", adds compatibility tooling for a candidate registry. It is **unmerged** and was not treated as accepted. |
| docs#24 (`openAMRobot/openamrobot-docs`) | **Not read.** This session could not get API read access to that repository. Its content has not been guessed. |
| UI work package, D-02 and D-03 (Google Drive) | **Not read**: they could not be reached from this session. This proposal uses only the field list in the task text, which cites WP section 4. |

The navigation reason codes cover navigation only. They were not
assumed to be a mission-wide registry. Every reason in this fixture uses the
namespaced **provisional** registry `FIXTURE_PROVISIONAL`, for example
`FIXTURE_PROVISIONAL.STEP_FAILED`. These codes exist so the tests can check
that reasons are typed. They are not proposed as permanent codes.

## 2. Envelope, version `openamr.run_state.fixture/0.1-proposed`

These are the WP section 4 fields as listed in the task text. Values that
are not known are sent as `null` and stay unknown. The consumer never fills
them in.

### Snapshot

`GET /api/dev/run-state-fixture/snapshot`

```json
{
  "schema_version": "openamr.run_state.fixture/0.1-proposed",
  "kind": "snapshot",
  "fixture": true,
  "scenario": "success",
  "producer": {
    "id": "openamr-ui-run-state-fixture",
    "epoch": "fixture-epoch-7f3c",
    "timebase": "producer_monotonic_ms",
    "time_ms": 4500,
    "heartbeat_seq": 3
  },
  "robot": { "robot_id": "fixture-robot-01", "config_id": "fixture-config-a" },
  "profile": "fixture-profile-default",
  "as_of_seq": 5,
  "operational_state": "READY",
  "reason": null,
  "allowed_operations": [],
  "run": {
    "run_id": "fixture-run-0001",
    "mission_id": "fixture-mission-a",
    "step": { "step_id": "step-3", "index": 2, "count": 3 },
    "progress": { "steps_completed": 3, "steps_total": 3 },
    "outcome": { "result": "SUCCEEDED", "reason": null },
    "started_at_ms": 1500,
    "ended_at_ms": 4500
  }
}
```

### Events

`GET /api/dev/run-state-fixture/events?epoch=<epoch>&after_seq=<n>`

The response returns the events of the current epoch whose `seq` is greater
than `n`. If `epoch` is not the current epoch, the response is
**409 EPOCH_MISMATCH** and the consumer must take a new snapshot.

```json
{
  "schema_version": "openamr.run_state.fixture/0.1-proposed",
  "kind": "events",
  "fixture": true,
  "producer": { "id": "...", "epoch": "...", "timebase": "producer_monotonic_ms", "time_ms": 0, "heartbeat_seq": 0 },
  "events": [
    {
      "run_id": "fixture-run-0001",
      "epoch": "fixture-epoch-7f3c",
      "seq": 2,
      "type": "RUN_STARTED",
      "producer_time_ms": 1500,
      "patch": {
        "run": { "step": { "step_id": "step-1", "index": 0, "count": 3 }, "started_at_ms": 1500 },
        "operational_state": "EXECUTING",
        "reason": null,
        "allowed_operations": ["FIXTURE_PROVISIONAL.CANCEL"]
      }
    }
  ]
}
```

Event types used by the fixture: `RUN_CREATED`, `RUN_STARTED`,
`STEP_CHANGED` and `RUN_ENDED`.

### Field notes

| Field | Meaning | Unknown |
|---|---|---|
| `schema_version` | Must match exactly. Any other value is **INVALID** and is never shown as healthy. | n/a |
| `producer.epoch` | An opaque identity for one producer lifetime. It is compared **for equality only**. It is not a clock and cannot be sorted: the fixture's second epoch deliberately sorts lower than its first. | n/a (required) |
| `producer.timebase`, `time_ms` | Milliseconds on the producer's monotonic clock, measured from the start of the epoch. It is only ever subtracted from other times in the same epoch. | n/a (required) |
| `producer.heartbeat_seq` | Advances while the producer is alive. Staleness is detected from it. | n/a (required) |
| `as_of_seq` | The last event `seq` of this epoch that the snapshot reflects. | n/a (required) |
| `operational_state` | One of READY, EXECUTING, WAITING, BLOCKED, RECOVERING, NEEDS_ASSISTANCE, SAFETY_STOPPED or FAULTED. | `null` is shown as UNKNOWN. An unrecognised value is also shown as UNKNOWN, with the raw value kept. |
| `run.outcome` | A **separate** terminal field: `{result: SUCCEEDED, FAILED or CANCELED, reason}`. Success and failure are never operational states. | `null` means no terminal outcome yet. |
| `reason`, `outcome.reason` | Typed as `{registry, code}`. | `null` means none reported. |
| `run.progress` | Sent only when the producer knows it. The UI never estimates progress or an ETA. | `null` is shown as "not reported". |
| `allowed_operations` | Display-only strings from the producer. The UI never infers them and never turns them into controls. | `null` means unknown, which is different from `[]` (none allowed). |
| `robot`, `profile` | Identity of the robot and its configuration. | `null` is shown as unknown. |

## 3. Semantics

- **Dedup identity and order.**
  - The identity of an event is `(run_id, epoch, seq)`.
  - Events are ordered by `seq`, which increases within one epoch.
  - A stable event id `${epoch}:${seq}` is *derived* from that tuple, not
    stored twice.
  - The consumer drops events with `seq <= appliedSeq` as duplicates.
  - Out-of-order batches are sorted by `seq` before they are applied.
  - A gap stops the batch and triggers a resync from a new snapshot.
- **Producer restart.**
  - A snapshot with a different epoch means the producer restarted. The
    consumer counts the restart, resets its event cursor to the new
    snapshot's `as_of_seq`, and rejects every event that still carries the
    old epoch. An event request made with the old epoch gets a 409.
  - In the fixture, the restarted producer keeps the run's identity but
    reports the operational state as unknown. This is a fixture assumption,
    not a proposed recovery policy.
  - A run start time from the previous epoch is not comparable with the new
    epoch, so elapsed time becomes unknown.
- **New run.** A different `run_id`, from `RUN_CREATED` or from a snapshot,
  is a new run. Events for any run other than the current one are rejected.
- **Snapshot/event race.** In the same epoch, a snapshot with
  `as_of_seq < appliedSeq` is older than state the consumer has already
  applied, so it is ignored.
- **Terminal non-regression.**
  - The first terminal outcome for a `run_id` is latched.
  - Later events for that run are rejected, and later snapshots cannot
    clear or change the latched outcome, even after a producer restart.
  - Every contradiction is counted as `terminalConflicts`.
- **Obsolete connection.** Each reconnect starts a new connection
  generation. Responses to requests from an earlier generation are dropped
  and never applied.
- **Stale deadlines.**
  - Data is FRESH only when all three hold: the connection is CONNECTED,
    the payload was valid, and the local time since `heartbeat_seq` last
    changed is at most `staleAfterMs` (3 s by default, with polling every
    1 s).
  - Otherwise it is STALE, or UNKNOWN when nothing valid has been received.
  - Freshness is worked out **when state is read**, from receipt times on
    the consumer's local monotonic clock. So a stopped, hung or
    unsubscribed connection cannot keep looking fresh.
  - The producer's clock and the browser's clock are never compared.
- **Connection states.** The possible states are IDLE, CONNECTING,
  CONNECTED, DISCONNECTED, DISABLED and INVALID.
  - DISCONNECTED: the network failed or the server returned a non-2xx
    status other than 404 or 409.
  - DISABLED: the server returned 404, meaning the fixture flag is off.
  - INVALID: validation failed.
- **Never fabricated.** While data is not fresh and valid, allowed
  operations are withheld (shown as `null`). The last reported operational
  state is still shown, but marked "last reported, not current".
- **Elapsed time.** Elapsed time is `(ended_at_ms or producer.time_ms) −
  started_at_ms`, computed only when both times come from the current
  epoch. It is shown as "producer clock, as of last producer report" or
  "final". It does not tick locally.

## 4. Exercised state and outcome mappings (proposed)

| Fixture scenario | Operational states in order | Outcome |
|---|---|---|
| `success` | (unknown), READY, EXECUTING ×3, READY | SUCCEEDED |
| `failed` | (unknown), READY, EXECUTING ×2, NEEDS_ASSISTANCE, with reason `FIXTURE_PROVISIONAL.OPERATOR_ATTENTION_REQUESTED` | FAILED, reason `FIXTURE_PROVISIONAL.STEP_FAILED` |
| `producer_restart` | READY, EXECUTING, then a new epoch with state unknown | none |
| `new_run` | Run 4: READY, EXECUTING, READY. Run 5: READY, EXECUTING. | SUCCEEDED for run 4; none for run 5 |
| `stale` | READY, EXECUTING, then the heartbeat stops | none |

The state after each terminal outcome (READY after success,
NEEDS_ASSISTANCE after failure) is an **illustrative fixture choice, not a
production recovery policy**. That decision belongs to the owner. The
fixture does not exercise WAITING, BLOCKED, RECOVERING, SAFETY_STOPPED or
FAULTED. It says nothing about physical safety: SAFETY_STOPPED here is a
vocabulary entry, not an E-stop implementation or validation.

## 5. Isolation

- **Backend-only flag.** The routes exist only when
  `OPENAMR_RUN_STATE_FIXTURES=1` (exactly `1`). Otherwise a guard returns 404
  for the whole `/api/dev/run-state-fixture` prefix; without it, the SPA
  catch-all in `flask_app.py` would answer with a 200.
- **No Config toggle, and Demo Mode is unchanged.**
- **Compose.** The default `docker-compose.yml` leaves the flag unset, and a
  test checks this. The only opt-in is the development override
  `docker-compose.run-state-fixture.yml`.
- **Read-only.** The fixture routes are GET-only.
- **Scenario control.** Scenarios are selected with
  `OPENAMR_RUN_STATE_FIXTURE_SCENARIO` and advanced by a server-side step
  clock (`OPENAMR_RUN_STATE_FIXTURE_STEP_SECONDS`), or directly by tests.
  No HTTP route selects, advances or starts anything.
- **Import allowlist.** The fixture module imports only the standard library
  and Flask. A test enforces this with an AST check, a check that the import
  loads no ROS module, and runtime fakes that fail on any use of
  `rclpy`, `roslibpy`, `subprocess` or the network.
- **Frontend.** The frontend reads the fixture with `fetch` GET only. Tests
  replace the `window.ROSLIB` constructors and the mission-runner command
  channel with spies that throw on any call.
- **Scope of the isolation.** This isolates the fixture view only. It does
  **not** make the rest of the application hardware-isolated.

## 6. Open questions for the owner

1. Which run-level states should follow a terminal outcome (see section 4)?
2. Is a pull model (snapshot plus events since a cursor) right for the
   production producer, or should the envelope ride on rosbridge? Polling was
   used here only because it is the existing Flask transport.
3. Where does the mission-wide reason registry live, and what is the
   relationship to the navigation groups and interfaces PR 12?
4. Should `allowed_operations` be typed operation identifiers from a
   registry?
5. What are the heartbeat period and stale deadline for the real producer?
6. How should restart continuity of `run_id` work: does a producer restart
   keep, end or orphan the run?
