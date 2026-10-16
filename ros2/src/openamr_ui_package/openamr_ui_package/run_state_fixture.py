"""Development/test-only run-state fixture for the current-run view.

PROPOSED, FIXTURE-ONLY, PENDING OWNER REVIEW. The envelope below is input to
the shared-envelope discussion (docs/proposals/i5-run-state-fixture.md); it
is not an accepted I5 contract and must not be treated as one.

What this module is:
  - a deterministic producer of run-state snapshots and events, served by
    Flask as read-only GET routes under FIXTURE_PREFIX;
  - disabled unless the backend environment sets
    OPENAMR_RUN_STATE_FIXTURES=1 (exactly "1"). There is no Config toggle.

What it is not:
  - not an executor, run lock or mission runner. Nothing here starts, stops
    or commands anything. Scenario selection and advancement live in the
    harness (environment variable + a server-side step clock, or direct
    calls from tests), never in an HTTP route.

Dependencies are deliberately limited to the standard library and Flask so
the module and its tests run without ROS (see
test/test_run_state_fixture.py, which also checks the import allowlist).
"""

import copy
import os
import threading
import time

from flask import Blueprint, jsonify, request

FLAG_ENV = "OPENAMR_RUN_STATE_FIXTURES"
SCENARIO_ENV = "OPENAMR_RUN_STATE_FIXTURE_SCENARIO"
STEP_SECONDS_ENV = "OPENAMR_RUN_STATE_FIXTURE_STEP_SECONDS"

FIXTURE_PREFIX = "/api/dev/run-state-fixture"
SCHEMA_VERSION = "openamr.run_state.fixture/0.1-proposed"
PRODUCER_ID = "openamr-ui-run-state-fixture"
TIMEBASE = "producer_monotonic_ms"
REASON_REGISTRY = "FIXTURE_PROVISIONAL"

# Proposed operator vocabulary (tracker openamrobot-ui#19). Terminal outcome
# is a separate field; SUCCEEDED/FAILED are never operational states.
OPERATIONAL_STATES = (
    "READY",
    "EXECUTING",
    "WAITING",
    "BLOCKED",
    "RECOVERING",
    "NEEDS_ASSISTANCE",
    "SAFETY_STOPPED",
    "FAULTED",
)
TERMINAL_RESULTS = ("SUCCEEDED", "FAILED", "CANCELED")

# Epoch identities. Deliberately NOT lexically ordered in restart order:
# epochs are compared for equality only, never sorted.
EPOCH_A = "fixture-epoch-7f3c"
EPOCH_B = "fixture-epoch-1a2b"


def _reason(code):
    return {"registry": REASON_REGISTRY, "code": f"{REASON_REGISTRY}.{code}"}


def _allowed(*ops):
    # Display-only strings supplied by the producer. The UI never turns
    # these into buttons or commands.
    return [f"{REASON_REGISTRY}.{op}" for op in ops]


def _created(run_id, mission_id, steps_total):
    return (
        "event",
        "RUN_CREATED",
        {
            "run": {
                "run_id": run_id,
                "mission_id": mission_id,
                "step": None,
                "progress": {"steps_completed": 0, "steps_total": steps_total},
                "outcome": None,
                "started_at_ms": None,
                "ended_at_ms": None,
            },
            "operational_state": "READY",
            "reason": None,
            "allowed_operations": _allowed("START"),
        },
    )


def _step(index, total, completed, started=False):
    patch = {
        "run": {
            "step": {"step_id": f"step-{index + 1}", "index": index, "count": total},
            "progress": {"steps_completed": completed, "steps_total": total},
        },
        "operational_state": "EXECUTING",
        "reason": None,
        "allowed_operations": _allowed("CANCEL"),
    }
    if started:
        patch["run"]["started_at_ms"] = "$now"
    return ("event", "RUN_STARTED" if started else "STEP_CHANGED", patch)


def _ended(result, total, completed, state, outcome_reason=None, state_reason=None):
    return (
        "event",
        "RUN_ENDED",
        {
            "run": {
                "outcome": {"result": result, "reason": outcome_reason},
                "progress": {"steps_completed": completed, "steps_total": total},
                "ended_at_ms": "$now",
            },
            "operational_state": state,
            "reason": state_reason,
            "allowed_operations": [],
        },
    )


# Each scenario is an ordered list of harness steps. The fixture-chosen
# post-terminal operational states (READY after success, NEEDS_ASSISTANCE
# after failure) are illustrative only, not a production recovery policy.
SCENARIOS = {
    "success": [
        _created("fixture-run-0001", "fixture-mission-a", 3),
        _step(0, 3, 0, started=True),
        _step(1, 3, 1),
        _step(2, 3, 2),
        _ended("SUCCEEDED", 3, 3, "READY"),
    ],
    "failed": [
        _created("fixture-run-0002", "fixture-mission-b", 3),
        _step(0, 3, 0, started=True),
        _step(1, 3, 1),
        _ended(
            "FAILED",
            3,
            1,
            "NEEDS_ASSISTANCE",
            outcome_reason=_reason("STEP_FAILED"),
            state_reason=_reason("OPERATOR_ATTENTION_REQUESTED"),
        ),
    ],
    "producer_restart": [
        _created("fixture-run-0003", "fixture-mission-a", 3),
        _step(0, 3, 0, started=True),
        ("restart", EPOCH_B),
    ],
    "new_run": [
        _created("fixture-run-0004", "fixture-mission-a", 1),
        _step(0, 1, 0, started=True),
        _ended("SUCCEEDED", 1, 1, "READY"),
        _created("fixture-run-0005", "fixture-mission-b", 2),
        _step(0, 2, 0, started=True),
    ],
    "stale": [
        _created("fixture-run-0006", "fixture-mission-a", 3),
        _step(0, 3, 0, started=True),
        ("silence",),
    ],
}
DEFAULT_SCENARIO = "success"


def _merge(target, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def _resolve_now(value, now_ms):
    if value == "$now":
        return now_ms
    if isinstance(value, dict):
        return {k: _resolve_now(v, now_ms) for k, v in value.items()}
    return value


class FixtureRunStateSource:
    """Deterministic producer state. Read via snapshot()/events_after().

    advance() is the harness entry point; no HTTP route calls it directly.
    """

    def __init__(self, scenario, clock_ms=None):
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown fixture scenario {scenario!r}")
        self.scenario = scenario
        self._steps = SCENARIOS[scenario]
        self._cursor = 0
        self._clock_ms = clock_ms or (lambda: int(time.monotonic() * 1000))
        self._lock = threading.Lock()
        self._start_epoch(EPOCH_A)

    def _start_epoch(self, epoch):
        self.epoch = epoch
        self._epoch_origin_ms = self._clock_ms()
        self._seq = 0
        self._events = []
        self._silent_since_ms = None
        self._heartbeat_seq = 0
        # A restarted producer does not remember previous run state: it
        # reports the run it knows about with unknown operational state.
        prior_run = getattr(self, "_run", None)
        self._run = (
            {
                "run_id": prior_run["run_id"],
                "mission_id": prior_run["mission_id"],
                "step": None,
                "progress": None,
                "outcome": None,
                "started_at_ms": None,
                "ended_at_ms": None,
            }
            if prior_run
            else None
        )
        self._operational_state = None
        self._reason = None
        self._allowed_operations = None

    def _now_ms(self):
        # Producer timebase: monotonic milliseconds since this epoch began.
        now = self._clock_ms() - self._epoch_origin_ms
        return now if self._silent_since_ms is None else self._silent_since_ms

    @property
    def finished(self):
        return self._cursor >= len(self._steps)

    def advance(self):
        """Apply the next harness step. Returns False when exhausted."""
        with self._lock:
            if self.finished:
                return False
            step = self._steps[self._cursor]
            self._cursor += 1
            kind = step[0]
            if kind == "restart":
                self._start_epoch(step[1])
            elif kind == "silence":
                self._silent_since_ms = self._now_ms()
            else:
                _, event_type, raw_patch = step
                patch = _resolve_now(raw_patch, self._now_ms())
                run_patch = patch.get("run", {})
                if event_type == "RUN_CREATED":
                    self._run = {}
                _merge(self._run, run_patch)
                self._operational_state = patch["operational_state"]
                self._reason = patch["reason"]
                self._allowed_operations = patch["allowed_operations"]
                self._seq += 1
                self._events.append(
                    {
                        "run_id": self._run["run_id"],
                        "epoch": self.epoch,
                        "seq": self._seq,
                        "type": event_type,
                        "producer_time_ms": self._now_ms(),
                        "patch": patch,
                    }
                )
            return True

    def advance_to(self, count):
        while self._cursor < count and self.advance():
            pass

    def heartbeat(self):
        """Harness tick: advances the heartbeat unless the producer is silent."""
        with self._lock:
            if self._silent_since_ms is None:
                self._heartbeat_seq += 1

    def _producer(self):
        return {
            "id": PRODUCER_ID,
            "epoch": self.epoch,
            "timebase": TIMEBASE,
            "time_ms": self._now_ms(),
            "heartbeat_seq": self._heartbeat_seq,
        }

    def snapshot(self):
        with self._lock:
            return {
                "schema_version": SCHEMA_VERSION,
                "kind": "snapshot",
                "fixture": True,
                "scenario": self.scenario,
                "producer": self._producer(),
                "robot": {
                    "robot_id": "fixture-robot-01",
                    "config_id": "fixture-config-a",
                },
                "profile": "fixture-profile-default",
                "as_of_seq": self._seq,
                "operational_state": self._operational_state,
                "reason": copy.deepcopy(self._reason),
                "allowed_operations": copy.deepcopy(self._allowed_operations),
                "run": copy.deepcopy(self._run),
            }

    def events_after(self, epoch, after_seq):
        """Return current-epoch events with seq > after_seq.

        Returns None when the caller's epoch is not the current one; the
        caller must then take a fresh snapshot.
        """
        with self._lock:
            if epoch != self.epoch:
                return None
            return {
                "schema_version": SCHEMA_VERSION,
                "kind": "events",
                "fixture": True,
                "producer": self._producer(),
                "events": [
                    copy.deepcopy(e) for e in self._events if e["seq"] > after_seq
                ],
            }


class ClockDrivenHarness:
    """Development harness: advances one scenario step every step_seconds.

    The step index is a function of elapsed server time only; clients can
    read the result but cannot select or advance scenarios.
    """

    def __init__(self, scenario, step_seconds=3.0, clock_s=time.monotonic):
        self._clock_s = clock_s
        self._origin = clock_s()
        self._step_seconds = max(0.1, float(step_seconds))
        self._last_tick = 0
        self.source = FixtureRunStateSource(
            scenario, clock_ms=lambda: int(clock_s() * 1000)
        )

    def current(self):
        elapsed = self._clock_s() - self._origin
        ticks = int(elapsed / self._step_seconds)
        while self._last_tick < ticks:
            self._last_tick += 1
            self.source.heartbeat()
        self.source.advance_to(ticks)
        return self.source


def fixtures_enabled(environ=None):
    environ = os.environ if environ is None else environ
    return environ.get(FLAG_ENV, "").strip() == "1"


def _disabled_blueprint():
    bp = Blueprint("run_state_fixture_disabled", __name__)

    # Without this guard the SPA catch-all in flask_app.py would answer
    # these paths with index.html and a 200.
    methods = ["GET", "POST", "PUT", "DELETE", "PATCH"]

    @bp.route(FIXTURE_PREFIX, defaults={"rest": ""}, methods=methods)
    @bp.route(f"{FIXTURE_PREFIX}/<path:rest>", methods=methods)
    def fixture_disabled(rest):
        return jsonify({"code": 404, "message": "Run-state fixture disabled"}), 404

    return bp


def _enabled_blueprint(get_source):
    bp = Blueprint("run_state_fixture", __name__)

    def _no_store(response):
        response.headers["Cache-Control"] = "no-store"
        return response

    @bp.route(f"{FIXTURE_PREFIX}/snapshot", methods=["GET"])
    def fixture_snapshot():
        return _no_store(jsonify(get_source().snapshot()))

    @bp.route(f"{FIXTURE_PREFIX}/events", methods=["GET"])
    def fixture_events():
        epoch = request.args.get("epoch", "")
        try:
            after_seq = int(request.args.get("after_seq", "0"))
        except ValueError:
            return jsonify({"code": 400, "message": "after_seq must be an integer"}), 400
        body = get_source().events_after(epoch, after_seq)
        if body is None:
            return _no_store(
                jsonify({"code": 409, "message": "EPOCH_MISMATCH: re-snapshot required"})
            ), 409
        return _no_store(jsonify(body))

    return bp


def register_run_state_fixture(app, environ=None, get_source=None):
    """Register fixture routes when enabled, else a 404 guard. Returns bool."""
    environ = os.environ if environ is None else environ
    if not fixtures_enabled(environ):
        app.register_blueprint(_disabled_blueprint())
        return False
    if get_source is None:
        scenario = environ.get(SCENARIO_ENV, DEFAULT_SCENARIO).strip() or DEFAULT_SCENARIO
        try:
            step_seconds = float(environ.get(STEP_SECONDS_ENV, "3"))
        except ValueError:
            step_seconds = 3.0
        harness = ClockDrivenHarness(scenario, step_seconds=step_seconds)
        get_source = harness.current
    app.register_blueprint(_enabled_blueprint(get_source))
    print(
        f"[openamr_ui] WARNING: {FLAG_ENV}=1 — development run-state fixture "
        f"routes are enabled under {FIXTURE_PREFIX}. Not for production.",
        flush=True,
    )
    return True
