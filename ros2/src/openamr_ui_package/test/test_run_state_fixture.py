"""Pure (no-ROS) tests for the development run-state fixture.

Run from ros2/src/openamr_ui_package:
    python3 -m pip install -r test/requirements-run-state-fixture.txt
    python3 -m pytest test/test_run_state_fixture.py
"""

import ast
import os
import subprocess
import sys
import types

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.abspath(os.path.join(PKG_ROOT, "..", "..", ".."))
MODULE_PATH = os.path.join(PKG_ROOT, "openamr_ui_package", "run_state_fixture.py")

flask = pytest.importorskip("flask")  # declared in requirements-run-state-fixture.txt

from openamr_ui_package import run_state_fixture as fx  # noqa: E402


class ManualClock:
    def __init__(self):
        self.ms = 1_000_000

    def __call__(self):
        return self.ms


def make_app(environ, get_source=None):
    app = flask.Flask("fixture_test")
    app.config.update(TESTING=True)
    enabled = fx.register_run_state_fixture(app, environ=environ, get_source=get_source)
    return app, enabled


def enabled_client(scenario="success"):
    clock = ManualClock()
    source = fx.FixtureRunStateSource(scenario, clock_ms=clock)
    app, enabled = make_app({fx.FLAG_ENV: "1"}, get_source=lambda: source)
    assert enabled is True
    return app.test_client(), source, clock


# ── Flag / 404 / read-only ────────────────────────────────────────────────


@pytest.mark.parametrize("environ", [{}, {fx.FLAG_ENV: "0"}, {fx.FLAG_ENV: ""},
                                     {fx.FLAG_ENV: "true"}, {fx.FLAG_ENV: "yes"}])
def test_routes_404_when_flag_absent_or_off(environ):
    app, enabled = make_app(environ)
    assert enabled is False

    # Stand-in for flask_app.py's SPA catch-all: without the guard these
    # paths would fall through to it and return 200.
    @app.route("/<path:path>")
    def spa(path):
        return "index.html", 200

    client = app.test_client()
    for path in ("/snapshot", "/events?epoch=x&after_seq=0", "", "/anything"):
        for method in ("get", "post", "delete"):
            response = getattr(client, method)(fx.FIXTURE_PREFIX + path)
            assert response.status_code == 404, (method, path)
    assert client.get("/some/spa/route").status_code == 200


def test_enabled_routes_are_get_only():
    client, _, _ = enabled_client()
    assert client.get(f"{fx.FIXTURE_PREFIX}/snapshot").status_code == 200
    for method in ("post", "put", "delete", "patch"):
        assert getattr(client, method)(f"{fx.FIXTURE_PREFIX}/snapshot").status_code == 405
        assert getattr(client, method)(f"{fx.FIXTURE_PREFIX}/events").status_code == 405
    # No scenario-selection / advancement route exists.
    for path in ("/advance", "/scenario", "/start", "/reset"):
        assert client.post(fx.FIXTURE_PREFIX + path).status_code in (404, 405)
        assert client.get(fx.FIXTURE_PREFIX + path).status_code == 404


def test_reads_do_not_advance_the_scenario():
    client, source, _ = enabled_client()
    first = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    for _ in range(5):
        client.get(f"{fx.FIXTURE_PREFIX}/snapshot")
        client.get(f"{fx.FIXTURE_PREFIX}/events?epoch={source.epoch}&after_seq=0")
    assert client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json() == first


# ── Envelope and scenarios ────────────────────────────────────────────────


def test_initial_snapshot_keeps_unknowns_unknown():
    client, _, _ = enabled_client()
    snap = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    assert snap["schema_version"] == fx.SCHEMA_VERSION
    assert snap["fixture"] is True
    assert snap["producer"]["epoch"] == fx.EPOCH_A
    assert snap["producer"]["timebase"] == fx.TIMEBASE
    assert snap["run"] is None
    assert snap["operational_state"] is None
    assert snap["allowed_operations"] is None  # unknown, not "nothing allowed"
    assert snap["as_of_seq"] == 0


def test_success_scenario_ready_executing_then_separate_outcome():
    client, source, clock = enabled_client("success")
    source.advance()
    snap = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    assert snap["operational_state"] == "READY"
    assert snap["run"]["outcome"] is None

    clock.ms += 1500
    source.advance()
    snap = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    assert snap["operational_state"] == "EXECUTING"
    assert snap["run"]["step"]["index"] == 0
    assert snap["run"]["started_at_ms"] == 1500

    clock.ms += 2000
    source.advance_to(len(fx.SCENARIOS["success"]))
    snap = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    assert snap["operational_state"] == "READY"
    assert snap["operational_state"] not in fx.TERMINAL_RESULTS
    assert snap["run"]["outcome"] == {"result": "SUCCEEDED", "reason": None}
    assert snap["run"]["ended_at_ms"] == 3500
    assert snap["allowed_operations"] == []


def test_failed_scenario_has_typed_provisional_reason():
    client, source, _ = enabled_client("failed")
    source.advance_to(len(fx.SCENARIOS["failed"]))
    snap = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    assert snap["run"]["outcome"]["result"] == "FAILED"
    assert snap["run"]["outcome"]["reason"] == {
        "registry": "FIXTURE_PROVISIONAL",
        "code": "FIXTURE_PROVISIONAL.STEP_FAILED",
    }
    assert snap["operational_state"] == "NEEDS_ASSISTANCE"
    assert snap["operational_state"] in fx.OPERATIONAL_STATES


def test_every_scenario_uses_only_proposed_vocabulary():
    for name, steps in fx.SCENARIOS.items():
        source = fx.FixtureRunStateSource(name, clock_ms=ManualClock())
        for _ in steps:
            source.advance()
            snap = source.snapshot()
            assert snap["operational_state"] in (None,) + fx.OPERATIONAL_STATES
            outcome = (snap["run"] or {}).get("outcome")
            if outcome:
                assert outcome["result"] in fx.TERMINAL_RESULTS


def test_events_have_run_epoch_seq_identity_and_order():
    client, source, _ = enabled_client("new_run")
    source.advance_to(len(fx.SCENARIOS["new_run"]))
    body = client.get(
        f"{fx.FIXTURE_PREFIX}/events?epoch={fx.EPOCH_A}&after_seq=0"
    ).get_json()
    seqs = [e["seq"] for e in body["events"]]
    assert seqs == list(range(1, len(seqs) + 1))
    assert {e["epoch"] for e in body["events"]} == {fx.EPOCH_A}
    assert [e["run_id"] for e in body["events"]] == [
        "fixture-run-0004", "fixture-run-0004", "fixture-run-0004",
        "fixture-run-0005", "fixture-run-0005",
    ]
    later = client.get(
        f"{fx.FIXTURE_PREFIX}/events?epoch={fx.EPOCH_A}&after_seq=3"
    ).get_json()
    assert [e["seq"] for e in later["events"]] == [4, 5]


def test_producer_restart_changes_epoch_and_rejects_old_epoch_cursor():
    client, source, _ = enabled_client("producer_restart")
    source.advance_to(2)
    before = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    source.advance()  # restart
    after = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
    assert before["producer"]["epoch"] == fx.EPOCH_A
    assert after["producer"]["epoch"] == fx.EPOCH_B
    # Epochs are identities, not sortable clocks: the new one sorts lower.
    assert fx.EPOCH_B < fx.EPOCH_A
    assert after["run"]["run_id"] == before["run"]["run_id"]
    assert after["operational_state"] is None
    assert after["allowed_operations"] is None
    assert after["as_of_seq"] == 0
    stale = client.get(f"{fx.FIXTURE_PREFIX}/events?epoch={fx.EPOCH_A}&after_seq=2")
    assert stale.status_code == 409


def test_stale_scenario_stops_heartbeat_and_producer_time():
    source = fx.FixtureRunStateSource("stale", clock_ms=ManualClock())
    source.advance_to(2)
    source.heartbeat()
    assert source.snapshot()["producer"]["heartbeat_seq"] == 1
    source.advance()  # silence
    frozen = source.snapshot()["producer"]
    source.heartbeat()
    assert source.snapshot()["producer"] == frozen


def test_clock_driven_harness_is_time_deterministic():
    now = [100.0]
    harness = fx.ClockDrivenHarness("success", step_seconds=2, clock_s=lambda: now[0])
    assert harness.current().snapshot()["as_of_seq"] == 0
    now[0] += 4.1
    assert harness.current().snapshot()["as_of_seq"] == 2
    now[0] += 100
    assert harness.current().snapshot()["run"]["outcome"]["result"] == "SUCCEEDED"


def test_unknown_scenario_is_rejected():
    with pytest.raises(ValueError):
        fx.FixtureRunStateSource("no-such-scenario")


# ── Default Compose leaves the flag unset; override is the only opt-in ─────


def test_default_compose_leaves_fixture_flag_unset():
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(REPO_ROOT, "docker-compose.yml"), encoding="utf-8") as f:
        default = yaml.safe_load(f)
    for service in default["services"].values():
        env = service.get("environment") or {}
        if isinstance(env, list):
            env = dict(item.split("=", 1) if "=" in item else (item, None) for item in env)
        assert fx.FLAG_ENV not in env
        assert fx.SCENARIO_ENV not in env
    with open(os.path.join(REPO_ROOT, "docker-compose.run-state-fixture.yml"),
              encoding="utf-8") as f:
        override = yaml.safe_load(f)
    assert override["services"]["openamr-ui"]["environment"][fx.FLAG_ENV] == "1"


# ── Isolation: static import allowlist + runtime fakes ────────────────────

ALLOWED_IMPORTS = {"copy", "os", "threading", "time", "flask"}


def test_static_import_allowlist():
    tree = ast.parse(open(MODULE_PATH, encoding="utf-8").read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.add((node.module or "").split(".")[0])
        elif isinstance(node, ast.Call) and getattr(node.func, "id", "") in (
            "__import__", "exec", "eval",
        ):
            pytest.fail("dynamic import/exec in fixture module")
    assert imported <= ALLOWED_IMPORTS, imported - ALLOWED_IMPORTS


def test_import_does_not_load_ros_modules():
    code = (
        "import sys; import openamr_ui_package.run_state_fixture; "
        "bad=[m for m in sys.modules if m.split('.')[0] in "
        "('rclpy','roslibpy','ament_index_python','xacro','serial')]; "
        "assert not bad, bad"
    )
    subprocess.run([sys.executable, "-c", code], cwd=PKG_ROOT, check=True)


class _Forbidden(types.ModuleType):
    def __getattr__(self, name):
        raise AssertionError(f"fixture touched forbidden module {self.__name__}.{name}")


def test_runtime_fakes_fail_on_any_command_attempt(monkeypatch):
    for name in ("rclpy", "rclpy.node", "roslibpy", "ament_index_python", "serial"):
        monkeypatch.setitem(sys.modules, name, _Forbidden(name))

    def forbid(*args, **kwargs):
        raise AssertionError("fixture attempted a process/network side effect")

    import socket
    import urllib.request

    monkeypatch.setattr(subprocess, "Popen", forbid)
    monkeypatch.setattr(subprocess, "run", forbid)
    monkeypatch.setattr(os, "system", forbid)
    monkeypatch.setattr(urllib.request, "urlopen", forbid)
    monkeypatch.setattr(socket.socket, "connect", forbid)

    for name, steps in fx.SCENARIOS.items():
        client, source, _ = enabled_client(name)
        for _ in range(len(steps) + 2):
            source.advance()
            source.heartbeat()
            snap = client.get(f"{fx.FIXTURE_PREFIX}/snapshot").get_json()
            client.get(
                f"{fx.FIXTURE_PREFIX}/events?epoch={snap['producer']['epoch']}&after_seq=0"
            )
    harness = fx.ClockDrivenHarness("success", step_seconds=0.1)
    harness.current()


def test_runtime_fake_actually_detects_a_forbidden_call(monkeypatch):
    """Negative control: the fake above must fail if code does touch ROS."""
    monkeypatch.setitem(sys.modules, "rclpy", _Forbidden("rclpy"))
    import importlib

    with pytest.raises(AssertionError, match="forbidden module rclpy.init"):
        importlib.import_module("rclpy").init()
