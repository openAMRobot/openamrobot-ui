"""Route and pose-freshness rules (openamrobot-ui#36).

Pure test: imports route_outcomes and pose_freshness only, so it runs with
``python3 -m pytest test/test_route_outcomes.py`` without ROS.
"""

import enum
import os
import sys

import pytest

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE_DIR)

from openamr_ui_package import pose_freshness, route_outcomes  # noqa: E402, I100
from openamr_ui_package.route_outcomes import (  # noqa: E402
    ABORTED, CANCELED, SUCCEEDED, TIMEOUT, UNKNOWN)


class FakeTaskResult(enum.Enum):
    """Same members as nav2_simple_commander's TaskResult (Jazzy)."""

    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


def test_route_succeeds_only_if_every_waypoint_succeeded():
    assert route_outcomes.route_outcome([SUCCEEDED] * 3, 3)[0] == SUCCEEDED


@pytest.mark.parametrize('failed', [ABORTED, CANCELED, TIMEOUT, UNKNOWN])
def test_any_failed_waypoint_fails_the_route(failed):
    outcome, message = route_outcomes.route_outcome([SUCCEEDED, failed], 3)
    assert outcome == failed
    assert message == f'Route {failed} at waypoint 2 of 3; 1 waypoint(s) reached.'
    assert 'SUCCEEDED' not in message


def test_route_stopped_between_waypoints_is_canceled():
    outcome, message = route_outcomes.route_outcome([SUCCEEDED], 3)
    assert outcome == CANCELED and 'before waypoint 2 of 3' in message


def test_empty_route_is_not_a_success():
    assert route_outcomes.route_outcome([], 0)[0] == UNKNOWN


@pytest.mark.parametrize('result,expected', [
    (FakeTaskResult.SUCCEEDED, SUCCEEDED), (FakeTaskResult.FAILED, ABORTED),
    (FakeTaskResult.CANCELED, CANCELED),
    (FakeTaskResult.UNKNOWN, UNKNOWN), (None, UNKNOWN)])
def test_nav2_result_maps_to_outcome(result, expected):
    assert route_outcomes.outcome_from_task_result(result) == expected


class FakeClock:
    """A monotonic clock the test sets by hand; it may even go backwards."""

    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


def _freshness(now=0.0, stale_after_s=pose_freshness.DEFAULT_STALE_AFTER_S):
    clock = FakeClock(now)
    return pose_freshness.PoseFreshness(stale_after_s, clock=clock), clock


def test_stationary_robot_stays_fresh_while_amcl_transform_arrives():
    freshness, clock = _freshness()
    freshness.on_pose()
    # No new pose for 10 minutes: AMCL does not publish while the robot
    # stands still, but its map to odom transform keeps arriving.
    for tick in range(6001):
        clock.now = tick * 0.1
        freshness.on_map_odom()
    assert freshness.is_fresh()


def test_pose_goes_stale_when_the_transform_stops():
    freshness, clock = _freshness(10.0)
    freshness.on_pose()
    freshness.on_map_odom()
    clock.now = 10.0 + pose_freshness.DEFAULT_STALE_AFTER_S
    assert freshness.is_fresh()
    clock.now += 0.01
    assert not freshness.is_fresh()


def test_reviewer_sequence_stays_stale_when_the_clock_goes_back():
    freshness, clock = _freshness(100.0)
    freshness.on_pose()
    freshness.on_map_odom()
    for now, fresh in ((100.0, True), (103.0, False), (0.0, False), (50.0, False)):
        clock.now = now
        assert freshness.is_fresh() is fresh, now


@pytest.mark.parametrize('now', [4.999, float('nan'), float('inf'), float('-inf')])
def test_negative_or_non_finite_age_is_never_fresh(now):
    freshness, clock = _freshness(5.0)
    freshness.on_pose()
    freshness.on_map_odom()
    clock.now = now  # 4.999 gives an age of -0.001 s
    assert not freshness.is_fresh()


def test_after_reset_a_new_transform_alone_does_not_revive_the_pose():
    freshness, clock = _freshness(100.0)
    freshness.on_pose()
    freshness.on_map_odom()
    freshness.reset()
    clock.now = 100.5
    freshness.on_map_odom()
    assert not freshness.is_fresh()


def test_after_reset_a_new_pose_and_transform_are_fresh():
    freshness, clock = _freshness(100.0)
    freshness.on_pose()
    freshness.on_map_odom()
    freshness.reset()
    clock.now = 100.5
    freshness.on_pose()
    freshness.on_map_odom()
    assert freshness.is_fresh()


def test_no_pose_or_no_transform_is_not_fresh():
    only_tf, _ = _freshness(1.0)
    only_tf.on_map_odom()
    assert not only_tf.is_fresh()
    only_pose, _ = _freshness(1.0)
    only_pose.on_pose()
    assert not only_pose.is_fresh()


def test_default_clock_is_monotonic():
    import time
    assert pose_freshness.PoseFreshness()._clock is time.monotonic


def test_default_threshold_is_twice_the_nav2_transform_tolerance():
    nav2_amcl_default_transform_tolerance_s = 1.0
    assert pose_freshness.DEFAULT_STALE_AFTER_S == 2 * nav2_amcl_default_transform_tolerance_s


@pytest.mark.parametrize('bad', [0, -1, float('nan'), float('inf'), '2', True, None])
def test_invalid_threshold_is_refused(bad):
    with pytest.raises(ValueError):
        pose_freshness.PoseFreshness(bad)
    with pytest.raises(ValueError):
        pose_freshness.require_finite_positive('waypoint_timeout_s', bad)
