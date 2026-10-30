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

from openamr_ui_package import pose_freshness, route_outcomes  # noqa: E402
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


def _tf_every(freshness, start, stop, period=0.1):
    t = start
    while t <= stop:
        freshness.on_map_odom(t)
        t += period


def test_stationary_robot_stays_fresh_while_amcl_transform_arrives():
    freshness = pose_freshness.PoseFreshness()
    freshness.on_pose(0.0)
    # No new pose for 10 minutes: AMCL does not publish while the robot
    # stands still, but its map to odom transform keeps arriving.
    _tf_every(freshness, 0.0, 600.0)
    assert freshness.is_fresh(600.0)


def test_pose_goes_stale_when_the_transform_stops():
    freshness = pose_freshness.PoseFreshness()
    freshness.on_pose(0.0)
    _tf_every(freshness, 0.0, 10.0)
    last = freshness.last_map_odom_at
    assert freshness.is_fresh(last + pose_freshness.DEFAULT_STALE_AFTER_S)
    assert not freshness.is_fresh(last + pose_freshness.DEFAULT_STALE_AFTER_S + 0.01)


def test_no_pose_or_no_transform_is_not_fresh():
    only_tf = pose_freshness.PoseFreshness()
    only_tf.on_map_odom(1.0)
    assert not only_tf.is_fresh(1.0)
    only_pose = pose_freshness.PoseFreshness()
    only_pose.on_pose(1.0)
    assert not only_pose.is_fresh(1.0)


def test_default_threshold_is_twice_the_nav2_transform_tolerance():
    nav2_amcl_default_transform_tolerance_s = 1.0
    assert pose_freshness.DEFAULT_STALE_AFTER_S == 2 * nav2_amcl_default_transform_tolerance_s


def test_threshold_must_be_positive():
    with pytest.raises(ValueError):
        pose_freshness.PoseFreshness(0)
