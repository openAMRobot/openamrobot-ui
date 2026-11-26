"""Plan limit checks (openamrobot-ui#35).

Pure test: imports voice_limits and PyYAML only, so it runs with
``python3 -m pytest test/test_voice_limits.py`` without ROS.
"""

import copy
import os
import sys

import pytest

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE_DIR)

from openamr_ui_package import voice_limits  # noqa: E402

LIMITS_FILE = os.path.join(PACKAGE_DIR, 'param', 'voice_limits.yaml')
LIMITS = voice_limits.load_limits(LIMITS_FILE)

OUT_OF_RANGE = [
    {'type': 'set_speed', 'linear': 0.21, 'angular': 0},
    {'type': 'set_speed', 'linear': -0.5, 'angular': 0},
    {'type': 'set_speed', 'linear': 0.1, 'angular': 2.1},
    {'type': 'drive_for', 'linear': 0.3, 'seconds': 1},
    {'type': 'drive_for', 'linear': 0.2, 'seconds': 31},
    {'type': 'drive_for', 'linear': 0.2, 'seconds': -1},
    {'type': 'rotate_for', 'angular': -2.5, 'seconds': 1},
    {'type': 'rotate_for', 'angular': 1, 'seconds': 30.5},
    {'type': 'wait', 'seconds': 31},
    {'type': 'battery_below', 'percent': 101, 'actions': []},
    {'type': 'repeat', 'times': 0, 'actions': []},
    {'type': 'repeat', 'times': 1.5, 'actions': []},
    {'type': 'wait_nav_complete', 'timeout': -1},
]

MALFORMED = [
    {'type': 'set_speed', 'linear': float('nan'), 'angular': 0},
    {'type': 'set_speed', 'linear': 0.1, 'angular': float('inf')},
    {'type': 'drive_for', 'linear': float('-inf'), 'seconds': 1},
    {'type': 'navigate', 'x': float('nan'), 'y': 0, 'yaw': 0},
    {'type': 'set_speed', 'linear': '0.1', 'angular': 0},
    {'type': 'set_speed', 'linear': True, 'angular': 0},
    {'type': 'drive_for', 'linear': 0.1},
    {'type': 'wait'},
]

WITHIN = [
    {'type': 'set_speed', 'linear': 0.2, 'angular': -2.0},
    {'type': 'drive_for', 'linear': -0.15, 'seconds': 30},
    {'type': 'rotate_for', 'angular': 2, 'seconds': 0},
    {'type': 'wait', 'seconds': 30},
    {'type': 'navigate', 'x': 12.0, 'y': -3.0, 'yaw': 1.57},
    {'type': 'wait_nav_complete', 'timeout': 120},
    {'type': 'stop_movement'},
    {'type': 'repeat', 'times': 3, 'actions': [{'type': 'wait', 'seconds': 1}]},
]


def test_package_limits_are_the_stated_values():
    assert LIMITS == {
        'max_linear_speed_mps': 0.2,
        'max_angular_speed_radps': 2.0,
        'max_distance_per_action_m': 5.0,
        'max_duration_per_action_s': 30.0,
    }


@pytest.mark.parametrize('action', OUT_OF_RANGE + MALFORMED)
def test_rejected_action_has_a_reason(action):
    (error,) = voice_limits.plan_errors([action], LIMITS)
    assert error['path'] == '0' and error['type'] == action['type'] and error['reasons']


@pytest.mark.parametrize('action', WITHIN)
def test_action_within_limits_passes(action):
    assert voice_limits.plan_errors([action], LIMITS) == []


def test_drive_distance_limit_is_speed_times_duration():
    tighter = dict(LIMITS, max_distance_per_action_m=1.0)
    (error,) = voice_limits.plan_errors(
        [{'type': 'drive_for', 'linear': 0.1, 'seconds': 11}], tighter)
    assert 'distance' in error['reasons'][0]


def test_nested_actions_are_checked_with_their_path():
    plan = [{'type': 'wait', 'seconds': 1},
            {'type': 'repeat', 'times': 2, 'actions': [
                {'type': 'wait', 'seconds': 1},
                {'type': 'set_speed', 'linear': 9, 'angular': 0}]}]
    (error,) = voice_limits.plan_errors(plan, LIMITS)
    assert error['path'] == '1.actions.1'


def test_values_are_rejected_not_clamped():
    plan = [{'type': 'set_speed', 'linear': 5, 'angular': 0}]
    before = copy.deepcopy(plan)
    assert voice_limits.plan_errors(plan, LIMITS)
    assert plan == before


@pytest.mark.parametrize('bad', ['0', '-1', '.nan', 'yes'])
def test_invalid_limits_file_is_refused(tmp_path, bad):
    path = tmp_path / 'voice_limits.yaml'
    path.write_text(
        'plan_limits:\n  max_linear_speed_mps: %s\n  max_angular_speed_radps: 2\n'
        '  max_distance_per_action_m: 5\n  max_duration_per_action_s: 30\n' % bad)
    with pytest.raises(ValueError, match='max_linear_speed_mps'):
        voice_limits.load_limits(str(path))
