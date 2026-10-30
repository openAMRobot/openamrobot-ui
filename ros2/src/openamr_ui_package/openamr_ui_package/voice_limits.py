"""Server-side limits for robot action plans (openamrobot-ui#35).

The backend checks every plan it returns from /api/voice-plan against the
limits in param/voice_limits.yaml and rejects, never clamps, a plan with a
non-finite number or a value above a limit. Imports nothing from ROS or
Flask, so it is tested with plain ``python3 -m pytest``.
"""

import math

import yaml

LIMIT_KEYS = (
    'max_linear_speed_mps',
    'max_angular_speed_radps',
    'max_distance_per_action_m',
    'max_duration_per_action_s',
)

# Numeric fields each action type must carry.
NUMERIC_FIELDS = {
    'navigate': ('x', 'y', 'yaw'),
    'wait': ('seconds',),
    'set_speed': ('linear', 'angular'),
    'drive_for': ('linear', 'seconds'),
    'rotate_for': ('angular', 'seconds'),
    'wait_nav_complete': ('timeout',),
    'battery_below': ('percent',),
    'repeat': ('times',),
}


def load_limits(path):
    """Read plan limits; every limit must be a finite positive number."""
    with open(path, encoding='utf-8') as config_file:
        data = (yaml.safe_load(config_file) or {}).get('plan_limits') or {}
    limits = {}
    for key in LIMIT_KEYS:
        value = data.get(key)
        if not _is_number(value) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{path}: plan_limits.{key} must be a finite positive number')
        limits[key] = float(value)
    return limits


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _action_reasons(action, limits):
    """Return the reasons one action (not its nested actions) is rejected."""
    kind = action.get('type')
    reasons = []
    values = {}
    for field in NUMERIC_FIELDS.get(kind, ()):
        value = action.get(field)
        if not _is_number(value):
            reasons.append(f'{field} must be a number')
        elif not math.isfinite(value):
            reasons.append(f'{field} must be finite')
        else:
            values[field] = value
    if reasons:
        return reasons

    max_linear = limits['max_linear_speed_mps']
    max_angular = limits['max_angular_speed_radps']
    max_distance = limits['max_distance_per_action_m']
    max_duration = limits['max_duration_per_action_s']

    if 'linear' in values and abs(values['linear']) > max_linear:
        reasons.append(f"linear speed {values['linear']} m/s is above the {max_linear} m/s limit")
    if 'angular' in values and abs(values['angular']) > max_angular:
        reasons.append(
            f"angular speed {values['angular']} rad/s is above the {max_angular} rad/s limit")
    if kind in ('wait', 'drive_for', 'rotate_for'):
        seconds = values['seconds']
        if seconds < 0:
            reasons.append('seconds must not be negative')
        elif seconds > max_duration:
            reasons.append(f'duration {seconds} s is above the {max_duration} s limit')
    if kind == 'drive_for':
        distance = abs(values['linear']) * abs(values['seconds'])
        if distance > max_distance:
            reasons.append(
                f'distance {distance:g} m (speed x duration) is above the {max_distance} m limit')
    if kind == 'wait_nav_complete' and values['timeout'] < 0:
        reasons.append('timeout must not be negative')
    if kind == 'battery_below' and not 0 <= values['percent'] <= 100:
        reasons.append('percent must be between 0 and 100')
    if kind == 'repeat' and (values['times'] != int(values['times']) or values['times'] < 1):
        reasons.append('times must be a whole number of at least 1')
    return reasons


def plan_errors(actions, limits, prefix=''):
    """Return one error per rejected action: its path, type and reasons."""
    errors = []
    for index, action in enumerate(actions):
        path = f'{prefix}{index}'
        reasons = _action_reasons(action, limits)
        if reasons:
            errors.append({'path': path, 'type': action.get('type'), 'reasons': reasons})
        nested = action.get('actions')
        if action.get('type') in ('repeat', 'battery_below') and isinstance(nested, list):
            errors.extend(plan_errors(nested, limits, prefix=f'{path}.actions.'))
    return errors
