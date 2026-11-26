"""When the relayed AMCL pose may still be shown as current (openamrobot-ui#36).

AMCL publishes /amcl_pose only when its filter updates, which needs the robot
to move, so a stationary robot's last pose message can be minutes old and
still correct. Message age alone therefore cannot mark a pose stale. While
AMCL is running it keeps broadcasting the map to odom transform on every
laser scan, so the age of the last map to odom transform received is the
liveness signal used here.

Imports nothing from ROS, so it is tested with plain ``python3 -m pytest``.
"""

import math
import time

# Seconds without a map to odom transform before the pose is stale.
# Derived: twice Nav2 AMCL's default transform_tolerance (1.0 s), the time by
# which AMCL post-dates each map to odom transform. A laser of 1 Hz or faster
# refreshes the transform well inside this. Accepted default (30 Oct 2026);
# overridden by the pose_stale_after_s parameter.
DEFAULT_STALE_AFTER_S = 2.0


def require_finite_positive(name, value):
    """Return value as a float, or raise ValueError unless it is finite and above 0."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a number, got {value!r}')
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and greater than 0, got {value!r}')
    return float(value)


class PoseFreshness:
    """Track when the last pose and map to odom transform were received.

    Receive times come from a monotonic clock (``time.monotonic`` unless a
    clock is injected for tests), never from ROS time, so a ROS clock jump
    cannot make an old pose look new. An age that is negative or not finite
    is never fresh.
    """

    def __init__(self, stale_after_s=DEFAULT_STALE_AFTER_S, clock=time.monotonic):
        self.stale_after_s = require_finite_positive('stale_after_s', stale_after_s)
        self._clock = clock
        self.reset()

    def reset(self):
        """Clear the pose and transform receive times (after a clock reset)."""
        self.last_pose_at = None
        self.last_map_odom_at = None

    def on_pose(self):
        """Record that a pose message arrived now."""
        self.last_pose_at = self._clock()

    def on_map_odom(self):
        """Record that a map to odom transform arrived now."""
        self.last_map_odom_at = self._clock()

    def is_fresh(self):
        """Return True when a pose exists and map to odom arrived within the threshold."""
        if self.last_pose_at is None or self.last_map_odom_at is None:
            return False
        age = self._clock() - self.last_map_odom_at
        return math.isfinite(age) and 0 <= age <= self.stale_after_s


def is_map_to_odom(transform, map_frame='map', odom_frame='odom'):
    """Return True for a TransformStamped from map_frame to odom_frame."""
    parent = transform.header.frame_id.lstrip('/')
    child = transform.child_frame_id.lstrip('/')
    return parent == map_frame and child == odom_frame
