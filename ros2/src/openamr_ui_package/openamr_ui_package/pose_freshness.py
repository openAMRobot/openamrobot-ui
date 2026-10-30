"""When the relayed AMCL pose may still be shown as current (openamrobot-ui#36).

AMCL publishes /amcl_pose only when its filter updates, which needs the robot
to move, so a stationary robot's last pose message can be minutes old and
still correct. Message age alone therefore cannot mark a pose stale. While
AMCL is running it keeps broadcasting the map to odom transform on every
laser scan, so the age of the last map to odom transform received is the
liveness signal used here.

Imports nothing from ROS, so it is tested with plain ``python3 -m pytest``.
"""

# Seconds without a map to odom transform before the pose is stale.
# Derived: twice Nav2 AMCL's default transform_tolerance (1.0 s), the time by
# which AMCL post-dates each map to odom transform. A laser of 1 Hz or faster
# refreshes the transform well inside this. Accepted default (30 Oct 2026);
# overridden by the pose_stale_after_s parameter.
DEFAULT_STALE_AFTER_S = 2.0


class PoseFreshness:
    """Track the last pose and the last map to odom transform, by receive time."""

    def __init__(self, stale_after_s=DEFAULT_STALE_AFTER_S):
        if not stale_after_s > 0:
            raise ValueError('stale_after_s must be positive')
        self.stale_after_s = stale_after_s
        self.last_pose_at = None
        self.last_map_odom_at = None

    def on_pose(self, now):
        """Record that a pose message arrived at ``now`` (seconds)."""
        self.last_pose_at = now

    def on_map_odom(self, now):
        """Record that a map to odom transform arrived at ``now`` (seconds)."""
        self.last_map_odom_at = now

    def is_fresh(self, now):
        """True when a pose exists and map to odom arrived within the threshold."""
        if self.last_pose_at is None or self.last_map_odom_at is None:
            return False
        return now - self.last_map_odom_at <= self.stale_after_s


def is_map_to_odom(transform, map_frame='map', odom_frame='odom'):
    """True for a TransformStamped from map_frame to odom_frame."""
    parent = transform.header.frame_id.lstrip('/')
    child = transform.child_frame_id.lstrip('/')
    return parent == map_frame and child == odom_frame
