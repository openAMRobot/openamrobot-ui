#!/usr/bin/env python3
"""
Relay TRANSIENT_LOCAL topics to /ui/* VOLATILE topics for rosbridge compatibility.
  /amcl_pose                         -> /ui/amcl_pose
  /navigate_to_pose/_action/status   -> /ui/navigate_to_pose/status
  /dock_robot/_action/status         -> /ui/dock_robot/status
  /undock_robot/_action/status       -> /ui/undock_robot/status

The pose is republished every second only while it is fresh: a pose has
arrived and AMCL's map to odom transform arrived within pose_stale_after_s
(see pose_freshness.py). After that the relay goes quiet, and the UI's own
/ui/amcl_pose timeouts show localization as stale.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatusArray
from openamr_ui_package.pose_freshness import DEFAULT_STALE_AFTER_S, is_map_to_odom, PoseFreshness
from tf2_msgs.msg import TFMessage


class NavRelays(Node):
    def __init__(self):
        super().__init__('nav_relays')

        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        vol_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._pose: PoseWithCovarianceStamped | None = None
        self.declare_parameter('pose_stale_after_s', DEFAULT_STALE_AFTER_S)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self._freshness = PoseFreshness(
            self.get_parameter('pose_stale_after_s').get_parameter_value().double_value)
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._was_fresh = False
        self._tf_sub = self.create_subscription(
            TFMessage, '/tf', self._tf_cb,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))

        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._pose_cb, tl_qos)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/ui/amcl_pose', vol_qos)

        self._status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', self._status_cb('/navigate_to_pose'), tl_qos)
        self._status_pub = self.create_publisher(
            GoalStatusArray, '/ui/navigate_to_pose/status', vol_qos)

        self._dock_status_sub = self.create_subscription(
            GoalStatusArray, '/dock_robot/_action/status', self._status_cb('/dock_robot'), tl_qos)
        self._dock_status_pub = self.create_publisher(
            GoalStatusArray, '/ui/dock_robot/status', vol_qos)

        self._undock_status_sub = self.create_subscription(
            GoalStatusArray, '/undock_robot/_action/status', self._status_cb('/undock_robot'), tl_qos)
        self._undock_status_pub = self.create_publisher(
            GoalStatusArray, '/ui/undock_robot/status', vol_qos)

        self._pubs = {
            '/navigate_to_pose': self._status_pub,
            '/dock_robot':       self._dock_status_pub,
            '/undock_robot':     self._undock_status_pub,
        }

        self._timer = self.create_timer(1.0, self._republish_pose)
        self.get_logger().info('nav_relays started: native topics -> /ui/* volatile topics')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        # A new AMCL pose is current by definition; forward it at once.
        self._pose = msg
        self._freshness.on_pose(self._now())
        self._pose_pub.publish(msg)

    def _tf_cb(self, msg: TFMessage) -> None:
        if any(is_map_to_odom(t, self._map_frame, self._odom_frame) for t in msg.transforms):
            self._freshness.on_map_odom(self._now())

    def _status_cb(self, server: str):
        def cb(msg: GoalStatusArray) -> None:
            self._pubs[server].publish(msg)
        return cb

    def _republish_pose(self) -> None:
        fresh = self._pose is not None and self._freshness.is_fresh(self._now())
        if fresh:
            self._pose_pub.publish(self._pose)
        elif self._was_fresh:
            self.get_logger().warn(
                'No map to odom transform for %.1f s; no longer republishing the AMCL pose.'
                % self._freshness.stale_after_s)
        self._was_fresh = fresh


def main() -> None:
    rclpy.init()
    node = NavRelays()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
