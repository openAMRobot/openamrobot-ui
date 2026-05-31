#!/usr/bin/env python3
"""
Relay TRANSIENT_LOCAL topics to /ui/* VOLATILE topics for rosbridge compatibility.
  /amcl_pose                         -> /ui/amcl_pose
  /navigate_to_pose/_action/status   -> /ui/navigate_to_pose/status
  /dock_robot/_action/status         -> /ui/dock_robot/status
  /undock_robot/_action/status       -> /ui/undock_robot/status
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatusArray


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

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose = msg
        self._pose_pub.publish(msg)

    def _status_cb(self, server: str):
        def cb(msg: GoalStatusArray) -> None:
            self._pubs[server].publish(msg)
        return cb

    def _republish_pose(self) -> None:
        if self._pose is not None:
            self._pose_pub.publish(self._pose)


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
