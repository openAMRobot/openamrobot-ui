#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
import os
import csv
import time
import yaml

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import (
    PoseWithCovarianceStamped, PoseArray, PoseStamped, PoseWithCovariance
)
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from openamr_ui_msgs.msg import ArrayPoseStampedWithCovariance


class WayPointMover(Node):
    def __init__(self, navigator: BasicNavigator):
        super().__init__('Way_points_handler')
        self.navigator = navigator
        self.waypoints = []
        self.current_wp = 0
        self.on_the_route = False
        self.break_mission = False

        package_share_dir = get_package_share_directory('openamr_ui_package')
        self.current_files = os.path.join(
            package_share_dir, 'param/current_map_route.yaml'
        )

        self.current_pos = PoseWithCovariance()
        self.home_pose = PoseWithCovariance()

        # ReentrantCallbackGroup allows ui_operation_callback to run concurrently
        # with other callbacks so navigation threads don't starve the executor.
        cb_group = ReentrantCallbackGroup()

        self.uiopsub = self.create_subscription(
            String, "ui_operation", self.ui_operation_callback, 10,
            callback_group=cb_group,
        )
        self.odomsub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10,
        )

        self.ui_message_pub = self.create_publisher(String, 'ui_message', 1)
        self.poseArray_publisher = self.create_publisher(ArrayPoseStampedWithCovariance, "/WayPoints_topic", 1)

        self.get_logger().info("------------ Way points handler started ------------")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pub(self, text: str):
        msg = String()
        msg.data = text
        self.ui_message_pub.publish(msg)

    def get_cur_files(self):
        with open(self.current_files, 'r') as f:
            return yaml.safe_load(f) or {"map_file": "", "route_file": ""}

    def read_wp(self):
        route_file = os.path.expanduser(self.get_cur_files().get("route_file", ""))
        del self.waypoints[:]
        if route_file:
            if not os.path.isfile(route_file):
                self.get_logger().warn(f"Route file does not exist: {route_file}")
            else:
                with open(route_file, 'r') as f:
                    for row_number, line in enumerate(csv.reader(f, delimiter=','), start=1):
                        if len(line) < 11:
                            self.get_logger().warn(f"Skipping malformed waypoint row {row_number} in {route_file}")
                            continue
                        try:
                            values = [float(value) for value in line[:11]]
                        except ValueError:
                            self.get_logger().warn(f"Skipping non-numeric waypoint row {row_number} in {route_file}")
                            continue

                        pose = PoseStamped()
                        pose.header.frame_id = "map"
                        pose.pose.position.x = values[0]
                        pose.pose.position.y = values[1]
                        pose.pose.position.z = values[2]
                        pose.pose.orientation.x = values[3]
                        pose.pose.orientation.y = values[4]
                        pose.pose.orientation.z = values[5]
                        pose.pose.orientation.w = values[6]
                        point_type = values[7]
                        purpose = values[10]
                        self.waypoints.append((pose, point_type, purpose))

        if not self.waypoints:
            self._pub("The waypoint queue is empty.")

    # ------------------------------------------------------------------
    # Navigation primitives
    # ------------------------------------------------------------------

    def _send_pose(self, pose: PoseStamped) -> bool:
        """Send a NavigateToPose goal and block until complete. Returns True on success."""
        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            if self.break_mission:
                self.navigator.cancelTask()
                return False
            time.sleep(0.1)
        return self.navigator.getResult() == TaskResult.SUCCEEDED

    # ------------------------------------------------------------------
    # Route operations (run in daemon threads to avoid blocking the executor)
    # ------------------------------------------------------------------

    def _follow_impl(self):
        self.break_mission = False
        self.on_the_route = True
        self.current_wp = 0
        self.read_wp()

        for index, (pose, _, purpose) in enumerate(self.waypoints):
            if self.break_mission:
                self._pub("break_mission")
                break

            self._pub(f"Following to {index + 1} waypoint...")
            success = self._send_pose(pose)
            self.current_wp = index + 1

            if self.break_mission:
                break

            if success:
                self._pub(f"Doing some action #{purpose} on {index + 1} point")
            else:
                self._pub(f"Failed to reach waypoint {index + 1}")
                break

            time.sleep(0.1)

        time.sleep(1)
        self._pub("Current route was successfully completed")
        self.on_the_route = False

    def follow_func(self):
        if self.on_the_route:
            self._pub("The robot on the route already")
            return
        threading.Thread(target=self._follow_impl, daemon=True).start()

    def _next_prev_impl(self, direction: int):
        self.read_wp()
        if not self.waypoints:
            return

        self.current_wp = max(1, min(self.current_wp + direction, len(self.waypoints)))
        pose, _, purpose = self.waypoints[self.current_wp - 1]
        self._pub(f"Following to {self.current_wp} waypoint...")
        if self._send_pose(pose):
            self._pub(f"Point #{self.current_wp} successfully reached")
            self._pub(f"Doing some action #{purpose} on {self.current_wp} point")
        else:
            self._pub(f"Failed to reach waypoint {self.current_wp}")

    def next_wp_func(self):
        self.get_logger().info("Following to next waypoint...")
        self._pub("Following to next waypoint...")
        threading.Thread(target=self._next_prev_impl, args=(1,), daemon=True).start()

    def previous_wp_func(self):
        self.get_logger().info("Following to previous waypoint...")
        self._pub("Following to previous waypoint...")
        threading.Thread(target=self._next_prev_impl, args=(-1,), daemon=True).start()

    def home_func(self):
        self.get_logger().info("Following to home position")
        self._pub("Following to home position")
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position = self.home_pose.pose.position
        pose.pose.orientation = self.home_pose.pose.orientation
        threading.Thread(target=self._send_pose, args=(pose,), daemon=True).start()

    def stop_func(self):
        self.get_logger().info("Canceling current route")
        self._pub("Canceling current route")
        self.break_mission = True
        self.navigator.cancelTask()

    def function(self, typeFunc):
        self._pub(f"Function {typeFunc} will be done")

    # ------------------------------------------------------------------
    # Odometry
    # ------------------------------------------------------------------

    def odom_callback(self, data: Odometry):
        self.current_pos.pose = data.pose.pose

    # ------------------------------------------------------------------
    # UI command dispatcher
    # ------------------------------------------------------------------

    def ui_operation_callback(self, data: String):
        self.get_logger().info(f"ui_operation_callback: {data.data}")

        if data.data in ("follow_route", "start"):
            self.follow_func()
        elif data.data == "next_point":
            self.next_wp_func()
        elif data.data == "previous_point":
            self.previous_wp_func()
        elif data.data == "home":
            self.home_func()
        elif data.data == "stop":
            self.stop_func()
        else:
            parts = data.data.split("_", 1)
            if parts[0] == "function" and len(parts) == 2:
                self.function(parts[1])


def main():
    rclpy.init()
    navigator = BasicNavigator()
    controller = WayPointMover(navigator)

    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
