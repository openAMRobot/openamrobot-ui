"""waypoint_nav reports real route outcomes; nav_relays stops relaying stale poses.

Needs rclpy, nav2_simple_commander and tf2_msgs, so it runs under colcon
test. The pure rules are tested in test_route_outcomes.py.
"""

import unittest
from unittest import mock


class WaypointNavOutcomeTest(unittest.TestCase):

    def setUp(self):
        import rclpy
        from rclpy.parameter import Parameter
        from nav2_simple_commander.robot_navigator import TaskResult
        from geometry_msgs.msg import PoseStamped
        import openamr_ui_package.waypoint_nav as waypoint_nav

        self.TaskResult = TaskResult
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.navigator = mock.Mock()
        self.navigator.goToPose.return_value = True
        self.navigator.isTaskComplete.return_value = True
        self.node = waypoint_nav.WayPointMover(self.navigator)
        self.addCleanup(self.node.destroy_node)
        self.node.set_parameters([Parameter('waypoint_timeout_s', value=0.3)])
        self.node.waypoint_timeout_s = 0.3
        self.messages = []
        self.node.ui_message_pub = mock.Mock(
            publish=lambda msg: self.messages.append(msg.data))
        poses = [(PoseStamped(), 0.0, 1.0) for _ in range(3)]

        def read_wp():
            self.node.waypoints[:] = poses
        self.node.read_wp = read_wp

    def _run(self, *results):
        self.navigator.getResult.side_effect = list(results)
        self.node._follow_impl()
        return self.messages[-1]

    def test_all_waypoints_succeeded(self):
        last = self._run(*[self.TaskResult.SUCCEEDED] * 3)
        self.assertEqual(last, 'Route SUCCEEDED: all 3 waypoints reached.')
        self.assertEqual(self.node.last_route_outcome, 'SUCCEEDED')

    def test_aborted_waypoint_fails_the_route(self):
        # BasicNavigator reports a Nav2 ABORTED goal as TaskResult.FAILED.
        last = self._run(self.TaskResult.SUCCEEDED, self.TaskResult.FAILED)
        self.assertEqual(last, 'Route ABORTED at waypoint 2 of 3; 1 waypoint(s) reached.')
        self.assertIn('Waypoint 2: ABORTED', self.messages)
        self.assertFalse(any('successfully completed' in m for m in self.messages))
        self.assertEqual(self.navigator.goToPose.call_count, 2)

    def test_canceled_waypoint_cancels_the_route(self):
        last = self._run(self.TaskResult.CANCELED)
        self.assertEqual(last, 'Route CANCELED at waypoint 1 of 3; 0 waypoint(s) reached.')

    def test_stop_during_a_waypoint_cancels_the_route(self):
        self.navigator.isTaskComplete.return_value = False

        def stop_after_goal(_pose):
            self.node.break_mission = True
            return True
        self.navigator.goToPose.side_effect = stop_after_goal
        self.node._follow_impl()
        self.navigator.cancelTask.assert_called()
        self.assertEqual(self.node.last_route_outcome, 'CANCELED')

    def test_waypoint_past_its_timeout_is_canceled_and_reported(self):
        self.navigator.isTaskComplete.return_value = False
        self.node._follow_impl()
        self.navigator.cancelTask.assert_called_once()
        self.assertEqual(self.messages[-1],
                         'Route TIMEOUT at waypoint 1 of 3; 0 waypoint(s) reached.')

    def test_rejected_goal_is_aborted(self):
        self.navigator.goToPose.return_value = False
        self.node._follow_impl()
        self.assertEqual(self.node.last_route_outcome, 'ABORTED')


class NavRelayFreshnessTest(unittest.TestCase):

    def setUp(self):
        import rclpy
        import openamr_ui_package.nav_relays as nav_relays

        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        # Two clocks: the monotonic clock that receive ages use, and the ROS
        # clock that is watched for backward jumps.
        self.now = 100.0
        self.ros = 1000.0
        self.node = nav_relays.NavRelays(monotonic=lambda: self.now)
        self.addCleanup(self.node.destroy_node)
        self.node._ros_now = lambda: self.ros
        self.published = []
        self.node._pose_pub = mock.Mock(publish=self.published.append)

    def _tf(self, parent='map', child='odom'):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        self.node._tf_cb(TFMessage(transforms=[transform]))

    def _pose(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.node._pose_cb(PoseWithCovarianceStamped())

    def test_stationary_pose_is_republished_while_map_to_odom_arrives(self):
        self._pose()
        for _ in range(600):  # 60 s of 10 Hz transforms, no new pose.
            self.now += 0.1
            self._tf()
        self.node._republish_pose()
        self.assertEqual(len(self.published), 2)

    def test_pose_is_not_republished_past_the_threshold(self):
        self._pose()
        self._tf()
        self.now += 2.0
        self.node._republish_pose()
        self.now += 0.1
        self.node._republish_pose()
        self.assertEqual(len(self.published), 2)  # The forward plus one republish.

    def test_other_transforms_do_not_keep_the_pose_fresh(self):
        self._pose()
        self._tf('odom', 'base_link')
        self.node._republish_pose()
        self.assertEqual(len(self.published), 1)

    def test_threshold_is_a_parameter(self):
        self.assertEqual(self.node.get_parameter('pose_stale_after_s').value, 2.0)

    def test_ros_clock_reset_drops_the_pose_and_new_tf_alone_does_not_revive_it(self):
        self._pose()
        self._tf()
        self.node._republish_pose()
        self.assertEqual(len(self.published), 2)
        self.ros = 0.0  # Simulation reset: the ROS clock jumps back.
        self.now += 0.5
        self._tf()
        self.node._republish_pose()
        self.assertEqual(len(self.published), 2)
        self.assertIsNone(self.node._pose)

    def test_after_a_reset_a_new_pose_and_tf_are_relayed_again(self):
        self._pose()
        self._tf()
        self.ros = 0.0
        self.now += 0.5
        self._pose()
        self._tf()
        self.node._republish_pose()
        self.assertEqual(len(self.published), 3)  # Two forwards plus one republish.

    def test_ros_time_moving_forward_keeps_the_pose(self):
        self._pose()
        self._tf()
        self.ros += 1.0
        self.now += 1.0
        self.node._republish_pose()
        self.assertEqual(len(self.published), 2)


class InvalidConfigurationTest(unittest.TestCase):
    """Non-finite or non-positive thresholds stop the node from starting."""

    def setUp(self):
        import rclpy

        rclpy.init()
        self.addCleanup(rclpy.shutdown)

    def test_nav_relays_refuses_an_invalid_stale_threshold(self):
        import openamr_ui_package.nav_relays as nav_relays

        for value in (0.0, -2.0, float('nan'), float('inf')):
            with self.subTest(value=value), mock.patch.object(
                    nav_relays.NavRelays, 'get_logger') as logger:
                with mock.patch('rclpy.node.Node.__init__', _with_overrides(
                        'pose_stale_after_s', value)):
                    with self.assertRaises(ValueError):
                        nav_relays.NavRelays()
                logger.return_value.error.assert_called_once()

    def test_waypoint_nav_refuses_an_invalid_timeout(self):
        import openamr_ui_package.waypoint_nav as waypoint_nav

        for value in (0.0, -1.0, float('nan'), float('inf')):
            with self.subTest(value=value), mock.patch.object(
                    waypoint_nav.WayPointMover, 'get_logger') as logger:
                with mock.patch('rclpy.node.Node.__init__', _with_overrides(
                        'waypoint_timeout_s', value)):
                    with self.assertRaises(ValueError):
                        waypoint_nav.WayPointMover(mock.Mock())
                logger.return_value.error.assert_called_once()


def _with_overrides(name, value):
    """Wrap Node.__init__ so the node starts with one parameter override."""
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    real_init = Node.__init__

    def init(self, node_name, **kwargs):
        kwargs['parameter_overrides'] = [Parameter(name, value=value)]
        real_init(self, node_name, **kwargs)
    return init


if __name__ == '__main__':
    unittest.main()
