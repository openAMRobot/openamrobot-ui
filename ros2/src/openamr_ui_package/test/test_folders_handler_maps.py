"""Maps actions use platform services only and never spawn a process.

Needs rclpy, nav2_msgs and openamr_ui_msgs, so it runs under colcon test.
The ROS-free check of the same rule is test_no_ui_launch.py.
"""

import os
import shutil
import tempfile
import unittest
from unittest import mock


class FoldersHandlerMapsTest(unittest.TestCase):

    def setUp(self):
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        import openamr_ui_package.folders_handler as handler

        self.handler = handler
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        real_share = get_package_share_directory('openamr_ui_package')
        os.makedirs(os.path.join(self.tmp, 'param'))
        shutil.copy(os.path.join(real_share, 'param', 'config.yaml'),
                    os.path.join(self.tmp, 'param', 'config.yaml'))
        with open(os.path.join(self.tmp, 'param', 'current_map_route.yaml'), 'w') as f:
            f.write("map_file: ''\nroute_file: ''\n")
        os.makedirs(os.path.join(self.tmp, 'maps', 'G'))
        os.makedirs(os.path.join(self.tmp, 'paths', 'G'))

        # Any attempt to start a process fails the test immediately.
        self.spawn = mock.MagicMock(side_effect=AssertionError('process spawned'))
        for target in ('subprocess.Popen', 'subprocess.run', 'subprocess.call',
                       'os.system', 'os.popen', 'os.execvp', 'os.spawnvp'):
            patcher = mock.patch(target, self.spawn)
            patcher.start()
            self.addCleanup(patcher.stop)
        for target, value in (
            ('webbrowser.open', mock.MagicMock()),
            ('openamr_ui_package.folders_handler.get_package_share_directory',
             lambda _name: self.tmp),
        ):
            patcher = mock.patch(target, value)
            patcher.start()
            self.addCleanup(patcher.stop)

        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = handler.UIFoldersHandler()
        self.addCleanup(self.node.destroy_node)
        self.messages = []
        self.node.ui_pub = mock.Mock(publish=lambda msg: self.messages.append(msg.data))

    def _command(self, text):
        from std_msgs.msg import String
        self.node.ui_callback(String(data=text))

    def test_build_map_launches_nothing_and_explains(self):
        self._command('build_map')
        self.spawn.assert_not_called()
        self.assertEqual(self.messages, [self.handler.MAPPING_NOT_STARTED_BY_UI])

    def test_save_map_without_platform_service_launches_nothing(self):
        self._command('save_map/{"group": "G", "map": "M"}')
        self.spawn.assert_not_called()
        self.assertTrue(any('not available' in m for m in self.messages), self.messages)
        self.assertFalse(os.path.exists(os.path.join(self.tmp, 'paths', 'G', 'M')))

    def test_save_map_calls_platform_saver_then_load_map(self):
        from nav2_msgs.srv import SaveMap
        from rclpy.task import Future

        save_future = Future()
        save_cli = mock.Mock(service_is_ready=lambda: True,
                             call_async=mock.Mock(return_value=save_future))
        load_cli = mock.Mock(service_is_ready=lambda: True, call_async=mock.Mock())
        self.node.save_map_cli = save_cli
        self.node.change_map_cli = load_cli

        self._command('save_map/{"group": "G", "map": "M"}')
        request = save_cli.call_async.call_args.args[0]
        self.assertEqual(request.map_url, os.path.join(self.tmp, 'maps', 'G', 'M'))
        self.assertEqual(request.map_topic, '/map')
        load_cli.call_async.assert_not_called()

        save_future.set_result(SaveMap.Response(result=True))
        self.spawn.assert_not_called()
        load_request = load_cli.call_async.call_args.args[0]
        self.assertEqual(load_request.map_url, os.path.join(self.tmp, 'maps', 'G', 'M.yaml'))
        self.assertTrue(os.path.isdir(os.path.join(self.tmp, 'paths', 'G', 'M')))
        self.assertIn('Map saved', self.messages)

    def test_failed_save_reports_and_loads_nothing(self):
        from nav2_msgs.srv import SaveMap
        from rclpy.task import Future

        save_future = Future()
        self.node.save_map_cli = mock.Mock(service_is_ready=lambda: True,
                                           call_async=mock.Mock(return_value=save_future))
        load_cli = mock.Mock(service_is_ready=lambda: True, call_async=mock.Mock())
        self.node.change_map_cli = load_cli
        self._command('save_map/{"group": "G", "map": "M"}')
        save_future.set_result(SaveMap.Response(result=False))
        load_cli.call_async.assert_not_called()
        self.assertIn('The platform could not save the map.', self.messages)

    def test_handler_has_no_initialpose_publisher(self):
        topics = [name for name, _types in self.node.get_publisher_names_and_types_by_node(
            'ui_folders', '/')]
        self.assertNotIn('/initialpose', topics)


if __name__ == '__main__':
    unittest.main()
