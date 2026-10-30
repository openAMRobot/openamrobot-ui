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

        # Keep any per-user data directory (~/.openamr_ui, see #47) inside
        # the test's temporary folder.
        patcher = mock.patch.dict(os.environ, {'HOME': os.path.join(self.tmp, 'home')})
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
        self.assertFalse(os.path.exists(os.path.join(self.node.routs_folder, 'G', 'M')))

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
        self.assertEqual(request.map_url, os.path.join(self.node.maps_folder, 'G', 'M'))
        self.assertEqual(request.map_topic, '/map')
        load_cli.call_async.assert_not_called()

        save_future.set_result(SaveMap.Response(result=True))
        self.spawn.assert_not_called()
        load_request = load_cli.call_async.call_args.args[0]
        self.assertEqual(load_request.map_url,
                         os.path.join(self.node.maps_folder, 'G', 'M.yaml'))
        self.assertTrue(os.path.isdir(os.path.join(self.node.routs_folder, 'G', 'M')))
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

    def test_failed_save_call_reports_and_loads_nothing(self):
        from rclpy.task import Future

        save_future = Future()
        self.node.save_map_cli = mock.Mock(service_is_ready=lambda: True,
                                           call_async=mock.Mock(return_value=save_future))
        load_cli = mock.Mock(service_is_ready=lambda: True, call_async=mock.Mock())
        self.node.change_map_cli = load_cli
        self._command('save_map/{"group": "G", "map": "M"}')
        save_future.set_exception(RuntimeError('service died'))
        load_cli.call_async.assert_not_called()
        self.spawn.assert_not_called()
        self.assertIn('The platform could not save the map.', self.messages)

    def _load_with(self, response):
        from rclpy.task import Future

        load_future = Future()
        self.node.change_map_cli = mock.Mock(service_is_ready=lambda: True,
                                             call_async=mock.Mock(return_value=load_future))
        self._write_map('M', ros_yaml=False)
        self._command('change_map/{"group": "G", "map": "M"}')
        if isinstance(response, Exception):
            load_future.set_exception(response)
        else:
            load_future.set_result(response)
        self.spawn.assert_not_called()

    def test_failed_load_reports_a_clear_error(self):
        from nav2_msgs.srv import LoadMap

        self._load_with(LoadMap.Response(result=LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST))
        self.assertTrue(any('could not load' in m and 'M.yaml' in m for m in self.messages),
                        self.messages)

    def test_load_call_error_reports_a_clear_error(self):
        self._load_with(RuntimeError('map_server gone'))
        self.assertTrue(any('could not load' in m and 'map_server gone' in m
                            for m in self.messages), self.messages)

    def test_successful_load_is_reported(self):
        from nav2_msgs.srv import LoadMap

        self._load_with(LoadMap.Response(result=LoadMap.Response.RESULT_SUCCESS))
        self.assertIn('Map loaded', self.messages)

    def test_unavailable_load_service_reports_and_starts_nothing(self):
        self.node.change_map_cli = mock.Mock(service_is_ready=lambda: False,
                                             wait_for_service=lambda timeout_sec: False)
        self._write_map('M', ros_yaml=False)
        self._command('change_map/{"group": "G", "map": "M"}')
        self.spawn.assert_not_called()
        self.assertTrue(any('unavailable' in m for m in self.messages), self.messages)

    # Rename and delete: maps saved by SaveMap have <name>.yaml and the image
    # only; legacy maps also have <name>_ros.yaml.

    def _write_map(self, name, ros_yaml, image=None):
        group = os.path.join(self.node.maps_folder, 'G')
        os.makedirs(group, exist_ok=True)
        image = image or f'{name}.png'
        with open(os.path.join(group, f'{name}.yaml'), 'w') as f:
            f.write(f'image: {image}\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        with open(os.path.join(group, os.path.basename(image)), 'wb') as f:
            f.write(b'png')
        if ros_yaml:
            with open(os.path.join(group, f'{name}_ros.yaml'), 'w') as f:
                f.write(f'map_server:\n  ros__parameters:\n    yaml_filename: "{name}.yaml"\n')
        os.makedirs(os.path.join(self.node.routs_folder, 'G', name), exist_ok=True)
        return group

    def _files(self, group):
        return sorted(os.listdir(group))

    def test_rename_a_savemap_map_without_ros_yaml(self):
        group = self._write_map('A', ros_yaml=False)
        self._command('rename_map/{"group": "G", "map_old": "A", "map_new": "B"}')
        self.assertEqual(self._files(group), ['B.png', 'B.yaml'])
        with open(os.path.join(group, 'B.yaml')) as f:
            self.assertIn('image: B.png', f.read())
        self.assertTrue(os.path.isdir(os.path.join(self.node.routs_folder, 'G', 'B')))

    def test_rename_a_legacy_map_with_ros_yaml(self):
        group = self._write_map('A', ros_yaml=True)
        self._command('rename_map/{"group": "G", "map_old": "A", "map_new": "B"}')
        self.assertEqual(self._files(group), ['B.png', 'B.yaml', 'B_ros.yaml'])
        with open(os.path.join(group, 'B_ros.yaml')) as f:
            self.assertIn('yaml_filename: B.yaml', f.read())
        # A relative image path stays relative, so the map still loads from
        # another container that mounts the folder at a different path.
        with open(os.path.join(group, 'B.yaml')) as f:
            self.assertIn('image: B.png', f.read())

    def test_rename_onto_an_existing_map_changes_nothing(self):
        group = self._write_map('A', ros_yaml=False)
        self._write_map('B', ros_yaml=False)
        self._command('rename_map/{"group": "G", "map_old": "A", "map_new": "B"}')
        self.assertEqual(self._files(group), ['A.png', 'A.yaml', 'B.png', 'B.yaml'])
        self.assertTrue(any('already exists' in m for m in self.messages), self.messages)

    def test_delete_a_savemap_map_without_ros_yaml(self):
        group = self._write_map('A', ros_yaml=False)
        self._write_map('Keep', ros_yaml=False)
        load_cli = self._load_cli()
        self._command('delete_map/{"group": "G", "map": "A"}')
        self.assertEqual(self._files(group), ['Keep.png', 'Keep.yaml'])
        self.assertFalse(os.path.exists(os.path.join(self.node.routs_folder, 'G', 'A')))
        self.assertEqual(load_cli.call_async.call_args.args[0].map_url,
                         os.path.join(group, 'Keep.yaml'))

    def _load_cli(self):
        load_cli = mock.Mock(service_is_ready=lambda: True, call_async=mock.Mock())
        self.node.change_map_cli = load_cli
        return load_cli

    def test_delete_a_legacy_map_with_ros_yaml_loads_the_next_map(self):
        group = self._write_map('A', ros_yaml=True)
        self._write_map('Keep', ros_yaml=True)
        load_cli = self._load_cli()
        self._command('delete_map/{"group": "G", "map": "A"}')
        self.assertEqual(self._files(group), ['Keep.png', 'Keep.yaml', 'Keep_ros.yaml'])
        self.assertEqual(load_cli.call_async.call_args.args[0].map_url,
                         os.path.join(group, 'Keep.yaml'))

    def test_delete_the_last_legacy_map(self):
        group = self._write_map('A', ros_yaml=True)
        self._command('delete_map/{"group": "G", "map": "A"}')
        self.assertEqual(self._files(group), [])
        self.assertIn('No maps in the group', self.messages)

    def test_handler_has_no_initialpose_publisher(self):
        topics = [name for name, _types in self.node.get_publisher_names_and_types_by_node(
            'ui_folders', '/')]
        self.assertNotIn('/initialpose', topics)


if __name__ == '__main__':
    unittest.main()
