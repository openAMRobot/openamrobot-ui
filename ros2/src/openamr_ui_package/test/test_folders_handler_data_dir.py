"""folders_handler saves maps and routes only under the data directory (#37).

Needs rclpy, nav2_msgs and openamr_ui_msgs, so it runs under colcon test.
The ROS-free rules are tested in test_data_dir.py.
"""

import json
import os
import shutil
import tempfile
import unittest
from unittest import mock


def _tree(root):
    found = {}
    for dirpath, _dirs, files in os.walk(root):
        for name in files:
            path = os.path.join(dirpath, name)
            with open(path, 'rb') as f:
                found[os.path.relpath(path, root)] = f.read()
    return found


class FoldersHandlerDataDirTest(unittest.TestCase):

    def setUp(self):
        import rclpy
        from ament_index_python.packages import get_package_share_directory

        self.real_share = get_package_share_directory('openamr_ui_package')
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.share = os.path.join(self.tmp, 'share')
        self.data = os.path.join(self.tmp, 'data')
        self._install()
        patcher = mock.patch.dict(os.environ)
        patcher.start()
        self.addCleanup(patcher.stop)
        os.environ.pop('OPENAMROBOT_MAPS_DIR', None)
        for target, value in (
            ('webbrowser.open', mock.MagicMock()),
            ('openamr_ui_package.folders_handler.get_package_share_directory',
             lambda _name: self.share),
            ('openamr_ui_package.data_dir.default_data_dir', lambda: self.data),
        ):
            patcher = mock.patch(target, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        rclpy.init()
        self.addCleanup(rclpy.shutdown)

    def _install(self):
        """Simulate a package (re)install into self.share."""
        shutil.rmtree(self.share, ignore_errors=True)
        for sub in ('maps', 'paths', 'param'):
            shutil.copytree(os.path.join(self.real_share, sub), os.path.join(self.share, sub))

    def _node(self):
        import openamr_ui_package.folders_handler as handler
        from std_msgs.msg import String

        node = handler.UIFoldersHandler()
        self.addCleanup(node.destroy_node)
        node.ui_pub = mock.Mock()
        node.change_map_cli = mock.Mock(service_is_ready=lambda: False,
                                        wait_for_service=lambda timeout_sec: False)
        return node, lambda text: node.ui_callback(String(data=text))

    def test_saved_group_and_route_live_in_the_data_dir_and_survive_reinstall(self):
        shipped = _tree(self.share)
        node, command = self._node()
        self.assertTrue(node.routs_folder.startswith(self.data))
        self.assertTrue(node.maps_folder.startswith(self.data))
        self.assertTrue(node.current_files.startswith(self.data))

        command('create_group/' + json.dumps({'group': 'Lab'}))
        os.makedirs(os.path.join(node.routs_folder, 'Lab', 'Floor1'))
        node.WPs = ['1.0,2.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0']
        command('save_route/' + json.dumps({'group': 'Lab', 'map': 'Floor1', 'route': 'R1'}))

        route = os.path.join(self.data, 'paths', 'Lab', 'Floor1', 'R1.csv')
        self.assertTrue(os.path.isfile(route))
        self.assertTrue(os.path.isdir(os.path.join(self.data, 'maps', 'Lab')))
        self.assertEqual(_tree(self.share), shipped)

        self._install()
        node, _command = self._node()
        self.assertTrue(os.path.isfile(route))
        structure = json.loads(node.get_paths())
        self.assertIn({'Lab': [{'Floor1': ['R1.csv']}]}, structure['structure'])
        self.assertEqual(structure['active_files']['route'], 'R1')

    def test_maps_go_to_openamrobot_maps_dir_and_routes_stay_in_the_data_dir(self):
        maps = os.path.join(self.tmp, 'shared_maps')
        os.environ['OPENAMROBOT_MAPS_DIR'] = maps
        node, command = self._node()
        self.assertEqual(node.maps_folder, maps)
        self.assertEqual(node.routs_folder, os.path.join(self.data, 'paths'))
        self.assertTrue(os.path.isfile(os.path.join(maps, 'Welcome', 'Start.yaml')))

        command('create_group/' + json.dumps({'group': 'Lab'}))
        self.assertTrue(os.path.isdir(os.path.join(maps, 'Lab')))
        self.assertTrue(os.path.isdir(os.path.join(self.data, 'paths', 'Lab')))
        self.assertFalse(os.path.exists(os.path.join(self.data, 'maps')))

    def test_recordings_share_the_data_dir(self):
        import openamr_ui_package.flask_app as flask_app

        self.assertEqual(os.path.dirname(flask_app.RECORDINGS_DIR),
                         os.path.join(os.path.expanduser('~'), '.openamr_ui'))


if __name__ == '__main__':
    unittest.main()
