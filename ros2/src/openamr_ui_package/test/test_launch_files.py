"""The UI's launch files must not start platform navigation nodes.

Pure test: reads launch sources as text and imports nothing from ROS, so it
runs with ``python3 -m pytest test/test_launch_files.py``.
"""

import glob
import os

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BRINGUP_LAUNCH = os.path.join(
    os.path.dirname(PACKAGE_DIR), 'openamr_ui_bringup', 'launch', 'ui.launch.py')
LEGACY = ('mapping_launch.py', 'navigation_launch.py', 'move_base_launch.py',
          'gmapping_launch.py', 'amcl_launch.py', 'map_server_launch.py')


def _read(path):
    with open(path, encoding='utf-8') as source:
        return source.read()


def test_no_launch_file_starts_a_map_server():
    # The platform owns map_server (openamrobot-ui#32).
    for path in glob.glob(os.path.join(PACKAGE_DIR, 'launch', '*.py')):
        assert 'nav2_map_server' not in _read(path), path


def test_supported_launches_include_no_legacy_file():
    for path in (BRINGUP_LAUNCH, os.path.join(PACKAGE_DIR, 'launch', 'new_ui_launch.py')):
        text = _read(path)
        for name in LEGACY:
            assert name not in text, f'{path} includes {name}'
