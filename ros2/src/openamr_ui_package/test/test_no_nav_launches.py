"""The UI package ships no SLAM, AMCL or Nav2 launch of its own (#32).

Pure test: reads files as text and imports nothing from ROS, so it runs
with ``python3 -m pytest test/test_no_nav_launches.py``.
"""

import glob
import os

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(PACKAGE_DIR)))

REMOVED = (
    'launch/mapping_launch.py',
    'launch/navigation_launch.py',
    'launch/gmapping_launch.py',
    'launch/move_base_launch.py',
    'launch/amcl_launch.py',
    'param/move_base',
)
# Nodes the robot platform owns. map_server is checked separately by
# test_launch_files.py once map_server_launch.py is gone (openamrobot-ui#41).
PLATFORM_NODE_PACKAGES = (
    'nav2_amcl', 'slam_toolbox', 'slam_gmapping',
    'nav2_controller', 'nav2_planner', 'nav2_bt_navigator',
)
CODE_PATTERNS = ('**/*.py', '**/*.sh', '**/*.yml', '**/*.yaml', '**/*.xml', '**/Dockerfile')


def _read(path):
    with open(path, encoding='utf-8', errors='replace') as source:
        return source.read()


def test_removed_launch_files_and_params_are_gone():
    for rel in REMOVED:
        assert not os.path.exists(os.path.join(PACKAGE_DIR, rel)), rel


def test_no_launch_file_starts_a_platform_navigation_node():
    for path in glob.glob(os.path.join(PACKAGE_DIR, 'launch', '*.py')):
        text = _read(path)
        for package in PLATFORM_NODE_PACKAGES:
            assert package not in text, f'{path} starts {package}'


def test_no_code_or_config_names_a_removed_file():
    names = [os.path.basename(rel) for rel in REMOVED[:-1]] + ['param/move_base']
    skip = ('node_modules', os.sep + 'build' + os.sep, os.sep + 'install' + os.sep,
            os.sep + 'log' + os.sep, os.path.abspath(__file__))
    for pattern in CODE_PATTERNS:
        for path in glob.glob(os.path.join(REPO_ROOT, pattern), recursive=True):
            if any(part in path for part in skip) or not os.path.isfile(path):
                continue
            text = _read(path)
            for name in names:
                assert name not in text, f'{path} names {name}'
