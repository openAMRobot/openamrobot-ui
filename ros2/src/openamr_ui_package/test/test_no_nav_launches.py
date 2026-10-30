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
SRC_DIR = os.path.dirname(PACKAGE_DIR)


def runtime_consumers():
    """Return the files that can launch or install a launch file at run time.

    Launch files, Python package code, setup.py, setup.cfg and package.xml of
    every package under ros2/src, plus Dockerfile, docker-compose.yml and
    scripts/. Test directories are left out: tests are not runtime callers,
    and test_launch_files.py names the removed files on purpose.
    """
    paths = []
    for package in sorted(glob.glob(os.path.join(SRC_DIR, '*', 'package.xml'))):
        root = os.path.dirname(package)
        name = os.path.basename(root)
        paths += glob.glob(os.path.join(root, 'launch', '*.py'))
        paths += glob.glob(os.path.join(root, name, '**', '*.py'), recursive=True)
        paths += [os.path.join(root, f) for f in ('setup.py', 'setup.cfg', 'package.xml')]
    paths += [os.path.join(REPO_ROOT, 'Dockerfile'), os.path.join(REPO_ROOT, 'docker-compose.yml')]
    paths += glob.glob(os.path.join(REPO_ROOT, 'scripts', '**', '*'), recursive=True)
    return sorted(p for p in set(paths)
                  if os.path.isfile(p) and os.sep + 'test' + os.sep not in p)


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


def test_no_runtime_consumer_names_a_removed_file():
    names = [os.path.basename(rel) for rel in REMOVED[:-1]] + ['param/move_base']
    consumers = runtime_consumers()
    assert os.path.join(PACKAGE_DIR, 'launch', 'new_ui_launch.py') in consumers
    for path in consumers:
        text = _read(path)
        for name in names:
            assert name not in text, f'{path} names {name}'


def test_test_directories_are_not_scanned():
    assert not any(os.sep + 'test' + os.sep in p for p in runtime_consumers())
