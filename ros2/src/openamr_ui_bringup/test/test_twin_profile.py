"""The twin launch profile's rosbridge is bound to localhost and allowlisted.

Pure test: loads twin.launch.py's helpers by path and parses its AST; imports
no ROS module. Runs with
  python3 -m pytest ros2/src/openamr_ui_bringup/test/test_twin_profile.py
and under colcon test.
"""
import ast
import fnmatch
import importlib.util
import os

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(PKG, 'launch', 'twin.launch.py')
ALLOWLIST = os.path.join(PKG, 'config', 'twin_bridge_allowlist.yaml')


def _module():
    spec = importlib.util.spec_from_file_location('twin_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _globs(value):
    """Same parsing as rosbridge_suite 2.7.1 parse_glob_string."""
    if value == '[]':
        return []
    return [s.strip().strip("'") for s in value[1:-1].split(',') if s.strip()]


def _allowed(globs, name):
    return any(fnmatch.fnmatch(name, g) for g in globs)


@pytest.fixture
def params():
    twin = _module()
    return twin.bridge_parameters(twin.load_allowlist(ALLOWLIST))


def _defaults():
    tree = ast.parse(open(LAUNCH, encoding='utf-8').read())
    out = {}
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and getattr(call.func, 'id', '') == 'DeclareLaunchArgument':
            for kw in call.keywords:
                if kw.arg == 'default_value':
                    out[call.args[0].value] = kw.value.value
    return out


def test_bound_to_localhost_by_default():
    defaults = _defaults()
    assert defaults['bridge_address'] == '127.0.0.1'
    assert defaults['ui_address'] == '127.0.0.1'
    assert defaults['manual_drive'] == 'false'
    assert defaults['stack'] == 'platform'


def test_every_glob_is_set(params):
    # An empty string would mean "no restriction" in rosbridge 2.7.1.
    for key in ('topics_glob', 'topics_sub_glob', 'topics_pub_glob', 'services_glob',
                'actions_glob'):
        assert params[key].startswith('[') and params[key].endswith(']'), key


def test_ui_cannot_write_cmd_vel_or_drive_by_default(params):
    pub = _globs(params['topics_pub_glob'])
    for name in ('/cmd_vel', '/cmd_vel_nav', '/cmd_vel_joy', '/cmd_vel_gated', '/cmd_vel_stop',
                 '/cmd_vel_mux', '/cmd_vel_dock', '/ui/cmd_vel', '/initialpose',
                 '/safety/stop_lock', '/dock_trigger', '/undock_robot'):
        assert not _allowed(pub, name), name
    assert pub == ['/goal_pose']


def test_manual_drive_adds_only_ui_cmd_vel():
    twin = _module()
    params = twin.bridge_parameters(twin.load_allowlist(ALLOWLIST), manual_drive=True)
    assert sorted(_globs(params['topics_pub_glob'])) == ['/goal_pose', '/ui/cmd_vel']


def test_cmd_vel_in_the_write_set_is_rejected():
    twin = _module()
    allowlist = twin.load_allowlist(ALLOWLIST)
    allowlist['write_topics'].append('/cmd_vel')
    with pytest.raises(ValueError):
        twin.bridge_parameters(allowlist)


def test_wildcards_are_rejected():
    twin = _module()
    with pytest.raises(ValueError):
        twin.glob_string(['/safety/*'])


def test_services_are_the_stop_path_cancels_and_read_only_queries(params):
    services = _globs(params['services_glob'])
    assert _allowed(services, '/safety/stop_lock_engage')
    assert _allowed(services, '/safety/stop_lock_release')
    assert _allowed(services, '/navigate_to_pose/_action/cancel_goal')
    for name in ('/controller_server/set_parameters', '/bt_navigator/change_state',
                 '/navigate_to_pose/_action/send_goal', '/reinitialize_global_localization',
                 '/dock_trigger/abort', '/map_server/load_map'):
        assert not _allowed(services, name), name
    assert all(s.endswith(('/cancel_goal', '/get_state', '/get_parameters'))
               or s.startswith('/safety/stop_lock_') for s in services)


def test_no_actions_and_rosapi_reads_no_parameter(params):
    assert _globs(params['actions_glob']) == []
    assert _module().rosapi_parameters() == {'params_glob': '[]'}


def test_read_set_covers_the_status_the_ui_shows(params):
    sub = _globs(params['topics_sub_glob'])
    for name in ('/navigation/status', '/safety/stop_state', '/safety/stop_lock',
                 '/baseline1/mission_status', '/ui/map', '/odom', '/ui/navigate_to_pose/status',
                 '/clock'):
        assert _allowed(sub, name), name


def test_docking_stays_off_on_the_platform_stack():
    text = open(LAUNCH, encoding='utf-8').read()
    assert "'use_docking': 'false'" in text
