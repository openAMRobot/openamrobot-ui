"""
rosbridge only lets clients reach what the operator UI uses, and never /cmd_vel.

PROPOSED (contract change request "rosbridge allowlist for the operator UI").

Pure test: parses new_ui_launch.py with ast and the web sources as text,
imports no ROS module, so it runs with
`python3 -m pytest ros2/src/openamr_ui_package/test/test_rosbridge_allowlist.py`
as well as under colcon test.

Why it matters: without topic globs rosbridge accepts any publish, so a browser
tab still running an older UI can publish /cmd_vel and skip the platform chain
of openamr-platform-sw#90 (twist_mux, stop gate, velocity_smoother,
collision_monitor). The stop gate watches only cmd_vel_nav, cmd_vel_dock and
cmd_vel_joy; the UI enters on /ui/cmd_vel through the relay.

rosbridge_suite 2.7.1 (the Jazzy release; same logic on the jazzy branch):
- rosbridge_server/scripts/rosbridge_websocket.py:102-106 declares
  topics_glob (legacy), topics_pub_glob, topics_sub_glob, services_glob and
  actions_glob as strings; there is no params_glob on this node.
- rosbridge_websocket.py:122-133 parse_glob_string: "" means no filtering,
  "[]" an empty list, otherwise "[a, b]" split on commas.
- rosbridge_websocket.py:148-149 appends "/rosapi/*" to services_glob.
- rosbridge_websocket.py:179-191 merges topics_glob into both pub and sub.
- rosbridge_library/.../capabilities/publish.py:83-87, advertise.py:142-146,
  subscribe.py:282-286, call_service.py:94-100, send_action_goal.py:106-110
  match with fnmatch.fnmatch(name, glob).
- rosapi/scripts/rosapi_node:87-91 declares params_glob;
  rosapi/src/rosapi/glob_helper.py:61-64 any_match, used by
  rosapi/src/rosapi/params.py:188-190 (set_param) and 270-272 (get_param).
"""
import ast
import fnmatch
import os
import json

import pytest
import re

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(TEST_DIR)
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(PKG_DIR)))
NEW_UI_LAUNCH = os.path.join(PKG_DIR, 'launch', 'new_ui_launch.py')
CONFIG_YAML = os.path.join(PKG_DIR, 'param', 'config.yaml')
WEB_SRC = os.path.join(REPO_ROOT, 'web', 'src')
CONSTANTS_JS = os.path.join(WEB_SRC, 'shared', 'constants', 'index.js')
NAV_PARAMS_JS = os.path.join(WEB_SRC, 'shared', 'constants', 'navParams.js')
NAV2D_JS = os.path.join(REPO_ROOT, 'web', 'public', 'ros', 'nav2d.js')

PROTOCOL_GLOBS = ('topics_pub_glob', 'topics_sub_glob', 'services_glob', 'actions_glob')

# Must never be publishable from a browser: the final command topic and every
# twist_mux input other than the UI's own relayed /ui/cmd_vel.
FORBIDDEN_PUBLISH = (
    '/cmd_vel', 'cmd_vel', '/cmd_vel_nav', '/cmd_vel_dock', '/cmd_vel_joy',
    '/cmd_vel_smoothed', '/cmd_vel_teleop', '/cmd_vel_unstamped', '/ui/cmd_vel_raw',
)
# Parameter services of the command chain nodes; none is a web app name.
FORBIDDEN_SERVICES = (
    '/safety_stop_gate_node/set_parameters',
    '/ui_cmd_vel_relay_node/set_parameters',
    '/twist_mux/set_parameters',
    '/velocity_smoother/set_parameters',
    '/collision_monitor/set_parameters',
)

# How the web app uses each name: 'pub', 'sub' or 'srv'. Keys are AppConfig
# keys, nav2d.js TOPICS keys (prefix "nav2d."), or literal names. The issue
# lists file:line for each.
WEB_APP_USES = {
    'UI_CMD_VEL_TOPIC': {'pub'},
    'GOAL_POSE_TOPIC': {'pub'},
    'INITIAL_POSE_TOPIC': {'pub'},
    'DOCK_TRIGGER_TOPIC': {'pub'},
    'UNDOCK_TRIGGER_TOPIC': {'pub'},
    'UI_OPERATION_TOPIC': {'pub'},
    'UI_MESSAGE_TOPIC': {'pub', 'sub'},
    'WP_REQ': {'pub'},
    'NAV_DATA_REQ_TOPIC': {'pub'},
    'NEW_WAYPOINT_TOPIC': {'pub'},
    'STOP_LOCK_TOPIC': {'sub'},
    'CLOCK_TOPIC': {'sub'},
    'ROBOT_POSE_TOPIC': {'sub'},
    'ROBOT_VELOCITY_TOPIC': {'sub'},
    'MAP_TOPIC': {'sub'},
    'GLOBAL_COSTMAP_TOPIC': {'sub'},
    'LOCAL_COSTMAP_TOPIC': {'sub'},
    'SCAN_TOPIC': {'sub'},
    'PLAN_TOPIC': {'sub'},
    'TF_TOPIC': {'sub'},
    'TF_STATIC_TOPIC': {'sub'},
    'AMCL_POSE_TOPIC': {'sub'},
    'NAV_STATUS_TOPIC': {'sub'},
    'NAV_FEEDBACK_TOPIC': {'sub'},
    'DOCK_TRIGGER_STATUS_TOPIC': {'sub'},
    'DOCK_STATUS_TOPIC': {'sub'},
    'UNDOCK_STATUS_TOPIC': {'sub'},
    'SENSORS_TOPIC': {'sub'},
    'UI_OPERATION': {'pub'},
    'SET_POINT': {'pub'},
    'BATTERY_TOPIC': {'sub'},
    'CHARGE_STATION_CONNECTED': {'sub'},
    'NAV_DATA_RESP_TOPIC': {'sub'},
    'JOINT_STATES_TOPIC': {'sub'},
    'STOP_LOCK_ENGAGE_SERVICE': {'srv'},
    'STOP_LOCK_RELEASE_SERVICE': {'srv'},
    'NAV_CANCEL_GOAL_SERVICE': {'srv'},
    'THROUGH_POSES_CANCEL_GOAL_SERVICE': {'srv'},
    'COMPUTE_PATH_CANCEL_GOAL_SERVICE': {'srv'},
    'DOCK_CANCEL_GOAL_SERVICE': {'srv'},
    'UNDOCK_CANCEL_GOAL_SERVICE': {'srv'},
    'COMPUTE_PATH_SEND_GOAL_SERVICE': {'srv'},
    'COMPUTE_PATH_GET_RESULT_SERVICE': {'srv'},
    'UI_CMD_VEL_RELAY_GET_PARAMETERS_SERVICE': {'srv'},
    'ROSAPI_GET_TIME_SERVICE': {'srv'},
    'nav2d.map': {'sub'},
    'nav2d.globalCostmap': {'sub'},
    'nav2d.localCostmap': {'sub'},
    'nav2d.waypoints': {'sub'},
    'nav2d.uiMessage': {'pub'},
    'nav2d.uiOperation': {'pub'},
    'nav2d.newWaypoint': {'pub'},
    'nav2d.plan': {'sub'},
    'nav2d.tf': {'sub'},
    'nav2d.tfStatic': {'sub'},
    'nav2d.amclPose': {'sub'},
    'nav2d.scan': {'sub'},
    '/diagnostics': {'sub'},
    '/rosout': {'sub'},
    '/reinitialize_global_localization': {'srv'},
}
# This legacy endpoint has no caller; do not grant publication just because a
# constant exists. Any future use must remove this exemption and add the policy.
INACTIVE_NAMES = {'SET_POINT': 'Unused legacy constant; no caller in web/src'}
NON_ROS_CONFIG_KEYS = {
    'ROSBRIDGE_SERVER_IP', 'ROSBRIDGE_SERVER_PORT', 'CAMERA_PORT',
    'RECONNECTION_TIME', 'MAX_LINEAR_SPEED', 'MAX_ANGULAR_SPEED',
}

KIND_GLOB = {'pub': 'topics_pub_glob', 'sub': 'topics_sub_glob', 'srv': 'services_glob'}


def _read(path):
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def _parse_glob_string(glob_string):
    """Copy of rosbridge_websocket.py:122-133 parse_glob_string (2.7.1)."""
    if not glob_string:
        return None
    if glob_string == '[]':
        return []
    return [s.strip().strip("'") for s in glob_string[1:-1].split(',') if s.strip()]


def _module_constant(tree, name):
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == name for t in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(
        f'new_ui_launch.py defines no {name}: rosbridge accepts every topic, '
        'service and action, so any client can publish /cmd_vel'
    )


def _glob_params_function(tree):
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == '_glob_params':
            namespace = {}
            exec(compile(ast.Module(body=[node], type_ignores=[]), NEW_UI_LAUNCH, 'exec'),
                 namespace)
            return namespace['_glob_params']
    raise AssertionError('new_ui_launch.py defines no _glob_params')


def _node_parameters(tree, executable):
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == 'Node'):
            continue
        kwargs = {k.arg: k.value for k in call.keywords}
        exe = kwargs.get('executable')
        if isinstance(exe, ast.Constant) and exe.value == executable:
            return kwargs.get('parameters')
    raise AssertionError(f'new_ui_launch.py starts no {executable}')


def _passes_allowlist(params, constant):
    """True when the Node's parameters contain _glob_params(<constant>)."""
    if not isinstance(params, ast.List):
        return False
    for elt in params.elts:
        if (isinstance(elt, ast.Call) and isinstance(elt.func, ast.Name)
                and elt.func.id == '_glob_params' and elt.args
                and isinstance(elt.args[0], ast.Name) and elt.args[0].id == constant):
            return True
    return False


def _effective_globs():
    """Globs exactly as rosbridge and rosapi will hold them after parsing."""
    tree = ast.parse(_read(NEW_UI_LAUNCH))
    bridge = _module_constant(tree, 'ROSBRIDGE_ALLOWLIST')
    rosapi = _module_constant(tree, 'ROSAPI_ALLOWLIST')
    render = _glob_params_function(tree)
    parsed_bridge = {k: _parse_glob_string(v) for k, v in render(bridge).items()}
    parsed_rosapi = {k: _parse_glob_string(v) for k, v in render(rosapi).items()}
    assert parsed_bridge == {k: list(v) for k, v in bridge.items()}, (
        'rendered rosbridge globs do not parse back to the allowlist'
    )
    assert parsed_rosapi == {k: list(v) for k, v in rosapi.items()}
    if parsed_bridge.get('services_glob') is not None:
        parsed_bridge['services_glob'] = parsed_bridge['services_glob'] + ['/rosapi/*']
    return tree, parsed_bridge, parsed_rosapi


def _allowed(name, globs):
    if globs is None:  # rosbridge: no glob, no check
        return True
    return any(fnmatch.fnmatch(name, glob) for glob in globs)


def _appconfig():
    block = re.search(r'export const AppConfig = \{(.*?)\n\};', _read(CONSTANTS_JS), re.S).group(1)
    keys = set(re.findall(r'^\s*([A-Z][A-Z0-9_]*):', block, re.M))
    values = dict((key, json.loads(value)) for key, value in re.findall(
        r'^\s*([A-Z][A-Z0-9_]*):\s*("[^"\n]*"|-?\d+(?:\.\d+)?),', block, re.M))
    assert set(values) == keys, 'AppConfig has an unparsed value: classify its syntax explicitly'
    return values


def _camera_topics():
    source = _read(CONSTANTS_JS)
    block = re.search(r'export const CAMERA_TOPIC_OPTIONS = \[(.*?)\];', source, re.S).group(1)
    topics = set(re.findall(r'value:\s*"([^"]+)"', block))
    default = re.search(r'export const DEFAULT_CAMERA_TOPIC = "([^"]+)"', source).group(1)
    assert default in topics, 'Default camera topic is not a selectable option'
    return topics


def _assert_classified(config, nav2d):
    classified = {key for key in WEB_APP_USES if not key.startswith(('/', 'nav2d.'))}
    assert classified == set(config) - NON_ROS_CONFIG_KEYS, (
        'AppConfig keys missing or stale in pub/sub/srv classification: '
        + str(sorted(classified ^ (set(config) - NON_ROS_CONFIG_KEYS))))
    assert NON_ROS_CONFIG_KEYS <= set(config), 'Stale non-ROS classification'
    assert {key[6:] for key in WEB_APP_USES if key.startswith('nav2d.')} == set(nav2d), (
        'nav2d keys missing or stale in pub/sub/srv classification')
    for key, kinds in WEB_APP_USES.items():
        assert kinds and kinds <= {'pub', 'sub', 'srv'}, f'Invalid classification: {key}'


def _nav2d_topics():
    block = re.search(r'const TOPICS = \{(.*?)\};', _read(NAV2D_JS), re.S).group(1)
    return dict(re.findall(r'(\w+):\s*"([^"]+)"', block))


def _resolve(key):
    if key.startswith('nav2d.'):
        return _nav2d_topics()[key[len('nav2d.'):]]
    if key.startswith('/'):
        return key
    return _appconfig()[key]


def _web_sources():
    for root, _dirs, files in os.walk(WEB_SRC):
        for name in files:
            if name.endswith(('.js', '.jsx')) and '.test.' not in name \
                    and 'testing' not in root and 'demo' not in root:
                yield os.path.join(root, name)


def test_rosbridge_and_rosapi_receive_the_allowlist():
    tree = ast.parse(_read(NEW_UI_LAUNCH))
    assert _passes_allowlist(_node_parameters(tree, 'rosbridge_websocket'),
                             'ROSBRIDGE_ALLOWLIST'), \
        'rosbridge_websocket starts without _glob_params(ROSBRIDGE_ALLOWLIST)'
    assert _passes_allowlist(_node_parameters(tree, 'rosapi_node'), 'ROSAPI_ALLOWLIST'), \
        'rosapi_node starts without _glob_params(ROSAPI_ALLOWLIST)'


def test_every_protocol_glob_is_set():
    _tree, bridge, rosapi = _effective_globs()
    for key in PROTOCOL_GLOBS:
        assert bridge.get(key) is not None, f'{key} unset: rosbridge does not filter it'
    assert 'topics_glob' not in bridge, 'legacy topics_glob merges into both pub and sub'
    assert rosapi.get('params_glob') is not None, 'params_glob unset: rosapi allows any param'
    for globs in list(bridge.values()) + list(rosapi.values()):
        for glob in globs:
            assert ',' not in glob and "'" not in glob, f'glob {glob!r} breaks parsing'


def test_config_yaml_sets_no_glob():
    # One source: a *_glob in config.yaml would be overridden silently or
    # disagree with the launch constant.
    assert '_glob' not in _read(CONFIG_YAML)


def test_cmd_vel_cannot_be_published():
    _tree, bridge, _rosapi = _effective_globs()
    pub = bridge['topics_pub_glob']
    allowed = [name for name in FORBIDDEN_PUBLISH if _allowed(name, pub)]
    assert not allowed, f'rosbridge would accept a publish on {allowed}'
    for glob in pub:
        assert not set(glob) & set('*?['), f'publish glob {glob!r} is not an exact name'


def test_command_chain_parameters_cannot_be_set():
    _tree, bridge, rosapi = _effective_globs()
    services = bridge['services_glob']
    allowed = [name for name in FORBIDDEN_SERVICES if _allowed(name, services)]
    assert not allowed, f'rosbridge would accept a call to {allowed}'
    for glob in services:
        if glob != '/rosapi/*':
            assert not set(glob) & set('*?['), f'service glob {glob!r} is not an exact name'
    # /rosapi/set_param is always reachable; rosapi must refuse every name.
    assert not _allowed('use_sim_time', rosapi['params_glob'])
    assert bridge['actions_glob'] == []


def test_every_web_app_name_is_allowed():
    _tree, bridge, _rosapi = _effective_globs()
    missing = []
    _assert_classified(_appconfig(), _nav2d_topics())
    for key, kinds in WEB_APP_USES.items():
        if key in INACTIVE_NAMES:
            continue
        name = _resolve(key)
        for kind in kinds:
            if not _allowed(name, bridge[KIND_GLOB[kind]]):
                missing.append(f'{kind} {name} ({key})')
    missing += [f'sub {name} (camera option)' for name in _camera_topics()
                if not _allowed(name, bridge['topics_sub_glob'])]
    assert not missing, f'web app names rosbridge would refuse: {missing}'


def test_lifecycle_and_param_services_are_allowed():
    _tree, bridge, _rosapi = _effective_globs()
    services = bridge['services_glob']
    lifecycle = re.findall(r'base:\s*"([^"]+)"', _read(CONSTANTS_JS))
    param_nodes = sorted(set(re.findall(r'node:\s*"([^"]+)"', _read(NAV_PARAMS_JS))))
    assert lifecycle and param_nodes
    wanted = [f'{b}/get_state' for b in lifecycle] + [f'{b}/change_state' for b in lifecycle]
    wanted += [f'{n}/{k}_parameters' for n in param_nodes for k in ('get', 'set')]
    missing = [name for name in wanted if not _allowed(name, services)]
    assert not missing, f'web app services rosbridge would refuse: {missing}'


def test_table_covers_every_name_the_web_app_references():
    # A new topic or service in the web app must be added to WEB_APP_USES
    # and the allowlist in the same change.
    config = _appconfig()
    _assert_classified(config, _nav2d_topics())
    used = set()
    for path in _web_sources():
        if os.path.abspath(path) == os.path.abspath(CONSTANTS_JS):
            continue
        used |= set(re.findall(r'AppConfig\.([A-Z][A-Z0-9_]*)', _read(path)))
        used |= set(re.findall(r'name:\s*"(/[^"]+)"', _read(path)))
    assert {key for key in used if not key.startswith('/')} <= set(config), 'Unknown AppConfig reference'
    cancel = re.search(r'ACTION_CANCEL_SERVICES = \[(.*?)\];', _read(CONSTANTS_JS), re.S)
    used |= set(re.findall(r'AppConfig\.([A-Z][A-Z0-9_]*)', cancel.group(1)))
    assert not (used & set(INACTIVE_NAMES)), 'An inactive endpoint gained a caller; review its allowlist policy'
    used = {k for k in used if k.startswith('/') or
            (isinstance(config.get(k), str) and config[k].startswith('/'))}
    used |= {f'nav2d.{k}' for k in _nav2d_topics()}
    unlisted = sorted(used - set(WEB_APP_USES))
    assert not unlisted, f'names the web app uses but the test table does not list: {unlisted}'


@pytest.mark.parametrize('source', ['AppConfig', 'nav2d'])
def test_new_unclassified_key_fails_even_without_a_caller(source):
    config, nav2d = _appconfig(), _nav2d_topics()
    (config if source == 'AppConfig' else nav2d)['NEW_UNCLASSIFIED_TOPIC'] = '/new_topic'
    with pytest.raises(AssertionError, match='classification'):
        _assert_classified(config, nav2d)


@pytest.mark.parametrize('topic', ['/ui/dock_robot/status', '/ui/undock_robot/status', '/sensors',
                                  '/camera/color/image_raw', '/rgb_image', '/camera/image_raw'])
def test_panel_subscriptions_stay_allowed(topic):
    _tree, bridge, _rosapi = _effective_globs()
    assert _allowed(topic, bridge['topics_sub_glob'])



def test_introspection_does_not_grant_unlisted_topic_access():
    _tree, bridge, rosapi = _effective_globs()
    assert _allowed('/rosapi/topics', bridge['services_glob'])
    assert not _allowed('/unlisted_topic', bridge['topics_pub_glob'])
    assert not _allowed('/unlisted_topic', bridge['topics_sub_glob'])
    assert rosapi['params_glob'] == []
