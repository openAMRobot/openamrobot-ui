"""
No UI launch file or UI source publishes or remaps to the final /cmd_vel.

Pure test: parses files with ast and plain text, imports no ROS module, so it
runs with `python3 -m pytest ros2/src/openamr_ui_package/test/test_no_direct_cmd_vel.py`
as well as under colcon test.

Why it matters (issue #31, P0-D command-path audit): on openamr-platform-sw#90
every velocity source enters twist_mux (config/twist_mux.yaml) and then
passes the stop gate, velocity_smoother and collision_monitor, which is the
only publisher of the final /cmd_vel. A UI process that writes /cmd_vel
itself skips all of them. Nav2 controller_server and behavior_server publish
on the relative name cmd_vel, so a UI launch that starts one without a
remap away from cmd_vel also writes /cmd_vel.

Checked:
- launch files of the UI packages: every remapping target, every launch
  argument default that a remapping uses, and every Nav2 velocity producer
  node (controller_server, behavior_server) must not resolve to /cmd_vel;
- Python nodes of the UI packages: no create_publisher on /cmd_vel;
- web/src (non-test .js/.jsx): no string literal "/cmd_vel" or "cmd_vel".
"""
import ast
import os
import re

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(TEST_DIR)
ROS2_SRC = os.path.dirname(PKG_DIR)
REPO_ROOT = os.path.dirname(os.path.dirname(ROS2_SRC))
WEB_SRC = os.path.join(REPO_ROOT, 'web', 'src')

UI_PACKAGES = ('openamr_ui_package', 'openamr_ui_bringup')
FINAL_TOPICS = {'cmd_vel', '/cmd_vel'}
# Nav2 nodes that publish geometry_msgs on the relative name cmd_vel.
VELOCITY_PRODUCERS = {
    ('nav2_controller', 'controller_server'),
    ('nav2_behaviors', 'behavior_server'),
}
WEB_LITERAL = re.compile(r"""(["'`])/?cmd_vel\1""")


def _launch_files():
    files = []
    for pkg in UI_PACKAGES:
        launch_dir = os.path.join(ROS2_SRC, pkg, 'launch')
        if not os.path.isdir(launch_dir):
            continue
        for name in sorted(os.listdir(launch_dir)):
            if name.endswith('.py'):
                files.append(os.path.join(launch_dir, name))
    return files


def _python_sources():
    files = []
    for pkg in UI_PACKAGES:
        mod_dir = os.path.join(ROS2_SRC, pkg, pkg)
        for root, _dirs, names in os.walk(mod_dir):
            if 'static' in root.split(os.sep):
                continue
            files.extend(os.path.join(root, n) for n in names if n.endswith('.py'))
    return sorted(files)


def _web_sources():
    files = []
    for root, _dirs, names in os.walk(WEB_SRC):
        if os.sep + 'testing' in root:
            continue
        for name in names:
            if re.search(r'\.(js|jsx)$', name) and not re.search(r'\.test\.(js|jsx)$', name):
                files.append(os.path.join(root, name))
    return sorted(files)


def _parse(path):
    with open(path, encoding='utf-8') as handle:
        return ast.parse(handle.read(), filename=path)


def _call_name(call):
    func = call.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def _kwarg(call, name):
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def _const(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _arg_defaults(tree):
    defaults = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _call_name(node) == 'DeclareLaunchArgument':
            name = _const(node.args[0]) if node.args else _const(_kwarg(node, 'name'))
            default = _const(_kwarg(node, 'default_value'))
            if name is not None:
                defaults[name] = default
    return defaults


def _resolve(node, defaults):
    """Return the topic a remap target resolves to by default, if knowable."""
    value = _const(node)
    if value is not None:
        return value
    if isinstance(node, ast.Call) and _call_name(node) == 'LaunchConfiguration' and node.args:
        return defaults.get(_const(node.args[0]))
    return None


def _remap_pairs(remappings):
    if not isinstance(remappings, (ast.List, ast.Tuple)):
        return []
    return [
        elt.elts for elt in remappings.elts
        if isinstance(elt, ast.Tuple) and len(elt.elts) == 2
    ]


def launch_violations(path):
    tree = _parse(path)
    defaults = _arg_defaults(tree)
    rel = os.path.relpath(path, REPO_ROOT)
    found = []
    for name, default in defaults.items():
        if default in FINAL_TOPICS:
            found.append(f'{rel}: launch argument {name} defaults to {default}')
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _call_name(node) not in ('Node', 'ComposableNode'):
            continue
        pairs = _remap_pairs(_kwarg(node, 'remappings'))
        for source, target in pairs:
            resolved = _resolve(target, defaults)
            if resolved in FINAL_TOPICS:
                found.append(
                    f'{rel}:{node.lineno}: remaps {_const(source)!r} to {resolved}')
        key = (_const(_kwarg(node, 'package')),
               _const(_kwarg(node, 'executable')) or _const(_kwarg(node, 'plugin')))
        if key in VELOCITY_PRODUCERS:
            moved = [
                _resolve(target, defaults) for source, target in pairs
                if _const(source) in FINAL_TOPICS
            ]
            if not moved or any(t in FINAL_TOPICS or t is None for t in moved):
                found.append(
                    f'{rel}:{node.lineno}: {key[1]} is not remapped away from /cmd_vel')
    return found


def python_violations(path):
    tree = _parse(path)
    rel = os.path.relpath(path, REPO_ROOT)
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and _call_name(node) == 'create_publisher':
            topic = _const(node.args[1]) if len(node.args) > 1 else _const(_kwarg(node, 'topic'))
            if topic in FINAL_TOPICS:
                found.append(f'{rel}:{node.lineno}: create_publisher on {topic}')
    return found


def web_violations(path):
    rel = os.path.relpath(path, REPO_ROOT)
    found = []
    with open(path, encoding='utf-8') as handle:
        for lineno, line in enumerate(handle, start=1):
            if WEB_LITERAL.search(line):
                found.append(f'{rel}:{lineno}: string literal {WEB_LITERAL.search(line).group(0)}')
    return found


def test_scopes_are_not_empty():
    assert _launch_files(), 'no UI launch file found; the test would prove nothing'
    assert _python_sources(), 'no UI Python source found; the test would prove nothing'
    assert _web_sources(), 'no web source found; the test would prove nothing'


def test_no_ui_launch_file_writes_cmd_vel():
    found = [v for path in _launch_files() for v in launch_violations(path)]
    assert found == [], '\n'.join(found)


def test_no_ui_python_node_publishes_cmd_vel():
    found = [v for path in _python_sources() for v in python_violations(path)]
    assert found == [], '\n'.join(found)


def test_no_web_source_names_cmd_vel():
    found = [v for path in _web_sources() for v in web_violations(path)]
    assert found == [], '\n'.join(found)


def test_detector_catches_the_legacy_patterns(tmp_path):
    legacy = tmp_path / 'legacy_launch.py'
    legacy.write_text(
        "DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')\n"
        "Node(package='nav2_controller', executable='controller_server',\n"
        "     remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))])\n"
        "Node(package='nav2_behaviors', executable='behavior_server')\n")
    found = launch_violations(str(legacy))
    assert len(found) == 4, found
    node = tmp_path / 'node.py'
    node.write_text("self.create_publisher(Twist, '/cmd_vel', 10)\n")
    assert len(python_violations(str(node))) == 1
    web = tmp_path / 'web.js'
    web.write_text('export const T = { CMD_VEL_TOPIC: "/cmd_vel" };\n')
    assert len(web_violations(str(web))) == 1
    ok = tmp_path / 'ok_launch.py'
    ok.write_text(
        "Node(package='nav2_controller', executable='controller_server',\n"
        "     remappings=[('cmd_vel', 'cmd_vel_nav')])\n")
    assert launch_violations(str(ok)) == []
