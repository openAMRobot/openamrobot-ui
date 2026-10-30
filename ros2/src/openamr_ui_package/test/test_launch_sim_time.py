"""
Every node in new_ui_launch.py receives the use_sim_time launch argument.

Pure test: parses the launch file with ast, imports no ROS module, so it runs
with `python3 -m pytest ros2/src/openamr_ui_package/test/test_launch_sim_time.py`
as well as under colcon test.

Why it matters: rosapi_node, rosbridge and the UI relays must run on the same
clock as the platform. ui.launch.py has always passed use_sim_time, but
new_ui_launch.py dropped it, so every UI node ran on wall time in simulation.
"""
import ast
import os

LAUNCH_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'launch')
NEW_UI_LAUNCH = os.path.join(LAUNCH_DIR, 'new_ui_launch.py')
UI_BRINGUP_LAUNCH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
    'openamr_ui_bringup',
    'launch',
    'ui.launch.py',
)


def _calls(tree, name):
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == name
    ]


def _kwarg(call, name):
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def _parse(path):
    with open(path, encoding='utf-8') as handle:
        return ast.parse(handle.read())


def test_use_sim_time_is_declared():
    tree = _parse(NEW_UI_LAUNCH)
    declared = [
        call.args[0].value
        for call in _calls(tree, 'DeclareLaunchArgument')
        if call.args and isinstance(call.args[0], ast.Constant)
    ]
    assert 'use_sim_time' in declared


def test_every_node_gets_sim_time():
    tree = _parse(NEW_UI_LAUNCH)
    nodes = _calls(tree, 'Node')
    assert nodes, 'new_ui_launch.py starts no Node; the test would prove nothing'
    missing = []
    for call in nodes:
        executable = _kwarg(call, 'executable')
        label = executable.value if isinstance(executable, ast.Constant) else '?'
        params = _kwarg(call, 'parameters')
        names = (
            {elt.id for elt in params.elts if isinstance(elt, ast.Name)}
            if isinstance(params, ast.List)
            else set()
        )
        if 'sim_time' not in names:
            missing.append(label)
    assert not missing, f'nodes without use_sim_time: {missing}'


def test_rosapi_node_is_covered():
    tree = _parse(NEW_UI_LAUNCH)
    executables = {
        _kwarg(call, 'executable').value
        for call in _calls(tree, 'Node')
        if isinstance(_kwarg(call, 'executable'), ast.Constant)
    }
    assert 'rosapi_node' in executables


def test_ui_bringup_passes_use_sim_time():
    with open(UI_BRINGUP_LAUNCH, encoding='utf-8') as handle:
        source = handle.read()
    assert '"use_sim_time": use_sim_time' in source
