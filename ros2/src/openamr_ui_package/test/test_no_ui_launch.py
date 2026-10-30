"""folders_handler.py must never start a process (openamrobot-ui#32).

Pure test: it parses the source and imports nothing from ROS, so it runs
with ``python3 -m pytest test/test_no_ui_launch.py`` on any machine.
"""

import ast
import os

HANDLER = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'openamr_ui_package', 'folders_handler.py')

PROCESS_MODULES = {'subprocess', 'multiprocessing', 'pty'}
OS_PROCESS_CALLS = {
    'system', 'popen', 'fork', 'forkpty',
    'execl', 'execle', 'execlp', 'execlpe', 'execv', 'execve', 'execvp', 'execvpe',
    'spawnl', 'spawnle', 'spawnlp', 'spawnlpe', 'spawnv', 'spawnve', 'spawnvp',
    'spawnvpe', 'posix_spawn', 'posix_spawnp',
}
FORBIDDEN_TEXT = ('ros2 launch', 'ros2 run', 'ros2 lifecycle', 'lifecycle set', 'map_saver -f')


def _tree():
    with open(HANDLER, encoding='utf-8') as source:
        return ast.parse(source.read(), HANDLER)


def test_no_process_module_is_imported():
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Import):
            names = {alias.name.split('.')[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            names = {(node.module or '').split('.')[0]}
        else:
            continue
        assert not names & PROCESS_MODULES, f'line {node.lineno} imports {names}'


def test_no_os_process_call():
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            if node.value.id == 'os':
                assert node.attr not in OS_PROCESS_CALLS, f'line {node.lineno}: os.{node.attr}'
        if isinstance(node, ast.Name):
            assert node.id != 'Popen', f'line {node.lineno}: Popen'


def test_no_launch_or_lifecycle_command_text():
    for node in ast.walk(_tree()):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            for text in FORBIDDEN_TEXT:
                assert text not in node.value, f'line {node.lineno}: {node.value!r}'
