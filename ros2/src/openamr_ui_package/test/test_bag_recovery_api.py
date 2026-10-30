"""flask_app records its ros2 bag processes and stops leftovers on start.

Imports flask_app (rclpy, Flask), so it runs under colcon test. The pure
identity-matching test is test_bag_processes.py.
"""

import json
import os
import tempfile
import unittest
from unittest import mock

RECORD_CMD = ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'bag', 'record']


class FakeTable:
    def __init__(self):
        self.processes = {}
        self.terminated = []

    def identity(self, pid):
        return self.processes.get(pid)

    def terminate(self, pid, expected):
        self.terminated.append(pid)
        self.processes.pop(pid, None)


class BagRecoveryApiTest(unittest.TestCase):

    def setUp(self):
        import openamr_ui_package.flask_app as flask_app

        self.app = flask_app
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        recordings = os.path.join(self.tmp.name, 'recordings')
        self.table = FakeTable()
        self.proc = mock.MagicMock(pid=4242)
        self.proc.poll.return_value = None
        self.table.processes[4242] = (1000, RECORD_CMD)
        for name, value in (
            ('RECORDINGS_DIR', recordings),
            ('RECORDINGS_INDEX_FILE', os.path.join(recordings, 'index.json')),
            ('BAG_PROCESSES_FILE', os.path.join(recordings, 'bag_processes.json')),
            ('PROCESS_TABLE', self.table),
        ):
            patcher = mock.patch.object(flask_app, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        patcher = mock.patch.object(flask_app.subprocess, 'Popen', return_value=self.proc)
        patcher.start()
        self.addCleanup(patcher.stop)
        flask_app._recording.update({'proc': None, 'id': None})
        self.addCleanup(flask_app._recording.update, {'proc': None, 'id': None})
        flask_app.app.config.update(TESTING=True)
        self.client = flask_app.app.test_client()

    def _state(self):
        with open(self.app.BAG_PROCESSES_FILE) as f:
            return json.load(f)

    def test_started_recording_is_written_to_the_state_file(self):
        response = self.client.post('/api/recordings/start', json={'name': 'run'})
        self.assertEqual(response.status_code, 201)
        (entry,) = self._state()
        self.assertEqual((entry['pid'], entry['start_time'], entry['cmdline'], entry['kind']),
                         (4242, 1000, RECORD_CMD, 'record'))
        self.assertTrue(os.path.dirname(self.app.BAG_PROCESSES_FILE)
                        == self.app.RECORDINGS_DIR)

    def test_clean_stop_removes_the_state_entry(self):
        self.client.post('/api/recordings/start', json={'name': 'run'})
        self.proc.poll.return_value = 0
        self.client.post('/api/recordings/stop')
        self.assertEqual(self._state(), [])

    def test_restart_terminates_a_matching_leftover_and_marks_it_interrupted(self):
        recording_id = self.client.post(
            '/api/recordings/start', json={'name': 'run'}).get_json()['id']
        self.app._recording.update({'proc': None, 'id': None})  # Flask restarted.
        reports = self.app.recover_bag_processes()
        self.assertEqual(self.table.terminated, [4242])
        self.assertTrue(reports[0]['terminated'])
        entry = next(e for e in self.app.read_recordings_index() if e['id'] == recording_id)
        self.assertEqual(entry['status'], 'interrupted')

    def test_restart_leaves_a_reused_pid_alone(self):
        self.client.post('/api/recordings/start', json={'name': 'run'})
        self.app._recording.update({'proc': None, 'id': None})
        self.table.processes[4242] = (5000, RECORD_CMD)  # Same pid, new process.
        self.app.recover_bag_processes()
        self.assertEqual(self.table.terminated, [])
        self.assertIn(4242, self.table.processes)

    def test_recovery_runs_from_main_only(self):
        # Importing flask_app (as tests do) must never signal a process.
        import ast
        import inspect

        tree = ast.parse(inspect.getsource(self.app))
        top_level_calls = [
            node.value.func.id for node in tree.body
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name)]
        self.assertNotIn('recover_bag_processes', top_level_calls)
        self.assertIn('recover_bag_processes()', inspect.getsource(self.app.main))


if __name__ == '__main__':
    unittest.main()
