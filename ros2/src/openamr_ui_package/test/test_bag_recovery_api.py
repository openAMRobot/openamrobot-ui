"""flask_app records its ros2 bag processes and stops leftovers on start.

Imports flask_app (rclpy, Flask), so it runs under colcon test. The pure
identity-matching test is test_bag_processes.py.
"""

import json
import os
import subprocess
import tempfile
import unittest
from unittest import mock

RECORD_CMD = ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'bag', 'record']
# Taken before any test patches subprocess.Popen.
REAL_POPEN = subprocess.Popen


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

    def _replayable(self, rec_id='rec_old'):
        # A finished recording with one telemetry topic, valid also under the
        # replay topic policy of #43.
        bag = os.path.join(self.app.RECORDINGS_DIR, rec_id)
        os.makedirs(bag)
        with open(os.path.join(bag, 'metadata.yaml'), 'w') as f:
            json.dump({'rosbag2_bagfile_information': {'topics_with_message_count': [
                {'topic_metadata': {'name': '/ui/map'}}]}}, f)
        entries = self.app.read_recordings_index()
        entries.append({'id': rec_id, 'name': rec_id, 'status': 'complete'})
        self.app.write_recordings_index(entries)

    def _procs(self, *pids):
        procs = []
        for pid in pids:
            proc = mock.MagicMock(pid=pid)
            proc.poll.return_value = None
            self.table.processes[pid] = (pid, RECORD_CMD)
            procs.append(proc)
        self.app.subprocess.Popen.side_effect = procs
        return procs

    def _concurrently(self, *requests):
        import threading

        from openamr_ui_package import bag_processes

        barrier = threading.Barrier(len(requests), timeout=0.5)

        def meet():
            try:
                barrier.wait()
            except threading.BrokenBarrierError:
                pass
        patcher = mock.patch.object(bag_processes, '_between_read_and_write', meet)
        patcher.start()
        self.addCleanup(patcher.stop)
        statuses = [None] * len(requests)

        def run(index, method, path, body):
            client = self.app.app.test_client()
            statuses[index] = getattr(client, method)(path, json=body).status_code
        threads = [threading.Thread(target=run, args=(i, *r)) for i, r in enumerate(requests)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        patcher.stop()
        return statuses

    def test_two_concurrent_starts_both_end_up_in_the_registry(self):
        self._replayable()
        self._procs(5001, 5002)
        self.app._replay.update({'proc': None, 'id': None})
        self.addCleanup(self.app._replay.update, {'proc': None, 'id': None})
        statuses = self._concurrently(
            ('post', '/api/recordings/start', {'name': 'run'}),
            ('post', '/api/recordings/rec_old/replay/start', {}))
        self.assertEqual(statuses, [201, 201])
        self.assertEqual(sorted(e['pid'] for e in self._state()), [5001, 5002])

    def test_concurrent_start_and_stop_keep_only_the_started_entry(self):
        self._replayable()
        replay, record = self._procs(5001, 5002)
        self.addCleanup(self.app._replay.update, {'proc': None, 'id': None})
        self.assertEqual(
            self.client.post('/api/recordings/rec_old/replay/start', json={}).status_code, 201)
        replay.poll.return_value = 0
        statuses = self._concurrently(
            ('post', '/api/recordings/replay/stop', {}),
            ('post', '/api/recordings/start', {'name': 'run'}))
        self.assertEqual(statuses, [200, 201])
        self.assertEqual([e['pid'] for e in self._state()], [5002])

    def test_persistence_failure_after_popen_leaves_no_child_running(self):
        from openamr_ui_package import bag_processes

        children = []

        def spawn(_cmd, **kwargs):
            child = REAL_POPEN(['sleep', '30'], **kwargs)
            children.append(child)
            return child
        self.app.subprocess.Popen.side_effect = spawn
        self.addCleanup(lambda: [c.kill() for c in children if c.poll() is None])

        def disk_full(*_args):
            raise OSError('disk full')
        with mock.patch.object(self.app, 'PROCESS_TABLE', bag_processes.ProcTable()), \
                mock.patch.object(bag_processes, 'write_state', disk_full):
            response = self.client.post('/api/recordings/start', json={'name': 'run'})
        self.assertEqual(response.status_code, 500)
        self.assertIn('disk full', response.get_json()['message'])
        (child,) = children
        self.assertIsNotNone(child.poll(), 'the bag child is still running')
        self.assertIsNone(self.app._recording['proc'])
        self.assertFalse(os.path.exists(self.app.BAG_PROCESSES_FILE))

    def test_live_child_with_unreadable_identity_fails_closed(self):
        # Review of #44: the child is alive (poll() is None) but its /proc
        # identity cannot be read, so no registry entry can be written.
        self.table.processes.clear()
        response = self.client.post('/api/recordings/start', json={'name': 'run'})
        self.assertEqual(response.status_code, 500)
        self.assertIn('could not be read', response.get_json()['message'])
        self.proc.terminate.assert_called_once_with()
        self.proc.wait.assert_called()
        self.assertIsNone(self.app._recording['proc'])
        self.assertFalse(os.path.exists(self.app.BAG_PROCESSES_FILE))

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
