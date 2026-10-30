"""POST /api/recordings/<id>/replay/start applies the replay policy.

Imports flask_app (rclpy, Flask), so it runs under colcon test. The ROS-free
policy test is test_replay_policy.py.
"""

import json
import os
import subprocess
import tempfile
import unittest
from unittest import mock

DENIED = ['/cmd_vel', '/ui/cmd_vel', '/cmd_vel_nav', '/tf', '/tf_static', '/goal_pose',
          '/initialpose', '/safety/estop_state', '/navigate_to_pose/_action/status',
          '/map_server/load_map/_service_event']

FAKE_PID = 4242
FAKE_IDENTITY = (1000, ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'bag', 'play'])
# Taken before any test patches subprocess.Popen.
REAL_POPEN = subprocess.Popen


class FakeProcTable:
    """Process identity reader for the fake bag process (#44 tracks it)."""

    def identity(self, pid):
        return FAKE_IDENTITY if pid == FAKE_PID else None

    def terminate(self, pid, expected):
        raise AssertionError('no leftover is stopped in these tests')


class ReplayApiTest(unittest.TestCase):

    def setUp(self):
        import openamr_ui_package.flask_app as flask_app

        self.flask_app = flask_app
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        recordings = os.path.join(self.tmp.name, 'recordings')
        # #44 records every started bag process by pid, start time and
        # command line, and stops one whose details cannot be read. The fake
        # process below therefore has readable details, and the registry
        # stays inside the test's temporary folder.
        for name, value in (('RECORDINGS_DIR', recordings),
                            ('RECORDINGS_INDEX_FILE', os.path.join(recordings, 'index.json')),
                            ('BAG_PROCESSES_FILE', os.path.join(recordings, 'bag_processes.json')),
                            ('PROCESS_TABLE', FakeProcTable())):
            patcher = mock.patch.object(flask_app, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.popen = mock.MagicMock()
        self.popen.return_value.pid = FAKE_PID
        self.popen.return_value.poll.return_value = None
        patcher = mock.patch.object(flask_app.subprocess, 'Popen', self.popen)
        patcher.start()
        self.addCleanup(patcher.stop)
        flask_app._replay.update({'proc': None, 'id': None})
        self.addCleanup(flask_app._replay.update, {'proc': None, 'id': None})
        flask_app.app.config.update(TESTING=True)
        self.client = flask_app.app.test_client()

    def app_replay_reset(self):
        self.flask_app._replay.update({'proc': None, 'id': None})

    def _recording(self, topics, rec_id='rec_1'):
        bag = os.path.join(self.flask_app.RECORDINGS_DIR, rec_id)
        os.makedirs(bag)
        with open(os.path.join(bag, 'metadata.yaml'), 'w') as f:
            json.dump({'rosbag2_bagfile_information': {'topics_with_message_count': [
                {'topic_metadata': {'name': t}} for t in topics]}}, f)
        entries = self.flask_app.read_recordings_index()
        entries.append({'id': rec_id, 'name': rec_id, 'status': 'complete'})
        self.flask_app.write_recordings_index(entries)

    def _start(self, body=None, rec_id='rec_1'):
        return self.client.post(f'/api/recordings/{rec_id}/replay/start', json=body or {})

    def test_without_a_topic_list_only_allowlisted_topics_play_and_skips_are_listed(self):
        for number, topic in enumerate(DENIED):
            with self.subTest(topic=topic):
                rec_id = f'rec_{number}'
                self._recording(['/ui/map', '/scan_filtered', topic], rec_id)
                self.popen.reset_mock()
                self.app_replay_reset()
                response = self._start(rec_id=rec_id)
                self.assertEqual(response.status_code, 201, response.get_json())
                cmd = self.popen.call_args.args[0]
                self.assertEqual(cmd[cmd.index('--topics') + 1:], ['/ui/map'])
                body = response.get_json()
                self.assertEqual(body['topics'], ['/ui/map'])
                self.assertEqual([s['topic'] for s in body['skipped']],
                                 ['/scan_filtered', topic])

    def test_request_naming_each_denied_topic_is_a_400_before_any_process(self):
        self._recording(['/ui/map'])
        for topic in DENIED:
            with self.subTest(topic=topic):
                response = self._start({'topics': ['/ui/map', topic]})
                self.assertEqual(response.status_code, 400)
                self.assertIn(topic, response.get_json()['message'])
        self.popen.assert_not_called()

    def test_requested_topic_off_the_allowlist_is_skipped_not_refused(self):
        self._recording(['/ui/map', '/scan_filtered'])
        response = self._start({'topics': ['/ui/map', '/scan_filtered']})
        self.assertEqual(response.status_code, 201)
        self.assertEqual(response.get_json()['skipped'],
                         [{'topic': '/scan_filtered',
                           'reason': 'not on the replay telemetry allowlist'}])

    def test_nothing_to_play_is_a_409_and_starts_no_process(self):
        self._recording(['/scan_filtered', '/cmd_vel'])
        response = self._start()
        self.assertEqual(response.status_code, 409)
        self.popen.assert_not_called()

    def test_play_passes_only_allowlisted_topics(self):
        self._recording(['/ui/map', '/scan_filtered', '/battery_status'])
        response = self._start()
        self.assertEqual(response.status_code, 201, response.get_json())
        cmd = self.popen.call_args.args[0]
        self.assertEqual(cmd[:3], ['ros2', 'bag', 'play'])
        self.assertEqual(cmd[cmd.index('--topics') + 1:], ['/ui/map', '/battery_status'])
        self.assertEqual(response.get_json()['topics'], ['/ui/map', '/battery_status'])

    def test_invalid_allowlist_disables_replay(self):
        self._recording(['/ui/map'])
        bad = os.path.join(self.tmp.name, 'bad.yaml')
        with open(bad, 'w') as f:
            f.write('replay_allowed_topics: [/ui/map, /tf]\n')
        with mock.patch.object(self.flask_app, 'REPLAY_ALLOWLIST_FILE', bad):
            response = self._start()
        self.assertEqual(response.status_code, 500)
        self.popen.assert_not_called()

    def test_real_process_with_unreadable_details_is_stopped_and_returns_500(self):
        # #44 is not weakened for replay: a real child whose /proc details
        # cannot be read is stopped and reaped, and the request fails.
        from openamr_ui_package import bag_processes

        self._recording(['/ui/map'])
        children = []

        def spawn(_cmd, **kwargs):
            child = REAL_POPEN(['sleep', '30'], **kwargs)
            children.append(child)
            return child
        self.popen.side_effect = spawn
        self.addCleanup(lambda: [c.kill() for c in children if c.poll() is None])
        unreadable = bag_processes.ProcTable(proc_root=os.path.join(self.tmp.name, 'no_proc'))
        with mock.patch.object(self.flask_app, 'PROCESS_TABLE', unreadable):
            response = self._start()
        self.assertEqual(response.status_code, 500)
        self.assertIn('could not be read', response.get_json()['message'])
        (child,) = children
        self.assertIsNotNone(child.poll(), 'the bag child is still running')
        self.assertIsNone(self.flask_app._replay['proc'])
        self.assertEqual(bag_processes.read_state(self.flask_app.BAG_PROCESSES_FILE), [])


if __name__ == '__main__':
    unittest.main()
