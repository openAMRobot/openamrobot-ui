"""Leftover ros2 bag processes are stopped only on a full identity match.

Pure test: imports bag_processes only, so it runs with
``python3 -m pytest test/test_bag_processes.py`` without ROS. The last test
uses a real ``sleep`` process and Linux /proc.
"""

import os
import subprocess
import sys

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE_DIR)

from openamr_ui_package import bag_processes  # noqa: E402

RECORD_CMD = ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'bag', 'record', '-o', 'x']


class FakeTable:
    """A process table the test controls; records every terminate call."""

    def __init__(self, processes):
        self.processes = dict(processes)
        self.terminated = []

    def identity(self, pid):
        return self.processes.get(pid)

    def terminate(self, pid, expected):
        assert self.processes.get(pid) == expected
        self.terminated.append(pid)
        del self.processes[pid]


def _state(tmp_path, table, *started):
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')
    for kind, recording_id, pid in started:
        bag_processes.remember(state_file, table, kind, recording_id, pid)
    return state_file


def test_matching_process_is_terminated_and_reported(tmp_path):
    table = FakeTable({4242: (1000, RECORD_CMD)})
    state_file = _state(tmp_path, table, ('record', 'rec_1', 4242))
    reports = bag_processes.recover(state_file, table)
    assert table.terminated == [4242]
    assert reports[0]['recording_id'] == 'rec_1' and reports[0]['terminated']
    assert bag_processes.read_state(state_file) == []


def test_reused_pid_with_a_different_start_time_is_not_killed(tmp_path):
    table = FakeTable({4242: (1000, RECORD_CMD)})
    state_file = _state(tmp_path, table, ('record', 'rec_1', 4242))
    # The old process exited and the kernel gave pid 4242 to a new process,
    # even one with the same command line.
    table.processes[4242] = (2000, RECORD_CMD)
    reports = bag_processes.recover(state_file, table)
    assert table.terminated == []
    assert reports[0]['terminated'] is False
    assert table.processes[4242] == (2000, RECORD_CMD)


def test_reused_pid_with_a_different_command_line_is_not_killed(tmp_path):
    table = FakeTable({4242: (1000, RECORD_CMD)})
    state_file = _state(tmp_path, table, ('record', 'rec_1', 4242))
    table.processes[4242] = (1000, ['/usr/sbin/sshd', '-D'])
    bag_processes.recover(state_file, table)
    assert table.terminated == []


def test_exited_process_is_not_signalled_but_cleared(tmp_path):
    table = FakeTable({4242: (1000, RECORD_CMD)})
    state_file = _state(tmp_path, table, ('replay', 'rec_1', 4242))
    del table.processes[4242]
    reports = bag_processes.recover(state_file, table)
    assert table.terminated == [] and reports[0]['terminated'] is False
    assert bag_processes.read_state(state_file) == []


def test_only_the_matching_entry_of_several_is_terminated(tmp_path):
    table = FakeTable({10: (1, RECORD_CMD), 11: (2, RECORD_CMD)})
    state_file = _state(tmp_path, table, ('record', 'a', 10), ('replay', 'b', 11))
    table.processes[11] = (3, RECORD_CMD)
    bag_processes.recover(state_file, table)
    assert table.terminated == [10]


def test_process_group_pids_are_never_signalled(tmp_path):
    table = FakeTable({0: (1, RECORD_CMD), 1: (1, RECORD_CMD), -5: (1, RECORD_CMD)})
    state_file = str(tmp_path / 'bag_processes.json')
    bag_processes.write_state(state_file, [
        {'kind': 'record', 'recording_id': 'x', 'pid': pid, 'start_time': 1,
         'cmdline': RECORD_CMD} for pid in (0, 1, -5)])
    bag_processes.recover(state_file, table)
    assert table.terminated == []


def test_forget_removes_a_cleanly_stopped_process(tmp_path):
    table = FakeTable({4242: (1000, RECORD_CMD)})
    state_file = _state(tmp_path, table, ('record', 'rec_1', 4242))
    bag_processes.forget(state_file, 4242)
    assert bag_processes.recover(state_file, table) == []
    assert table.terminated == []


def test_corrupt_state_file_signals_nothing(tmp_path):
    state_file = tmp_path / 'bag_processes.json'
    state_file.write_text('{not json')
    table = FakeTable({4242: (1000, RECORD_CMD)})
    assert bag_processes.recover(str(state_file), table) == []
    assert table.terminated == []


def test_real_process_is_matched_by_proc_and_stopped(tmp_path):
    table = bag_processes.ProcTable()
    proc = subprocess.Popen(['sleep', '30'])
    try:
        state_file = _state(tmp_path, table, ('record', 'rec_1', proc.pid))
        entries = bag_processes.read_state(state_file)
        start_time, cmdline = entries[0]['start_time'], entries[0]['cmdline']
        assert cmdline == ['sleep', '30'] and isinstance(start_time, int)

        # A tampered start time stands in for a reused pid: left running.
        bag_processes.write_state(state_file, [dict(entries[0], start_time=start_time + 1)])
        bag_processes.recover(state_file, table)
        assert proc.poll() is None

        bag_processes.write_state(state_file, entries)
        reports = bag_processes.recover(state_file, table)
        assert reports[0]['terminated']
        assert proc.wait(timeout=5) is not None
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
