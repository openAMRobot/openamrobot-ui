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


# Concurrency. A hook between the registry read and write makes two threads
# meet at a barrier: without the registry lock both read the same old state
# and one write is lost; with the lock the second thread cannot reach its
# read, the first one's barrier wait times out and it finishes alone.
def _interleave(monkeypatch, *calls):
    import threading

    barrier = threading.Barrier(len(calls), timeout=0.5)

    def meet():
        try:
            barrier.wait()
        except threading.BrokenBarrierError:
            pass
    monkeypatch.setattr(bag_processes, '_between_read_and_write', meet)
    errors = []

    def run(call):
        try:
            call()
        except Exception as error:  # surfaced below
            errors.append(error)
    threads = [threading.Thread(target=run, args=(call,)) for call in calls]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert errors == []


def test_two_concurrent_remembers_both_end_up_in_the_registry(tmp_path, monkeypatch):
    table = FakeTable({10: (1, RECORD_CMD), 11: (2, RECORD_CMD)})
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')
    _interleave(
        monkeypatch,
        lambda: bag_processes.remember(state_file, table, 'record', 'a', 10),
        lambda: bag_processes.remember(state_file, table, 'replay', 'b', 11))
    assert sorted(e['pid'] for e in bag_processes.read_state(state_file)) == [10, 11]


def test_concurrent_remember_and_forget_keep_only_the_started_entry(tmp_path, monkeypatch):
    table = FakeTable({10: (1, RECORD_CMD), 11: (2, RECORD_CMD)})
    state_file = _state(tmp_path, table, ('replay', 'old', 10))
    _interleave(
        monkeypatch,
        lambda: bag_processes.forget(state_file, 10),
        lambda: bag_processes.remember(state_file, table, 'record', 'new', 11))
    assert [e['pid'] for e in bag_processes.read_state(state_file)] == [11]


def test_registry_lock_excludes_another_process(tmp_path):
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')
    probe = ('import fcntl, sys\n'
             'f = open(sys.argv[1], "a")\n'
             'try:\n'
             '    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)\n'
             'except BlockingIOError:\n'
             '    sys.exit(3)\n')
    with bag_processes.registry_lock(state_file):
        held = subprocess.run([sys.executable, '-c', probe, state_file + '.lock'])
    free = subprocess.run([sys.executable, '-c', probe, state_file + '.lock'])
    assert (held.returncode, free.returncode) == (3, 0)


def test_each_write_uses_a_unique_temporary_file_and_fsync(tmp_path, monkeypatch):
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')
    replaced, synced = [], []
    real_replace, real_fsync = os.replace, os.fsync
    monkeypatch.setattr(os, 'replace', lambda a, b: (replaced.append(a), real_replace(a, b)))
    monkeypatch.setattr(os, 'fsync', lambda fd: (synced.append(fd), real_fsync(fd)))
    bag_processes.write_state(state_file, [])
    bag_processes.write_state(state_file, [])
    assert len(set(replaced)) == 2 and len(synced) == 2
    for tmp in replaced:
        assert os.path.dirname(tmp) == os.path.dirname(state_file)
        assert tmp != state_file + '.tmp'
    assert os.listdir(os.path.dirname(state_file)) == ['bag_processes.json']


def test_failed_write_leaves_no_temporary_file(tmp_path, monkeypatch):
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')

    def broken_replace(a, b):
        raise OSError('disk full')
    monkeypatch.setattr(os, 'replace', broken_replace)
    try:
        bag_processes.write_state(state_file, [])
    except OSError:
        pass
    assert os.listdir(os.path.dirname(state_file)) == []


# Review of #44: a live child whose /proc identity cannot be read must not be
# accepted untracked. track_or_stop() stops and reaps it and raises.

class FakeChild:
    """A started bag child that stays alive until terminated."""

    def __init__(self, pid):
        self.pid = pid
        self.calls = []

    def poll(self):
        return None if 'terminate' not in self.calls else -2

    def terminate(self):
        self.calls.append('terminate')

    def kill(self):
        self.calls.append('kill')

    def wait(self, timeout=None):
        self.calls.append('wait')
        return -2


def test_live_child_with_unreadable_identity_is_stopped_reaped_and_reported(tmp_path):
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')
    table = FakeTable({})  # identity(pid) is None: /proc could not be read
    child = FakeChild(4242)
    assert child.poll() is None

    try:
        bag_processes.track_or_stop(state_file, table, child, 'record', 'rec')
    except bag_processes.UntrackedProcess as error:
        assert 'pid 4242' in str(error)
    else:
        raise AssertionError('an untracked live child was accepted')

    assert child.calls == ['terminate', 'wait']
    assert child.poll() is not None
    assert bag_processes.read_state(state_file) == []


def test_real_child_with_unreadable_identity_is_not_left_running(tmp_path):
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')
    table = bag_processes.ProcTable(proc_root=str(tmp_path / 'no_proc'))
    child = subprocess.Popen(['sleep', '30'])
    try:
        try:
            bag_processes.track_or_stop(state_file, table, child, 'replay', 'rec')
        except bag_processes.UntrackedProcess:
            pass
        else:
            raise AssertionError('an untracked live child was accepted')
        assert child.poll() is not None, 'the bag child is still running'
        assert bag_processes.read_state(state_file) == []
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()


def test_readable_child_is_tracked_and_left_running(tmp_path):
    state_file = str(tmp_path / 'recordings' / 'bag_processes.json')
    table = FakeTable({4242: (1000, RECORD_CMD)})
    child = FakeChild(4242)

    bag_processes.track_or_stop(state_file, table, child, 'record', 'rec')

    assert child.calls == []
    assert [e['pid'] for e in bag_processes.read_state(state_file)] == [4242]
