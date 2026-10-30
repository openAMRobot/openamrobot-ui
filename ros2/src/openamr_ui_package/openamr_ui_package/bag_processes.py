"""Find and stop ros2 bag processes left behind by an earlier Flask run.

flask_app.py starts ``ros2 bag record`` and ``ros2 bag play`` as child
processes and keeps their handles only in memory. If Flask restarts, those
processes keep running with no owner. Each one is therefore written to a
state file with its pid, its start time and its command line. On the next
start, a process is terminated only when all three still match the live
process, so a pid that the kernel has since given to another process is
never signalled.

Imports nothing from ROS or Flask, so it is tested with ``python3 -m pytest``.
"""

import json
import os
import signal
import time

# Seconds to wait for a bag process to exit after SIGINT before SIGKILL.
# Basis: flask_app.py already waits 10 s for ros2 bag after SIGINT.
TERMINATE_TIMEOUT_S = 10.0


class ProcTable:
    """Reads and signals live processes through Linux /proc."""

    def __init__(self, proc_root='/proc'):
        self.proc_root = proc_root

    def identity(self, pid):
        """Return (start_time, cmdline) for a live pid, or None if it is gone."""
        try:
            with open(os.path.join(self.proc_root, str(pid), 'stat'), encoding='utf-8') as f:
                stat = f.read()
            with open(os.path.join(self.proc_root, str(pid), 'cmdline'), 'rb') as f:
                raw = f.read()
        except OSError:
            return None
        # Field 22 (starttime, clock ticks since boot). The command name in
        # field 2 may contain spaces, so count fields after its closing ')'.
        fields = stat[stat.rindex(')') + 2:].split()
        if fields[0] == 'Z':
            return None
        cmdline = [part.decode('utf-8', 'replace') for part in raw.split(b'\0') if part]
        return int(fields[19]), cmdline

    def terminate(self, pid, expected):
        """Stop pid with SIGINT, then SIGKILL, re-checking identity each time."""
        for sig in (signal.SIGCONT, signal.SIGINT):
            if self.identity(pid) != expected:
                return
            os.kill(pid, sig)
        deadline = time.monotonic() + TERMINATE_TIMEOUT_S
        while time.monotonic() < deadline:
            if self.identity(pid) != expected:
                return
            time.sleep(0.1)
        if self.identity(pid) == expected:
            os.kill(pid, signal.SIGKILL)


def read_state(state_file):
    """Return the recorded bag processes, or [] when the file is missing or unreadable."""
    try:
        with open(state_file, encoding='utf-8') as f:
            entries = json.load(f)
    except (OSError, ValueError):
        return []
    return [e for e in entries if isinstance(e, dict)] if isinstance(entries, list) else []


def write_state(state_file, entries):
    """Replace the state file atomically."""
    os.makedirs(os.path.dirname(state_file), exist_ok=True)
    tmp = f'{state_file}.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(entries, f, indent=2, sort_keys=True)
    os.replace(tmp, state_file)


def remember(state_file, table, kind, recording_id, pid):
    """Record a bag process the UI has just started."""
    identity = table.identity(pid)
    if identity is None:
        return
    start_time, cmdline = identity
    entries = [e for e in read_state(state_file) if e.get('pid') != pid]
    entries.append({'kind': kind, 'recording_id': recording_id, 'pid': pid,
                    'start_time': start_time, 'cmdline': cmdline})
    write_state(state_file, entries)


def forget(state_file, pid):
    """Drop a bag process that the UI stopped or reaped itself."""
    entries = read_state(state_file)
    remaining = [e for e in entries if e.get('pid') != pid]
    if len(remaining) != len(entries):
        write_state(state_file, remaining)


def recover(state_file, table):
    """Terminate leftover bag processes whose pid, start time and command line match.

    Returns one report per state entry: its fields plus ``terminated``.
    Every entry is cleared from the state file, matched or not: an entry
    that no longer matches belongs to a process that has already exited.
    """
    reports = []
    for entry in read_state(state_file):
        expected = (entry.get('start_time'), entry.get('cmdline'))
        pid = entry.get('pid')
        # pid 0, 1 and negative values address process groups or init.
        valid_pid = isinstance(pid, int) and not isinstance(pid, bool) and pid > 1
        matched = valid_pid and table.identity(pid) == expected
        if matched:
            table.terminate(pid, expected)
        reports.append(dict(entry, terminated=matched))
    if reports:
        write_state(state_file, [])
    return reports
