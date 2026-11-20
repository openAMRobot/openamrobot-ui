#!/usr/bin/env python3
"""Probe the restricted rosbridge of twin.launch.py from a client's point of view.

  python3 scripts/twin_bridge_probe.py --url ws://127.0.0.1:9090 --out probe.json

Each check sends one rosbridge v2 operation and records what came back.
rosbridge_suite 2.7.1 refuses an operation outside its globs without telling
the client (it logs "No match found ..., cancelling ..." on the bridge node),
so the workflow also matches that log line per check and checks the ROS side
(no publisher created) with `ros2 topic info -v`. --remote-host adds a connection
attempt to a non-loopback address of this machine, which must be refused.

Exit code 0 only when every check matched its expectation. Needs the
`websockets` package (python3-websockets, 10.x API). Simulation tooling only.
"""
import argparse
import asyncio
import json
import sys

import websockets


async def _exchange(ws, message, wait_s=3.0):
    await ws.send(json.dumps(message))
    replies = []
    try:
        while True:
            raw = await asyncio.wait_for(ws.recv(), timeout=wait_s)
            replies.append(json.loads(raw))
            if replies[-1].get('op') in ('service_response', 'action_result'):
                break
    except asyncio.TimeoutError:
        pass
    return replies


def _refused(replies):
    # rosbridge 2.7.1 refuses outside its globs silently: it logs "No match found ...
    # cancelling ..." on its own node and sends the client nothing. So a refusal is
    # "nothing useful came back"; the workflow matches the bridge log line per check.
    for r in replies:
        if r.get('op') == 'service_response' and r.get('result', False):
            return False, 'service answered: ' + json.dumps(r.get('values'))[:200]
        if r.get('op') in ('publish', 'action_result', 'action_feedback'):
            return False, 'data came back: ' + r.get('op')
    return True, 'no reply' if not replies else json.dumps(replies)[:300]


def _answered(replies):
    for r in replies:
        if r.get('op') == 'service_response' and r.get('result', False):
            return True, json.dumps(r.get('values'))[:200]
    return False, json.dumps(replies)[:300]


CHECKS = [
    # (id, expectation, message)
    ('publish_cmd_vel_refused', 'refused', {
        'op': 'advertise', 'id': 'a1', 'topic': '/cmd_vel', 'type': 'geometry_msgs/msg/Twist'}),
    ('publish_ui_cmd_vel_refused_without_manual_drive', 'refused', {
        'op': 'advertise', 'id': 'a2', 'topic': '/ui/cmd_vel',
        'type': 'geometry_msgs/msg/TwistStamped'}),
    ('publish_initialpose_refused', 'refused', {
        'op': 'advertise', 'id': 'a3', 'topic': '/initialpose',
        'type': 'geometry_msgs/msg/PoseWithCovarianceStamped'}),
    ('subscribe_cmd_vel_nav_refused', 'refused', {
        'op': 'subscribe', 'id': 's1', 'topic': '/cmd_vel_nav', 'type': 'geometry_msgs/msg/Twist'}),
    ('nav2_set_parameters_refused', 'refused', {
        'op': 'call_service', 'id': 'c1', 'service': '/controller_server/set_parameters',
        'type': 'rcl_interfaces/srv/SetParameters', 'args': {'parameters': []}}),
    ('lifecycle_change_state_refused', 'refused', {
        'op': 'call_service', 'id': 'c2', 'service': '/bt_navigator/change_state',
        'type': 'lifecycle_msgs/srv/ChangeState', 'args': {'transition': {'id': 4}}}),
    ('rosapi_set_param_refused', 'refused', {
        'op': 'call_service', 'id': 'c3', 'service': '/rosapi/set_param',
        'args': {'name': '/controller_server:FollowPath.max_vel_x', 'value': '5.0'}}),
    ('send_action_goal_refused', 'refused', {
        'op': 'send_action_goal', 'id': 'g1', 'action': '/navigate_to_pose',
        'action_type': 'nav2_msgs/action/NavigateToPose', 'args': {}}),
    ('rosapi_get_time_allowed', 'answered', {
        'op': 'call_service', 'id': 'c4', 'service': '/rosapi/get_time'}),
    ('lifecycle_get_state_allowed', 'answered', {
        'op': 'call_service', 'id': 'c5', 'service': '/bt_navigator/get_state',
        'type': 'lifecycle_msgs/srv/GetState', 'args': {}}),
]


async def probe(url, remote_url):
    results = []
    async with websockets.connect(url, open_timeout=10) as ws:
        for check_id, expect, message in CHECKS:
            replies = await _exchange(ws, message)
            ok, detail = (_refused if expect == 'refused' else _answered)(replies)
            results.append({'check': check_id, 'expect': expect,
                            'verdict': 'PASS' if ok else 'FAIL', 'detail': detail})
    if remote_url:
        try:
            async with websockets.connect(remote_url, open_timeout=5):
                verdict, detail = 'FAIL', 'connected'
        except (OSError, asyncio.TimeoutError, websockets.exceptions.WebSocketException) as error:
            verdict, detail = 'PASS', f'{type(error).__name__}: {error}'
        results.append({'check': 'non_loopback_connection_refused', 'expect': 'refused',
                        'verdict': verdict, 'detail': f'{remote_url}: {detail}'})
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='ws://127.0.0.1:9090')
    parser.add_argument('--remote-host', default='')
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    port = args.url.rsplit(':', 1)[-1]
    remote = f'ws://{args.remote_host}:{port}' if args.remote_host else ''
    results = asyncio.run(probe(args.url, remote))
    for r in results:
        print(f"{r['verdict']:4} {r['check']}: {r['detail']}")
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as handle:
            json.dump(results, handle, indent=2)
    failed = [r for r in results if r['verdict'] != 'PASS']
    print(f'{len(results) - len(failed)} of {len(results)} checks as expected')
    return 1 if failed or not results else 0


if __name__ == '__main__':
    sys.exit(main())
