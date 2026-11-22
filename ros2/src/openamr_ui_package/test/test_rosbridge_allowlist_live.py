"""
A running rosbridge with the launch file's allowlist refuses /cmd_vel.

Live test, companion to test_rosbridge_allowlist.py (which reads the launch
file only). It starts rosbridge_websocket (rosbridge_server is an exec_depend
of this package) with the parameters new_ui_launch.py passes, connects as a
browser would over a plain websocket, and publishes on /cmd_vel and on
/ui/cmd_vel. A ROS subscriber counts what arrives: /ui/cmd_vel must arrive
(the websocket path works) and /cmd_vel must not (the allowlist holds).

Without the allowlist parameters rosbridge forwards /cmd_vel, so this test
fails when the allowlist is removed from the launch file.

Simulation-free and hardware-free: this is a software check of rosbridge's
topic filter only, not safety evidence.
"""
import base64
import importlib.util
import json
import os
import random
import socket
import struct
import subprocess
import tempfile
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NEW_UI_LAUNCH = os.path.join(PKG_DIR, 'launch', 'new_ui_launch.py')
PUBLISH_COUNT = 20


def _launch_module():
    spec = importlib.util.spec_from_file_location('new_ui_launch', NEW_UI_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def _params_file(port, globs):
    lines = ['/**:', '  ros__parameters:', f'    port: {port}', '    address: "127.0.0.1"']
    lines += [f'    {name}: {json.dumps(value)}' for name, value in globs.items()]
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    handle.write('\n'.join(lines) + '\n')
    handle.close()
    return handle.name


class _WebSocket:
    """Minimal RFC 6455 text client: enough to send rosbridge JSON ops."""

    def __init__(self, port, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                self.sock = socket.create_connection(('127.0.0.1', port), timeout=2.0)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.2)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            'GET / HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n'
            f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
            'Sec-WebSocket-Version: 13\r\n\r\n').encode())
        reply = b''
        while b'\r\n\r\n' not in reply:
            chunk = self.sock.recv(4096)
            assert chunk, 'rosbridge closed the connection during the handshake'
            reply += chunk
        assert reply.startswith(b'HTTP/1.1 101'), reply[:80]

    def send(self, message):
        data = json.dumps(message).encode()
        mask = os.urandom(4)
        if len(data) < 126:
            header = struct.pack('!BB', 0x81, 0x80 | len(data))
        else:
            header = struct.pack('!BBH', 0x81, 0x80 | 126, len(data))
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        self.sock.sendall(header + mask + masked)

    def close(self):
        self.sock.close()


def _counter(received, topic):
    def count(_msg):
        received[topic] += 1
    return count


def test_rosbridge_refuses_cmd_vel_and_forwards_ui_cmd_vel():
    launch = _launch_module()
    globs = launch._glob_params(launch.ROSBRIDGE_ALLOWLIST)
    port = _free_port()
    params = _params_file(port, globs)
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(random.randint(100, 199))
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    for name in ('ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE'):
        os.environ[name] = env[name]
    bridge = subprocess.Popen(
        ['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
         '--ros-args', '--params-file', params],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    received = {'/cmd_vel': 0, '/ui/cmd_vel': 0}
    rclpy.init()
    node = rclpy.create_node('allowlist_live_probe')
    try:
        for msg_class, topic in ((Twist, '/cmd_vel'), (TwistStamped, '/ui/cmd_vel')):
            node.create_subscription(msg_class, topic, _counter(received, topic), 10)
        ws = _WebSocket(port)
        topics = (
            ('/cmd_vel', 'geometry_msgs/msg/Twist', {'linear': {'x': 0.1}}),
            ('/ui/cmd_vel', 'geometry_msgs/msg/TwistStamped', {'twist': {'linear': {'x': 0.1}}}),
        )
        for topic, msg_type, _ in topics:
            ws.send({'op': 'advertise', 'topic': topic, 'type': msg_type})
        # Discovery between rosbridge's publishers and this subscriber takes a moment.
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        for _ in range(PUBLISH_COUNT):
            for topic, _, msg in topics:
                ws.send({'op': 'publish', 'topic': topic, 'msg': msg})
            rclpy.spin_once(node, timeout_sec=0.05)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        ws.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        bridge.terminate()
        try:
            output = bridge.communicate(timeout=10)[0]
        except subprocess.TimeoutExpired:
            bridge.kill()
            output = bridge.communicate()[0]
        os.unlink(params)
    print(output[-3000:])
    assert received['/ui/cmd_vel'] > 0, (
        f'no /ui/cmd_vel message arrived; the websocket path itself failed: {received}')
    assert received['/cmd_vel'] == 0, (
        f'rosbridge forwarded {received["/cmd_vel"]} /cmd_vel messages: '
        'the allowlist does not hold')
