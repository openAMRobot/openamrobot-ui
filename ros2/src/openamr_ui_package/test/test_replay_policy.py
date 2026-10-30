"""Replay topic policy (openamrobot-ui#33).

Pure test: imports only replay_policy and PyYAML, so it runs with
``python3 -m pytest test/test_replay_policy.py`` without ROS.
"""

import os
import sys

import pytest

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PACKAGE_DIR)

from openamr_ui_package import replay_policy  # noqa: E402

ALLOWLIST_FILE = os.path.join(PACKAGE_DIR, 'param', 'replay_topics.yaml')

# One example per denied class named in the policy.
DENIED = [
    '/cmd_vel',
    '/ui/cmd_vel',
    '/cmd_vel_nav',
    '/diff_cont/cmd_vel_unstamped',
    '/tf',
    '/tf_static',
    '/goal_pose',
    '/initialpose',
    '/safety/estop_state',
    '/safety',
    '/navigate_to_pose/_action/status',
    '/navigate_to_pose/_action/feedback',
    '/map_server/load_map/_service_event',
]


@pytest.mark.parametrize('topic', DENIED)
def test_each_denied_class_has_a_reason(topic):
    assert replay_policy.denied_reason(topic)


@pytest.mark.parametrize('topic', ['/ui/map', '/battery_status', '/plan', '/rosout'])
def test_telemetry_is_not_denied(topic):
    assert replay_policy.denied_reason(topic) is None


def test_package_allowlist_loads_and_names_no_denied_topic():
    allowlist = replay_policy.load_allowlist(ALLOWLIST_FILE)
    assert allowlist
    assert all(replay_policy.denied_reason(t) is None for t in allowlist)


def test_allowlist_naming_a_denied_topic_is_refused(tmp_path):
    bad = tmp_path / 'replay_topics.yaml'
    bad.write_text('replay_allowed_topics: [/ui/map, /cmd_vel]\n')
    with pytest.raises(ValueError, match='/cmd_vel'):
        replay_policy.load_allowlist(str(bad))


@pytest.mark.parametrize('topic', DENIED)
def test_request_with_a_denied_topic_is_rejected(topic):
    with pytest.raises(replay_policy.ReplayPolicyError, match='never publishes'):
        replay_policy.select_topics(['/ui/map', topic], ['/ui/map', topic], [topic, '/ui/map'])


@pytest.mark.parametrize('topic', DENIED)
def test_recording_with_a_denied_topic_is_rejected(topic):
    with pytest.raises(replay_policy.ReplayPolicyError, match='This recording'):
        replay_policy.select_topics(None, ['/ui/map', topic], ['/ui/map'])


def test_requested_topic_off_the_allowlist_is_rejected():
    with pytest.raises(replay_policy.ReplayPolicyError, match='allowlist: /scan'):
        replay_policy.select_topics(['/scan'], ['/scan'], ['/ui/map'])


def test_only_allowlisted_topics_are_played_from_a_recording():
    topics = replay_policy.select_topics(
        None, ['/ui/map', '/scan_filtered', '/battery_status'], ['/ui/map', '/battery_status'])
    assert topics == ['/ui/map', '/battery_status']


def test_requested_subset_is_played():
    topics = replay_policy.select_topics(
        ['/battery_status'], ['/ui/map', '/battery_status'], ['/ui/map', '/battery_status'])
    assert topics == ['/battery_status']


def test_recording_without_allowlisted_topics_is_rejected():
    with pytest.raises(replay_policy.ReplayPolicyError, match='no topics'):
        replay_policy.select_topics(None, ['/scan_filtered'], ['/ui/map'])


@pytest.mark.parametrize('requested', [[], 'cmd', [1]])
def test_malformed_request_is_rejected(requested):
    with pytest.raises(replay_policy.ReplayPolicyError):
        replay_policy.select_topics(requested, ['/ui/map'], ['/ui/map'])


def test_bag_topics_are_read_from_metadata(tmp_path):
    (tmp_path / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  topics_with_message_count:\n'
        '    - topic_metadata: {name: /ui/map}\n'
        '    - topic_metadata: {name: /cmd_vel}\n')
    assert replay_policy.read_bag_topics(str(tmp_path)) == ['/ui/map', '/cmd_vel']


def test_unreadable_metadata_is_rejected(tmp_path):
    with pytest.raises(replay_policy.ReplayPolicyError, match='could not be read'):
        replay_policy.read_bag_topics(str(tmp_path))
