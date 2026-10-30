"""Which recorded topics a replay may publish (openamrobot-ui#33).

Replay only plays topics on an explicit telemetry allowlist, passed to
``ros2 bag play --topics``. A request or a recording that contains a command,
transform, goal, pose-reset, safety, action or service topic is rejected
before any process starts. This module imports nothing from ROS or Flask so
it can be tested with plain ``python3 -m pytest``.
"""

import os

import yaml

# Exact topic names that must never be replayed.
DENIED_EXACT = {
    '/cmd_vel': 'velocity command topic',
    '/ui/cmd_vel': 'velocity command topic',
    '/tf': 'transform topic',
    '/tf_static': 'transform topic',
    '/goal_pose': 'navigation goal topic',
    '/initialpose': 'pose reset topic',
}


class ReplayPolicyError(ValueError):
    """A replay request that must be refused, with the reason for the operator."""


def denied_reason(topic):
    """Return why ``topic`` may never be replayed, or None if it is not denied."""
    if topic in DENIED_EXACT:
        return f'{topic} is a {DENIED_EXACT[topic]}'
    segments = [part for part in topic.split('/') if part]
    if any(part.startswith('cmd_vel') for part in segments):
        return f'{topic} is a velocity command topic (cmd_vel*)'
    if segments and segments[0] == 'safety':
        return f'{topic} is a safety topic (/safety/*)'
    if '_action' in segments:
        return f'{topic} is an action topic'
    if '_service' in segments or topic.endswith('/_service_event'):
        return f'{topic} is a service topic'
    return None


def load_allowlist(path):
    """Read the telemetry allowlist; refuse a list that names a denied topic."""
    with open(path, encoding='utf-8') as config_file:
        data = yaml.safe_load(config_file) or {}
    topics = data.get('replay_allowed_topics')
    if not isinstance(topics, list) or not all(isinstance(t, str) for t in topics):
        raise ValueError(f'{path}: replay_allowed_topics must be a list of topic names')
    for topic in topics:
        reason = denied_reason(topic)
        if reason:
            raise ValueError(f'{path}: allowlist names a denied topic: {reason}')
    return list(dict.fromkeys(topics))


def read_bag_topics(bag_path):
    """Return the topic names stored in a rosbag2 directory's metadata.yaml."""
    metadata_path = os.path.join(bag_path, 'metadata.yaml')
    try:
        with open(metadata_path, encoding='utf-8') as metadata_file:
            info = (yaml.safe_load(metadata_file) or {})['rosbag2_bagfile_information']
        return [entry['topic_metadata']['name']
                for entry in info.get('topics_with_message_count') or []]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise ReplayPolicyError(
            'The recording topic list could not be read, so it cannot be checked '
            f'for command topics ({error.__class__.__name__}).') from error


def select_topics(requested, bag_topics, allowlist):
    """Return the topics to pass to ``ros2 bag play --topics``.

    ``requested`` is the request's topic list or None. Without a request the
    selection is every topic in the recording. Any denied topic in the
    selection rejects the whole replay. Requested topics must all be on the
    allowlist; from a whole recording, topics off the allowlist are skipped.
    """
    if requested is not None:
        if not isinstance(requested, list) or not requested:
            raise ReplayPolicyError('topics must be a non-empty list of topic names.')
        if not all(isinstance(topic, str) for topic in requested):
            raise ReplayPolicyError('topics must be a non-empty list of topic names.')
        selection = requested
    else:
        selection = bag_topics

    reasons = [r for r in (denied_reason(topic) for topic in selection) if r]
    if reasons:
        source = 'The request' if requested is not None else 'This recording'
        raise ReplayPolicyError(
            f'{source} contains topics that replay never publishes: '
            + '; '.join(reasons) + '.')

    if requested is not None:
        missing = [topic for topic in requested if topic not in allowlist]
        if missing:
            raise ReplayPolicyError(
                'Not on the replay telemetry allowlist: ' + ', '.join(missing) + '.')
        absent = [topic for topic in requested if topic not in bag_topics]
        if absent:
            raise ReplayPolicyError('Not in this recording: ' + ', '.join(absent) + '.')

    topics = [topic for topic in selection if topic in allowlist]
    if not topics:
        raise ReplayPolicyError('This recording has no topics on the replay telemetry allowlist.')
    return topics
