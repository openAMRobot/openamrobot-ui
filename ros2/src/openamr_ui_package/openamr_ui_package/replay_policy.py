"""Which recorded topics a replay may publish (openamrobot-ui#33).

Replay only plays topics on an explicit telemetry allowlist, passed to
``ros2 bag play --topics``. Only a request that explicitly names a command,
transform, goal, pose-reset, safety, action or service topic is rejected
(HTTP 400). A recording that contains such topics is not rejected: they are
skipped, like any topic off the allowlist, and reported with the reason,
and the allowlisted topics play. A replay with nothing left to play is
refused (HTTP 409). Every check runs before any process starts. This module
imports nothing from ROS or Flask so it can be tested with plain
``python3 -m pytest``.
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
    """A replay that must be refused; ``status`` is the HTTP status to return."""

    status = 409


class DeniedTopicRequested(ReplayPolicyError):
    """The request explicitly names a topic that replay never publishes."""

    status = 400


class MalformedTopics(ReplayPolicyError):
    """The request's topics field is not a non-empty list of topic names."""

    status = 422


class BagUnreadable(ReplayPolicyError):
    """The recording's topic list cannot be read, so it cannot be checked."""

    status = 500


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
        raise BagUnreadable(
            'The recording topic list could not be read, so it cannot be checked '
            f'for command topics ({error.__class__.__name__}).') from error


def select_topics(requested, bag_topics, allowlist):
    """Return (topics to pass to ``ros2 bag play --topics``, skipped topics).

    ``requested`` is the request's topic list or None. Without a request the
    selection is every topic in the recording. Only a request that names a
    denied topic is refused outright (DeniedTopicRequested, HTTP 400). Every
    other topic that cannot be played (denied topics found in the recording,
    topics off the allowlist, requested topics missing from the recording) is
    skipped and reported with its reason. Nothing left to play is a
    ReplayPolicyError (HTTP 409).
    """
    if requested is not None:
        if (not isinstance(requested, list) or not requested
                or not all(isinstance(topic, str) for topic in requested)):
            raise MalformedTopics('topics must be a non-empty list of topic names.')
        reasons = [r for r in (denied_reason(topic) for topic in requested) if r]
        if reasons:
            raise DeniedTopicRequested(
                'The request names topics that replay never publishes: '
                + '; '.join(reasons) + '.')
        selection = requested
    else:
        selection = bag_topics

    topics, skipped = [], []
    for topic in dict.fromkeys(selection):
        reason = denied_reason(topic)
        if reason:
            skipped.append({'topic': topic, 'reason': f'denied: {reason}'})
        elif topic not in allowlist:
            skipped.append({'topic': topic, 'reason': 'not on the replay telemetry allowlist'})
        elif topic not in bag_topics:
            skipped.append({'topic': topic, 'reason': 'not in this recording'})
        else:
            topics.append(topic)
    if not topics:
        raise ReplayPolicyError(
            'Nothing to replay: no requested or recorded topic is on the replay '
            'telemetry allowlist.')
    return topics, skipped
