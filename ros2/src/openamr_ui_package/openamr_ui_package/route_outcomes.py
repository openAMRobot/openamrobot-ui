"""Per-waypoint and per-route outcomes for route following (openamrobot-ui#36).

A route succeeds only if every waypoint's NavigateToPose goal SUCCEEDED.
Imports nothing from ROS, so it is tested with plain ``python3 -m pytest``.
"""

SUCCEEDED = 'SUCCEEDED'
ABORTED = 'ABORTED'
CANCELED = 'CANCELED'
TIMEOUT = 'TIMEOUT'
UNKNOWN = 'UNKNOWN'

# Seconds one waypoint may take before its goal is canceled and the route
# ends with TIMEOUT. Proposed default, owner to confirm: at the UI's 0.2 m/s
# linear limit this allows a 60 m leg. Overridden by the waypoint_timeout_s
# parameter of waypoint_nav.
DEFAULT_WAYPOINT_TIMEOUT_S = 300.0


# nav2_simple_commander's TaskResult names, by the Nav2 goal status behind
# them: BasicNavigator.getResult() reports GoalStatus ABORTED as FAILED.
_TASK_RESULT_OUTCOMES = {
    'SUCCEEDED': SUCCEEDED,
    'FAILED': ABORTED,
    'CANCELED': CANCELED,
}


def outcome_from_task_result(result):
    """Map a nav2_simple_commander TaskResult (or its name) to an outcome."""
    return _TASK_RESULT_OUTCOMES.get(getattr(result, 'name', result), UNKNOWN)


def route_outcome(waypoint_outcomes, waypoint_count):
    """Return the route outcome and a one-line operator message.

    SUCCEEDED only when there was at least one waypoint and every one of
    them SUCCEEDED; otherwise the outcome of the first waypoint that did not.
    """
    if waypoint_count == 0:
        return UNKNOWN, 'Route not run: it has no waypoints.'
    for number, outcome in enumerate(waypoint_outcomes, start=1):
        if outcome != SUCCEEDED:
            return outcome, (
                f'Route {outcome} at waypoint {number} of {waypoint_count}; '
                f'{number - 1} waypoint(s) reached.')
    if len(waypoint_outcomes) == waypoint_count:
        return SUCCEEDED, f'Route SUCCEEDED: all {waypoint_count} waypoints reached.'
    number = len(waypoint_outcomes) + 1
    return CANCELED, (
        f'Route CANCELED before waypoint {number} of {waypoint_count}; '
        f'{number - 1} waypoint(s) reached.')
