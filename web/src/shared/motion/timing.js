// Every timing number used by the manual drive path, with its basis.
// Platform values are read from openamr-platform-sw#90 at head
// 710c98bc3d384a3d0612e87ff5e8c0c00b804dc0.

// Platform value: ui_cmd_vel_relay_node STALE_AFTER_S. The relay drops a
// /ui/cmd_vel message whose stamp is older than this against the robot clock.
export const RELAY_STALE_AFTER_S = 0.25;

// Platform value: ui_cmd_vel_relay_node FUTURE_TOLERANCE_S. A stamp further
// ahead of the robot clock than this is dropped too.
export const RELAY_FUTURE_TOLERANCE_S = 0.01;

// 20 Hz while a manual source is held. Basis: at least 4 samples inside the
// relay's 0.25 s window and inside the twist_mux manual input timeout of
// 0.25 s (task decision for PR A1), so one lost sample never times out the input.
export const MANUAL_PUBLISH_PERIOD_MS = 50;

// Real robot: /rosapi/get_time resync period and sample window (task
// decision for PR A1): resync every 2 s, keep the lowest-RTT sample of the last 5.
export const CLOCK_RESYNC_PERIOD_MS = 2000;
export const CLOCK_SAMPLE_WINDOW = 5;

// Real robot: fixed extra margin subtracted from every stamp (task decision
// for PR A1). Stamp = estimatedRobotNow - (RTT/2 + 5 ms).
export const STAMP_EXTRA_MARGIN_S = 0.005;

// Disable manual drive when RTT/2 plus the margin exceeds this. Basis: the
// worst-case stamp age at creation is then at most 0.15 s, which leaves at
// least 0.1 s of the relay's 0.25 s window for delivery to the relay.
export const MAX_STAMP_AGE_AT_CREATION_S = 0.15;

// Real robot: disable manual drive when the last good /rosapi/get_time sample
// is older than this (task decision for PR A1: three missed 2 s resyncs).
export const MAX_SYNC_AGE_MS = 6000;

// Simulation: disable manual drive when no /clock message arrived for this
// long. Basis: same 0.15 s bound as above, since the stamp is the last /clock
// value and is at least this old when /clock goes quiet.
export const SIM_CLOCK_MAX_SILENCE_MS = 150;

// Simulation: rosbridge throttle for the /clock subscription. Proposed
// default, owner to confirm: Gazebo publishes /clock once per physics step,
// far faster than the browser needs; 10 ms (up to 100 Hz) adds at most 10 ms
// to the stamp age and keeps the websocket load small.
export const SIM_CLOCK_THROTTLE_MS = 10;

// Retry period for reading the relay's use_sim_time until it succeeds.
// Proposed default, owner to confirm: same as the 2 s clock resync period.
export const MODE_RETRY_PERIOD_MS = 2000;

// How often the clock and lock states are re-evaluated for time-based rules.
// Basis: equal to the 20 Hz publish period, so a sync that goes stale is
// noticed before the next manual sample would be stamped.
export const GATE_EVALUATE_PERIOD_MS = MANUAL_PUBLISH_PERIOD_MS;

// Stop lock: treat the lock as unknown when no /safety/stop_lock sample
// arrived for this long. Basis: safety_stop_gate_node republishes the lock on
// every 10 Hz tick (TICK_HZ = 10), so 1 s is 10 missed ticks. Proposed
// default, owner to confirm.
export const STOP_LOCK_STALE_AFTER_MS = 1000;
