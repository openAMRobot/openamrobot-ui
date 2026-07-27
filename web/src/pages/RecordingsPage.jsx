import React, { useEffect, useRef, useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { AppConfig } from "../shared/constants";
import {
  fetchRecordings,
  fetchRecordingsStatus,
  startRecording,
  stopRecording,
  deleteRecording,
  startReplay,
  stopReplay,
  pauseReplay,
  resumeReplay,
  downloadRecording,
} from "../features/recordings/recordingsApi";
import {
  DashboardCard,
  EmptyState,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";
import Button from "../shared/ui/Button";

const TOPIC_CHOICES = [
  {
    topic: AppConfig.SCAN_TOPIC,
    label: "Laser scan",
    description: "Raw distance readings from the robot's laser sensor",
  },
  {
    topic: AppConfig.ROBOT_POSE_TOPIC,
    label: "Odometry",
    description: "The robot's estimated position and speed from its wheels/motors",
  },
  {
    topic: AppConfig.MAP_TOPIC,
    label: "Map",
    description: "The map the robot is navigating on",
  },
  {
    topic: AppConfig.AMCL_POSE_TOPIC,
    label: "AMCL pose",
    description: "Where the robot thinks it is",
  },
  {
    topic: AppConfig.NAV_STATUS_TOPIC,
    label: "Nav status",
    description: "Navigation progress and outcome",
  },
  {
    topic: AppConfig.BATTERY_TOPIC,
    label: "Battery",
    description: "Battery charge level and status",
  },
  {
    topic: AppConfig.JOINT_STATES_TOPIC,
    label: "Joint states",
    description: "Position of movable robot parts",
  },
  {
    topic: AppConfig.TF_TOPIC,
    label: "TF",
    description: "How the robot's parts are positioned relative to each other",
  },
  {
    topic: AppConfig.TF_STATIC_TOPIC,
    label: "TF static",
    description: "Fixed robot geometry (rarely changes)",
  },
];

const STATUS_LABELS = {
  recording: "Recording",
  complete: "Complete",
  interrupted: "Didn't finish cleanly",
};

const Field = ({ label, hint, children }) => (
  <label className="block">
    <span className="text-xs font-semibold uppercase tracking-wider text-themeTextGray">
      {label}
    </span>
    <div className="mt-1.5">{children}</div>
    {hint ? <p className="mt-1 text-[11px] text-themeTextGray/70">{hint}</p> : null}
  </label>
);

const inputClass =
  "w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

const formatBytes = (bytes) => {
  if (!bytes) return "0 KB";
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
};

const formatDuration = (seconds) => {
  const s = Math.max(0, Math.floor(seconds));
  const mm = String(Math.floor(s / 60)).padStart(2, "0");
  const ss = String(s % 60).padStart(2, "0");
  return `${mm}:${ss}`;
};

const RecordingsPage = () => {
  const [recordings, setRecordings] = useState([]);
  const [status, setStatus] = useState({ recording: null, replay: null });
  const [nowTick, setNowTick] = useState(Date.now());

  const [form, setForm] = useState({ name: "", description: "" });
  const [allTopics, setAllTopics] = useState(true);
  const [selectedTopics, setSelectedTopics] = useState([]);
  const [replayRate, setReplayRate] = useState(1);
  const [stoppingReplay, setStoppingReplay] = useState(false);

  const pollRef = useRef(null);

  const refresh = async () => {
    try {
      const [recData, statusData] = await Promise.all([
        fetchRecordings(),
        fetchRecordingsStatus(),
      ]);
      setRecordings(recData.recordings || []);
      setStatus(statusData);
    } catch {
      // Backend unreachable — leave last-known state, don't thrash the UI.
    }
  };

  useEffect(() => {
    refresh();
    pollRef.current = setInterval(refresh, 2000);
    const tickId = setInterval(() => setNowTick(Date.now()), 1000);
    return () => {
      clearInterval(pollRef.current);
      clearInterval(tickId);
    };
  }, []);

  const toggleTopic = (topic) => {
    setSelectedTopics((prev) =>
      prev.includes(topic) ? prev.filter((t) => t !== topic) : [...prev, topic],
    );
  };

  const handleStart = async () => {
    const name = form.name.trim();
    if (!name) {
      toast.warn("Enter a name for this recording");
      return;
    }
    if (!allTopics && selectedTopics.length === 0) {
      toast.warn("Select at least one topic, or record all topics");
      return;
    }
    try {
      await startRecording({
        name,
        description: form.description.trim(),
        topics: allTopics ? null : selectedTopics,
      });
      toast.success(`Recording "${name}" started`);
      setForm({ name: "", description: "" });
      setSelectedTopics([]);
      setAllTopics(true);
      refresh();
    } catch (err) {
      toast.error(err.message);
    }
  };

  const handleStop = async () => {
    try {
      await stopRecording();
      toast.success("Recording stopped");
      refresh();
    } catch (err) {
      toast.error(err.message);
    }
  };

  const handleDelete = async (id, name) => {
    if (!window.confirm("Delete this recording? This can't be undone.")) return;
    try {
      await deleteRecording(id);
      toast.success(`Deleted "${name}"`);
      refresh();
    } catch (err) {
      toast.error(err.message);
    }
  };

  const handleReplay = async (id, name) => {
    try {
      await startReplay(id, replayRate);
      toast.success(`Replaying "${name}"`);
      refresh();
    } catch (err) {
      toast.error(err.message);
    }
  };

  const handleStopReplay = async () => {
    // ros2 bag play takes several seconds to shut down cleanly after
    // SIGINT (unlike recording, which stops almost instantly) — without
    // this, the button looks unresponsive for that whole window.
    setStoppingReplay(true);
    try {
      await stopReplay();
      toast.success("Replay stopped");
      refresh();
    } catch (err) {
      toast.error(err.message);
    } finally {
      setStoppingReplay(false);
    }
  };

  const handlePauseResume = async () => {
    try {
      if (status.replay?.paused) {
        await resumeReplay();
      } else {
        await pauseReplay();
      }
      refresh();
    } catch (err) {
      toast.error(err.message);
    }
  };

  const recording = status.recording;
  const replay = status.replay;
  const recordingElapsed = recording
    ? (nowTick - new Date(recording.startedAt).getTime()) / 1000
    : 0;
  const replayElapsed = replay
    ? (nowTick - new Date(replay.startedAt).getTime()) / 1000
    : 0;
  const replayingEntry = replay
    ? recordings.find((r) => r.id === replay.id)
    : null;

  return (
    <div className="sectionHeight space-y-5 py-4 sm:space-y-6 sm:py-6">
      <SectionHeader
        eyebrow="Debugging & lessons"
        title="Recordings"
        description="Record a session of everything the robot sensed and did, so you can play it back later — useful for debugging, demos, and training data. Replayed telemetry is always clearly labeled; it's never presented as a live robot."
      />

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          {recording ? "Recording in progress" : "Start a recording"}
        </p>

        {recording ? (
          <div className="mt-3 flex flex-wrap items-center justify-between gap-3">
            <div>
              <p className="text-sm font-semibold text-textWhiteHover">
                {recording.name}
              </p>
              <p className="font-[RobotoMono] text-xs text-themeTextGray">
                {recording.topics ? `${recording.topics.length} topic(s)` : "All topics"}
                {" · "}
                {formatDuration(recordingElapsed)} elapsed
              </p>
            </div>
            <StatusBadge status="active" label="Recording" pulse />
            <Button type="danger" onBtnClick={handleStop}>
              Stop recording
            </Button>
          </div>
        ) : (
          <>
            <div className="mt-4 grid gap-4 sm:grid-cols-2">
              <Field label="Name">
                <input
                  type="text"
                  value={form.name}
                  onChange={(e) => setForm((p) => ({ ...p, name: e.target.value }))}
                  placeholder="e.g. Warehouse loop demo"
                  className={inputClass}
                />
              </Field>
              <Field label="Description (optional)">
                <input
                  type="text"
                  value={form.description}
                  onChange={(e) =>
                    setForm((p) => ({ ...p, description: e.target.value }))
                  }
                  placeholder="What this recording is for"
                  className={inputClass}
                />
              </Field>
            </div>

            <div className="mt-4">
              <label className="flex items-center gap-2 text-sm text-textWhiteHover">
                <input
                  type="checkbox"
                  checked={allTopics}
                  onChange={(e) => setAllTopics(e.target.checked)}
                />
                Record all topics
              </label>

              {!allTopics && (
                <div className="mt-3 grid grid-cols-2 gap-2 sm:grid-cols-3">
                  {TOPIC_CHOICES.map(({ topic, label, description }) => (
                    <label
                      key={topic}
                      className="flex items-start gap-2 text-xs text-themeTextGray"
                    >
                      <input
                        type="checkbox"
                        checked={selectedTopics.includes(topic)}
                        onChange={() => toggleTopic(topic)}
                        className="mt-0.5"
                      />
                      <span>
                        {label}
                        {description ? (
                          <span className="block text-[10px] text-themeTextGray/60">
                            {description}
                          </span>
                        ) : null}
                      </span>
                    </label>
                  ))}
                </div>
              )}
            </div>

            <div className="mt-4 sm:max-w-xs">
              <Button type="orange" onBtnClick={handleStart}>
                Start recording
              </Button>
            </div>
          </>
        )}
      </DashboardCard>

      {replay && (
        <DashboardCard className="border border-statusYellow/30 bg-statusYellow/5 p-4">
          <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-statusYellow">
            Replaying
          </p>
          <div className="mt-3 flex flex-wrap items-center justify-between gap-3">
            <div>
              <p className="text-sm font-semibold text-textWhiteHover">
                {replayingEntry?.name || replay.id}
              </p>
              <p className="font-[RobotoMono] text-xs text-themeTextGray">
                Rate {replay.rate}x · {formatDuration(replayElapsed)} elapsed
                {replay.paused ? " · Paused" : ""}
              </p>
            </div>
            <div className="flex gap-2">
              <Button
                type={stoppingReplay ? "disabled" : ""}
                onBtnClick={handlePauseResume}
              >
                {replay.paused ? "Resume" : "Pause"}
              </Button>
              <Button
                type={stoppingReplay ? "disabled" : "danger"}
                onBtnClick={handleStopReplay}
              >
                {stoppingReplay
                  ? "Stopping… this can take a few seconds."
                  : "Stop replay"}
              </Button>
            </div>
          </div>
        </DashboardCard>
      )}

      <DashboardCard className="p-4">
        <p className="mb-3 font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Saved recordings
        </p>

        {recordings.length === 0 ? (
          <EmptyState
            title="No recordings yet"
            description="Start one above to see it here."
          />
        ) : (
          <div className="space-y-2">
            {recordings.map((entry) => {
              const isActive = recording?.id === entry.id || replay?.id === entry.id;
              return (
                <div
                  key={entry.id}
                  className="flex flex-wrap items-center justify-between gap-3 rounded-lg border border-borderSubtle bg-bgSurface px-3 py-2.5"
                >
                  <div className="min-w-0">
                    <div className="flex items-center gap-2">
                      <span className="truncate text-sm font-semibold text-textWhiteHover">
                        {entry.name}
                      </span>
                      <StatusBadge
                        status={
                          entry.status === "complete"
                            ? "success"
                            : entry.status === "recording"
                            ? "active"
                            : "warning"
                        }
                        label={STATUS_LABELS[entry.status] || entry.status}
                      />
                    </div>
                    <p className="mt-0.5 font-[RobotoMono] text-xs text-themeTextGray">
                      {entry.topics ? `${entry.topics.length} topic(s)` : "All topics"}
                      {" · "}
                      {formatBytes(entry.sizeBytes)}
                      {entry.startedAt && entry.endedAt
                        ? ` · ${formatDuration(
                            (new Date(entry.endedAt) - new Date(entry.startedAt)) / 1000,
                          )}`
                        : ""}
                    </p>
                    {entry.description ? (
                      <p className="mt-0.5 truncate text-xs text-themeTextGray/70">
                        {entry.description}
                      </p>
                    ) : null}
                  </div>

                  <div className="flex shrink-0 items-center gap-2">
                    {entry.status !== "recording" && (
                      <Button
                        type={isActive ? "disabled" : ""}
                        onBtnClick={() => handleReplay(entry.id, entry.name)}
                      >
                        Replay
                      </Button>
                    )}
                    {entry.status !== "recording" && (
                      <Button onBtnClick={() => downloadRecording(entry.id)}>
                        Download
                      </Button>
                    )}
                    <button
                      onClick={() => handleDelete(entry.id, entry.name)}
                      disabled={isActive}
                      className="text-themeTextGray hover:text-statusRed disabled:opacity-30"
                      aria-label={`Delete ${entry.name}`}
                      title={`Delete ${entry.name}`}
                    >
                      ×
                    </button>
                  </div>
                </div>
              );
            })}
          </div>
        )}
      </DashboardCard>

      <ToastContainer theme="dark" position="bottom-right" />
    </div>
  );
};

export default RecordingsPage;
