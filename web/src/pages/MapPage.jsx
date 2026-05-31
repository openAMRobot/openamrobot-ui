import React, {
  useContext,
  useState,
  useRef,
  useEffect,
  useCallback,
} from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants";

import Map from "../components/Map";
import Camera from "../components/Camera";
import Joystick from "../components/Joystick";
import NavStatus from "../components/NavStatus";
import MapLayers from "../components/MapLayers";
import SystemAlerts from "../components/SystemAlerts";

const INITIAL_POSE_COV = [
  0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685,
];

const MapPage = () => {
  const ros = useContext(RosContext);
  const mapRef = useRef(null);

  // mode: null | 'goal' | 'pose' | 'waypoint'
  const [mode, setModeState] = useState(null);
  const modeRef = useRef(null);
  const setMode = (m) => {
    modeRef.current = m;
    setModeState(m);
  };

  const [waypointQueue, setWaypointQueueState] = useState([]);
  const waypointQueueRef = useRef([]);
  const setWaypointQueue = (updater) => {
    const next =
      typeof updater === "function"
        ? updater(waypointQueueRef.current)
        : updater;
    waypointQueueRef.current = next;
    setWaypointQueueState(next);
  };

  const [queueExecuting, setQueueExecuting] = useState(false);
  const queueExecutingRef = useRef(false);
  const queueIdxRef = useRef(0);

  const goalPoseTopic = useRef(null);
  const initialPoseTopic = useRef(null);
  const cancelClient = useRef(null);
  const pendingInitialPoseRef = useRef(false);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    goalPoseTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.GOAL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseStamped",
    });

    initialPoseTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.INITIAL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    cancelClient.current = new window.ROSLIB.Service({
      ros,
      name: "/navigate_to_pose/_action/cancel_goal",
      serviceType: "action_msgs/CancelGoal",
    });

    const amclTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.AMCL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });
    amclTopic.subscribe(() => {
      if (!pendingInitialPoseRef.current) return;
      pendingInitialPoseRef.current = false;
      toast.success("Localization updated from AMCL");
    });

    return () => amclTopic.unsubscribe();
  }, [ros]);

  // Nav status: advance waypoint queue on Succeeded
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const statusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_STATUS_TOPIC,
      messageType: "action_msgs/GoalStatusArray",
    });

    statusTopic.subscribe((msg) => {
      if (!queueExecutingRef.current) return;
      if (!msg.status_list?.length) return;
      const latest = msg.status_list[msg.status_list.length - 1];

      if (latest.status === 4) {
        const next = queueIdxRef.current + 1;
        if (next < waypointQueueRef.current.length) {
          queueIdxRef.current = next;
          publishGoal(waypointQueueRef.current[next]);
          toast.info(
            `Waypoint ${next + 1} / ${waypointQueueRef.current.length}`,
          );
        } else {
          queueExecutingRef.current = false;
          setQueueExecuting(false);
          toast.success("All waypoints complete!");
        }
      } else if (latest.status === 5 || latest.status === 6) {
        queueExecutingRef.current = false;
        setQueueExecuting(false);
        toast.warn("Queue stopped: goal was canceled or failed");
      }
    });

    return () => statusTopic.unsubscribe();
  }, [ros]);

  const publishGoal = (pose) => {
    if (!goalPoseTopic.current) return;
    goalPoseTopic.current.publish(
      new window.ROSLIB.Message({
        header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x: pose.position.x, y: pose.position.y, z: 0 },
          orientation: {
            x: 0,
            y: 0,
            z: pose.orientation.z,
            w: pose.orientation.w,
          },
        },
      }),
    );
    window.NAV2D?.setGoalPose?.(pose);
  };

  // Install the direct NAV2D callback — fires synchronously from stagemouseup,
  // no DOM bubbling or setTimeout needed.
  const installCallback = useCallback(() => {
    if (!window.NAV2D) return;
    window.NAV2D._poseCallback = (pose) => {
      const m = modeRef.current;
      if (m === "goal") {
        publishGoal(pose);
        toast.success(
          `Goal: (${pose.position.x.toFixed(2)}, ${pose.position.y.toFixed(
            2,
          )})`,
        );
      } else if (m === "pose") {
        if (!initialPoseTopic.current) return;
        initialPoseTopic.current.publish(
          new window.ROSLIB.Message({
            header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
            pose: {
              pose: {
                position: { x: pose.position.x, y: pose.position.y, z: 0 },
                orientation: {
                  x: 0,
                  y: 0,
                  z: pose.orientation.z,
                  w: pose.orientation.w,
                },
              },
              covariance: INITIAL_POSE_COV,
            },
          }),
        );
        pendingInitialPoseRef.current = true;
        if (window.NAV2D?.clearTrail) window.NAV2D.clearTrail();
        window.NAV2D?.clearGoalPose?.();
        toast.success(
          `Initial pose set: (${pose.position.x.toFixed(
            2,
          )}, ${pose.position.y.toFixed(2)})`,
        );
        deactivateMode();
      } else if (m === "waypoint") {
        window.NAV2D?.clearGoalPose?.();
        const idx = waypointQueueRef.current.length;
        setWaypointQueue((prev) => [...prev, pose]);
        toast.info(`Waypoint ${idx + 1} added`);
      }
    };
  }, []);

  const activateMode = useCallback(
    (m) => {
      if (!window.NAV2D) return;
      window.NAV2D.arePointsSettable = true;
      installCallback();
      setMode(m);
    },
    [installCallback],
  );

  const deactivateMode = useCallback(() => {
    if (window.NAV2D) {
      window.NAV2D.arePointsSettable = false;
      window.NAV2D._poseCallback = null;
    }
    setMode(null);
  }, []);

  // Re-install callback whenever mode changes so the closure always has the right mode
  useEffect(() => {
    if (modeRef.current !== null) installCallback();
  }, [mode, installCallback]);

  const cancelGoal = useCallback(() => {
    if (!cancelClient.current) return;
    cancelClient.current.callService(
      new window.ROSLIB.ServiceRequest({
        goal_info: {
          goal_id: { uuid: new Array(16).fill(0) },
          stamp: { sec: 0, nanosec: 0 },
        },
      }),
      () => toast.info("Navigation canceled"),
      (err) => console.warn("Cancel failed:", err),
    );
  }, []);

  const executeQueue = useCallback(() => {
    if (!waypointQueueRef.current.length) return;
    queueIdxRef.current = 0;
    queueExecutingRef.current = true;
    setQueueExecuting(true);
    publishGoal(waypointQueueRef.current[0]);
    toast.info(`Executing ${waypointQueueRef.current.length} waypoints`);
  }, []);

  const stopQueue = useCallback(() => {
    queueExecutingRef.current = false;
    setQueueExecuting(false);
    cancelGoal();
  }, [cancelGoal]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (window.NAV2D) {
        window.NAV2D.arePointsSettable = false;
        window.NAV2D._poseCallback = null;
      }
    };
  }, []);

  const modeBtn = (label, shortLabel, m, activeLabel, shortActiveLabel) => {
    const active = mode === m;
    return (
      <button
        onClick={() => (active ? deactivateMode() : activateMode(m))}
        className={`flex-1 rounded-xl border px-2 py-3 font-[RobotoMono] text-[10px] font-semibold transition-colors sm:px-3 sm:text-sm ${
          active
            ? "border-themeBlue bg-themeBlue text-white"
            : "border-borderSubtle bg-bgCard text-themeBlue hover:border-themeBlue"
        }`}
      >
        <span className="sm:hidden">{active ? shortActiveLabel : shortLabel}</span>
        <span className="hidden sm:inline">{active ? activeLabel : label}</span>
      </button>
    );
  };

  return (
    <>
      <ToastContainer
        position="bottom-right"
        theme="light"
        toastStyle={{ backgroundColor: "#ffffff", border: "1px solid #c9d8e6" }}
      />

      <div className="flex h-[calc(100vh-72px)] min-h-0 flex-col gap-2 overflow-y-auto py-2 xl:overflow-hidden">
        <SystemAlerts />
        <NavStatus onCancelGoal={cancelGoal} />
        <MapLayers />

        {/* Map + Camera */}
        <section className="flex min-h-0 flex-1 flex-col gap-3 xl:flex-row">
          <div className="min-h-[300px] w-full xl:h-full xl:min-h-0 xl:w-[58%]">
            <Map ref={mapRef} />
          </div>
          <div className="min-h-[220px] w-full xl:h-full xl:min-h-0 xl:w-[42%]">
            <Camera />
          </div>
        </section>

        {/* Controls row */}
        <section className="flex w-full shrink-0 items-stretch gap-3 xl:h-[112px]">
          {/* Joystick */}
          <div className="flex w-[110px] shrink-0 flex-col items-center justify-center gap-1 rounded-xl border border-borderSubtle bg-bgCard p-2 sm:w-[124px] 2xl:w-[136px]">
            <p className="font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
              Manual
            </p>
            <Joystick compact />
          </div>

          {/* Mode buttons + queue */}
          <div className="flex min-w-0 flex-1 flex-col gap-2">
            <div className="flex gap-2">
              {modeBtn("○ Goal Mode", "Goal", "goal", "● Goal Mode ON", "● Goal")}
              {modeBtn("⊕ Set Pose", "Pose", "pose", "● Click to Set Pose", "● Pose")}
              {modeBtn("＋ Add Waypoint", "Waypoint", "waypoint", "● Adding Waypoints", "● Adding")}
            </div>

            <div className="rounded-xl border border-borderSubtle bg-bgCard px-3 py-1">
              <p className="font-[RobotoMono] text-xs leading-5 text-themeTextGray">
                {mode === "goal" &&
                  "Click map to navigate. Drag before releasing to set heading."}
                {mode === "pose" &&
                  "Click map to set AMCL initial pose. Drag to set heading. One-shot."}
                {mode === "waypoint" &&
                  "Each click adds a waypoint. Drag to set heading. Execute all below."}
                {!mode && "Select a mode above to interact with the map."}
              </p>
            </div>

            {waypointQueue.length > 0 && (
              <div className="rounded-xl border border-borderSubtle bg-bgCard p-3 font-[RobotoMono]">
                <div className="mb-2 flex items-center justify-between">
                  <p className="text-xs uppercase tracking-wider text-themeTextGray">
                    Waypoint Queue ({waypointQueue.length})
                  </p>
                  <button
                    onClick={() => {
                      setWaypointQueue([]);
                      stopQueue();
                    }}
                    className="text-xs text-statusRed hover:underline"
                  >
                    Clear
                  </button>
                </div>
                <div className="mb-3 flex flex-wrap gap-1.5">
                  {waypointQueue.map((wp, i) => (
                    <span
                      key={i}
                      className={`rounded border px-2 py-0.5 text-xs ${
                        queueExecuting && i === queueIdxRef.current
                          ? "border-themeBlue bg-themeBlue/20 text-themeBlue"
                          : "border-borderSubtle text-themeTextGray"
                      }`}
                    >
                      {i + 1}: ({wp.position.x.toFixed(1)},{" "}
                      {wp.position.y.toFixed(1)})
                    </span>
                  ))}
                </div>
                {queueExecuting ? (
                  <button
                    onClick={stopQueue}
                    className="w-full rounded-lg border border-statusRed bg-bgCard py-1.5 text-xs font-semibold text-statusRed transition-colors hover:bg-statusRed hover:text-white"
                  >
                    Stop Queue
                  </button>
                ) : (
                  <button
                    onClick={executeQueue}
                    className="w-full rounded-lg border border-themeBlue bg-themeBlue/10 py-1.5 text-xs font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
                  >
                    Execute Queue
                  </button>
                )}
              </div>
            )}
          </div>
        </section>
      </div>
    </>
  );
};

export default MapPage;
