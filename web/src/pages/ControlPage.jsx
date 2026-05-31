import React, {
  useContext,
  useRef,
  useEffect,
  useCallback,
  useState,
} from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { AppConfig } from "../shared/constants/index";
import { RosContext } from "../app/App";

import Map from "../components/Map";
import Joystick from "../components/Joystick";
import RobotState from "../components/RobotState";
import NavStatus from "../components/NavStatus";
import DockingControl from "../components/DockingControl";
import LifecycleStatus from "../components/LifecycleStatus";
import SystemHealth from "../components/SystemHealth";
import MapLayers from "../components/MapLayers";
import SystemAlerts from "../components/SystemAlerts";

const INITIAL_POSE_COV = [
  0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685,
];

const ControlPage = () => {
  const ros = useContext(RosContext);
  const mapRef = useRef(null);

  // mode: null | 'goal' | 'pose'
  const [mode, setModeState] = useState(null);
  const modeRef = useRef(null);
  const setMode = (m) => {
    modeRef.current = m;
    setModeState(m);
  };

  const [maxSpeed, setMaxSpeed] = useState(AppConfig.MAX_LINEAR_SPEED);

  const goalPoseTopic = useRef(null);
  const initialPoseTopic = useRef(null);
  const cancelClient = useRef(null);
  const cmdVelTopic = useRef(null);
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

    cmdVelTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
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

  const installCallback = useCallback(() => {
    if (!window.NAV2D) return;
    window.NAV2D._poseCallback = (pose) => {
      const m = modeRef.current;
      if (m === "goal") {
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

  // Re-install callback when mode changes so the closure is always fresh
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

  const emergencyStop = useCallback(() => {
    if (!cmdVelTopic.current) return;
    cmdVelTopic.current.publish(
      new window.ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 },
      }),
    );
    cancelGoal();
    toast.warn("Emergency stop — robot halted");
  }, [cancelGoal]);

  const sendHome = useCallback(() => {
    if (!goalPoseTopic.current) return;
    goalPoseTopic.current.publish(
      new window.ROSLIB.Message({
        header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      }),
    );
    window.NAV2D?.setGoalPose?.({
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    });
    toast.info("Navigating to home position (0, 0)");
  }, []);

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

      <div className="flex h-[calc(100vh-72px)] min-h-0 flex-col gap-3 overflow-y-auto py-3 xl:flex-row xl:overflow-hidden">
        {/* Left: Map */}
        <section className="flex min-h-0 w-full flex-col gap-2 xl:w-[60%]">
          <SystemAlerts />
          <NavStatus onCancelGoal={cancelGoal} />

          <div className="min-h-[300px] flex-1 xl:min-h-0">
            <Map ref={mapRef} />
          </div>

          {/* Map controls */}
          <div className="flex shrink-0 gap-2">
            {modeBtn("○ Goal Mode", "Goal", "goal", "● Goal Mode ON", "● Goal")}
            {modeBtn("⊕ Set Pose", "Pose", "pose", "● Click to Set Pose", "● Pose")}
            <button
              onClick={sendHome}
              className="flex-1 rounded-xl border border-borderSubtle bg-bgCard px-2 py-3 font-[RobotoMono] text-[10px] text-textWhiteHover transition-colors hover:border-themeBlue hover:text-themeBlue sm:px-3 sm:text-sm"
            >
              ⌂ Go Home
            </button>
          </div>
        </section>

        {/* Right: Controls */}
        <section className="grid min-h-0 w-full grid-cols-1 gap-3 xl:w-[40%] xl:grid-rows-[112px_150px_80px_minmax(0,1fr)]">
          <div className="grid gap-3 sm:grid-cols-[150px_1fr] xl:min-h-0">
            {/* Emergency stop */}
            <div className="flex items-center justify-center">
            <button
              onClick={emergencyStop}
              className="h-[104px] w-[104px] rounded-full border-4 border-statusRed bg-statusRed/10 font-[RobotoMono] text-base font-bold text-statusRed shadow-lg shadow-red-200 transition-all hover:bg-statusRed hover:text-white"
            >
              STOP
            </button>
            </div>

            {/* Speed slider */}
            <div className="flex flex-col justify-center rounded-xl border border-borderSubtle bg-bgCard p-3 font-[RobotoMono]">
              <div className="mb-2 flex items-center justify-between">
                <p className="text-xs uppercase tracking-wider text-themeTextGray">
                  Max Speed
                </p>
                <span className="text-sm font-semibold text-themeBlue">
                  {maxSpeed.toFixed(2)} m/s
                </span>
              </div>
              <input
                type="range"
                min={0.05}
                max={0.5}
                step={0.05}
                value={maxSpeed}
                onChange={(e) => setMaxSpeed(parseFloat(e.target.value))}
                className="w-full accent-themeBlue"
              />
              <div className="mt-1 flex justify-between text-[10px] text-themeTextGray opacity-60">
                <span>0.05</span>
                <span>0.5 m/s</span>
              </div>
            </div>
          </div>

          <div className="grid min-h-0 gap-3 sm:grid-cols-[180px_1fr]">
            {/* Joystick */}
            <div className="flex min-h-0 flex-col items-center justify-center gap-1 rounded-xl border border-borderSubtle bg-bgCard p-3">
              <p className="font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
                Manual Drive
              </p>
              <Joystick maxSpeed={maxSpeed} compact />
            </div>

            {/* Robot state */}
            <RobotState />
          </div>

          <MapLayers />

          <div className="grid min-h-0 grid-cols-[1.15fr_0.85fr] gap-4 rounded-xl border border-borderSubtle bg-bgCard p-4">
            <div className="min-h-0">
              <SystemHealth compact />
            </div>
            <div className="grid min-h-0 grid-rows-[1fr_auto] gap-4">
              <LifecycleStatus compact />
              <DockingControl compact />
            </div>
          </div>
        </section>
      </div>
    </>
  );
};

export default ControlPage;
