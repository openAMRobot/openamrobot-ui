import React, { useContext, useEffect, useMemo, useRef, useState } from "react";
import * as Blockly from "blockly";
import "blockly/blocks";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { RosContext, RosStatusContext } from "../app/App";
import {
  registerOpenAmrBlocks,
  workspaceToRobotPlan,
} from "../features/blocks/blockDefinitions";
import {
  executeRobotPlan,
  createRobotActionClient,
} from "../features/blocks/robotActions";
import { openAmrToolbox } from "../features/blocks/toolbox";

const STORAGE_KEY = "openamr_blockly_workspace";

const defaultWorkspace = {
  blocks: {
    languageVersion: 0,
    blocks: [
      {
        type: "openamr_start",
        id: "start",
        x: 48,
        y: 48,
        next: {
          block: {
            type: "openamr_navigate",
            id: "navigate",
            fields: { X: 0, Y: 0, YAW: 0 },
            next: {
              block: {
                type: "openamr_wait",
                id: "wait",
                fields: { SECONDS: 2 },
              },
            },
          },
        },
      },
    ],
  },
};

const actionLabel = (action) => {
  switch (action.type) {
    case "navigate":
      if (action.location) {
        return `Navigate to ${action.location}`;
      }
      return `Navigate to x ${action.x}, y ${action.y}, yaw ${action.yaw}`;
    case "wait":
      return `Wait ${action.seconds}s`;
    case "set_speed":
      return `Set speed ${action.linear} m/s, ${action.angular} rad/s`;
    case "drive_for":
      return `Drive ${action.linear} m/s for ${action.seconds}s`;
    case "rotate_for":
      return `Rotate ${action.angular} rad/s for ${action.seconds}s`;
    case "stop_movement":
      return "Stop movement";
    case "wait_nav_complete":
      return `Wait for navigation complete, timeout ${action.timeout}s`;
    case "repeat":
      return `${action.label === "patrol" ? "Patrol" : "Repeat"} ${
        action.times
      } times (${action.actions.length} nested steps)`;
    case "log":
      return `Log "${action.message}"`;
    case "battery_below":
      return `If battery below ${action.percent}%, run ${action.actions.length} nested steps`;
    case "set_mode":
      return `Set mode ${action.mode}`;
    case "dock":
      return "Dock robot";
    case "undock":
      return "Undock robot";
    case "stop":
      return "Emergency stop";
    default:
      return action.type;
  }
};

const loadBlocklyState = (serializedState) => {
  if (!serializedState) return defaultWorkspace;

  try {
    return JSON.parse(serializedState);
  } catch {
    localStorage.removeItem(STORAGE_KEY);
    return defaultWorkspace;
  }
};

const BlocksPage = () => {
  const ros = useContext(RosContext);
  const rosStatus = useContext(RosStatusContext);
  const blocklyDivRef = useRef(null);
  const workspaceRef = useRef(null);
  const stopRequestedRef = useRef(false);

  const [plan, setPlan] = useState([]);
  const [running, setRunning] = useState(false);
  const [activeStep, setActiveStep] = useState(null);

  const canRun = useMemo(
    () => rosStatus === "connected" && plan.length > 0 && !running,
    [plan.length, rosStatus, running],
  );

  useEffect(() => {
    registerOpenAmrBlocks();

    const workspace = Blockly.inject(blocklyDivRef.current, {
      toolbox: openAmrToolbox,
      grid: { spacing: 24, length: 2, colour: "#d8e4ed", snap: true },
      trashcan: true,
      move: { scrollbars: true, drag: true, wheel: true },
      zoom: {
        controls: true,
        wheel: true,
        startScale: 0.9,
        maxScale: 1.4,
        minScale: 0.5,
        scaleSpeed: 1.1,
      },
    });

    workspaceRef.current = workspace;

    Blockly.serialization.workspaces.load(
      loadBlocklyState(localStorage.getItem(STORAGE_KEY)),
      workspace,
    );

    const updatePlan = () => {
      setPlan(workspaceToRobotPlan(workspace));
    };

    updatePlan();
    workspace.addChangeListener(updatePlan);

    const resize = () => Blockly.svgResize(workspace);
    window.addEventListener("resize", resize);
    setTimeout(resize, 0);

    return () => {
      window.removeEventListener("resize", resize);
      workspace.dispose();
      workspaceRef.current = null;
    };
  }, []);

  const saveWorkspace = () => {
    if (!workspaceRef.current) return;
    const state = Blockly.serialization.workspaces.save(workspaceRef.current);
    localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
    toast.success("Blocks saved in this browser");
  };

  const loadWorkspace = () => {
    if (!workspaceRef.current) return;
    const savedState = localStorage.getItem(STORAGE_KEY);
    if (!savedState) {
      toast.info("No saved block program found");
      return;
    }
    Blockly.serialization.workspaces.load(
      loadBlocklyState(savedState),
      workspaceRef.current,
    );
    setPlan(workspaceToRobotPlan(workspaceRef.current));
    toast.success("Blocks loaded");
  };

  const clearWorkspace = () => {
    if (!workspaceRef.current) return;
    workspaceRef.current.clear();
    Blockly.serialization.workspaces.load(
      defaultWorkspace,
      workspaceRef.current,
    );
    setPlan(workspaceToRobotPlan(workspaceRef.current));
  };

  const runPlan = async () => {
    if (!canRun) return;
    stopRequestedRef.current = false;
    setRunning(true);
    setActiveStep(0);

    try {
      await executeRobotPlan(ros, plan, {
        shouldStop: () => stopRequestedRef.current,
        onStep: (index) => setActiveStep(index),
      });
      if (stopRequestedRef.current) {
        toast.info("Block program stopped");
      } else {
        toast.success("Block program sent");
      }
    } catch (err) {
      toast.error(err.message || "Block program failed");
    } finally {
      setRunning(false);
      setActiveStep(null);
    }
  };

  const stopPlan = () => {
    stopRequestedRef.current = true;
    if (ros && window.ROSLIB) {
      createRobotActionClient(ros).stop();
    }
    setRunning(false);
    setActiveStep(null);
    toast.warn("Stop command sent");
  };

  return (
    <section className="sectionHeight grid min-h-0 grid-cols-1 gap-4 py-4 lg:grid-cols-[minmax(0,1fr)_360px]">
      <div className="min-h-[620px] overflow-hidden rounded-lg border border-borderSubtle bg-bgCard shadow-sm shadow-slate-200/70">
        <div className="flex items-center justify-between border-b border-borderSubtle px-4 py-3">
          <div>
            <h1 className="font-[RobotoMono] text-lg font-bold text-themeBlue">
              Blockly Robot Program
            </h1>
            <p className="text-sm text-themeTextGray">
              Build a robot action chain from the start block.
            </p>
          </div>
          <div className="flex flex-wrap justify-end gap-2 font-[RobotoMono] text-xs">
            <button
              onClick={saveWorkspace}
              className="rounded-lg border border-borderSubtle px-3 py-2 text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            >
              Save
            </button>
            <button
              onClick={loadWorkspace}
              className="rounded-lg border border-borderSubtle px-3 py-2 text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            >
              Load
            </button>
            <button
              onClick={clearWorkspace}
              className="rounded-lg border border-borderSubtle px-3 py-2 text-themeTextGray hover:border-statusRed hover:text-statusRed"
            >
              Reset
            </button>
          </div>
        </div>
        <div
          ref={blocklyDivRef}
          className="h-[calc(100%-73px)] min-h-[545px]"
        />
      </div>

      <aside className="flex min-h-0 flex-col rounded-lg border border-borderSubtle bg-bgCard p-4 shadow-sm shadow-slate-200/70">
        <div className="mb-4">
          <p className="font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
            ROSBridge
          </p>
          <p
            className={`font-[RobotoMono] text-sm font-semibold ${
              rosStatus === "connected" ? "text-statusGreen" : "text-statusRed"
            }`}
          >
            {rosStatus}
          </p>
        </div>

        <div className="mb-4 grid grid-cols-2 gap-2 font-[RobotoMono] text-sm">
          <button
            onClick={runPlan}
            disabled={!canRun}
            className="rounded-lg border border-themeBlue bg-themeBlue px-3 py-2 font-semibold text-white transition-colors hover:bg-themeDarkBlue disabled:cursor-not-allowed disabled:opacity-40"
          >
            {running ? "Running" : "Run"}
          </button>
          <button
            onClick={stopPlan}
            disabled={!running && rosStatus !== "connected"}
            className="rounded-lg border border-statusRed px-3 py-2 font-semibold text-statusRed transition-colors hover:bg-statusRed hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
          >
            Stop
          </button>
        </div>

        <div className="min-h-0 flex-1 overflow-hidden">
          <div className="mb-2 flex items-center justify-between">
            <h2 className="font-[RobotoMono] text-sm font-bold text-textWhiteHover">
              Generated Plan
            </h2>
            <span className="font-[RobotoMono] text-xs text-themeTextGray">
              {plan.length} steps
            </span>
          </div>

          {plan.length === 0 ? (
            <div className="rounded-lg border border-dashed border-borderSubtle bg-bgSurface p-4 text-sm text-themeTextGray">
              Connect robot actions below the start block to create a plan.
            </div>
          ) : (
            <ol className="space-y-2 overflow-auto pr-1">
              {plan.map((action, index) => (
                <li
                  key={`${action.type}-${index}`}
                  className={`rounded-lg border px-3 py-2 text-sm ${
                    activeStep === index
                      ? "border-themeBlue bg-themeBlue/10 text-themeBlue"
                      : "border-borderSubtle bg-bgSurface text-textWhiteHover"
                  }`}
                >
                  <span className="font-[RobotoMono] text-xs text-themeTextGray">
                    {index + 1}.
                  </span>{" "}
                  {actionLabel(action)}
                </li>
              ))}
            </ol>
          )}
        </div>
      </aside>
      <ToastContainer position="bottom-right" theme="light" />
    </section>
  );
};

export default BlocksPage;
