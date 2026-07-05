import React, {
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import * as Blockly from "blockly";
import "blockly/blocks";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { RosContext, RosStatusContext } from "../app/App";
import {
  registerOpenAmrBlocks,
  setOpenAmrLocations,
  workspaceToRobotPlan,
  planToWorkspace,
} from "../features/blocks/blockDefinitions";
import {
  executeRobotPlan,
  createRobotActionClient,
} from "../features/blocks/robotActions";
import { generatePlanFromVoice } from "../features/blocks/voicePlan";
import {
  createSpeechRecognizer,
  isSpeechRecognitionSupported,
} from "../features/blocks/voiceCapture";
import {
  hasValidationErrors,
  validateRobotPlan,
} from "../features/blocks/planValidation";
import { openAmrToolbox } from "../features/blocks/toolbox";
import {
  deleteBlockProgram,
  fetchBlockProgram,
  fetchBlockPrograms,
  saveBlockProgram,
} from "../features/blocks/backendPrograms";
import {
  deleteBlockLocation,
  fetchBlockLocations,
  saveBlockLocation,
} from "../features/blocks/backendLocations";
import {
  defaultProgramTemplateId,
  programTemplates,
} from "../features/blocks/programTemplates";
import {
  clearBlockRunHistory,
  fetchBlockRunHistory,
  saveBlockRunHistory,
} from "../features/blocks/backendRunHistory";
import { AppConfig } from "../shared/constants";

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

const initialStepStatuses = (length, status = "queued") =>
  Array.from({ length }, () => status);

const statusClasses = {
  queued: "border-borderSubtle bg-bgSurface text-themeTextGray",
  running: "border-themeBlue bg-themeBlue/10 text-themeBlue",
  done: "border-statusGreen/30 bg-statusGreen/10 text-statusGreen",
  failed: "border-statusRed/30 bg-statusRed/10 text-statusRed",
  stopped: "border-statusRed/30 bg-statusRed/10 text-statusRed",
};

const statusLabel = {
  queued: "Queued",
  running: "Running",
  done: "Done",
  failed: "Failed",
  stopped: "Stopped",
};

const parseInputNumber = (value) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : 0;
};

const formatRunTime = (value) => {
  if (!value) return "Unknown time";
  const date = new Date(value);
  return Number.isNaN(date.getTime()) ? "Unknown time" : date.toLocaleString();
};

const formatDuration = (durationMs) => {
  const seconds = Math.max(0, Math.round(Number(durationMs || 0) / 1000));
  return `${seconds}s`;
};

const historyStatusClasses = {
  success: "border-statusGreen/30 bg-statusGreen/10 text-statusGreen",
  failed: "border-statusRed/30 bg-statusRed/10 text-statusRed",
  stopped: "border-yellow-500/30 bg-yellow-50 text-yellow-700",
};

const riskyActionTypes = new Set([
  "set_speed",
  "drive_for",
  "rotate_for",
  "dock",
  "undock",
  "stop",
]);

const flattenActions = (actions) =>
  actions.flatMap((action) => [
    action,
    ...flattenActions(action.actions || []),
  ]);

const hasRiskyActions = (actions) =>
  flattenActions(actions).some((action) => riskyActionTypes.has(action.type));

const BlocksPage = () => {
  const ros = useContext(RosContext);
  const rosStatus = useContext(RosStatusContext);
  const blocklyDivRef = useRef(null);
  const workspaceRef = useRef(null);
  const importInputRef = useRef(null);
  const stopRequestedRef = useRef(false);
  const activeRecognizerRef = useRef(null);

  const [plan, setPlan] = useState([]);
  const [voiceListening, setVoiceListening] = useState(false);
  const [voiceTranscript, setVoiceTranscript] = useState("");
  const [voiceBusy, setVoiceBusy] = useState(false);
  const [validationMessages, setValidationMessages] = useState([]);
  const [running, setRunning] = useState(false);
  const [activeStep, setActiveStep] = useState(null);
  const [stepStatuses, setStepStatuses] = useState([]);
  const [programName, setProgramName] = useState("");
  const [selectedProgram, setSelectedProgram] = useState("");
  const [savedPrograms, setSavedPrograms] = useState([]);
  const [programsLoading, setProgramsLoading] = useState(false);
  const [locations, setLocations] = useState({});
  const [locationName, setLocationName] = useState("");
  const [selectedLocation, setSelectedLocation] = useState("");
  const [locationPose, setLocationPose] = useState({ x: 0, y: 0, yaw: 0 });
  const [locationsLoading, setLocationsLoading] = useState(false);
  const [selectedTemplate, setSelectedTemplate] = useState(
    defaultProgramTemplateId,
  );
  const [runHistory, setRunHistory] = useState([]);
  const [historyLoading, setHistoryLoading] = useState(false);

  const validationHasErrors = useMemo(
    () => hasValidationErrors(validationMessages),
    [validationMessages],
  );

  const canRun = useMemo(
    () =>
      rosStatus === "connected" &&
      plan.length > 0 &&
      !validationHasErrors &&
      !running,
    [plan.length, rosStatus, running, validationHasErrors],
  );

  const speechSupported = useMemo(() => isSpeechRecognitionSupported(), []);

  const selectedTemplateInfo = useMemo(
    () => programTemplates.find((item) => item.id === selectedTemplate),
    [selectedTemplate],
  );

  const refreshSavedPrograms = useCallback(async () => {
    setProgramsLoading(true);
    try {
      const programs = await fetchBlockPrograms();
      setSavedPrograms(programs);
      setSelectedProgram((current) => current || programs[0]?.name || "");
    } catch (err) {
      toast.error(err.message || "Could not load backend programs");
    } finally {
      setProgramsLoading(false);
    }
  }, []);

  const refreshRunHistory = useCallback(async () => {
    setHistoryLoading(true);
    try {
      setRunHistory(await fetchBlockRunHistory());
    } catch (err) {
      toast.error(err.message || "Could not load run history");
    } finally {
      setHistoryLoading(false);
    }
  }, []);

  const updatePlanFromWorkspace = useCallback((workspace) => {
    const nextPlan = workspaceToRobotPlan(workspace);
    setPlan(nextPlan);
    setValidationMessages(validateRobotPlan(workspace, nextPlan));
    setStepStatuses(initialStepStatuses(nextPlan.length));
    return nextPlan;
  }, []);

  const refreshLocations = useCallback(async () => {
    setLocationsLoading(true);
    try {
      const nextLocations = await fetchBlockLocations();
      setLocations(nextLocations);
      setOpenAmrLocations(nextLocations);
      const firstLocation = Object.keys(nextLocations)[0] || "";
      setSelectedLocation((current) => {
        return current && nextLocations[current] ? current : firstLocation;
      });
      if (workspaceRef.current) {
        updatePlanFromWorkspace(workspaceRef.current);
      }
    } catch (err) {
      toast.error(err.message || "Could not load backend locations");
    } finally {
      setLocationsLoading(false);
    }
  }, [updatePlanFromWorkspace]);

  useEffect(() => {
    if (!selectedLocation || !locations[selectedLocation]) return;
    setLocationName(selectedLocation);
    setLocationPose(locations[selectedLocation]);
  }, [locations, selectedLocation]);

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

    const updatePlan = () => updatePlanFromWorkspace(workspace);

    updatePlan();
    workspace.addChangeListener(updatePlan);
    refreshSavedPrograms();
    refreshLocations();
    refreshRunHistory();

    const resize = () => Blockly.svgResize(workspace);
    window.addEventListener("resize", resize);
    setTimeout(resize, 0);

    return () => {
      window.removeEventListener("resize", resize);
      workspace.dispose();
      workspaceRef.current = null;
    };
  }, [
    refreshLocations,
    refreshRunHistory,
    refreshSavedPrograms,
    updatePlanFromWorkspace,
  ]);

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
    updatePlanFromWorkspace(workspaceRef.current);
    toast.success("Blocks loaded");
  };

  const exportWorkspace = () => {
    if (!workspaceRef.current) return;
    const state = Blockly.serialization.workspaces.save(workspaceRef.current);
    const blob = new Blob([JSON.stringify(state, null, 2)], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    const safeName = (programName.trim() || "openamr-block-program")
      .toLowerCase()
      .replace(/[^a-z0-9._-]+/g, "-");

    link.href = url;
    link.download = `${safeName}.json`;
    link.click();
    URL.revokeObjectURL(url);
    toast.success("Program JSON exported");
  };

  const importWorkspace = (event) => {
    const file = event.target.files?.[0];
    if (!file || !workspaceRef.current) return;

    const reader = new FileReader();
    reader.onload = () => {
      try {
        const importedState = JSON.parse(reader.result);
        workspaceRef.current.clear();
        Blockly.serialization.workspaces.load(
          importedState,
          workspaceRef.current,
        );
        updatePlanFromWorkspace(workspaceRef.current);
        setProgramName(file.name.replace(/\.json$/i, ""));
        toast.success("Program JSON imported");
      } catch {
        toast.error("Could not import Blockly JSON");
      } finally {
        event.target.value = "";
      }
    };
    reader.readAsText(file);
  };

  const clearWorkspace = () => {
    if (!workspaceRef.current) return;
    workspaceRef.current.clear();
    Blockly.serialization.workspaces.load(
      defaultWorkspace,
      workspaceRef.current,
    );
    updatePlanFromWorkspace(workspaceRef.current);
  };

  const loadProgramTemplate = () => {
    if (!workspaceRef.current) return;
    const template = programTemplates.find(
      (item) => item.id === selectedTemplate,
    );
    if (!template) return;

    workspaceRef.current.clear();
    Blockly.serialization.workspaces.load(
      template.createWorkspace(),
      workspaceRef.current,
    );
    updatePlanFromWorkspace(workspaceRef.current);
    setProgramName(template.name);
    toast.success(`Loaded template "${template.name}"`);
  };

  const saveBackendWorkspace = async () => {
    if (!workspaceRef.current) return;
    const name = programName.trim();
    if (!name) {
      toast.info("Enter a program name before saving");
      return;
    }

    try {
      const workspace = Blockly.serialization.workspaces.save(
        workspaceRef.current,
      );
      await saveBlockProgram({ name, workspace, plan });
      setSelectedProgram(name);
      await refreshSavedPrograms();
      toast.success(`Saved "${name}" to backend`);
    } catch (err) {
      toast.error(err.message || "Could not save backend program");
    }
  };

  const loadBackendWorkspace = async () => {
    if (!workspaceRef.current || !selectedProgram) return;

    try {
      const savedProgram = await fetchBlockProgram(selectedProgram);
      Blockly.serialization.workspaces.load(
        savedProgram.workspace,
        workspaceRef.current,
      );
      updatePlanFromWorkspace(workspaceRef.current);
      setProgramName(savedProgram.name || selectedProgram);
      toast.success(`Loaded "${savedProgram.name || selectedProgram}"`);
    } catch (err) {
      toast.error(err.message || "Could not load backend program");
    }
  };

  const deleteBackendWorkspace = async () => {
    if (!selectedProgram) return;

    try {
      await deleteBlockProgram(selectedProgram);
      setProgramName("");
      setSelectedProgram("");
      await refreshSavedPrograms();
      toast.success(`Deleted "${selectedProgram}"`);
    } catch (err) {
      toast.error(err.message || "Could not delete backend program");
    }
  };

  const saveBackendLocation = async () => {
    const name = locationName.trim();
    if (!name) {
      toast.info("Enter a location name before saving");
      return;
    }

    try {
      await saveBlockLocation({
        name,
        x: parseInputNumber(locationPose.x),
        y: parseInputNumber(locationPose.y),
        yaw: parseInputNumber(locationPose.yaw),
      });
      setSelectedLocation(name);
      await refreshLocations();
      toast.success(`Saved location "${name}"`);
    } catch (err) {
      toast.error(err.message || "Could not save backend location");
    }
  };

  const deleteBackendLocation = async () => {
    if (!selectedLocation) return;

    try {
      await deleteBlockLocation(selectedLocation);
      setLocationName("");
      setSelectedLocation("");
      setLocationPose({ x: 0, y: 0, yaw: 0 });
      await refreshLocations();
      toast.success(`Deleted location "${selectedLocation}"`);
    } catch (err) {
      toast.error(err.message || "Could not delete backend location");
    }
  };

  const clearRunHistory = async () => {
    try {
      await clearBlockRunHistory();
      setRunHistory([]);
      toast.success("Run history cleared");
    } catch (err) {
      toast.error(err.message || "Could not clear run history");
    }
  };

  const recordRunHistory = async (entry) => {
    try {
      const data = await saveBlockRunHistory(entry);
      setRunHistory(data.history || []);
    } catch (err) {
      toast.error(err.message || "Could not save run history");
    }
  };

  const runPlan = async () => {
    if (!canRun) return;
    const warningCount = validationMessages.filter(
      (message) => message.severity === "warning",
    ).length;
    if (hasRiskyActions(plan) || warningCount > 0) {
      const confirmed = window.confirm(
        `This program contains ${
          hasRiskyActions(plan) ? "direct robot actions" : "validation warnings"
        }${
          warningCount > 0 ? ` and ${warningCount} warning(s)` : ""
        }.\n\nConfirm the robot area is clear before running.`,
      );
      if (!confirmed) return;
    }

    const startedAt = new Date();
    let completedSteps = 0;
    stopRequestedRef.current = false;
    setRunning(true);
    setActiveStep(0);
    setStepStatuses(initialStepStatuses(plan.length));

    try {
      await executeRobotPlan(ros, plan, {
        shouldStop: () => stopRequestedRef.current,
        onStepStart: (index) => {
          setActiveStep(index);
          setStepStatuses((current) =>
            current.map((status, statusIndex) =>
              statusIndex === index ? "running" : status,
            ),
          );
        },
        onStepSuccess: (index) => {
          completedSteps = Math.max(completedSteps, index + 1);
          setStepStatuses((current) =>
            current.map((status, statusIndex) =>
              statusIndex === index ? "done" : status,
            ),
          );
        },
        onStepError: (index) => {
          setStepStatuses((current) =>
            current.map((status, statusIndex) =>
              statusIndex === index ? "failed" : status,
            ),
          );
        },
      });
      if (stopRequestedRef.current) {
        setStepStatuses((current) =>
          current.map((status) => (status === "queued" ? "stopped" : status)),
        );
        await recordRunHistory({
          program_name: programName.trim() || selectedTemplateInfo?.name,
          status: "stopped",
          started_at: startedAt.toISOString(),
          duration_ms: Date.now() - startedAt.getTime(),
          steps_total: plan.length,
          steps_completed: completedSteps,
        });
        toast.info("Block program stopped");
      } else {
        await recordRunHistory({
          program_name: programName.trim() || selectedTemplateInfo?.name,
          status: "success",
          started_at: startedAt.toISOString(),
          duration_ms: Date.now() - startedAt.getTime(),
          steps_total: plan.length,
          steps_completed: completedSteps,
        });
        toast.success("Block program sent");
      }
    } catch (err) {
      await recordRunHistory({
        program_name: programName.trim() || selectedTemplateInfo?.name,
        status: stopRequestedRef.current ? "stopped" : "failed",
        started_at: startedAt.toISOString(),
        duration_ms: Date.now() - startedAt.getTime(),
        steps_total: plan.length,
        steps_completed: completedSteps,
        error_message: err.message || "Block program failed",
      });
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
    setStepStatuses((current) =>
      current.map((status) => (status === "running" ? "stopped" : status)),
    );
    toast.warn("Stop command sent");
  };

  const applyVoicePlan = async (transcript) => {
    if (!workspaceRef.current) return;

    setVoiceBusy(true);
    try {
      const { plan: generatedPlan } = await generatePlanFromVoice({
        transcript,
        locations,
      });
      workspaceRef.current.clear();
      Blockly.serialization.workspaces.load(
        planToWorkspace(generatedPlan),
        workspaceRef.current,
      );
      updatePlanFromWorkspace(workspaceRef.current);
      toast.success(
        `Generated a ${generatedPlan.length}-step plan from voice. Review the blocks, then press Run.`,
      );
    } catch (err) {
      toast.error(err.message || "Could not generate a plan from voice");
    } finally {
      setVoiceBusy(false);
    }
  };

  const startVoiceListening = () => {
    if (!isSpeechRecognitionSupported()) {
      toast.error("Voice input isn't supported in this browser");
      return;
    }

    const recognizer = createSpeechRecognizer({
      onInterimResult: setVoiceTranscript,
      onFinalResult: (text) => {
        setVoiceTranscript(text);
        applyVoicePlan(text);
      },
      onError: (error) => toast.error(`Voice input error: ${error}`),
      onEnd: () => setVoiceListening(false),
    });

    activeRecognizerRef.current = recognizer;
    setVoiceTranscript("");
    setVoiceListening(true);
    recognizer.start();
  };

  const stopVoiceListening = () => {
    activeRecognizerRef.current?.stop();
  };

  return (
    <section className="grid min-h-[520px] grid-cols-1 gap-3 py-3 lg:h-[calc(100vh-96px)] lg:min-h-0 lg:grid-cols-[minmax(0,1fr)_340px] lg:overflow-hidden">
      <div className="flex min-h-[480px] flex-col overflow-hidden rounded-lg border border-borderSubtle bg-bgCard shadow-sm shadow-slate-200/70 dark:shadow-slate-950/40 lg:min-h-0">
        <div className="flex shrink-0 items-center justify-between border-b border-borderSubtle px-3 py-2">
          <div>
            <h1 className="font-[RobotoMono] text-lg font-bold text-themeBlue">
              Blockly Robot Program
            </h1>
            <p className="text-sm text-themeTextGray">
              Build a robot action chain from the start block.
            </p>
          </div>
          <div className="flex flex-wrap justify-end gap-2 font-[RobotoMono] text-xs">
            <input
              ref={importInputRef}
              type="file"
              accept="application/json,.json"
              onChange={importWorkspace}
              className="hidden"
            />
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
              onClick={() => importInputRef.current?.click()}
              className="rounded-lg border border-borderSubtle px-3 py-2 text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            >
              Import
            </button>
            <button
              onClick={exportWorkspace}
              className="rounded-lg border border-borderSubtle px-3 py-2 text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
            >
              Export
            </button>
            <button
              onClick={clearWorkspace}
              className="rounded-lg border border-borderSubtle px-3 py-2 text-themeTextGray hover:border-statusRed hover:text-statusRed"
            >
              Reset
            </button>
          </div>
        </div>
        <div ref={blocklyDivRef} className="min-h-[420px] flex-1 lg:min-h-0" />
      </div>

      <aside className="flex min-h-0 flex-col overflow-y-auto rounded-lg border border-borderSubtle bg-bgCard p-3 shadow-sm shadow-slate-200/70 dark:shadow-slate-950/40 lg:h-full">
        <div className="mb-3">
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

        <div className="mb-3 grid grid-cols-2 gap-2 font-[RobotoMono] text-sm">
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

        <details
          open
          className="mb-3 rounded-lg border border-borderSubtle bg-bgSurface p-2.5"
        >
          <summary className="cursor-pointer font-[RobotoMono] text-sm font-bold text-textWhiteHover">
            Voice Command
          </summary>

          {!speechSupported ? (
            <div className="mt-2 rounded-lg border border-dashed border-borderSubtle bg-bgCard p-3 text-xs text-themeTextGray">
              Voice input isn't supported in this browser. Try Chrome on
              http://localhost.
            </div>
          ) : (
            <>
              <p className="mb-2 mt-2 text-xs text-themeTextGray">
                Speak a command to generate blocks below. Review them, then
                press Run.
              </p>
              <button
                onClick={
                  voiceListening ? stopVoiceListening : startVoiceListening
                }
                disabled={voiceBusy}
                className={`mb-2 w-full rounded-lg border py-2 font-[RobotoMono] text-xs font-semibold disabled:cursor-not-allowed disabled:opacity-40 ${
                  voiceListening
                    ? "border-statusRed bg-statusRed/10 text-statusRed"
                    : "border-themeBlue bg-themeBlue/10 text-themeBlue hover:bg-themeBlue hover:text-white"
                }`}
              >
                {voiceBusy
                  ? "Generating plan..."
                  : voiceListening
                    ? "Listening... (tap to stop)"
                    : "Tap to speak a command"}
              </button>
              <div className="min-h-[40px] rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-xs text-textWhiteHover">
                {voiceTranscript || "Transcript will appear here"}
              </div>
            </>
          )}
        </details>

        <details
          open
          className="mb-3 rounded-lg border border-borderSubtle bg-bgSurface p-2.5"
        >
          <summary className="cursor-pointer font-[RobotoMono] text-sm font-bold text-textWhiteHover">
            Program Templates
          </summary>
          <p className="mb-2 mt-2 text-xs text-themeTextGray">
            Load a ready-made starter program into the workspace.
          </p>

          <select
            value={selectedTemplate}
            onChange={(event) => setSelectedTemplate(event.target.value)}
            className="mb-2 w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue"
          >
            {programTemplates.map((template) => (
              <option key={template.id} value={template.id}>
                {template.name}
              </option>
            ))}
          </select>

          <p className="mb-2 min-h-[32px] text-xs text-themeTextGray">
            {selectedTemplateInfo?.description}
          </p>

          <button
            onClick={loadProgramTemplate}
            className="w-full rounded-lg border border-themeBlue bg-themeBlue/10 py-2 font-[RobotoMono] text-xs font-semibold text-themeBlue hover:bg-themeBlue hover:text-white"
          >
            Load Template
          </button>
        </details>

        <details
          open
          className="mb-3 rounded-lg border border-borderSubtle bg-bgSurface p-2.5"
        >
          <summary className="cursor-pointer font-[RobotoMono] text-sm font-bold text-textWhiteHover">
            Run History
          </summary>
          <div className="mb-2 mt-2 flex justify-end">
            <div className="flex gap-1">
              <button
                onClick={refreshRunHistory}
                disabled={historyLoading}
                className="rounded border border-borderSubtle px-2 py-1 font-[RobotoMono] text-[10px] text-themeTextGray hover:border-themeBlue hover:text-themeBlue disabled:cursor-not-allowed disabled:opacity-40"
              >
                Refresh
              </button>
              <button
                onClick={clearRunHistory}
                disabled={historyLoading || runHistory.length === 0}
                className="rounded border border-statusRed px-2 py-1 font-[RobotoMono] text-[10px] text-statusRed hover:bg-statusRed hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
              >
                Clear
              </button>
            </div>
          </div>

          {runHistory.length === 0 ? (
            <div className="rounded-lg border border-dashed border-borderSubtle bg-bgCard p-3 text-xs text-themeTextGray">
              Run a program to create the first history entry.
            </div>
          ) : (
            <ol className="max-h-36 space-y-2 overflow-auto pr-1">
              {runHistory.slice(0, 5).map((entry, index) => (
                <li
                  key={`${entry.started_at}-${index}`}
                  className="rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-xs text-textWhiteHover"
                >
                  <div className="mb-1 flex items-center justify-between gap-2">
                    <span className="truncate font-[RobotoMono] font-semibold">
                      {entry.program_name}
                    </span>
                    <span
                      className={`shrink-0 rounded border px-2 py-0.5 font-[RobotoMono] text-[10px] uppercase ${
                        historyStatusClasses[entry.status] ||
                        "border-borderSubtle bg-bgSurface text-themeTextGray"
                      }`}
                    >
                      {entry.status}
                    </span>
                  </div>
                  <p className="text-themeTextGray">
                    {formatRunTime(entry.finished_at)} · {entry.steps_completed}
                    /{entry.steps_total} steps ·{" "}
                    {formatDuration(entry.duration_ms)}
                  </p>
                  {entry.error_message ? (
                    <p className="mt-1 text-statusRed">{entry.error_message}</p>
                  ) : null}
                </li>
              ))}
            </ol>
          )}
        </details>

        <details
          open
          className="mb-3 rounded-lg border border-borderSubtle bg-bgSurface p-2.5"
        >
          <summary className="cursor-pointer font-[RobotoMono] text-sm font-bold text-textWhiteHover">
            Backend Programs
          </summary>
          <div className="mb-2 mt-2 flex justify-end">
            <button
              onClick={refreshSavedPrograms}
              disabled={programsLoading}
              className="rounded border border-borderSubtle px-2 py-1 font-[RobotoMono] text-[10px] text-themeTextGray hover:border-themeBlue hover:text-themeBlue disabled:cursor-not-allowed disabled:opacity-40"
            >
              Refresh
            </button>
          </div>

          <input
            value={programName}
            onChange={(event) => setProgramName(event.target.value)}
            placeholder="Program name"
            className="mb-2 w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue"
          />

          <select
            value={selectedProgram}
            onChange={(event) => {
              setSelectedProgram(event.target.value);
              setProgramName(event.target.value);
            }}
            className="mb-2 w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue"
          >
            <option value="">
              {savedPrograms.length === 0
                ? "No backend programs"
                : "Select saved program"}
            </option>
            {savedPrograms.map((program) => (
              <option key={program.name} value={program.name}>
                {program.name}
              </option>
            ))}
          </select>

          <div className="grid grid-cols-3 gap-2 font-[RobotoMono] text-xs">
            <button
              onClick={saveBackendWorkspace}
              disabled={programsLoading}
              className="rounded-lg border border-themeBlue bg-themeBlue/10 py-2 font-semibold text-themeBlue hover:bg-themeBlue hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
            >
              Save
            </button>
            <button
              onClick={loadBackendWorkspace}
              disabled={!selectedProgram || programsLoading}
              className="rounded-lg border border-borderSubtle bg-bgCard py-2 font-semibold text-textWhiteHover hover:border-themeBlue hover:text-themeBlue disabled:cursor-not-allowed disabled:opacity-40"
            >
              Load
            </button>
            <button
              onClick={deleteBackendWorkspace}
              disabled={!selectedProgram || programsLoading}
              className="rounded-lg border border-statusRed bg-bgCard py-2 font-semibold text-statusRed hover:bg-statusRed hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
            >
              Delete
            </button>
          </div>
        </details>

        <details className="mb-3 rounded-lg border border-borderSubtle bg-bgSurface p-2.5">
          <summary className="cursor-pointer font-[RobotoMono] text-sm font-bold text-textWhiteHover">
            Named Locations
          </summary>
          <div className="mb-2 mt-2 flex justify-end">
            <button
              onClick={refreshLocations}
              disabled={locationsLoading}
              className="rounded border border-borderSubtle px-2 py-1 font-[RobotoMono] text-[10px] text-themeTextGray hover:border-themeBlue hover:text-themeBlue disabled:cursor-not-allowed disabled:opacity-40"
            >
              Refresh
            </button>
          </div>

          <input
            value={locationName}
            onChange={(event) => setLocationName(event.target.value)}
            placeholder="Location name"
            className="mb-2 w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue"
          />

          <select
            value={selectedLocation}
            onChange={(event) => {
              const name = event.target.value;
              setSelectedLocation(name);
              setLocationName(name);
              if (locations[name]) {
                setLocationPose(locations[name]);
              }
            }}
            className="mb-2 w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue"
          >
            <option value="">
              {Object.keys(locations).length === 0
                ? "No backend locations"
                : "Select location"}
            </option>
            {Object.keys(locations).map((name) => (
              <option key={name} value={name}>
                {name}
              </option>
            ))}
          </select>

          <div className="mb-2 grid grid-cols-3 gap-2">
            {["x", "y", "yaw"].map((field) => (
              <label
                key={field}
                className="font-[RobotoMono] text-[10px] uppercase text-themeTextGray"
              >
                {field}
                <input
                  type="number"
                  step="0.01"
                  value={locationPose[field]}
                  onChange={(event) =>
                    setLocationPose((current) => ({
                      ...current,
                      [field]: event.target.value,
                    }))
                  }
                  className="mt-1 w-full rounded-lg border border-borderSubtle bg-bgCard px-2 py-2 text-sm normal-case text-textWhiteHover outline-none focus:border-themeBlue"
                />
              </label>
            ))}
          </div>

          <div className="grid grid-cols-2 gap-2 font-[RobotoMono] text-xs">
            <button
              onClick={saveBackendLocation}
              disabled={locationsLoading}
              className="rounded-lg border border-themeBlue bg-themeBlue/10 py-2 font-semibold text-themeBlue hover:bg-themeBlue hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
            >
              Save Location
            </button>
            <button
              onClick={deleteBackendLocation}
              disabled={!selectedLocation || locationsLoading}
              className="rounded-lg border border-statusRed bg-bgCard py-2 font-semibold text-statusRed hover:bg-statusRed hover:text-white disabled:cursor-not-allowed disabled:opacity-40"
            >
              Delete
            </button>
          </div>
        </details>

        <div className="min-h-0">
          <details
            open
            className="mb-3 rounded-lg border border-borderSubtle bg-bgSurface p-2.5"
          >
            <summary className="cursor-pointer font-[RobotoMono] text-sm font-bold text-textWhiteHover">
              Plan Checks
            </summary>
            <div className="mb-2 mt-2 flex justify-end">
              <span className="font-[RobotoMono] text-xs text-themeTextGray">
                max {AppConfig.MAX_LINEAR_SPEED} m/s,{" "}
                {AppConfig.MAX_ANGULAR_SPEED} rad/s
              </span>
            </div>

            {validationMessages.length === 0 ? (
              <div className="rounded-lg border border-statusGreen/30 bg-statusGreen/10 px-3 py-2 text-sm text-statusGreen">
                Ready. No validation warnings.
              </div>
            ) : (
              <ul className="max-h-28 space-y-2 overflow-auto pr-1">
                {validationMessages.map((message, index) => (
                  <li
                    key={`${message.severity}-${index}`}
                    className={`rounded-lg border px-3 py-2 text-xs ${
                      message.severity === "error"
                        ? "border-statusRed/30 bg-statusRed/10 text-statusRed"
                        : "border-yellow-500/30 bg-yellow-50 text-yellow-700"
                    }`}
                  >
                    <span className="font-[RobotoMono] font-bold uppercase">
                      {message.severity}
                    </span>
                    : {message.text}
                  </li>
                ))}
              </ul>
            )}
          </details>

          <details
            open
            className="rounded-lg border border-borderSubtle bg-bgSurface p-2.5"
          >
            <summary className="cursor-pointer font-[RobotoMono] text-sm font-bold text-textWhiteHover">
              Generated Plan
            </summary>
            <div className="mb-2 mt-2 flex justify-end">
              <span className="font-[RobotoMono] text-xs text-themeTextGray">
                {plan.length} steps
              </span>
            </div>

            {plan.length === 0 ? (
              <div className="rounded-lg border border-dashed border-borderSubtle bg-bgCard p-4 text-sm text-themeTextGray">
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
                        : "border-borderSubtle bg-bgCard text-textWhiteHover"
                    }`}
                  >
                    <span className="font-[RobotoMono] text-xs text-themeTextGray">
                      {index + 1}.
                    </span>{" "}
                    {actionLabel(action)}
                    <span
                      className={`mt-2 block w-fit rounded border px-2 py-0.5 font-[RobotoMono] text-[10px] uppercase ${
                        statusClasses[stepStatuses[index] || "queued"]
                      }`}
                    >
                      {statusLabel[stepStatuses[index] || "queued"]}
                    </span>
                  </li>
                ))}
              </ol>
            )}
          </details>
        </div>
      </aside>
      <ToastContainer position="bottom-right" theme="light" />
    </section>
  );
};

export default BlocksPage;
