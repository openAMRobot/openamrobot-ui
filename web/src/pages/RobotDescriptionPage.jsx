import React, { useEffect, useMemo, useState } from "react";

import { useRos, useRosStatus } from "../app/App";
import { SectionHeader } from "../shared/ui/Dashboard";
import { fetchRobotDescriptionManifest } from "../features/robotDescription/api/robotDescriptionApi";
import useRobotScene from "../features/robotDescription/three/useRobotScene";
import useJointStates from "../features/robotDescription/hooks/useJointStates";
import useRobotPose from "../features/robotDescription/hooks/useRobotPose";
import usePlanPath from "../features/robotDescription/hooks/usePlanPath";
import RobotModelViewer from "../features/robotDescription/components/RobotModelViewer";
import KinematicTree from "../features/robotDescription/components/KinematicTree";
import ComponentInspector from "../features/robotDescription/components/ComponentInspector";
import JointSliderPanel from "../features/robotDescription/components/JointSliderPanel";
import DisplayLayerControls from "../features/robotDescription/components/DisplayLayerControls";
import ModelLoadStatus from "../features/robotDescription/components/ModelLoadStatus";
import DescriptionModeBanner from "../features/robotDescription/components/DescriptionModeBanner";
import useRobotDescriptionTopic from "../features/upperBody/hooks/useRobotDescriptionTopic";
import useUpperBodyJointStates from "../features/upperBody/hooks/useUpperBodyJointStates";
import CandidateModelBanner from "../features/upperBody/components/CandidateModelBanner";
import UpperBodyPanel from "../features/upperBody/components/UpperBodyPanel";
import LiftPanel from "../features/upperBody/components/LiftPanel";
import {
  FLOOR_FRAME,
  LIFT_JOINT,
  SHOULDER_LINKS,
  findPlaceholderLinks,
  isCombinedDescription,
  linkHeightAbove,
} from "../features/upperBody/upperBodyModel";

const DEFAULT_LAYERS = {
  visual: true,
  collision: false,
  tfAxes: false,
  jointAxes: false,
  linkNames: false,
  jointNames: false,
  centerOfMass: false,
  footprint: false,
};

const RobotDescriptionPage = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();

  const [manifest, setManifest] = useState(null);
  const [manifestError, setManifestError] = useState(null);
  const [mode, setMode] = useState(() => localStorage.getItem("robotDescriptionMode") || "description");
  const [selected, setSelected] = useState(null);
  const [layers, setLayers] = useState(DEFAULT_LAYERS);

  useEffect(() => {
    localStorage.setItem("robotDescriptionMode", mode);
  }, [mode]);

  useEffect(() => {
    let cancelled = false;
    fetchRobotDescriptionManifest()
      .then((data) => {
        if (!cancelled) setManifest(data);
      })
      .catch((error) => {
        if (!cancelled) setManifestError(error.message);
      });
    return () => {
      cancelled = true;
    };
  }, []);

  // The combined OpenAMRobot 2.0 model, when the robot publishes it. It
  // replaces the backend's platform-only model; meshes still resolve through
  // the backend's package map.
  const publishedUrdf = useRobotDescriptionTopic({ ros });
  const combinedUrdf = useMemo(
    () => (isCombinedDescription(publishedUrdf) ? publishedUrdf : null),
    [publishedUrdf],
  );
  const placeholderLinks = useMemo(() => findPlaceholderLinks(combinedUrdf), [combinedUrdf]);
  const sceneManifest = useMemo(() => {
    if (!combinedUrdf) return manifest;
    return {
      ...(manifest || {}),
      displayName: "OpenAMRobot 2.0 (candidate model)",
      available: true,
      urdfText: combinedUrdf,
      packages: manifest?.packages || {},
    };
  }, [manifest, combinedUrdf]);

  const { positions: upperBodyPositions, freshness: upperBodyFreshness } = useUpperBodyJointStates({ ros });
  const shoulderM = useMemo(
    () => (combinedUrdf ? linkHeightAbove(combinedUrdf, FLOOR_FRAME, SHOULDER_LINKS.left, upperBodyPositions) : null),
    [combinedUrdf, upperBodyPositions],
  );

  const handleSelect = (name, kind) => setSelected({ name, kind });

  const { pose, source: poseSource } = useRobotPose({ mode, ros });
  const path = usePlanPath({ mode, ros });

  const {
    containerRef,
    status,
    robot,
    resetView,
    fitToScreen,
    toggleFullscreen,
    isFullscreen,
    setJointValues: applyJointValuesToScene,
  } = useRobotScene({ manifest: sceneManifest, layers, selected, onSelect: handleSelect, pose, path });

  const movableJointNames = useMemo(
    () =>
      robot
        ? Object.values(robot.joints)
            .filter((j) => j.jointType !== "fixed")
            .map((j) => j.urdfName || j.name)
        : [],
    [robot],
  );

  const {
    values: jointValues,
    setJointValue,
    resetJoints,
    liveConnected,
  } = useJointStates({ mode, ros, jointNames: movableJointNames });

  useEffect(() => {
    applyJointValuesToScene(jointValues);
  }, [jointValues, applyJointValuesToScene]);

  const handleLayerChange = (key, value) => setLayers((prev) => ({ ...prev, [key]: value }));

  return (
    <div className="sectionHeight flex flex-col gap-5 py-4 sm:gap-6 sm:py-6 xl:h-[calc(100vh-145px)] xl:min-h-0">
      <SectionHeader
        eyebrow={sceneManifest ? sceneManifest.displayName : "Digital twin"}
        title="Robot description"
        description="Explore a 3D model of the robot's parts and joints, built from its design files — no robot connection needed. (Technically: its URDF/Xacro description.)"
      />

      {manifestError ? (
        <p className="text-sm text-statusRed">Could not load the robot description manifest: {manifestError}</p>
      ) : null}

      {combinedUrdf ? <CandidateModelBanner placeholderLinks={placeholderLinks} /> : null}

      <DescriptionModeBanner
        mode={mode}
        onModeChange={setMode}
        rosStatus={rosStatus}
        liveConnected={liveConnected}
        poseSource={poseSource}
        pathPointCount={path?.length || 0}
      />

      <div className="grid min-w-0 flex-1 gap-4 xl:min-h-0 xl:grid-cols-[minmax(0,1.45fr)_minmax(360px,0.85fr)] xl:gap-5">
        <section className="flex min-w-0 flex-col gap-4 xl:min-h-0">
          <div className="h-[420px] min-w-0 sm:h-[520px] xl:h-auto xl:flex-1 xl:min-h-0">
            <RobotModelViewer
              containerRef={containerRef}
              status={status}
              resetView={resetView}
              fitToScreen={fitToScreen}
              toggleFullscreen={toggleFullscreen}
              isFullscreen={isFullscreen}
            />
          </div>
          <DisplayLayerControls robot={robot} layers={layers} onChange={handleLayerChange} />
          <ModelLoadStatus manifest={sceneManifest} status={status} />
        </section>

        <section className="flex min-w-0 flex-col gap-4 xl:min-h-0 xl:overflow-y-auto">
          <div className="h-[280px] min-w-0 xl:h-auto xl:flex-1 xl:min-h-0">
            <KinematicTree robot={robot} selected={selected} onSelect={handleSelect} />
          </div>
          <UpperBodyPanel positions={upperBodyPositions} freshness={upperBodyFreshness} />
          <LiftPanel
            liftPositionM={upperBodyPositions[LIFT_JOINT]}
            shoulderM={shoulderM}
            freshness={upperBodyFreshness.lift}
          />
          <ComponentInspector robot={robot} selected={selected} meshesReady={status.stage === "ready"} />
          <div className="h-[360px] min-w-0 xl:h-auto xl:flex-1 xl:min-h-0">
            <JointSliderPanel
              robot={robot}
              jointValues={jointValues}
              onSetJointValue={setJointValue}
              onReset={resetJoints}
              mode={mode}
              selected={selected}
              onSelect={handleSelect}
            />
          </div>
        </section>
      </div>
    </div>
  );
};

export default RobotDescriptionPage;
