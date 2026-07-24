import React, { useState, useEffect, useRef } from "react";
import * as Three from "three";

import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants/index";
import { MetricCard, StatusBadge } from "../shared/ui/Dashboard";

const quatToYaw = (q) =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

const State = ({ compact = false }) => {
  const ros = useRos();

  const [linear, setLinear] = useState("0.00");
  const [angular, setAngular] = useState("0.00");
  const [xCoord, setXCoord] = useState("—");
  const [yCoord, setYCoord] = useState("—");
  const [orientation, setOrientation] = useState("—");
  const [amclActive, setAmclActive] = useState(false);
  const amclActiveRef = useRef(false);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    // Position from AMCL (accurate, map-corrected)
    const amclTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.AMCL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    amclTopic.subscribe((msg) => {
      const pos = msg?.pose?.pose?.position;
      const ori = msg?.pose?.pose?.orientation;
      if (!pos || !ori) return;
      amclActiveRef.current = true;
      setAmclActive(true);
      setXCoord(pos.x.toFixed(2));
      setYCoord(pos.y.toFixed(2));
      setOrientation((quatToYaw(ori) * (180 / Math.PI)).toFixed(1));
    });

    // Velocity from odom (continuous, real-time)
    const odomTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROBOT_POSE_TOPIC,
      messageType: "nav_msgs/Odometry",
    });

    odomTopic.subscribe((msg) => {
      const vel = msg?.twist?.twist;
      if (!vel) return;
      setLinear(vel.linear.x.toFixed(2));
      setAngular(vel.angular.z.toFixed(2));

      // Fall back to odom position if AMCL is not running
      if (!amclActiveRef.current) {
        const pos = msg?.pose?.pose?.position;
        const ori = msg?.pose?.pose?.orientation;
        if (pos && ori) {
          setXCoord(pos.x.toFixed(2));
          setYCoord(pos.y.toFixed(2));
          const euler = new Three.Euler().setFromQuaternion(
            new Three.Quaternion(ori.x, ori.y, ori.z, ori.w),
          );
          setOrientation((euler._z * (180 / Math.PI)).toFixed(1));
        }
      }
    });

    return () => {
      amclTopic.unsubscribe();
      odomTopic.unsubscribe();
    };
  }, [ros]);

  return (
    <div
      className={`grid w-full min-w-0 grid-cols-1 sm:grid-cols-2 ${compact ? "gap-2" : "gap-3"}`}
    >
      <MetricCard
        compact={compact}
        label="Linear velocity"
        value={linear}
        unit="m/s"
        meta={
          <span className="font-[RobotoMono]">
            Angular{" "}
            <strong className="font-semibold text-textWhiteHover">
              {angular}
            </strong>{" "}
            rad/s
          </span>
        }
      />
      <MetricCard
        compact={compact}
        label="Map position"
        value={`${xCoord}, ${yCoord}`}
        unit="x / y"
        meta={
          <div className="flex flex-wrap items-center justify-between gap-2">
            <span className="font-[RobotoMono]">
              Heading{" "}
              <strong className="font-semibold text-textWhiteHover">
                {orientation}
                {orientation !== "—" ? "°" : ""}
              </strong>
            </span>
            <StatusBadge
              status={amclActive ? "online" : "info"}
              label={amclActive ? "AMCL" : "ODOM"}
            />
          </div>
        }
      />
    </div>
  );
};

export default State;
