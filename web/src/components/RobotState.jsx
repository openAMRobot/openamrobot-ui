import React, { useState, useEffect, useContext, useRef } from "react";
import * as Three from "three";

import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants/index";

const StatCard = ({ title, badge, children }) => (
  <div className="flex-1 rounded-xl border border-borderSubtle bg-bgCard p-3">
    <div className="mb-2 flex items-center gap-2">
      <p className="text-xs font-semibold uppercase tracking-wider text-themeBlue">
        {title}
      </p>
      {badge && (
        <span className="bg-themeBlue/15 rounded px-1.5 py-0.5 text-[9px] font-bold uppercase tracking-wider text-themeBlue">
          {badge}
        </span>
      )}
    </div>
    <div className="space-y-1 font-[RobotoMono] text-sm text-textWhiteHover">
      {children}
    </div>
  </div>
);

const quatToYaw = (q) =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

const State = () => {
  const ros = useContext(RosContext);

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
  }, [ros, amclActive]);

  return (
    <div className="flex w-full gap-3">
      <StatCard title="Velocity">
        <p>
          Linear: <span className="text-themeBlue">{linear}</span> m/s
        </p>
        <p>
          Angular: <span className="text-themeBlue">{angular}</span> rad/s
        </p>
      </StatCard>
      <StatCard title="Position" badge={amclActive ? "AMCL" : "ODOM"}>
        <p>
          X: <span className="text-themeBlue">{xCoord}</span> Y:{" "}
          <span className="text-themeBlue">{yCoord}</span>
        </p>
        <p>
          Heading: <span className="text-themeBlue">{orientation}</span>
          {orientation !== "—" ? "°" : ""}
        </p>
      </StatCard>
    </div>
  );
};

export default State;
