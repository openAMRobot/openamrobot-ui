import { AppConfig, LIFECYCLE_NODES } from "../constants";

const TICK_MS = 300;
const LOOP_RADIUS = 1.5;
const LOOP_ANGULAR_SPEED = 0.15; // rad/s
const MAP_RESOLUTION = 0.05;
const MAP_CELLS = 100; // 5m x 5m room

const quatFromYaw = (yaw) => ({
  x: 0,
  y: 0,
  z: Math.sin(yaw / 2),
  w: Math.cos(yaw / 2),
});

const IDENTITY_QUAT = { x: 0, y: 0, z: 0, w: 1 };
const ZERO_POINT = { x: 0, y: 0, z: 0 };

// Built once — a small square room with a solid border, matching what
// ROS2D.OccupancyGrid needs (info.origin.position/orientation are required,
// not optional — a real publisher always includes them, so the synthetic
// one must too).
const buildDemoMapData = () => {
  const data = new Array(MAP_CELLS * MAP_CELLS).fill(0);
  for (let row = 0; row < MAP_CELLS; row++) {
    for (let col = 0; col < MAP_CELLS; col++) {
      const isBorder =
        row === 0 || col === 0 || row === MAP_CELLS - 1 || col === MAP_CELLS - 1;
      data[row * MAP_CELLS + col] = isBorder ? 100 : 0;
    }
  }
  return data;
};

const DEMO_MAP_DATA = buildDemoMapData();
const mapMessage = () => ({
  info: {
    width: MAP_CELLS,
    height: MAP_CELLS,
    resolution: MAP_RESOLUTION,
    origin: {
      position: { x: -(MAP_CELLS * MAP_RESOLUTION) / 2, y: -(MAP_CELLS * MAP_RESOLUTION) / 2, z: 0 },
      orientation: IDENTITY_QUAT,
    },
  },
  data: DEMO_MAP_DATA,
});

const EXPECTED_TOPIC_NAMES = [
  AppConfig.SCAN_TOPIC,
  AppConfig.ROBOT_POSE_TOPIC,
  AppConfig.MAP_TOPIC,
  AppConfig.GLOBAL_COSTMAP_TOPIC,
  AppConfig.PLAN_TOPIC,
  AppConfig.AMCL_POSE_TOPIC,
  AppConfig.NAV_STATUS_TOPIC,
  AppConfig.BATTERY_TOPIC,
  AppConfig.JOINT_STATES_TOPIC,
  AppConfig.TF_TOPIC,
  AppConfig.TF_STATIC_TOPIC,
];

const fakeServiceValues = (message) => {
  if (message.service?.endsWith("/get_state")) {
    return { current_state: { id: 3, label: "active" } };
  }
  if (message.service === "/rosapi/topics") {
    return {
      topics: EXPECTED_TOPIC_NAMES,
      types: EXPECTED_TOPIC_NAMES.map(() => ""),
    };
  }
  if (message.service === "/rosapi/nodes") {
    return { nodes: LIFECYCLE_NODES.map((n) => n.base) };
  }
  return {};
};

/**
 * Installs the demo transport override on the shared ROSLIB.Ros instance —
 * called directly during render (not inside a useEffect). This has to be
 * synchronous and installed before any child component's own effects run:
 * React runs child effects before parent effects on mount, so a child like
 * LifecycleStatus fires its first service call before App's own effect
 * would otherwise get a chance to patch callOnConnection, and that first
 * call would hit the real implementation (which tries to write to a
 * WebSocket that was never opened). Idempotent — safe to call every
 * render.
 */
export function installDemoTransport(ros) {
  if (ros.__demoOriginalCallOnConnection) return;
  ros.__demoOriginalCallOnConnection = ros.callOnConnection.bind(ros);
  ros.callOnConnection = (message) => {
    if (message.op === "call_service") {
      setTimeout(() => {
        ros.emit(message.id, {
          op: "service_response",
          service: message.service,
          id: message.id,
          values: fakeServiceValues(message),
          result: true,
        });
      }, 30);
    }
    // subscribe/unsubscribe/advertise/advertise_service: nothing to send —
    // the local ros.on()/off() bookkeeping already happened in roslib.js.
  };
}

export function uninstallDemoTransport(ros) {
  if (!ros.__demoOriginalCallOnConnection) return;
  ros.callOnConnection = ros.__demoOriginalCallOnConnection;
  delete ros.__demoOriginalCallOnConnection;
}

/**
 * Starts the periodic synthetic-telemetry tick loop. Safe to start slightly
 * after mount (unlike the transport override above, nothing needs this to
 * be synchronous) — it just calls ros.emit(topicName, message) directly,
 * exactly mimicking what SocketAdapter.handleMessage does for a real
 * "publish" frame, so every existing Topic subscriber gets fed with no
 * component code changes needed.
 */
export function startDemoTicking(ros) {
  const startedAt = Date.now();

  const tick = () => {
    const t = (Date.now() - startedAt) / 1000;
    const yaw = LOOP_ANGULAR_SPEED * t + Math.PI / 2;
    const x = LOOP_RADIUS * Math.cos(LOOP_ANGULAR_SPEED * t);
    const y = LOOP_RADIUS * Math.sin(LOOP_ANGULAR_SPEED * t);
    const orientation = quatFromYaw(yaw);
    const position = { x, y, z: 0 };
    const linearSpeed = LOOP_RADIUS * LOOP_ANGULAR_SPEED;

    ros.emit(AppConfig.ROBOT_POSE_TOPIC, {
      pose: { pose: { position, orientation } },
      twist: { twist: { linear: { x: linearSpeed, y: 0, z: 0 }, angular: { x: 0, y: 0, z: LOOP_ANGULAR_SPEED } } },
    });

    ros.emit(AppConfig.AMCL_POSE_TOPIC, {
      pose: { pose: { position, orientation }, covariance: new Array(36).fill(0) },
    });

    ros.emit(AppConfig.TF_TOPIC, {
      transforms: [
        {
          header: { frame_id: "map" },
          child_frame_id: "odom",
          transform: { translation: ZERO_POINT, rotation: IDENTITY_QUAT },
        },
        {
          header: { frame_id: "odom" },
          child_frame_id: "base_link",
          transform: { translation: position, rotation: orientation },
        },
        {
          header: { frame_id: "base_link" },
          child_frame_id: "lidar_link",
          transform: { translation: ZERO_POINT, rotation: IDENTITY_QUAT },
        },
      ],
    });

    ros.emit(AppConfig.SCAN_TOPIC, {
      angle_min: -Math.PI,
      angle_max: Math.PI,
      angle_increment: (2 * Math.PI) / 180,
      range_min: 0.1,
      range_max: 6,
      ranges: new Array(180).fill(0).map(() => 1.1 + Math.random() * 1.3),
    });

    ros.emit(AppConfig.MAP_TOPIC, mapMessage());
    ros.emit(AppConfig.GLOBAL_COSTMAP_TOPIC, mapMessage());

    ros.emit(AppConfig.PLAN_TOPIC, {
      poses: new Array(5).fill(0).map((_, i) => {
        const futureT = t + i * 2;
        return {
          pose: {
            position: {
              x: LOOP_RADIUS * Math.cos(LOOP_ANGULAR_SPEED * futureT),
              y: LOOP_RADIUS * Math.sin(LOOP_ANGULAR_SPEED * futureT),
              z: 0,
            },
            orientation: IDENTITY_QUAT,
          },
        };
      }),
    });

    ros.emit(AppConfig.NAV_STATUS_TOPIC, {
      status_list: [{ status: 2, goal_info: { goal_id: { id: "demo" } } }],
    });

    ros.emit(AppConfig.BATTERY_TOPIC, { data: 78 - ((t % 300) / 300) * 20 });
    ros.emit(AppConfig.CHARGE_STATION_CONNECTED, { data: false });

    ros.emit(AppConfig.JOINT_STATES_TOPIC, {
      name: ["left_wheel_joint", "right_wheel_joint"],
      position: [t * 2, t * 2],
      velocity: [1, 1],
    });
  };

  tick();
  const intervalId = setInterval(tick, TICK_MS);

  return function stopDemoTicking() {
    clearInterval(intervalId);
  };
}
