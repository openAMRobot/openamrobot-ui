import { AppConfig } from "./index";

const STORAGE_KEY = "openamrRuntimeConfig";

// Overridable at runtime from the Config page, persisted to localStorage.
// An empty rosbridgeHost means "auto" — keep the existing behavior of
// following the page's own host in production, falling back to
// AppConfig.ROSBRIDGE_SERVER_IP only for local dev on :3000.
export const DEFAULT_RUNTIME_CONFIG = {
  rosbridgeHost: "",
  rosbridgePort: AppConfig.ROSBRIDGE_SERVER_PORT,
  cameraPort: AppConfig.CAMERA_PORT,
  maxLinearSpeed: AppConfig.MAX_LINEAR_SPEED,
  maxAngularSpeed: AppConfig.MAX_ANGULAR_SPEED,
  notificationsEnabled: false,
  lowBatteryThreshold: 20,
  demoMode: false,
};

export function loadRuntimeConfig() {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY) || "{}");
    return { ...DEFAULT_RUNTIME_CONFIG, ...stored };
  } catch {
    return { ...DEFAULT_RUNTIME_CONFIG };
  }
}

export function saveRuntimeConfig(config) {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(config));
}

// Same "auto-detect vs explicit override" rule previously duplicated inline
// in App.jsx and Camera.jsx: prefer a user-set host override; otherwise use
// the page's own hostname (correct automatically once deployed on the
// robot), falling back to AppConfig's dev default only when running the
// CRA dev server on :3000.
export function resolveRosbridgeHost(config) {
  if (config.rosbridgeHost) return config.rosbridgeHost;
  const isLocalDevServer = window.location.href.includes(
    "http://localhost:3000/",
  );
  return isLocalDevServer
    ? AppConfig.ROSBRIDGE_SERVER_IP
    : window.location.hostname;
}
