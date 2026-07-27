import packageJson from "../../../package.json";
import { getEvents } from "../events/eventLog";
import { loadParamRows, readParamValue } from "../constants/navParams";

const METRICS_STORAGE_KEY = "openamrMetrics";
const PARAM_READ_TIMEOUT_MS = 2500;
const RECENT_EVENTS_LIMIT = 50;

const readMetricsSnapshot = () => {
  try {
    return JSON.parse(localStorage.getItem(METRICS_STORAGE_KEY) || "null");
  } catch {
    return null;
  }
};

// Best-effort, read-only snapshot of the curated Nav2 param list — a fresh
// ROSLIB.Service per node/get_parameters call, raced against a timeout so one
// unresponsive node can't hang the whole export. Deliberately doesn't reuse
// ParamsPage's live component state (this can be called from a page that
// never mounted it, e.g. Health Centre).
const snapshotNavParams = (ros, rosConnected) => {
  if (!ros || !rosConnected || !window.ROSLIB) {
    return Promise.resolve(
      loadParamRows().map(({ node, param, type }) => ({
        node,
        param,
        type,
        value: null,
        status: "not-read (rosbridge offline)",
      })),
    );
  }

  const services = {};
  const svc = (node) => {
    if (!services[node]) {
      services[node] = new window.ROSLIB.Service({
        ros,
        name: `${node}/get_parameters`,
        serviceType: "rcl_interfaces/srv/GetParameters",
      });
    }
    return services[node];
  };

  const readOne = ({ node, param, type }) =>
    new Promise((resolve) => {
      let done = false;
      const finish = (value, status) => {
        if (done) return;
        done = true;
        resolve({ node, param, type, value, status });
      };
      const timer = setTimeout(() => finish(null, "timeout"), PARAM_READ_TIMEOUT_MS);
      try {
        svc(node).callService(
          new window.ROSLIB.ServiceRequest({ names: [param] }),
          (res) => {
            clearTimeout(timer);
            const pv = res?.values?.[0];
            if (!pv || pv.type === 0) finish(null, "not-set");
            else finish(readParamValue(pv), "read");
          },
          () => {
            clearTimeout(timer);
            finish(null, "error");
          },
        );
      } catch {
        clearTimeout(timer);
        finish(null, "error");
      }
    });

  return Promise.all(loadParamRows().map(readOne));
};

/**
 * Assembles a single JSON-serializable snapshot for offline troubleshooting:
 * connection info, the Health Centre rollup, recent events, the persisted
 * "track record" metrics, runtime config, and a best-effort Nav2 param
 * snapshot. Nothing here is secret (no credentials exist in this app), so no
 * redaction is applied — host/IP is left in deliberately, since it's exactly
 * what a support ticket needs to reproduce a connection issue.
 *
 * Callers pass in data already computed by hooks they've mounted themselves
 * (issues/overall from useSystemDiagnostics, config from useRuntimeConfig) —
 * this function doesn't re-subscribe to anything, so it's safe to call from
 * any page without doubling up ROS subscriptions or metrics accumulators.
 */
export default async function buildSupportPackage({
  ros,
  rosStatus,
  resolvedHost,
  config,
  health,
}) {
  const navParams = await snapshotNavParams(ros, rosStatus === "connected");

  return {
    generatedAt: new Date().toISOString(),
    appVersion: packageJson.version,
    userAgent: typeof navigator !== "undefined" ? navigator.userAgent : null,
    connection: {
      rosbridgeUrl: `ws://${resolvedHost}:${config.rosbridgePort}`,
      cameraPort: config.cameraPort,
      rosStatus,
    },
    health: {
      overall: health.overall,
      overallLabel: health.overallLabel,
      issues: health.issues,
    },
    recentEvents: getEvents().slice(0, RECENT_EVENTS_LIMIT),
    metrics: readMetricsSnapshot(),
    runtimeConfig: config,
    navParams,
  };
}
