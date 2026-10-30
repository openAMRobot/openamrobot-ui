import { installMockRoslib, uninstallMockRoslib } from "../motion/testing/mockRoslib";
import { AppConfig } from "../constants";
import { createReportingSource } from "./reportingSource";
import { DISPLAY } from "./reportingAdapter";
import { battery, diagnostics, lock, navStatus } from "./testing/fixtures";

// reportingSource imports App, which pulls in every page; three.js examples
// and urdf-loader ship as ES modules that CRA's Jest does not transform.
jest.mock("three/examples/jsm/controls/OrbitControls.js", () => ({ OrbitControls: function OrbitControls() {} }));
jest.mock("urdf-loader", () => function URDFLoader() {});
jest.mock("../events/eventLog", () => ({ addEvent: jest.fn() }));

// The source stamps each event with the wall clock, so a latched
// diagnostics fault names the panel-local time of its last error.
it("a latched diagnostics fault shows the wall-clock time of the last error", () => {
  const mock = installMockRoslib();
  const ros = new window.ROSLIB.Ros();
  let local = 1000;
  const wall = new Date(2026, 9, 30, 14, 5, 9).getTime();
  const source = createReportingSource({ ros, ROSLIB: window.ROSLIB, now: () => local, wallNow: () => wall });
  const emit = (name, msg) => mock.topicsNamed(name).forEach((t) => t.emit(msg));
  source.setConnection("connected");
  emit(AppConfig.STOP_LOCK_TOPIC, lock(false));
  emit(AppConfig.NAVIGATION_STATUS_TOPIC, navStatus());
  emit(AppConfig.BATTERY_TOPIC, battery(80));
  emit(AppConfig.DIAGNOSTICS_TOPIC, diagnostics(2, "lidar", "no data"));
  local = 1200;
  emit(AppConfig.STOP_LOCK_TOPIC, lock(false));
  const v = source.getView();
  expect(v.display).toBe(DISPLAY.FAULTED);
  expect(v.reason).toBe("Diagnostics error: lidar: no data (last error at 14:05:09)");
  source.stop();
  uninstallMockRoslib();
});
