/**
 * Static guard for the manual drive path (issue #31): parse every source file
 * under web/src and fail if
 * - a ROSLIB Topic with a Twist or TwistStamped message type is constructed
 *   anywhere except shared/motion/uiCommand.js,
 * - uiCommand.js publishes on any name other than AppConfig.UI_CMD_VEL_TOPIC,
 * - any topic name, or any AppConfig value, is /cmd_vel or a cmd_vel_* topic,
 * - anything but the Resume path calls the stop lock release service.
 * Help text, comments and tests that mention cmd_vel are allowed: only string
 * values used as code are checked.
 */
import fs from "fs";
import path from "path";
import { parse } from "@babel/parser";

import { AppConfig } from "../constants";

const SRC = path.resolve(__dirname, "../..");
const UI_COMMAND = path.join("shared", "motion", "uiCommand.js");
const VELOCITY_TYPE = /(^|\/)Twist(Stamped)?$/;
const FORBIDDEN_TOPIC = /^\/?cmd_vel(_[A-Za-z0-9_]+)?$/;

const listSources = (dir) =>
  fs.readdirSync(dir, { withFileTypes: true }).flatMap((entry) => {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) return listSources(full);
    if (!/\.(js|jsx)$/.test(entry.name) || /\.test\.(js|jsx)$/.test(entry.name)) return [];
    if (full.includes(`${path.sep}testing${path.sep}`)) return [];
    return [full];
  });

const parseFile = (file) =>
  parse(fs.readFileSync(file, "utf8"), {
    sourceType: "module",
    plugins: ["jsx", "classProperties", "optionalChaining"],
    errorRecovery: true,
  });

const walk = (node, visit, parent = null) => {
  if (!node || typeof node.type !== "string") return;
  visit(node, parent);
  Object.keys(node).forEach((key) => {
    if (key === "loc" || key === "start" || key === "end") return;
    const child = node[key];
    if (Array.isArray(child)) child.forEach((c) => walk(c, visit, node));
    else if (child && typeof child.type === "string") walk(child, visit, node);
  });
};

const memberPath = (node) => {
  if (!node) return "";
  if (node.type === "Identifier") return node.name;
  if (node.type === "MemberExpression" || node.type === "OptionalMemberExpression") {
    const prop = node.computed ? "[]" : node.property.name;
    return `${memberPath(node.object)}.${prop}`;
  }
  return "";
};

const isRoslibTopic = (node) =>
  node.type === "NewExpression" && /(^|\.)ROSLIB\.Topic$/.test(memberPath(node.callee));

const objectProp = (obj, name) =>
  obj?.type === "ObjectExpression"
    ? obj.properties.find((p) => p.key && (p.key.name === name || p.key.value === name))
    : undefined;

const literal = (node) =>
  node?.type === "StringLiteral" ? node.value : node?.type === "TemplateLiteral" ? null : undefined;

const files = listSources(SRC).map((file) => ({
  file,
  rel: path.relative(SRC, file),
  ast: parseFile(file),
}));

describe("static guard: velocity publishers", () => {
  it("scans a real source tree", () => {
    expect(files.length).toBeGreaterThan(50);
    expect(files.map((f) => f.rel)).toContain(UI_COMMAND);
  });

  it("constructs no Twist or TwistStamped Topic outside uiCommand.js", () => {
    const offenders = [];
    files.forEach(({ rel, ast }) => {
      walk(ast, (node) => {
        if (!isRoslibTopic(node)) return;
        const type = literal(objectProp(node.arguments[0], "messageType")?.value);
        if (typeof type === "string" && VELOCITY_TYPE.test(type) && rel !== UI_COMMAND) {
          offenders.push(`${rel}:${node.loc.start.line} ${type}`);
        }
      });
    });
    expect(offenders).toEqual([]);
  });

  it("uiCommand.js publishes only on AppConfig.UI_CMD_VEL_TOPIC", () => {
    const { ast } = files.find((f) => f.rel === UI_COMMAND);
    const topics = [];
    walk(ast, (node) => {
      if (!isRoslibTopic(node)) return;
      const arg = node.arguments[0];
      topics.push({
        name: memberPath(objectProp(arg, "name")?.value) || literal(objectProp(arg, "name")?.value),
        type: literal(objectProp(arg, "messageType")?.value),
      });
    });
    expect(topics).toEqual([{ name: "AppConfig.UI_CMD_VEL_TOPIC", type: "geometry_msgs/TwistStamped" }]);
    expect(AppConfig.UI_CMD_VEL_TOPIC).toBe("/ui/cmd_vel");
  });

  it("no Topic name literal and no AppConfig value is /cmd_vel or cmd_vel_*", () => {
    const offenders = [];
    files.forEach(({ rel, ast }) => {
      walk(ast, (node) => {
        if (!isRoslibTopic(node)) return;
        const name = literal(objectProp(node.arguments[0], "name")?.value);
        if (typeof name === "string" && FORBIDDEN_TOPIC.test(name)) offenders.push(`${rel} ${name}`);
      });
    });
    Object.entries(AppConfig).forEach(([key, value]) => {
      if (typeof value === "string" && FORBIDDEN_TOPIC.test(value)) offenders.push(`AppConfig.${key}`);
    });
    expect(offenders).toEqual([]);
  });

  it("release: only stopLock.js names the service, only softwareStop.js calls it, only the Resume banner resumes", () => {
    const releaseNames = [];
    const releaseCallers = [];
    const resumeCallers = [];
    files.forEach(({ rel, ast }) => {
      walk(ast, (node) => {
        if (memberPath(node) === "AppConfig.STOP_LOCK_RELEASE_SERVICE") releaseNames.push(rel);
        if (node.type === "StringLiteral" && node.value === "/safety/stop_lock_release" && rel !== path.join("shared", "constants", "index.js")) {
          releaseNames.push(rel);
        }
        if (node.type === "CallExpression" && node.callee.name === "releaseStopLock") releaseCallers.push(rel);
        if (node.type === "CallExpression" && node.callee.name === "resumeFromSoftwareStop") resumeCallers.push(rel);
      });
    });
    expect([...new Set(releaseNames)]).toEqual([path.join("shared", "motion", "stopLock.js")]);
    expect([...new Set(releaseCallers)]).toEqual([path.join("shared", "motion", "softwareStop.js")]);
    expect([...new Set(resumeCallers)]).toEqual([path.join("components", "MotionStopBanner.jsx")]);
  });

  it("flags a forbidden publisher in a sample (the guard is not vacuous)", () => {
    const sample = parse(
      'const t = new window.ROSLIB.Topic({ ros, name: "/cmd_vel", messageType: "geometry_msgs/Twist" });',
      { sourceType: "module" },
    );
    const found = [];
    walk(sample, (node) => {
      if (!isRoslibTopic(node)) return;
      const arg = node.arguments[0];
      found.push([literal(objectProp(arg, "name").value), literal(objectProp(arg, "messageType").value)]);
    });
    expect(found).toEqual([["/cmd_vel", "geometry_msgs/Twist"]]);
    expect(FORBIDDEN_TOPIC.test("/cmd_vel")).toBe(true);
    expect(FORBIDDEN_TOPIC.test("cmd_vel_joy")).toBe(true);
    expect(FORBIDDEN_TOPIC.test("/ui/cmd_vel")).toBe(false);
    expect(VELOCITY_TYPE.test("geometry_msgs/msg/TwistStamped")).toBe(true);
  });
});
