// The upper-body view is read-only. Besides the runtime check in
// RobotDescriptionPage.test.jsx, fail if any of its source files reaches for
// a publisher, service, action or parameter API at all.
import fs from "fs";
import path from "path";

const FORBIDDEN = [
  /\.publish\s*\(/,
  /\badvertise\s*\(/,
  /callService/,
  /ROSLIB\.(Service|ServiceRequest|ActionClient|Action|Goal|Param)\b/,
  /\bsendGoal\b/,
];

const sourceFiles = (dir) =>
  fs.readdirSync(dir, { withFileTypes: true }).flatMap((entry) => {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) return sourceFiles(full);
    return /\.(js|jsx)$/.test(entry.name) && !/\.test\.(js|jsx)$/.test(entry.name) ? [full] : [];
  });

it("has source files to check", () => {
  expect(sourceFiles(__dirname).length).toBeGreaterThan(0);
});

it.each(sourceFiles(__dirname).map((f) => [path.relative(__dirname, f), f]))("%s has no command path", (_name, file) => {
  const text = fs.readFileSync(file, "utf8");
  FORBIDDEN.forEach((pattern) => expect(text).not.toMatch(pattern));
});
