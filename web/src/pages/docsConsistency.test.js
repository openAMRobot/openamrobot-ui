/**
 * @jest-environment node
 */
// The README's page table matches the page registry and plugins, and the
// operator docs use the current names: "Stop (software)" (not E-STOP) and
// the Getac ZX80 panel (not ZX10).
const fs = require("fs");
const path = require("path");

const REPO = path.join(__dirname, "..", "..", "..");
const read = (rel) => fs.readFileSync(path.join(REPO, rel), "utf8");

const listMarkdown = (dir) =>
  fs.readdirSync(path.join(REPO, dir), { withFileTypes: true }).flatMap((entry) => {
    const rel = path.join(dir, entry.name);
    if (entry.isDirectory()) return listMarkdown(rel);
    return entry.name.endsWith(".md") ? [rel] : [];
  });

// docs/proposals/ and docs/reporting-mode.md are owned by the Reporting Mode
// work and may quote superseded names on purpose.
const NOT_CHECKED = [path.join("docs", "proposals"), path.join("docs", "reporting-mode.md")];
const OPERATOR_DOCS = [
  "README.md",
  ...listMarkdown("docs").filter((rel) => !NOT_CHECKED.some((skip) => rel.startsWith(skip))),
];

const pathsIn = (rel) =>
  [...read(rel).matchAll(/\bpath:\s*"([^"]+)"/g)].map((m) => m[1]);

describe("operator documentation", () => {
  it("README page table lists exactly the registered pages", () => {
    const registered = [
      ...pathsIn("web/src/pages/registry.js"),
      ...pathsIn("web/src/plugins/notesPlugin/index.js"),
    ].sort();
    const table = read("README.md").split("## Main pages")[1].split("\n## ")[0];
    const documented = [...table.matchAll(/^\| [^|]+ \| `(\/[^`]*)` \|/gm)]
      .map((m) => m[1])
      .sort();
    expect(documented).toEqual(registered);
  });

  it.each(OPERATOR_DOCS)("%s names the UI control Stop (software), not E-STOP", (rel) => {
    expect(read(rel)).not.toMatch(/E-STOP/);
  });

  it.each(OPERATOR_DOCS)("%s does not name the superseded ZX10 panel", (rel) => {
    expect(read(rel)).not.toMatch(/ZX10/i);
  });

  // Stop (software) on main is a one-shot stop that does not latch; #50
  // replaces it with a latched stop and explicit Resume and updates the
  // docs. Each stop passage must say one or the other, never describe the
  // future lock as today's behaviour.
  it.each([
    "README.md",
    "docs/installation.md",
    "docs/lessons/00-your-first-10-minutes.md",
    "docs/lessons/02-ros2-core-concepts.md",
    "docs/lessons/06-the-pages.md",
    "docs/lessons/07-ui-components.md",
  ])("%s describes Stop (software) as it works today", (rel) => {
    const text = read(rel);
    const oneShotToday = /does not latch/.test(text) && /#50/.test(text);
    const latchedAfter50 = /Resume/.test(text) && !/does not latch/.test(text);
    expect(oneShotToday || latchedAfter50).toBe(true);
    expect(text).not.toMatch(/non-latched/);
  });

  it("README states the amended replay policy", () => {
    const policy = read("README.md").split("### Recording replay policy")[1].split("\n## ")[0];
    expect(policy).toMatch(/plays the recording's topics that are on\s+the allowlist/);
    expect(policy).toMatch(/skipped/);
    expect(policy).toMatch(/request that names one of the never-played topics is refused \(HTTP\s+400/);
    expect(policy).not.toMatch(/cannot be\s+replayed/);
  });

  it("README and the installation guide give the maps location of #53", () => {
    const readme = read("README.md");
    for (const term of ["OPENAMROBOT_MAPS_DIR", "openamrobot_maps", "/data/maps"]) {
      expect(readme).toContain(term);
    }
    expect(readme).toMatch(/platform\s+must mount the same volume at `\/data\/maps`/);
    expect(readme).not.toContain(
      "`~/.openamr_ui/maps/`, `~/.openamr_ui/paths/`, `~/.openamr_ui/current_map_route.yaml`",
    );
    expect(read("docs/installation.md")).toContain("OPENAMROBOT_MAPS_DIR");
  });

  it("README names the Getac ZX80 panel and documents the replay policy and data directory", () => {
    const readme = read("README.md");
    expect(readme).toMatch(/Getac ZX80/);
    expect(readme).toMatch(/### Recording replay policy/);
    expect(readme).toMatch(/~\/\.openamr_ui\/maps\//);
  });
});
