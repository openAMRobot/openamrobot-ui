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

  it("README names the Getac ZX80 panel and documents the replay policy and data directory", () => {
    const readme = read("README.md");
    expect(readme).toMatch(/Getac ZX80/);
    expect(readme).toMatch(/### Recording replay policy/);
    expect(readme).toMatch(/~\/\.openamr_ui\/maps\//);
  });
});
