/**
 * @jest-environment node
 */
// README and docs describe the Stop control as PR #50 implements it: a
// latched Stop (software) on the platform stop lock, released only by an
// explicit Resume, with controls disabled while the lock is engaged or
// unknown. Old one-shot wording must not come back.
const fs = require("fs");
const path = require("path");

const REPO = path.join(__dirname, "..", "..", "..", "..");
const read = (rel) => fs.readFileSync(path.join(REPO, rel), "utf8");

const STOP_DOCS = [
  "README.md",
  "docs/installation.md",
  "docs/lessons/00-your-first-10-minutes.md",
  "docs/lessons/01-what-is-this-ui.md",
  "docs/lessons/02-ros2-core-concepts.md",
  "docs/lessons/06-the-pages.md",
  "docs/lessons/07-ui-components.md",
];

const ONE_SHOT_WORDING = [
  /E-STOP/,
  /non-latched/i,
  /not\s+latched/i,
  /one-shot software intervention/i,
  /publishes one zero `Twist`/,
  /zeroes `\/cmd_vel`/,
  /sends one\s+zero-velocity command/i,
];

describe("Stop (software) documentation", () => {
  it.each(STOP_DOCS)("%s has no one-shot stop wording", (rel) => {
    const text = read(rel);
    ONE_SHOT_WORDING.forEach((re) => expect(text).not.toMatch(re));
  });

  it.each(["README.md", "docs/installation.md", "docs/lessons/00-your-first-10-minutes.md", "docs/lessons/01-what-is-this-ui.md", "docs/lessons/06-the-pages.md"])(
    "%s describes the latched stop, Resume and disabled controls",
    (rel) => {
      // Join wrapped lines, including Markdown blockquote markers.
      const text = read(rel).replace(/\n>\s?/g, " ").replace(/\s+/g, " ");
      expect(text).toMatch(/Stop \(software\)/);
      expect(text).toMatch(/Resume/);
      expect(text).toMatch(/latche[sd]|stays engaged|lock stays/i);
      expect(text).toMatch(/engaged,? or (while )?(its state is |the lock is )?unknown/i);
      expect(text).toMatch(/disabled/i);
    },
  );

  it("no doc says the UI publishes /cmd_vel", () => {
    [...STOP_DOCS, "docs/lessons/09-blockly-programming.md", "web/src/features/blocks/README.md"].forEach((rel) => {
      const text = read(rel).replace(/\s+/g, " ");
      expect(text).not.toMatch(/`\/cmd_vel` carries velocity commands the UI publishes/);
      expect(text).not.toMatch(/Direct `\/cmd_vel` commands/);
      expect(text).not.toMatch(/Publishes to `AppConfig\.CMD_VEL_TOPIC`/);
    });
  });
});
