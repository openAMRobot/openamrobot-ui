// Copies Blockly's sounds, sprites and cursors from the installed blockly
// package into public/blockly-media/, so the app serves them itself and the
// Programs page needs no internet (Blockly's default media host is
// blockly-demo.appspot.com). Runs before `npm run dev` and `npm run build`;
// the copy is not committed (see .gitignore).
const fs = require("fs");
const path = require("path");

// blockly's "exports" map hides package.json, so locate the package root
// from its main entry.
const root = path.dirname(require.resolve("blockly"));
const source = path.join(root, "media");
const target = path.join(__dirname, "..", "public", "blockly-media");

fs.rmSync(target, { recursive: true, force: true });
fs.mkdirSync(target, { recursive: true });
for (const name of fs.readdirSync(source)) {
  fs.copyFileSync(path.join(source, name), path.join(target, name));
}
const { version, license } = JSON.parse(
  fs.readFileSync(path.join(root, "package.json"), "utf8"),
);
fs.writeFileSync(
  path.join(target, "SOURCE.txt"),
  `Copied from the blockly npm package ${version} (licence ${license}, ` +
    "https://github.com/google/blockly), media/ folder, unmodified.\n",
);
console.log(`[copy-blockly-media] ${fs.readdirSync(source).length} files -> public/blockly-media/`);
