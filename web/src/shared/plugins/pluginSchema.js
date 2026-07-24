import appPackageJson from "../../../package.json";

/**
 * The shape a plugin manifest passed to registerPlugin() must have.
 * Plain-JS documentation (this repo has no TypeScript), enforced at runtime
 * by validateManifest() below rather than at compile time.
 *
 *   {
 *     id: string,               // unique, e.g. "example-notes"
 *     name: string,             // human-readable
 *     version: string,          // the plugin's own version, e.g. "1.0.0"
 *     compatibleWith: string,   // minimum app version, e.g. ">=0.0.1"
 *     pages: [{ path, label, icon, component }],       // optional
 *     deviceTypes: [{ value, label }],                 // optional
 *   }
 */

const parseVersion = (v) =>
  String(v || "")
    .split(".")
    .map((n) => parseInt(n, 10) || 0);

const compareVersions = (a, b) => {
  const [aMaj, aMin, aPatch] = parseVersion(a);
  const [bMaj, bMin, bPatch] = parseVersion(b);
  if (aMaj !== bMaj) return aMaj - bMaj;
  if (aMin !== bMin) return aMin - bMin;
  return (aPatch || 0) - (bPatch || 0);
};

// Only ">=x.y.z" is supported — the one comparison that actually matters for
// "does this plugin work with the app version currently running." No need
// for a full semver-range parser/dependency for a single-deployment app
// that doesn't dynamically load third-party plugins from anywhere.
export function isCompatible(compatibleWith, appVersion = appPackageJson.version) {
  if (!compatibleWith) return true;
  const match = /^>=\s*(\d+\.\d+\.\d+)$/.exec(compatibleWith.trim());
  if (!match) return true; // unrecognized format — don't block on it, just skip the check
  return compareVersions(appVersion, match[1]) >= 0;
}

export function validateManifest(manifest) {
  const errors = [];
  if (!manifest || typeof manifest !== "object") {
    return { valid: false, errors: ["manifest must be an object"] };
  }
  if (!manifest.id) errors.push("id is required");
  if (!manifest.name) errors.push("name is required");
  if (!manifest.version) errors.push("version is required");
  (manifest.pages || []).forEach((p, i) => {
    if (!p.path || !p.component) {
      errors.push(`pages[${i}] needs at least { path, component }`);
    }
  });
  (manifest.deviceTypes || []).forEach((d, i) => {
    if (!d.value || !d.label) {
      errors.push(`deviceTypes[${i}] needs { value, label }`);
    }
  });
  return { valid: errors.length === 0, errors };
}
