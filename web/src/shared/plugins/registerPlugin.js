import { registerPage } from "../../pages/registry";
import { registerDeviceType } from "../../features/devices/deviceTypeRegistry";
import { validateManifest, isCompatible } from "./pluginSchema";

/**
 * Validates a plugin manifest and, if valid, registers its pages and device
 * types into the app's existing registries (pages/registry.js,
 * features/devices/deviceTypeRegistry.js) — a thin, validating front door
 * over those two so a plugin author fills in one manifest instead of
 * learning both registries' internals separately.
 *
 * This app has no dynamic/remote plugin loading (no fetching a manifest.json
 * from a URL and eval-ing a bundle) — plugins are in-tree JS modules,
 * explicitly imported and installed from web/src/index.js before the app
 * renders. That's a deliberate scope decision: dynamically executing
 * arbitrary third-party code in a robot control UI is a real attack
 * surface with no real consumer asking for it yet, not a missing feature.
 *
 * An incompatible `compatibleWith` only warns (the plugin often still
 * works) — an invalid manifest (missing required fields) registers nothing
 * at all, since a half-registered plugin is worse than none.
 */
export function registerPlugin(manifest) {
  const { valid, errors } = validateManifest(manifest);
  if (!valid) {
    console.error(`[plugin:${manifest?.id || "unknown"}] invalid manifest, not registered:`, errors);
    return { ok: false, errors };
  }

  if (!isCompatible(manifest.compatibleWith)) {
    console.warn(
      `[plugin:${manifest.id}] declares compatibleWith "${manifest.compatibleWith}", which this app version may not satisfy — installing anyway.`,
    );
  }

  (manifest.pages || []).forEach((page) => registerPage(page));
  (manifest.deviceTypes || []).forEach((deviceType) => registerDeviceType(deviceType));

  return { ok: true, errors: [] };
}
