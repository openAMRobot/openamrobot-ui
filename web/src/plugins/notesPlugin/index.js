import { registerPlugin } from "../../shared/plugins/registerPlugin";
import NotesPage from "./NotesPage";

/**
 * Example plugin manifest — copy this folder as a starting point for a real
 * one. See ../../shared/plugins/pluginSchema.js for the full field list and
 * ../../shared/plugins/registerPlugin.js for what happens with it.
 */
const manifest = {
  id: "example-notes",
  name: "Notes (example plugin)",
  version: "1.0.0",
  compatibleWith: ">=0.0.1",
  pages: [{ path: "/notes", label: "Notes", icon: "notes", component: NotesPage }],
};

export function installNotesPlugin() {
  return registerPlugin(manifest);
}
