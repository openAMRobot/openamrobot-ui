import * as Blockly from "blockly";

// Blockly renders its own SVG/DOM chrome (workspace background, toolbox,
// flyout, scrollbars) independent of the page's CSS custom properties, so
// it needs its own theme object rather than inheriting global.css tokens.
// Block category colours (toolbox.js / blockDefinitions.js) are left as-is
// deliberately — they identify block *type* at a glance, a distinct
// function from the app's accent palette, not just leftover theming.
export const openAmrDarkTheme = Blockly.Theme.defineTheme("openamrDark", {
  base: Blockly.Themes.Classic,
  componentStyles: {
    workspaceBackgroundColour: "#0d0d11", // surface-2
    toolboxBackgroundColour: "#08080a", // surface-1, recessed relative to workspace
    toolboxForegroundColour: "#98989f", // text-secondary
    flyoutBackgroundColour: "#121217", // surface-3
    flyoutForegroundColour: "#98989f",
    flyoutOpacity: 1,
    scrollbarColour: "#28282f",
    scrollbarOpacity: 0.7,
    insertionMarkerColour: "#8b5cf6", // accent-violet
    insertionMarkerOpacity: 0.4,
    markerColour: "#38bdf8", // technical-cyan — this is Blockly's keyboard-nav cursor, a spatial/technical indicator
    cursorColour: "#38bdf8",
  },
  fontStyle: {
    family: "Inter, ui-sans-serif, system-ui, sans-serif",
    weight: "normal",
    size: 11,
  },
});

// Workspace grid dot colour — passed separately to Blockly.inject's `grid`
// option, not part of the Theme object.
export const BLOCKLY_GRID_COLOUR_DARK = "#1c1c22";
export const BLOCKLY_GRID_COLOUR_LIGHT = "#d8e4ed";
