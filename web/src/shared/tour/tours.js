/**
 * Guided-tour step lists, keyed by the same absolute path used in
 * web/src/pages/registry.js. Each selector must match a real
 * data-tour="..." attribute somewhere in that page's rendered DOM (or
 * Header.jsx, which is always mounted). Read by HelpWidget.jsx via
 * TourOverlay.jsx.
 */
export const TOURS = {
  "/": [
    {
      selector: '[data-tour="nav"]',
      title: "Get around",
      body: "This is the main navigation — every page in the app is one click away from here.",
    },
    {
      selector: '[data-tour="connection-status"]',
      title: "Connection status",
      body: "Shows whether this browser is actually connected to a robot (or, in Demo Mode, to simulated data).",
    },
    {
      selector: '[data-tour="map-layers"]',
      title: "Map layers",
      body: "Toggle what's drawn on the map — costmap, laser scan, planned path, saved waypoints, the robot's trail — and adjust opacity.",
    },
    {
      selector: '[data-tour="map-canvas"]',
      title: "The map itself",
      body: "Drag to pan, scroll or pinch to zoom. Right-click anywhere for a quick menu — send a goal, save a waypoint, or set the initial pose.",
    },
    {
      selector: '[data-tour="map-actions"]',
      title: "Goal, pose, and waypoints",
      body: "Switch what a click on the map does: send a navigation goal, set the robot's initial pose, or drop a saved waypoint.",
    },
    {
      selector: '[data-tour="manual-drive"]',
      title: "Manual drive",
      body: "Drive the robot directly at any time — this works whether or not you're in Goal Mode.",
    },
  ],
};
