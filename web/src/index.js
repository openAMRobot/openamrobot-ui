import React from "react";
import ReactDOM from "react-dom/client";
import App from "./app/App";

// Plugins must install before the router/nav render, since they mutate
// pages/registry.js's PAGE_REGISTRY (read once by Header.jsx/pages/index.jsx
// on first render). See shared/plugins/registerPlugin.js.
import { installNotesPlugin } from "./plugins/notesPlugin";
installNotesPlugin();

const root = ReactDOM.createRoot(document.getElementById("root"));
root.render(<App />);
