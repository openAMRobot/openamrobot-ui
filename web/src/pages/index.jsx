import React from "react";
import { Routes, Route } from "react-router";

import AppLayout from "../layouts/appLayout";
import NotFoundPage from "./NotFoundPage";
import CurrentRunFixturePage from "./CurrentRunFixturePage";
import { PAGE_REGISTRY } from "./registry";

const Routing = () => (
  <Routes>
    <Route path="/" element={<AppLayout />}>
      {PAGE_REGISTRY.map(({ path, component: Component }) =>
        path === "/" ? (
          <Route key={path} index element={<Component />} />
        ) : (
          <Route key={path} path={path.slice(1)} element={<Component />} />
        ),
      )}
      {/* Development-only fixture view: reachable by URL, deliberately not in
          PAGE_REGISTRY/nav. Shows "Fixture backend disabled" unless the
          backend runs with OPENAMR_RUN_STATE_FIXTURES=1. */}
      <Route path="dev/current-run" element={<CurrentRunFixturePage />} />
      <Route path="*" element={<NotFoundPage />} />
    </Route>
  </Routes>
);
export default Routing;
