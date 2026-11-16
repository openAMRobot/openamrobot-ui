import React from "react";
import { Routes, Route } from "react-router";

import AppLayout from "../layouts/appLayout";
import NotFoundPage from "./NotFoundPage";
import { PAGE_REGISTRY } from "./registry";

const Routing = () => (
  <Routes>
    {/* Standalone pages keep their registry route but render outside AppLayout. */}
    {PAGE_REGISTRY.filter(({ standalone }) => standalone).map(({ path, component: Component }) => (
      <Route key={path} path={path} element={<Component />} />
    ))}
    <Route path="/" element={<AppLayout />}>
      {PAGE_REGISTRY.filter(({ standalone }) => !standalone).map(({ path, component: Component }) =>
        path === "/" ? (
          <Route key={path} index element={<Component />} />
        ) : (
          <Route key={path} path={path.slice(1)} element={<Component />} />
        ),
      )}
      <Route path="*" element={<NotFoundPage />} />
    </Route>
  </Routes>
);
export default Routing;
