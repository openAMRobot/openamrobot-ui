import React from "react";
import { Routes, Route } from "react-router";

import AppLayout from "../layouts/appLayout";
import NotFoundPage from "./NotFoundPage";
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
      <Route path="*" element={<NotFoundPage />} />
    </Route>
  </Routes>
);
export default Routing;
