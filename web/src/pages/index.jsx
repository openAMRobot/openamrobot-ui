import React from "react";
import { Routes, Route } from "react-router";

import AppLayout from "../layouts/appLayout";
import MapPage from "./MapPage";
import RoutePage from "./RoutePage";
import ControlPage from "./ControlPage";
import InfoPage from "./InfoPage";
import NotFoundPage from "./NotFoundPage";

const Routing = () => (
  <Routes>
    <Route path="/" element={<AppLayout />}>
      <Route index element={<MapPage />} />
      <Route path="route" element={<RoutePage />} />
      <Route path="control" element={<ControlPage />} />
      <Route path="info" element={<InfoPage />} />
      <Route path="*" element={<NotFoundPage />} />
    </Route>
  </Routes>
);
export default Routing;
