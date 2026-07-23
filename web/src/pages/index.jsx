import React from "react";
import { Routes, Route } from "react-router";

import AppLayout from "../layouts/appLayout";
import MapPage from "./MapPage";
import RoutePage from "./RoutePage";
import ControlPage from "./ControlPage";
import InfoPage from "./InfoPage";
import BlocksPage from "./BlocksPage";
import RobotDescriptionPage from "./RobotDescriptionPage";
import DevicesPage from "./DevicesPage";
import ConfigPage from "./ConfigPage";
import NotFoundPage from "./NotFoundPage";

const Routing = () => (
  <Routes>
    <Route path="/" element={<AppLayout />}>
      <Route index element={<MapPage />} />
      <Route path="route" element={<RoutePage />} />
      <Route path="control" element={<ControlPage />} />
      <Route path="blocks" element={<BlocksPage />} />
      <Route path="info" element={<InfoPage />} />
      <Route path="robot" element={<RobotDescriptionPage />} />
      <Route path="devices" element={<DevicesPage />} />
      <Route path="config" element={<ConfigPage />} />
      <Route path="*" element={<NotFoundPage />} />
    </Route>
  </Routes>
);
export default Routing;
