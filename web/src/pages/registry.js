import MapPage from "./MapPage";
import RoutePage from "./RoutePage";
import InfoPage from "./InfoPage";
import BlocksPage from "./BlocksPage";
import RobotDescriptionPage from "./RobotDescriptionPage";
import DevicesPage from "./DevicesPage";
import HealthPage from "./HealthPage";
import RecordingsPage from "./RecordingsPage";
import ConsolePage from "./ConsolePage";
import EventsPage from "./EventsPage";
import MetricsPage from "./MetricsPage";
import ParamsPage from "./ParamsPage";
import FleetPage from "./FleetPage";
import MapsPage from "./MapsPage";
import SchedulerPage from "./SchedulerPage";
import ConfigPage from "./ConfigPage";

/**
 * Single source of truth for every top-level page. Header.jsx's sidebar/
 * mobile nav and pages/index.jsx's routing both read this array instead of
 * each keeping their own hardcoded list — a contributor adding a new page
 * appends one entry here and nothing else needs editing.
 *
 * `path` is always the absolute nav path (what NavLink's `to` wants);
 * pages/index.jsx derives the relative <Route path> from it. `icon` must
 * match a case in Header.jsx's NavIcon (unregistered names fall back to a
 * generic dot, so a new entry never renders as a blank slot).
 */
export const PAGE_REGISTRY = [
  { path: "/", label: "Map", icon: "map", component: MapPage },
  { path: "/route", label: "Routes", icon: "route", component: RoutePage },
  { path: "/maps", label: "Maps", icon: "maps", component: MapsPage },
  { path: "/blocks", label: "Programs", icon: "blocks", component: BlocksPage },
  { path: "/scheduler", label: "Scheduler", icon: "scheduler", component: SchedulerPage },
  { path: "/info", label: "Status", icon: "status", component: InfoPage },
  { path: "/robot", label: "Robot", icon: "robot", component: RobotDescriptionPage },
  { path: "/devices", label: "Devices", icon: "devices", component: DevicesPage },
  { path: "/health", label: "Health", icon: "health", component: HealthPage },
  { path: "/metrics", label: "Metrics", icon: "metrics", component: MetricsPage },
  { path: "/recordings", label: "Recordings", icon: "recordings", component: RecordingsPage },
  { path: "/events", label: "Events", icon: "events", component: EventsPage },
  { path: "/console", label: "Console", icon: "console", component: ConsolePage },
  { path: "/params", label: "Parameters", icon: "params", component: ParamsPage },
  { path: "/fleet", label: "Fleet", icon: "fleet", component: FleetPage },
  { path: "/config", label: "Config", icon: "config", component: ConfigPage },
];

/** Appends a page plugin to the registry. Call before the app renders. */
export function registerPage(entry) {
  PAGE_REGISTRY.push(entry);
}
