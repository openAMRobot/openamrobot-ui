import React from "react";

import CurrentRunFixtureView from "../components/CurrentRunFixtureView";
import { SectionHeader } from "../shared/ui/Dashboard";

const CurrentRunFixturePage = () => (
  <div className="space-y-4 py-4">
    <SectionHeader
      eyebrow="Development"
      title="Current run (fixture)"
      description="Read-only view of deterministic run-state fixture data served by the Flask backend."
    />
    <CurrentRunFixtureView />
  </div>
);

export default CurrentRunFixturePage;
