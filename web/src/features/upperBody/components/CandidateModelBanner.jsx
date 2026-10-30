import React from "react";

import { DashboardCard } from "../../../shared/ui/Dashboard";
import { CANDIDATE_MODEL_SOURCE } from "../upperBodyModel";

/** Shown whenever the page displays the combined OpenAMRobot 2.0 model. */
const CandidateModelBanner = ({ placeholderLinks }) => {
  const { repository, pullRequest, commit } = CANDIDATE_MODEL_SOURCE;
  return (
    <DashboardCard className="p-4 ring-1 ring-statusYellow/50" role="status">
      <p className="text-sm font-semibold text-statusYellow">Candidate model, provisional geometry</p>
      <p className="mt-1 text-[12px] leading-snug text-themeTextGray">
        Model as published by the robot (<code>robot_description</code>). Source: <code>{repository}</code> PR #
        {pullRequest}, checked against commit <code>{commit}</code>. The robot does not report which commit it
        runs. This model is not accepted (P-03 item 25).
      </p>
      {placeholderLinks.length ? (
        <p className="mt-1 text-[12px] leading-snug text-themeTextGray">
          Placeholder positions, pending M-09: <code>{placeholderLinks.join(", ")}</code>
        </p>
      ) : null}
    </DashboardCard>
  );
};

export default CandidateModelBanner;
