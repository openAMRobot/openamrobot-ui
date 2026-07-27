import React from "react";
import { Link } from "react-router-dom";
import { DashboardCard, EmptyState } from "../shared/ui/Dashboard";

const NotFoundPage = () => {
  return (
    <div className="flex min-h-[calc(100vh-120px)] items-center justify-center py-8">
      <DashboardCard className="w-full max-w-xl">
        <EmptyState
          title="Page not found"
          description="This workspace does not contain the requested screen."
          action={
            <Link
              to="/"
              className="inline-flex min-h-[42px] items-center rounded-xl bg-themeBlue px-4 py-2 text-sm font-semibold text-white hover:bg-themeMediumBlue"
            >
              Return to map
            </Link>
          }
        />
      </DashboardCard>
    </div>
  );
};

export default NotFoundPage;
