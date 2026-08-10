import React from "react";

const ErrorBoundary = () => {
  return (
    <div className="dark flex min-h-screen items-center justify-center bg-bgBase px-4 text-textWhiteHover">
      <div className="dashboard-card w-full max-w-md p-8 text-center">
        <div className="mx-auto flex h-12 w-12 items-center justify-center rounded-full border border-statusRed/30 bg-statusRed/10 text-statusRed">
          <svg
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="1.8"
            strokeLinecap="round"
            strokeLinejoin="round"
            className="h-6 w-6"
            aria-hidden="true"
          >
            <path d="M12 9v4M12 17h.01" />
            <path d="M10.29 3.86 1.82 18a1.5 1.5 0 0 0 1.3 2.25h17.76a1.5 1.5 0 0 0 1.3-2.25L13.71 3.86a1.5 1.5 0 0 0-2.42 0Z" />
          </svg>
        </div>
        <h1 className="mt-4 text-lg font-bold text-textWhiteHover">
          Something went wrong
        </h1>
        <p className="mt-2 text-sm text-themeTextGray">
          The interface hit an unexpected error and couldn't continue. This
          doesn't affect the robot itself — its ROS stack keeps running
          independently of this UI.
        </p>
        <button
          type="button"
          onClick={() => {
            window.location.href = "/";
          }}
          className="mt-6 min-h-[42px] w-full rounded-xl border border-themeBlue bg-themeBlue px-4 font-[RobotoMono] text-xs font-semibold text-white transition-colors hover:bg-themeMediumBlue"
        >
          Return to dashboard
        </button>
      </div>
    </div>
  );
};

export default ErrorBoundary;
