import React from "react";

const ErrorBoundary = () => {
  return (
    <div>
      <h1>Oops!</h1>
      <h2>Something went wrong!</h2>
      <button
        onClick={() => {
          window.location.href = "/";
        }}
      >
        Go to home page
      </button>
    </div>
  );
};

export default ErrorBoundary;
