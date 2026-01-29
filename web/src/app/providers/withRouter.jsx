import React, { Suspense } from "react";
import { BrowserRouter } from "react-router-dom";

const withRouter = (component) => () => (
  <BrowserRouter>
    {/* <Suspense fallback={<Spinner />}>{component()}</Suspense> */}
    <Suspense>{component()}</Suspense>
  </BrowserRouter>
);

export default withRouter;
