import React from "react";

import { Outlet } from "react-router-dom";
import Header from "../components/Header";

const AppLayout = () => {
  return (
    <div>
      <Header />
      <main className="mx-auto max-w-[1920px] px-6 lg:px-10 2xl:px-20">
        <Outlet />
      </main>
    </div>
  );
};
export default AppLayout;
