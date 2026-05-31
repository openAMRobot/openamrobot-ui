import React, { useState } from "react";
import { Outlet } from "react-router-dom";
import Header from "../components/Header";
import Logs from "../components/Logs";

const AppLayout = () => {
  const [showLogs, setShowLogs] = useState(false);

  return (
    <div className="flex min-h-screen flex-col bg-[radial-gradient(circle_at_top_left,rgba(14,159,188,0.10),transparent_28rem),linear-gradient(180deg,#f8fbfd_0%,#eef5f9_100%)] text-textWhiteHover">
      <Header showLogs={showLogs} onToggleLogs={() => setShowLogs(!showLogs)} />
      <main className="mx-auto min-h-0 w-full max-w-[1920px] flex-1 px-4 pb-4 lg:px-8 2xl:px-20">
        <Outlet />
      </main>

      {showLogs && (
        <section className="fixed bottom-0 left-0 right-0 z-50 h-[240px] border-t border-borderSubtle bg-bgCard px-6 py-2 shadow-2xl shadow-slate-300/40 transition-all duration-300 ease-in-out lg:px-10">
          <div className="flex h-full flex-col">
            <div className="flex items-center justify-between border-b border-borderSubtle pb-1">
              <span className="font-[RobotoMono] text-xs font-semibold uppercase tracking-wider text-themeTextGray">
                System Diagnostics Console
              </span>
              <button
                onClick={() => setShowLogs(false)}
                className="font-[RobotoMono] text-xs text-statusRed hover:underline"
              >
                Close [X]
              </button>
            </div>
            <div className="flex-1 overflow-hidden pt-1">
              <Logs />
            </div>
          </div>
        </section>
      )}
    </div>
  );
};
export default AppLayout;
