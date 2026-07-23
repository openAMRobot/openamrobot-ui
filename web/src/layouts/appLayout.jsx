import React, { useState } from "react";
import { Outlet } from "react-router-dom";
import Header from "../components/Header";
import Logs from "../components/Logs";
import NotificationsWatcher from "../components/NotificationsWatcher";
import AuthModeBanner from "../components/AuthModeBanner";

const AppLayout = () => {
  const [showLogs, setShowLogs] = useState(false);

  return (
    <div className="app-bg flex min-h-screen flex-col text-textWhiteHover md:pl-56">
      <a
        href="#main-content"
        className="fixed left-4 top-4 z-[100] -translate-y-20 rounded-lg bg-themeBlue px-4 py-2 text-sm font-semibold text-white focus:translate-y-0"
      >
        Skip to main content
      </a>
      <Header showLogs={showLogs} onToggleLogs={() => setShowLogs(!showLogs)} />
      <NotificationsWatcher />
      <main
        id="main-content"
        className="mx-auto min-h-0 w-full max-w-[1600px] flex-1 px-3 pb-4 sm:px-4 lg:px-6"
      >
        <AuthModeBanner />
        <Outlet />
      </main>

      {showLogs && (
        <section
          aria-label="System diagnostics console"
          className="fixed bottom-3 left-3 right-3 z-50 h-[260px] rounded-[18px] border border-borderSubtle bg-bgCard/95 p-3 shadow-2xl shadow-black/60 backdrop-blur-xl sm:bottom-4 sm:left-4 sm:right-4 md:left-60 lg:right-6"
        >
          <div className="mx-auto flex h-full max-w-[1560px] flex-col">
            <div className="mb-2 flex items-center justify-between">
              <span className="font-[RobotoMono] text-[10px] font-semibold uppercase tracking-[0.16em] text-themeTextGray">
                System Diagnostics Console
              </span>
              <button
                type="button"
                onClick={() => setShowLogs(false)}
                className="min-h-[36px] rounded-lg border border-borderSubtle px-3 font-[RobotoMono] text-[11px] text-themeTextGray hover:border-statusRed/40 hover:text-statusRed"
              >
                Close
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
