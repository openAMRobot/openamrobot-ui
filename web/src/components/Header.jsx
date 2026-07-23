import React, { useContext, useState } from "react";
import { NavLink } from "react-router-dom";
import { ThemeContext, useRosStatus, useRuntimeConfig } from "../app/App";
import { resolveRosbridgeHost } from "../shared/constants/runtimeConfig";
import { IconButton, StatusBadge } from "../shared/ui/Dashboard";

const statusConfig = {
  connected: { label: "Robot connected", pulse: true },
  disconnected: { label: "Robot offline", pulse: false },
  error: { label: "Connection error", pulse: false },
};

const NAV_LINKS = [
  { to: "/", label: "Map", icon: "map" },
  { to: "/route", label: "Routes", icon: "route" },
  { to: "/control", label: "Control", icon: "control" },
  { to: "/blocks", label: "Programs", icon: "blocks" },
  { to: "/info", label: "Status", icon: "status" },
  { to: "/robot", label: "Robot", icon: "robot" },
  { to: "/devices", label: "Devices", icon: "devices" },
  { to: "/config", label: "Config", icon: "config" },
];

const NavIcon = ({ name }) => {
  const paths = {
    map: (
      <>
        <path d="m3 6 5-2 8 3 5-2v13l-5 2-8-3-5 2V6Z" />
        <path d="M8 4v13M16 7v13" />
      </>
    ),
    route: (
      <>
        <circle cx="6" cy="18" r="2" />
        <circle cx="18" cy="6" r="2" />
        <path d="M8 18h3a3 3 0 0 0 3-3V9a3 3 0 0 1 3-3h-1" />
      </>
    ),
    control: (
      <>
        <path d="M4 7h10M18 7h2M4 17h2M10 17h10" />
        <circle cx="16" cy="7" r="2" />
        <circle cx="8" cy="17" r="2" />
      </>
    ),
    blocks: (
      <>
        <rect x="3" y="3" width="7" height="7" rx="2" />
        <rect x="14" y="3" width="7" height="7" rx="2" />
        <rect x="3" y="14" width="7" height="7" rx="2" />
        <path d="M14 17.5h7M17.5 14v7" />
      </>
    ),
    status: (
      <>
        <path d="M4 19V9M10 19V5M16 19v-7M22 19V3" />
      </>
    ),
    robot: (
      <>
        <rect x="5" y="9" width="14" height="10" rx="2" />
        <circle cx="9.5" cy="14" r="1.4" />
        <circle cx="14.5" cy="14" r="1.4" />
        <path d="M12 9V5M9 5h6" />
        <circle cx="12" cy="3.6" r="1.1" />
      </>
    ),
    devices: (
      <>
        <path d="M9 3v4M15 3v4" />
        <rect x="6" y="7" width="12" height="7" rx="2" />
        <path d="M9 14v1.5A3.5 3.5 0 0 0 12.5 19h0A3.5 3.5 0 0 0 16 15.5V14" />
        <path d="M12.5 19v2" />
      </>
    ),
    config: (
      <>
        <circle cx="12" cy="12" r="3" />
        <path d="M19.4 13a1.7 1.7 0 0 0 .34 1.87l.06.06a2 2 0 1 1-2.83 2.83l-.06-.06a1.7 1.7 0 0 0-1.87-.34 1.7 1.7 0 0 0-1.04 1.56V19a2 2 0 1 1-4 0v-.09A1.7 1.7 0 0 0 9 17.35a1.7 1.7 0 0 0-1.87.34l-.06.06a2 2 0 1 1-2.83-2.83l.06-.06A1.7 1.7 0 0 0 4.65 13a1.7 1.7 0 0 0-1.56-1.04H3a2 2 0 1 1 0-4h.09A1.7 1.7 0 0 0 4.65 6.96a1.7 1.7 0 0 0-.34-1.87l-.06-.06a2 2 0 1 1 2.83-2.83l.06.06A1.7 1.7 0 0 0 9 2.65a1.7 1.7 0 0 0 1.04-1.56V1a2 2 0 1 1 4 0v.09a1.7 1.7 0 0 0 1.04 1.56 1.7 1.7 0 0 0 1.87-.34l.06-.06a2 2 0 1 1 2.83 2.83l-.06.06A1.7 1.7 0 0 0 19.35 9c.15.62.62 1.13 1.56 1.04H21a2 2 0 1 1 0 4h-.09a1.7 1.7 0 0 0-1.51 1z" />
      </>
    ),
  };

  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.8"
      strokeLinecap="round"
      strokeLinejoin="round"
      className="h-4 w-4"
      aria-hidden="true"
    >
      {paths[name]}
    </svg>
  );
};

const SunIcon = () => (
  <svg
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="1.8"
    strokeLinecap="round"
    className="h-[18px] w-[18px]"
    aria-hidden="true"
  >
    <circle cx="12" cy="12" r="4" />
    <path d="M12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M6.3 17.7l-1.4 1.4M19.1 4.9l-1.4 1.4" />
  </svg>
);

const MoonIcon = () => (
  <svg
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="1.8"
    strokeLinecap="round"
    strokeLinejoin="round"
    className="h-[18px] w-[18px]"
    aria-hidden="true"
  >
    <path d="M20.5 14.2A8 8 0 0 1 9.8 3.5 8.5 8.5 0 1 0 20.5 14.2Z" />
  </svg>
);

const MenuIcon = ({ open }) => (
  <svg
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="1.8"
    strokeLinecap="round"
    className="h-5 w-5"
    aria-hidden="true"
  >
    {open ? (
      <path d="m6 6 12 12M18 6 6 18" />
    ) : (
      <path d="M4 7h16M4 12h16M4 17h16" />
    )}
  </svg>
);

const Logo = ({ onClick }) => (
  <NavLink
    to="/"
    onClick={onClick}
    className="group flex min-w-0 items-center gap-2.5 rounded-xl focus-visible:outline-none"
    aria-label="OpenAMR map dashboard"
  >
    <span className="relative flex h-9 w-9 shrink-0 items-center justify-center overflow-hidden rounded-xl bg-bgSurface text-white ring-1 ring-white/10">
      <span className="absolute inset-0 bg-gradient-to-br from-violet-500 via-purple-500 to-pink-400 opacity-90 transition-opacity group-hover:opacity-100" />
      <svg
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="1.8"
        className="relative h-5 w-5"
        aria-hidden="true"
      >
        <rect x="5" y="7" width="14" height="11" rx="4" />
        <path d="M9 7V5m6 2V5M9 13h.01M15 13h.01M9 17h6" strokeLinecap="round" />
      </svg>
    </span>
    <span className="min-w-0">
      <span className="block truncate text-[15px] font-bold tracking-[-0.03em] text-textWhiteHover">
        OpenAMR
      </span>
      <span className="hidden font-[RobotoMono] text-[9px] uppercase tracking-[0.16em] text-themeTextGray sm:block">
        Robot workspace
      </span>
    </span>
  </NavLink>
);

const Header = ({ showLogs, onToggleLogs }) => {
  const status = useRosStatus();
  const { theme, toggleTheme } = useContext(ThemeContext);
  const { config } = useRuntimeConfig();
  const { label, pulse } = statusConfig[status] || statusConfig.disconnected;
  const [menuOpen, setMenuOpen] = useState(false);
  const resolvedAddress = `${resolveRosbridgeHost(config)}:${config.rosbridgePort}`;

  const closeMenu = () => setMenuOpen(false);

  return (
    <>
      {/* Desktop: fixed vertical sidebar */}
      <aside className="fixed inset-y-0 left-0 z-40 hidden w-56 flex-col border-r border-borderSubtle bg-bgRaised/95 px-3 pb-4 pt-5 backdrop-blur-xl md:flex">
        <div className="px-1 pb-5">
          <Logo />
        </div>

        <nav
          className="flex flex-1 flex-col gap-1 overflow-y-auto"
          aria-label="Primary navigation"
        >
          {NAV_LINKS.map(({ to, label: navLabel, icon }) => (
            <NavLink
              key={to}
              to={to}
              end={to === "/"}
              className={({ isActive }) =>
                `flex items-center gap-3 rounded-xl px-3 py-2.5 text-sm font-semibold transition-all ${
                  isActive
                    ? "bg-bgCard text-textWhiteHover shadow-md shadow-black/10 ring-1 ring-white/5"
                    : "text-themeTextGray hover:bg-bgCard/60 hover:text-textWhiteHover"
                }`
              }
            >
              <NavIcon name={icon} />
              <span>{navLabel}</span>
            </NavLink>
          ))}
        </nav>

        <div className="flex flex-col gap-2 border-t border-borderSubtle pt-3">
          <button
            type="button"
            onClick={onToggleLogs}
            aria-expanded={showLogs}
            className={`flex min-h-[42px] items-center gap-2 rounded-xl border px-3 font-[RobotoMono] text-[11px] font-semibold ${
              showLogs
                ? "border-themeBlue/40 bg-themeBlue/10 text-themeBlue"
                : "border-borderSubtle bg-bgSurface/70 text-themeTextGray hover:border-themeBlue/40 hover:text-textWhiteHover"
            }`}
          >
            <span className="h-1.5 w-1.5 rounded-full bg-current" aria-hidden="true" />
            Console
          </button>

          <div className="flex items-center justify-between gap-2">
            <StatusBadge status={status} label={label} pulse={pulse} />
            <IconButton
              label={theme === "dark" ? "Use light theme" : "Use dark theme"}
              onClick={toggleTheme}
            >
              {theme === "dark" ? <SunIcon /> : <MoonIcon />}
            </IconButton>
          </div>
          <NavLink
            to="/config"
            className="truncate text-center font-[RobotoMono] text-[10px] text-themeTextGray/70 hover:text-themeBlue"
            title={`ws://${resolvedAddress} — edit in Config`}
          >
            {resolvedAddress}
          </NavLink>
        </div>
      </aside>

      {/* Mobile/tablet: top bar with hamburger menu */}
      <header className="relative z-40 w-full px-3 pb-2 pt-3 sm:px-4 md:hidden">
        <div className="relative flex min-h-[64px] w-full items-center justify-between rounded-[18px] border border-borderSubtle bg-bgCard/90 px-3 shadow-2xl shadow-black/20 backdrop-blur-xl sm:px-4">
          <Logo onClick={closeMenu} />

          <div className="flex items-center gap-2">
            <IconButton
              label={theme === "dark" ? "Use light theme" : "Use dark theme"}
              onClick={toggleTheme}
            >
              {theme === "dark" ? <SunIcon /> : <MoonIcon />}
            </IconButton>

            <div
              className="flex h-3 w-3 items-center justify-center"
              title={label}
              aria-label={label}
            >
              <span
                className={`h-2.5 w-2.5 rounded-full ${
                  status === "connected"
                    ? "bg-statusGreen"
                    : status === "error"
                    ? "bg-statusYellow"
                    : "bg-statusRed"
                }`}
              />
            </div>

            <IconButton
              label={menuOpen ? "Close navigation" : "Open navigation"}
              aria-expanded={menuOpen}
              onClick={() => setMenuOpen((open) => !open)}
            >
              <MenuIcon open={menuOpen} />
            </IconButton>
          </div>

          {menuOpen ? (
            <div className="absolute left-0 right-0 top-[calc(100%+8px)] rounded-[18px] border border-borderSubtle bg-bgCard p-2 shadow-2xl shadow-black/40">
              <div className="mb-2 flex items-center justify-between border-b border-borderSubtle px-2 pb-2 pt-1">
                <div>
                  <span className="font-[RobotoMono] text-[10px] uppercase tracking-[0.14em] text-themeTextGray">
                    Navigation
                  </span>
                  <p className="font-[RobotoMono] text-[10px] text-themeTextGray/60">
                    {resolvedAddress}
                  </p>
                </div>
                <StatusBadge status={status} label={label} pulse={pulse} />
              </div>
              <nav className="grid gap-1" aria-label="Mobile navigation">
                {NAV_LINKS.map(({ to, label: navLabel, icon }) => (
                  <NavLink
                    key={to}
                    to={to}
                    end={to === "/"}
                    onClick={closeMenu}
                    className={({ isActive }) =>
                      `flex min-h-[46px] items-center gap-3 rounded-xl px-3 text-sm font-semibold ${
                        isActive
                          ? "bg-themeBlue/10 text-themeBlue"
                          : "text-textWhiteHover hover:bg-bgSurface"
                      }`
                    }
                  >
                    <NavIcon name={icon} />
                    {navLabel}
                  </NavLink>
                ))}
                <button
                  type="button"
                  onClick={() => {
                    onToggleLogs();
                    closeMenu();
                  }}
                  className="flex min-h-[46px] items-center gap-3 rounded-xl px-3 text-left text-sm font-semibold text-textWhiteHover hover:bg-bgSurface"
                >
                  <span
                    className="flex h-4 w-4 items-center justify-center font-[RobotoMono] text-xs"
                    aria-hidden="true"
                  >
                    ›_
                  </span>
                  {showLogs ? "Hide console" : "Open console"}
                </button>
              </nav>
            </div>
          ) : null}
        </div>
      </header>
    </>
  );
};

export default Header;
