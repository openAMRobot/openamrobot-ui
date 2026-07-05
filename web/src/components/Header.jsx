import React, { useContext, useState } from "react";
import { NavLink } from "react-router-dom";
import { RosStatusContext, ThemeContext } from "../app/App";

const statusConfig = {
  connected:    { color: "bg-statusGreen", pulse: true,  label: "Connected"    },
  disconnected: { color: "bg-statusRed",   pulse: false, label: "Disconnected" },
  error:        { color: "bg-statusYellow",pulse: false, label: "Error"        },
};

const NAV_LINKS = [
  { to: "/",        label: "Map"     },
  { to: "/route",   label: "Route"   },
  { to: "/control", label: "Control" },
  { to: "/blocks",  label: "Blocks"  },
  { to: "/info",    label: "Info"    },
];

const SunIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="h-4 w-4">
    <circle cx="12" cy="12" r="4" />
    <path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M6.34 17.66l-1.41 1.41M19.07 4.93l-1.41 1.41" />
  </svg>
);

const MoonIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="h-4 w-4">
    <path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z" />
  </svg>
);

const Header = ({ showLogs, onToggleLogs }) => {
  const status = useContext(RosStatusContext);
  const { theme, toggleTheme } = useContext(ThemeContext);
  const { color, pulse, label } = statusConfig[status] || statusConfig.disconnected;
  const [menuOpen, setMenuOpen] = useState(false);

  const closeMenu = () => setMenuOpen(false);

  return (
    <header className="relative flex items-center justify-between border-b border-borderSubtle bg-bgCard/95 px-4 py-2.5 shadow-sm shadow-slate-200/80 dark:shadow-slate-950/50 backdrop-blur lg:px-10">
      {/* Logo */}
      <div className="flex items-center gap-3">
        <div className="flex h-8 w-8 items-center justify-center rounded-lg bg-themeBlue shadow-sm shadow-cyan-800/20">
          <svg viewBox="0 0 24 24" fill="currentColor" className="h-5 w-5 text-white">
            <path d="M12 2a5 5 0 1 1 0 10A5 5 0 0 1 12 2zm0 12c5.33 0 8 2.67 8 4v2H4v-2c0-1.33 2.67-4 8-4z" />
          </svg>
        </div>
        <span className="font-[RobotoMono] text-lg font-bold tracking-wider text-themeBlue">
          OpenAMR
        </span>
      </div>

      {/* Desktop nav */}
      <nav className="hidden gap-6 font-[RobotoMono] text-sm sm:flex lg:gap-10 lg:text-base">
        {NAV_LINKS.map(({ to, label: navLabel }) => (
          <NavLink
            key={to}
            to={to}
            className={({ isActive }) =>
              isActive
                ? "border-b-2 border-themeBlue pb-1 text-themeBlue"
                : "pb-1 text-textWhiteHover transition-colors hover:text-themeBlue"
            }
          >
            {navLabel}
          </NavLink>
        ))}
      </nav>

      {/* Right side */}
      <div className="flex items-center gap-3 font-[RobotoMono] text-sm">
        {/* Console button — desktop only */}
        <button
          onClick={onToggleLogs}
          className={`hidden items-center gap-1.5 rounded-lg border px-3 py-1.5 text-xs font-semibold transition-colors sm:flex ${
            showLogs
              ? "border-themeBlue bg-themeBlue/10 text-themeBlue"
              : "border-borderSubtle text-themeTextGray hover:border-themeBlue hover:text-themeBlue"
          }`}
        >
          {showLogs ? "Hide Console" : "Console Logs"}
        </button>

        {/* Theme toggle */}
        <button
          onClick={toggleTheme}
          aria-label="Toggle dark mode"
          className="flex h-8 w-8 items-center justify-center rounded-lg border border-borderSubtle text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
        >
          {theme === "dark" ? <SunIcon /> : <MoonIcon />}
        </button>

        {/* Status dot */}
        <div className="flex items-center gap-2">
          <div className="relative flex h-2.5 w-2.5">
            {pulse && (
              <span className={`absolute inline-flex h-full w-full animate-ping rounded-full ${color} opacity-60`} />
            )}
            <span className={`relative inline-flex h-2.5 w-2.5 rounded-full ${color}`} />
          </div>
          <span className="hidden text-themeTextGray sm:inline">{label}</span>
        </div>

        {/* Hamburger — mobile only */}
        <button
          onClick={() => setMenuOpen((o) => !o)}
          className="flex h-9 w-9 items-center justify-center rounded-lg border border-borderSubtle text-themeTextGray sm:hidden"
          aria-label="Toggle menu"
        >
          {menuOpen ? (
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className="h-5 w-5">
              <path strokeLinecap="round" strokeLinejoin="round" d="M6 18L18 6M6 6l12 12" />
            </svg>
          ) : (
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className="h-5 w-5">
              <path strokeLinecap="round" strokeLinejoin="round" d="M4 6h16M4 12h16M4 18h16" />
            </svg>
          )}
        </button>
      </div>

      {/* Mobile dropdown */}
      {menuOpen && (
        <div className="absolute left-0 right-0 top-full z-50 border-b border-borderSubtle bg-bgCard/95 px-4 py-3 shadow-lg backdrop-blur sm:hidden">
          <nav className="flex flex-col gap-1 font-[RobotoMono]">
            {NAV_LINKS.map(({ to, label: navLabel }) => (
              <NavLink
                key={to}
                to={to}
                onClick={closeMenu}
                className={({ isActive }) =>
                  `rounded-lg px-3 py-3 text-sm font-medium transition-colors ${
                    isActive
                      ? "bg-themeBlue/10 text-themeBlue"
                      : "text-textWhiteHover hover:bg-bgSurface hover:text-themeBlue"
                  }`
                }
              >
                {navLabel}
              </NavLink>
            ))}
            <div className="mt-1 border-t border-borderSubtle pt-1">
              <button
                onClick={() => { onToggleLogs(); closeMenu(); }}
                className={`w-full rounded-lg px-3 py-3 text-left text-sm font-medium transition-colors ${
                  showLogs
                    ? "bg-themeBlue/10 text-themeBlue"
                    : "text-themeTextGray hover:bg-bgSurface hover:text-themeBlue"
                }`}
              >
                {showLogs ? "Hide Console" : "Console Logs"}
              </button>
            </div>
          </nav>
        </div>
      )}
    </header>
  );
};

export default Header;
