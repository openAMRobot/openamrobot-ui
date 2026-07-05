/** @type {import('tailwindcss').Config} */
// eslint-disable-next-line no-undef
module.exports = {
  content: ["./src/**/*.{js,jsx,ts,tsx}"],
  darkMode: "class",
  theme: {
    extend: {
      colors: {
        // All colors are driven by CSS custom properties so the entire
        // palette flips automatically when the .dark class is toggled on <html>.
        // The `<alpha-value>` placeholder lets Tailwind's opacity modifiers
        // (e.g. bg-themeBlue/10) work correctly with CSS variables.
        bgBase:          "rgb(var(--color-bg-base)          / <alpha-value>)",
        bgCard:          "rgb(var(--color-bg-card)          / <alpha-value>)",
        bgSurface:       "rgb(var(--color-bg-surface)       / <alpha-value>)",
        borderSubtle:    "rgb(var(--color-border-subtle)    / <alpha-value>)",

        themeLightGray:  "rgb(var(--color-theme-light-gray)  / <alpha-value>)",
        themeMediumGray: "rgb(var(--color-theme-medium-gray) / <alpha-value>)",
        themeTextGray:   "rgb(var(--color-theme-text-gray)   / <alpha-value>)",

        themeBlue:       "rgb(var(--color-theme-blue)        / <alpha-value>)",
        themeMediumBlue: "rgb(var(--color-theme-medium-blue) / <alpha-value>)",
        themeDarkBlue:   "rgb(var(--color-theme-dark-blue)   / <alpha-value>)",

        textWhiteHover:  "rgb(var(--color-text-white-hover)  / <alpha-value>)",
        textWhiteActive: "rgb(var(--color-text-white-active) / <alpha-value>)",

        statusGreen:     "rgb(var(--color-status-green)  / <alpha-value>)",
        statusRed:       "rgb(var(--color-status-red)    / <alpha-value>)",
        statusYellow:    "rgb(var(--color-status-yellow) / <alpha-value>)",
        statusBlue:      "rgb(var(--color-status-blue)   / <alpha-value>)",
      },
    },
  },
  plugins: [],
};
