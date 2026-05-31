/** @type {import('tailwindcss').Config} */
// eslint-disable-next-line no-undef
module.exports = {
  content: ["./src/**/*.{js,jsx,ts,tsx}"],
  theme: {
    extend: {
      colors: {
        // Light control-room theme
        bgBase: "#f4f8fb",
        bgCard: "#ffffff",
        bgSurface: "#eaf1f7",
        borderSubtle: "#c9d8e6",

        // Existing semantic names mapped to light-friendly variants
        themeLightGray: "#eaf1f7",
        themeMediumGray: "#d7e3ee",
        themeTextGray: "#64748b",

        themeBlue: "#087ea4",
        themeMediumBlue: "#0e9fbc",
        themeDarkBlue: "#075985",

        textWhiteHover: "#172033",
        textWhiteActive: "#334155",

        // Status colors
        statusGreen: "#16a34a",
        statusRed: "#dc2626",
        statusYellow: "#d97706",
        statusBlue: "#2563eb",
      },
    },
  },
  plugins: [],
};
