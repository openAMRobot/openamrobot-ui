/** @type {import('tailwindcss').Config} */
// eslint-disable-next-line no-undef
module.exports = {
  content: ["./src/**/*.{js,jsx,ts,tsx}"],
  theme: {
    extend: {
      colors: {
        themeLightGray: "#F5F5F5",
        themeMediumGray: "#C3C3C3",
        themeTextGray: "#696969",

        themeBlue: "#22b7fc",
        themeMediumBlue: "#1c97d4",
        themeDarkBlue: "#197fb4",

        textWhiteHover: "#d1d5db",
        textWhiteActive: "#6b7280",
      },
    },
  },
  plugins: [],
};
