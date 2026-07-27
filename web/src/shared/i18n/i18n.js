import { useEffect, useState } from "react";

// Lightweight i18n. Keys ARE the English strings, so any untranslated string
// falls back to readable English — this lets translation be adopted
// incrementally (wrap a string in t() and add a German entry when ready)
// without a big-bang migration. German (de) is seeded for the app shell
// (nav, status bar, common actions); page bodies can be translated over time.

const STORAGE_KEY = "openamrLang";

const DE = {
  // Navigation
  Map: "Karte",
  Routes: "Routen",
  Programs: "Programme",
  Status: "Status",
  Robot: "Roboter",
  Devices: "Geräte",
  Health: "Zustand",
  Metrics: "Kennzahlen",
  Recordings: "Aufnahmen",
  Events: "Ereignisse",
  Console: "Konsole",
  Parameters: "Parameter",
  Fleet: "Flotte",
  Maps: "Karten",
  Scheduler: "Zeitplan",
  Config: "Einstellungen",
  // Status bar / common
  Connected: "Verbunden",
  Offline: "Offline",
  Error: "Fehler",
  Batt: "Akku",
  "E-STOP": "NOT-AUS",
  Language: "Sprache",
};

const DICTS = { en: {}, de: DE };

let lang = load();
const listeners = new Set();

function load() {
  try {
    const stored = localStorage.getItem(STORAGE_KEY);
    return stored === "de" || stored === "en" ? stored : "en";
  } catch {
    return "en";
  }
}

export function getLang() {
  return lang;
}

export function setLang(next) {
  if (next !== "en" && next !== "de") return;
  lang = next;
  try {
    localStorage.setItem(STORAGE_KEY, next);
  } catch {
    // ignore
  }
  listeners.forEach((fn) => fn(lang));
}

export function translate(key) {
  return DICTS[lang]?.[key] ?? key;
}

// React binding: re-renders subscribers when the language changes.
export function useT() {
  const [current, setCurrent] = useState(lang);
  useEffect(() => {
    const fn = (l) => setCurrent(l);
    listeners.add(fn);
    return () => listeners.delete(fn);
  }, []);
  return { t: translate, lang: current, setLang };
}
