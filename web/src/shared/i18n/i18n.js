import { useEffect, useState } from "react";

// Lightweight i18n. Keys ARE the English strings, so any untranslated string
// falls back to readable English -- this lets translation be adopted
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
  "Reporting Mode": "Berichtsmodus",
  // Status bar / common
  Connected: "Verbunden",
  Offline: "Offline",
  Error: "Fehler",
  Batt: "Akku",
  "Stop (software)": "Stopp (Software)",
  Language: "Sprache",
  // Reporting Mode: scope header
  "Base and navigation": "Basis und Navigation",
  "Navigation status covers navigation only. It is not whole-robot readiness or E-stop state.":
    "Navigationsstatus gilt nur für die Navigation. Er zeigt nicht die Gesamtbereitschaft des Roboters oder den Not-Halt-Zustand.",
  // Reporting Mode: Demo Mode label (the page is outside AppLayout's banner)
  "Demo mode": "Demo-Modus",
  "Every value on this page is simulated. No robot is connected.":
    "Alle Werte auf dieser Seite sind simuliert. Kein Roboter ist verbunden.",
  READY: "BEREIT",
  EXECUTING: "IN AUSFÜHRUNG",
  WAITING: "WARTEND",
  BLOCKED: "BLOCKIERT",
  RECOVERING: "WIEDERHERSTELLUNG",
  NEEDS_ASSISTANCE: "HILFE ERFORDERLICH",
  SAFETY_STOPPED: "SICHERHEITSSTOPP",
  FAULTED: "GESTÖRT",
  "STOPPED (SOFTWARE)": "STOPP (SOFTWARE)",
  UNKNOWN: "UNBEKANNT",
  DISCONNECTED: "GETRENNT",
  Stale: "Veraltet",
  "Stale data": "Veraltete Daten",
  "Connection is up; source data is old": "Verbindung besteht; Quelldaten sind veraltet",
  "Last known state": "Letzter bekannter Zustand",
  "Disconnected since": "Getrennt seit",
  "Last known, not live": "Zuletzt bekannt, nicht live",
  // Reporting Mode: exit control
  "Hold to exit Reporting Mode": "Gedrückt halten zum Beenden",
  "Hold to exit": "Zum Beenden halten",
  "Keep holding to exit": "Weiter halten zum Beenden",
  // Reporting Mode: section labels
  "Current task": "Aktuelle Aufgabe",
  None: "Keine",
  Progress: "Fortschritt",
  "Last result": "Letztes Ergebnis",
  Warnings: "Warnungen",
  // Reporting Mode: progress
  "m remaining": "m verbleibend",
  "about": "ca.",
  "s": "s",
  "recoveries": "Wiederherstellungen",
  // Reporting Mode: result
  "Result unknown (invalid data)": "Ergebnis unbekannt (ungültige Daten)",
  "No result yet": "Noch kein Ergebnis",
  // Reporting Mode: footer
  "Screen wake lock": "Bildschirm-Wachhalten",
  "Speech off": "Sprache aus",
  "Speech on": "Sprache ein",
  "Speech not supported": "Sprache nicht unterstützt",
  "Last announcement": "Letzte Ansage",
  "No announcement yet": "Noch keine Ansage",
  // Reporting Mode: screen wake lock status values
  "not supported": "nicht unterstützt",
  on: "ein",
  refused: "abgelehnt",
  released: "freigegeben",
  "more not shown": "weitere nicht angezeigt",
  // Reporting Mode: source row labels
  Connection: "Verbindung",
  "Software stop lock": "Software-Stopp-Sperre",
  Navigation: "Navigation",
  Localization: "Lokalisierung",
  Battery: "Akku",
  Diagnostics: "Diagnose",
  Arms: "Arme",
  Lift: "Lift",
  "Physical E-stop": "Physischer Not-Halt",
  "Not reported": "Nicht gemeldet",
  // Reporting Mode: source row values (set in reportingAdapter.js)
  Disconnected: "Getrennt",
  Engaged: "Aktiv",
  Released: "Gelöst",
  Unknown: "Unbekannt",
  Ready: "Bereit",
  "Not ready": "Nicht bereit",
  Lost: "Verloren",
  Degraded: "Eingeschränkt",
  OK: "OK",
  Warning: "Warnung",
  // Reporting Mode: source freshness labels
  "(no data)": "(keine Daten)",
  "(invalid)": "(ungültig)",
  // Reporting Mode: Next: prefix on next-action line
  "Next:": "Weiter:",
  // Reporting Mode: "Completed:" prefix
  "Completed:": "Abgeschlossen:",
  // Reporting Mode: "Step N of M" is assembled in reportingAdapter, not translated here
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
