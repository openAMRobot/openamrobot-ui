import { useEffect, useState } from "react";

const STORAGE_KEY = "openamrSpeedPresets";

// Named max-linear-speed profiles ("Slow near people", "Fast in aisles").
// Seeded with sensible defaults the first time; fully editable after.
const DEFAULT_PRESETS = [
  { id: "slow", name: "Slow", linear: 0.1 },
  { id: "normal", name: "Normal", linear: 0.2 },
  { id: "fast", name: "Fast", linear: 0.4 },
];

const load = () => {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY) || "null");
    return Array.isArray(stored) ? stored : DEFAULT_PRESETS;
  } catch {
    return DEFAULT_PRESETS;
  }
};

export default function useSpeedPresets() {
  const [presets, setPresets] = useState(load);

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(presets));
  }, [presets]);

  const addPreset = (name, linear) =>
    setPresets((prev) => [
      ...prev,
      { id: `${Date.now()}`, name, linear: Number(linear) },
    ]);

  const removePreset = (id) =>
    setPresets((prev) => prev.filter((p) => p.id !== id));

  return { presets, addPreset, removePreset };
}
