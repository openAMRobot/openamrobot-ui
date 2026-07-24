import React, { useEffect, useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRuntimeConfig } from "../app/App";
import {
  DEFAULT_RUNTIME_CONFIG,
  resolveRosbridgeHost,
} from "../shared/constants/runtimeConfig";
import { SectionHeader, DashboardCard } from "../shared/ui/Dashboard";
import Button from "../shared/ui/Button";
import Switcher from "../shared/ui/Switcher";
import useRobotProfiles from "../shared/hooks/useRobotProfiles";

const Field = ({ label, hint, children }) => (
  <label className="block">
    <span className="text-xs font-semibold uppercase tracking-wider text-themeTextGray">
      {label}
    </span>
    <div className="mt-1.5">{children}</div>
    {hint ? <p className="mt-1 text-[11px] text-themeTextGray/70">{hint}</p> : null}
  </label>
);

const inputClass =
  "w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

const ConfigPage = () => {
  const { config, updateConfig } = useRuntimeConfig();
  const [form, setForm] = useState(config);
  const [notifPermission, setNotifPermission] = useState(
    typeof Notification !== "undefined" ? Notification.permission : "unsupported",
  );
  const { profiles, addProfile, removeProfile } = useRobotProfiles();
  const [profileName, setProfileName] = useState("");

  // Keep the form in sync if settings change elsewhere (e.g. reset from
  // another tab writing to the same localStorage key).
  useEffect(() => setForm(config), [config]);

  const requestNotificationPermission = async () => {
    if (typeof Notification === "undefined") return;
    const result = await Notification.requestPermission();
    setNotifPermission(result);
    if (result === "granted") toast.success("Browser notifications enabled");
    else toast.warn("Notification permission was not granted");
  };

  const resolvedHost = resolveRosbridgeHost(form);

  const setField = (key) => (e) => {
    const { value } = e.target;
    setForm((prev) => ({ ...prev, [key]: value }));
  };

  const handleSave = () => {
    updateConfig({
      rosbridgeHost: form.rosbridgeHost.trim(),
      rosbridgePort: String(form.rosbridgePort || DEFAULT_RUNTIME_CONFIG.rosbridgePort),
      cameraPort: String(form.cameraPort || DEFAULT_RUNTIME_CONFIG.cameraPort),
      maxLinearSpeed: parseFloat(form.maxLinearSpeed) || DEFAULT_RUNTIME_CONFIG.maxLinearSpeed,
      maxAngularSpeed: parseFloat(form.maxAngularSpeed) || DEFAULT_RUNTIME_CONFIG.maxAngularSpeed,
      notificationsEnabled: Boolean(form.notificationsEnabled),
      lowBatteryThreshold:
        parseFloat(form.lowBatteryThreshold) || DEFAULT_RUNTIME_CONFIG.lowBatteryThreshold,
    });
    toast.success("Settings saved");
  };

  const handleReset = () => {
    setForm(DEFAULT_RUNTIME_CONFIG);
    updateConfig(DEFAULT_RUNTIME_CONFIG);
    toast.info("Settings reset to defaults");
  };

  const handleSaveProfile = () => {
    const trimmed = profileName.trim();
    if (!trimmed) {
      toast.warn("Enter a name for this robot");
      return;
    }
    if (!form.rosbridgeHost.trim()) {
      toast.warn(
        "Set a specific host above first — “auto” always resolves to whichever page you're on, so it can't be saved as a switchable profile.",
      );
      return;
    }
    addProfile(trimmed, {
      rosbridgeHost: form.rosbridgeHost.trim(),
      rosbridgePort: String(form.rosbridgePort || DEFAULT_RUNTIME_CONFIG.rosbridgePort),
      cameraPort: String(form.cameraPort || DEFAULT_RUNTIME_CONFIG.cameraPort),
    });
    setProfileName("");
    toast.success(`Saved "${trimmed}"`);
  };

  const handleConnectProfile = (profile) => {
    const next = {
      rosbridgeHost: profile.rosbridgeHost,
      rosbridgePort: profile.rosbridgePort,
      cameraPort: profile.cameraPort,
    };
    setForm((prev) => ({ ...prev, ...next }));
    updateConfig(next);
    toast.success(`Connecting to "${profile.name}"`);
  };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:space-y-6 sm:py-6">
      <SectionHeader
        eyebrow="Settings"
        title="Configuration"
        description="Connection and safety defaults for this browser, saved locally — nothing here is shared with other operators or persisted on the robot."
      />

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Demo mode
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          Explore the whole interface with simulated telemetry — no robot or
          rosbridge required. Every page shows a permanent "Demo mode" badge
          while this is on, and nothing simulated is ever labeled as live.
          Turning it off hands control straight back to the connection
          settings below.
        </p>
        <div className="mt-4 flex items-center gap-3">
          <Switcher
            switcherValue={Boolean(config.demoMode)}
            onChange={(next) => updateConfig({ demoMode: next })}
          />
          <span className="text-sm text-textWhiteHover">
            {config.demoMode ? "Demo mode is on" : "Demo mode is off"}
          </span>
        </div>
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Connection
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          Currently resolving to{" "}
          <code className="rounded bg-bgSurface px-1.5 py-0.5 text-textWhiteHover">
            ws://{resolvedHost}:{form.rosbridgePort}
          </code>
          . Changing these fields reconnects the ROS bridge.
        </p>

        <div className="mt-4 grid gap-4 sm:grid-cols-2">
          <Field
            label="Rosbridge host override"
            hint='Leave blank to auto-use this page’s own host (the normal case once deployed on the robot).'
          >
            <input
              type="text"
              value={form.rosbridgeHost}
              onChange={setField("rosbridgeHost")}
              placeholder={`Auto — ${resolveRosbridgeHost({ rosbridgeHost: "" })}`}
              className={inputClass}
            />
          </Field>

          <Field label="Rosbridge port">
            <input
              type="number"
              min="1"
              max="65535"
              value={form.rosbridgePort}
              onChange={setField("rosbridgePort")}
              className={inputClass}
            />
          </Field>

          <Field label="Camera stream port" hint="web_video_server port used by the Camera panel.">
            <input
              type="number"
              min="1"
              max="65535"
              value={form.cameraPort}
              onChange={setField("cameraPort")}
              className={inputClass}
            />
          </Field>
        </div>
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Saved robots
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          Name the connection above and save it, then switch robots with one
          click instead of retyping host/port.
        </p>

        <div className="mt-4 flex gap-2">
          <input
            type="text"
            value={profileName}
            onChange={(e) => setProfileName(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && handleSaveProfile()}
            placeholder="e.g. Warehouse Robot 3"
            className={inputClass}
          />
          <button
            onClick={handleSaveProfile}
            className="shrink-0 rounded-lg border border-themeBlue px-3 py-1.5 text-xs font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
          >
            Save as profile
          </button>
        </div>

        {profiles.length === 0 ? (
          <p className="mt-3 text-xs text-themeTextGray opacity-70">
            No saved robots yet.
          </p>
        ) : (
          <div className="mt-3 flex flex-wrap gap-1.5">
            {profiles.map((profile) => (
              <div
                key={profile.id}
                className="flex items-center gap-1.5 rounded-lg border border-borderSubtle bg-bgSurface px-2 py-1 text-xs"
              >
                <button
                  onClick={() => handleConnectProfile(profile)}
                  className="text-themeBlue hover:underline"
                  title={`${profile.rosbridgeHost}:${profile.rosbridgePort}`}
                >
                  ▸ {profile.name}
                </button>
                <button
                  onClick={() => removeProfile(profile.id)}
                  className="text-themeTextGray hover:text-statusRed"
                  aria-label={`Delete ${profile.name}`}
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        )}
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Manual-drive safety limits
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          Ceiling values for the joystick and the Control page&apos;s max-speed
          slider. Lowering these takes effect immediately for new drive
          commands.
        </p>

        <div className="mt-4 grid gap-4 sm:grid-cols-2">
          <Field label="Max linear speed" hint="m/s">
            <input
              type="number"
              min="0.05"
              max="2"
              step="0.05"
              value={form.maxLinearSpeed}
              onChange={setField("maxLinearSpeed")}
              className={inputClass}
            />
          </Field>

          <Field label="Max angular speed" hint="rad/s">
            <input
              type="number"
              min="0.1"
              max="6"
              step="0.1"
              value={form.maxAngularSpeed}
              onChange={setField("maxAngularSpeed")}
              className={inputClass}
            />
          </Field>
        </div>
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Notifications
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          Browser notifications for nav goal completion, docking, and low
          battery — useful when this tab isn&apos;t in focus. Requires the
          browser&apos;s notification permission.
        </p>

        <div className="mt-4 flex flex-wrap items-center gap-3">
          <Switcher
            switcherValue={Boolean(form.notificationsEnabled)}
            onChange={(next) =>
              setForm((prev) => ({ ...prev, notificationsEnabled: next }))
            }
          />
          <span className="text-sm text-textWhiteHover">
            {form.notificationsEnabled ? "Enabled" : "Disabled"}
          </span>

          <button
            type="button"
            onClick={requestNotificationPermission}
            className="ml-auto rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeBlue hover:border-themeBlue"
          >
            {notifPermission === "granted"
              ? "Permission granted"
              : notifPermission === "denied"
              ? "Permission blocked — check browser settings"
              : "Request permission"}
          </button>
        </div>

        <div className="mt-4 max-w-xs">
          <Field label="Low battery threshold" hint="Notify once battery drops to or below this level, %">
            <input
              type="number"
              min="0"
              max="100"
              step="1"
              value={form.lowBatteryThreshold}
              onChange={setField("lowBatteryThreshold")}
              className={inputClass}
            />
          </Field>
        </div>
      </DashboardCard>

      <div className="grid grid-cols-2 gap-3 sm:max-w-sm">
        <Button type="orange" onBtnClick={handleSave}>
          Save settings
        </Button>
        <Button onBtnClick={handleReset}>Reset to defaults</Button>
      </div>

      <ToastContainer theme="dark" position="bottom-right" />
    </div>
  );
};

export default ConfigPage;
