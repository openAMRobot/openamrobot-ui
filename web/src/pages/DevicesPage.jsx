import React, { useEffect, useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRos } from "../app/App";
import useDevices from "../shared/hooks/useDevices";
import useDeviceStatuses from "../shared/hooks/useDeviceStatuses";
import { fetchSerialPorts } from "../features/devices/devicesApi";
import {
  DashboardCard,
  EmptyState,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";
import Button from "../shared/ui/Button";

const CONNECTION_TYPES = [
  { value: "usb", label: "USB", placeholder: "/dev/ttyUSB0" },
  { value: "can", label: "CAN bus", placeholder: "can0" },
  { value: "network", label: "Network", placeholder: "192.168.1.50:11411" },
  {
    value: "raspberry-pi",
    label: "Raspberry Pi (attached)",
    placeholder: "hostname or IP address",
  },
  { value: "other", label: "Other", placeholder: "connection target" },
];

const connectionLabel = (value) =>
  CONNECTION_TYPES.find((type) => type.value === value)?.label || value;

const STATUS_DISPLAY = {
  online: { status: "connected", label: "Online" },
  offline: { status: "disconnected", label: "Offline" },
  unmonitored: { status: "unknown", label: "No status topic" },
};

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

const EMPTY_FORM = {
  name: "",
  connectionType: "usb",
  target: "",
  statusTopic: "",
  statusMessageType: "",
  notes: "",
};

const DevicesPage = () => {
  const ros = useRos();
  const { devices, addDevice, removeDevice } = useDevices();
  const statuses = useDeviceStatuses(ros, devices);

  const [form, setForm] = useState(EMPTY_FORM);
  const [serialPorts, setSerialPorts] = useState([]);
  const [serialPortsSupported, setSerialPortsSupported] = useState(true);
  const [serialPortsLoading, setSerialPortsLoading] = useState(true);

  useEffect(() => {
    let cancelled = false;
    fetchSerialPorts()
      .then((data) => {
        if (cancelled) return;
        setSerialPorts(data.ports || []);
        setSerialPortsSupported(data.supported !== false);
      })
      .catch(() => {
        if (!cancelled) setSerialPortsSupported(false);
      })
      .finally(() => {
        if (!cancelled) setSerialPortsLoading(false);
      });
    return () => {
      cancelled = true;
    };
  }, []);

  const setField = (key) => (e) => {
    const { value } = e.target;
    setForm((prev) => ({ ...prev, [key]: value }));
  };

  const useSerialPort = (port) => {
    setForm((prev) => ({
      ...prev,
      connectionType: "usb",
      target: port.device,
      name: prev.name || port.description || "",
    }));
  };

  const handleAddDevice = () => {
    const name = form.name.trim();
    const target = form.target.trim();
    if (!name || !target) {
      toast.warn("Name and connection target are both required");
      return;
    }
    addDevice({
      name,
      connectionType: form.connectionType,
      target,
      statusTopic: form.statusTopic.trim(),
      statusMessageType: form.statusMessageType.trim(),
      notes: form.notes.trim(),
    });
    setForm(EMPTY_FORM);
    toast.success(`Registered "${name}"`);
  };

  const activeType = CONNECTION_TYPES.find((t) => t.value === form.connectionType);

  return (
    <div className="sectionHeight space-y-5 py-4 sm:space-y-6 sm:py-6">
      <SectionHeader
        eyebrow="Hardware"
        title="Devices"
        description="Manually registered external hardware — USB, CAN, network, and Raspberry-Pi-attached devices — with live status wherever a ROS topic is available. There's no plug-and-play auto-detection here: register what's connected, and point it at the topic its driver publishes."
      />

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Detected serial ports
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          Real serial ports (e.g. USB-serial adapters, Arduino-style boards)
          currently exposed on the machine running this backend. This won&apos;t
          see CAN interfaces, network devices, or hardware attached to a
          different Raspberry Pi than the one hosting this UI.
        </p>

        <div className="mt-3">
          {serialPortsLoading ? (
            <p className="text-xs text-themeTextGray opacity-70">Checking…</p>
          ) : !serialPortsSupported ? (
            <p className="text-xs text-themeTextGray opacity-70">
              Serial port detection isn&apos;t available on this backend.
            </p>
          ) : serialPorts.length === 0 ? (
            <p className="text-xs text-themeTextGray opacity-70">
              No USB-serial devices currently detected on this host.
            </p>
          ) : (
            <div className="flex flex-wrap gap-1.5">
              {serialPorts.map((port) => (
                <button
                  key={port.device}
                  type="button"
                  onClick={() => useSerialPort(port)}
                  className="rounded-lg border border-borderSubtle bg-bgSurface px-2.5 py-1.5 text-left text-xs hover:border-themeBlue"
                  title="Use this port as the new device's connection target"
                >
                  <span className="text-textWhiteHover">{port.device}</span>
                  {port.description ? (
                    <span className="ml-1.5 text-themeTextGray">
                      {port.description}
                    </span>
                  ) : null}
                </button>
              ))}
            </div>
          )}
        </div>
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Register a device
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          Status is optional — leave it blank for a device you&apos;re just
          keeping a record of, or point it at the ROS topic its driver
          publishes to get a live online/offline badge.
        </p>

        <div className="mt-4 grid gap-4 sm:grid-cols-2">
          <Field label="Name">
            <input
              type="text"
              value={form.name}
              onChange={setField("name")}
              placeholder="e.g. Gripper controller"
              className={inputClass}
            />
          </Field>

          <Field label="Connection type">
            <select
              value={form.connectionType}
              onChange={setField("connectionType")}
              className={inputClass}
            >
              {CONNECTION_TYPES.map((type) => (
                <option key={type.value} value={type.value}>
                  {type.label}
                </option>
              ))}
            </select>
          </Field>

          <Field
            label="Connection target"
            hint="Serial path, CAN interface name, or host:port — whatever identifies it."
          >
            <input
              type="text"
              value={form.target}
              onChange={setField("target")}
              placeholder={activeType?.placeholder}
              className={inputClass}
            />
          </Field>

          <Field
            label="Status topic (optional)"
            hint="A topic this device's driver publishes to when alive."
          >
            <input
              type="text"
              value={form.statusTopic}
              onChange={setField("statusTopic")}
              placeholder="/gripper/status"
              className={inputClass}
            />
          </Field>

          <Field
            label="Status message type (optional)"
            hint="Defaults to std_msgs/String if left blank."
          >
            <input
              type="text"
              value={form.statusMessageType}
              onChange={setField("statusMessageType")}
              placeholder="std_msgs/String"
              className={inputClass}
            />
          </Field>

          <Field label="Notes (optional)">
            <input
              type="text"
              value={form.notes}
              onChange={setField("notes")}
              placeholder="Driver package, firmware version, anything worth remembering"
              className={inputClass}
            />
          </Field>
        </div>

        <div className="mt-4 sm:max-w-xs">
          <Button type="orange" onBtnClick={handleAddDevice}>
            Register device
          </Button>
        </div>
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          Registered devices
        </p>

        {devices.length === 0 ? (
          <EmptyState
            className="mt-3"
            title="No devices registered yet"
            description="Add one above to start tracking it here."
          />
        ) : (
          <div className="mt-3 space-y-2">
            {devices.map((device) => {
              const state = statuses[device.id] || "unmonitored";
              const display = STATUS_DISPLAY[state];
              return (
                <div
                  key={device.id}
                  className="flex flex-wrap items-center justify-between gap-3 rounded-lg border border-borderSubtle bg-bgSurface px-3 py-2.5"
                >
                  <div className="min-w-0">
                    <div className="flex items-center gap-2">
                      <span className="truncate text-sm font-semibold text-textWhiteHover">
                        {device.name}
                      </span>
                      <span className="shrink-0 rounded-full border border-borderSubtle px-2 py-0.5 text-[10px] uppercase tracking-wider text-themeTextGray">
                        {connectionLabel(device.connectionType)}
                      </span>
                    </div>
                    <p className="mt-0.5 truncate font-[RobotoMono] text-xs text-themeTextGray">
                      {device.target}
                      {device.statusTopic ? ` · ${device.statusTopic}` : ""}
                    </p>
                    {device.notes ? (
                      <p className="mt-0.5 truncate text-xs text-themeTextGray/70">
                        {device.notes}
                      </p>
                    ) : null}
                  </div>

                  <div className="flex shrink-0 items-center gap-3">
                    <StatusBadge
                      status={display.status}
                      label={display.label}
                      pulse={state === "online"}
                    />
                    <button
                      onClick={() => removeDevice(device.id)}
                      className="text-themeTextGray hover:text-statusRed"
                      aria-label={`Remove ${device.name}`}
                    >
                      ×
                    </button>
                  </div>
                </div>
              );
            })}
          </div>
        )}
      </DashboardCard>

      <ToastContainer theme="dark" position="bottom-right" />
    </div>
  );
};

export default DevicesPage;
