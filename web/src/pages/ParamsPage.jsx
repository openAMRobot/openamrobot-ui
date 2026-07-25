import React, { useRef, useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRos, useRosStatus } from "../app/App";
import { DashboardCard, SectionHeader, StatusBadge } from "../shared/ui/Dashboard";
import {
  PARAM_ROWS_STORAGE_KEY,
  readParamValue,
  buildParamValue,
  loadParamRows,
} from "../shared/constants/navParams";

const inputClass =
  "w-full rounded-lg border border-borderSubtle bg-bgCard px-2 py-1 text-xs text-textWhiteHover outline-none focus:border-themeBlue";

/**
 * Live Nav2 parameter tuning. Reads and writes parameters on running nodes via
 * their standard rcl_interfaces get_parameters / set_parameters services, so
 * common tuning (speed caps, tolerances, inflation) doesn't need a terminal.
 * Changes are runtime-only — they revert when the node restarts.
 */
const ParamsPage = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const connected = rosStatus === "connected";

  const [rows, setRows] = useState(() =>
    loadParamRows().map((r, i) => ({ id: `${i}`, value: "", ...r })),
  );
  const servicesRef = useRef({});

  const persist = (next) => {
    setRows(next);
    localStorage.setItem(
      PARAM_ROWS_STORAGE_KEY,
      JSON.stringify(next.map(({ node, param, type }) => ({ node, param, type }))),
    );
  };

  const svc = (node, kind) => {
    const key = `${node}/${kind}`;
    if (!servicesRef.current[key]) {
      servicesRef.current[key] = new window.ROSLIB.Service({
        ros,
        name: `${node}/${kind}_parameters`,
        serviceType:
          kind === "get"
            ? "rcl_interfaces/srv/GetParameters"
            : "rcl_interfaces/srv/SetParameters",
      });
    }
    return servicesRef.current[key];
  };

  const patch = (id, fields) =>
    setRows((prev) => prev.map((r) => (r.id === id ? { ...r, ...fields } : r)));

  // Best-effort plain-English hint inferred from the parameter name. Returns
  // null (renders nothing) when the name doesn't hint at anything specific,
  // rather than guessing a unit that might be wrong.
  const paramHint = (param) => {
    if (!param) return null;
    const p = param.toLowerCase();
    if (p.includes("theta") || p.includes("angular") || p.includes("yaw")) {
      return "Top turning speed (rad/s)";
    }
    if (p.includes("accel")) {
      return "Acceleration limit (m/s²)";
    }
    if (p.includes("vel")) {
      return "Top driving speed (m/s)";
    }
    if (p.includes("tolerance")) {
      return "How close counts as close enough (m)";
    }
    if (p.includes("radius") || p.includes("distance")) {
      return "Distance (m)";
    }
    return null;
  };

  const readRow = (row) => {
    if (!ros || !window.ROSLIB) return;
    patch(row.id, { status: "reading" });
    svc(row.node, "get").callService(
      new window.ROSLIB.ServiceRequest({ names: [row.param] }),
      (res) => {
        const pv = res?.values?.[0];
        if (!pv || pv.type === 0) {
          patch(row.id, { status: "not-set" });
          return;
        }
        patch(row.id, { value: readParamValue(pv), status: "read" });
      },
      (err) => {
        patch(row.id, { status: "error" });
        toast.error(`Read failed: ${err}`);
      },
    );
  };

  const applyRow = (row) => {
    if (!ros || !window.ROSLIB) return;
    if (!window.confirm(`Set ${row.param} to ${row.value} on the robot right now?`)) return;
    patch(row.id, { status: "applying" });
    svc(row.node, "set").callService(
      new window.ROSLIB.ServiceRequest({
        parameters: [{ name: row.param, value: buildParamValue(row.type, row.value) }],
      }),
      (res) => {
        const ok = res?.results?.[0]?.successful;
        patch(row.id, { status: ok ? "applied" : "rejected" });
        if (ok) toast.success(`${row.param} set`);
        else toast.error(`Rejected: ${res?.results?.[0]?.reason || "unknown"}`);
      },
      (err) => {
        patch(row.id, { status: "error" });
        toast.error(`Set failed: ${err}`);
      },
    );
  };

  const readAll = () => rows.forEach(readRow);

  const addRow = () =>
    persist([
      ...rows,
      { id: `${Date.now()}`, node: "/controller_server", param: "", type: "double", value: "" },
    ]);

  const removeRow = (id) => persist(rows.filter((r) => r.id !== id));

  const statusBadge = (
    <div className="flex items-center gap-3">
      <StatusBadge
        status={connected ? "connected" : "disconnected"}
        pulse={connected}
        label={connected ? "live" : "offline"}
      />
      <button
        onClick={readAll}
        disabled={!connected}
        className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue disabled:opacity-40"
      >
        Read all
      </button>
    </div>
  );

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <ToastContainer position="bottom-right" theme="dark" />
      <SectionHeader
        eyebrow="Tuning"
        title="Parameters"
        description="Nav2 is the robot's navigation software — these settings control things like how fast it drives and how close it gets to obstacles before adjusting course. Read and set Nav2 parameters on running nodes. Runtime-only — values revert when the related software restarts."
        action={statusBadge}
      />

      <DashboardCard className="p-3 font-[RobotoMono]">
        <div className="hidden grid-cols-[1.4fr_1.8fr_0.7fr_1fr_auto] gap-2 px-1 pb-2 text-[10px] uppercase tracking-wider text-themeTextGray sm:grid">
          <span>Node</span>
          <span>Parameter</span>
          <span>Type</span>
          <span>Value</span>
          <span />
        </div>

        <div className="space-y-2">
          {rows.map((row) => (
            <div
              key={row.id}
              className="grid grid-cols-2 items-center gap-2 rounded-lg border border-borderSubtle bg-bgSurface px-2 py-2 sm:grid-cols-[1.4fr_1.8fr_0.7fr_1fr_auto]"
            >
              <input
                className={inputClass}
                value={row.node}
                onChange={(e) => patch(row.id, { node: e.target.value })}
                placeholder="/node"
              />
              <div className="flex flex-col gap-0.5">
                <input
                  className={inputClass}
                  value={row.param}
                  onChange={(e) => patch(row.id, { param: e.target.value })}
                  placeholder="param.name"
                />
                {paramHint(row.param) && (
                  <p className="text-xs text-themeTextGray">{paramHint(row.param)}</p>
                )}
              </div>
              <select
                className={inputClass}
                value={row.type}
                onChange={(e) => patch(row.id, { type: e.target.value })}
              >
                <option value="double">Decimal number</option>
                <option value="int">Whole number</option>
                <option value="bool">On/Off</option>
                <option value="string">Text</option>
              </select>
              <input
                className={inputClass}
                value={row.value}
                onChange={(e) => patch(row.id, { value: e.target.value })}
                placeholder="—"
              />
              <div className="col-span-2 flex items-center gap-1.5 sm:col-span-1">
                <button
                  onClick={() => readRow(row)}
                  disabled={!connected}
                  className="rounded-lg border border-borderSubtle px-2 py-1 text-xs text-themeTextGray hover:border-themeBlue hover:text-themeBlue disabled:opacity-40"
                >
                  Read
                </button>
                <button
                  onClick={() => applyRow(row)}
                  disabled={!connected}
                  className="rounded-lg border border-themeBlue px-2 py-1 text-xs text-themeBlue hover:bg-themeBlue hover:text-white disabled:opacity-40"
                >
                  Set
                </button>
                <button
                  onClick={() => removeRow(row.id)}
                  aria-label="Remove row"
                  className="px-1 text-themeTextGray hover:text-statusRed"
                >
                  ×
                </button>
                {row.status && (
                  <span
                    className={`text-[10px] ${
                      ["applied", "read"].includes(row.status)
                        ? "text-statusGreen"
                        : ["rejected", "error"].includes(row.status)
                          ? "text-statusRed"
                          : "text-themeTextGray"
                    }`}
                  >
                    {row.status}
                  </span>
                )}
              </div>
            </div>
          ))}
        </div>

        <button
          onClick={addRow}
          className="mt-3 rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
        >
          + Add parameter
        </button>
      </DashboardCard>
    </div>
  );
};

export default ParamsPage;
