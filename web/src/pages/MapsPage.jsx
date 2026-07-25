import React, { useEffect, useRef, useState } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRos, useRosStatus } from "../app/App";
import { AppConfig } from "../shared/constants";
import { DashboardCard, EmptyState, SectionHeader, StatusBadge } from "../shared/ui/Dashboard";

// Names travel over the wire with underscores for spaces; the UI shows spaces.
const toWire = (s) => String(s || "").trim().replace(/\s+/g, "_");
const toDisplay = (s) => String(s || "").replace(/_/g, " ");
const stripCsv = (s) => String(s || "").replace(".csv", "");

// structure: [ { group: [ { map: [route,...] }, ... ] }, ... ]
const parseStructure = (structure) =>
  (structure || []).map((groupObj) => {
    const group = Object.keys(groupObj)[0];
    const maps = (groupObj[group] || []).map((mapObj) => {
      const map = Object.keys(mapObj)[0];
      return { name: toDisplay(map), routes: (mapObj[map] || []).map((r) => toDisplay(stripCsv(r))) };
    });
    return { name: toDisplay(group), maps };
  });

const inputClass =
  "rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

/**
 * Map management — save the current map, switch between saved maps, rename and
 * delete them, and organise them into groups. Talks to the existing
 * folders_handler backend over /ui_operation (the same command protocol the
 * Routes page uses) and reads the catalog from /nav_data_resp.
 */
const MapsPage = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const connected = rosStatus === "connected";

  const [groups, setGroups] = useState([]);
  const [active, setActive] = useState({ group: "", map: "" });
  const [saveForm, setSaveForm] = useState({ group: "", name: "" });
  const [newGroup, setNewGroup] = useState("");
  const [renaming, setRenaming] = useState(null); // {group, map, value}

  const reqRef = useRef(null);
  const respRef = useRef(null);
  const opRef = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return undefined;

    reqRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_DATA_REQ_TOPIC,
      messageType: "std_msgs/Empty",
    });
    respRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_DATA_RESP_TOPIC,
      messageType: "std_msgs/String",
    });
    opRef.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    });

    respRef.current.subscribe((data) => {
      try {
        const obj = JSON.parse(data.data);
        setGroups(parseStructure(obj.structure));
        setActive({
          group: toDisplay(obj.active_files?.group),
          map: toDisplay(obj.active_files?.map),
        });
      } catch {
        // malformed nav_data — ignore this message
      }
    });

    reqRef.current.publish();
    return () => respRef.current?.unsubscribe();
  }, [ros]);

  const refresh = () => reqRef.current?.publish();

  const sendCmd = (path, data) => {
    if (!opRef.current) return;
    const wired = data
      ? Object.fromEntries(Object.entries(data).map(([k, v]) => [k, toWire(v)]))
      : null;
    const msg = wired ? `${path}/${JSON.stringify(wired)}` : path;
    opRef.current.publish(new window.ROSLIB.Message({ data: msg }));
    // Backend does file ops then re-publishes nav_data; nudge a refresh too.
    setTimeout(refresh, 900);
  };

  const switchMap = (group, map) => {
    window.NAV2D?.ClearMap?.();
    sendCmd("change_map", { group, map });
    toast.info(`Loading "${map}" — set the initial pose after it loads; old localization won't match.`);
  };

  const saveMap = () => {
    const group = saveForm.group.trim();
    const name = saveForm.name.trim();
    if (!group || !name) {
      toast.warn("Pick a group and a name for the map");
      return;
    }
    sendCmd("save_map", { group, map: name });
    toast.success(`Saving current map as "${name}"…`);
    setSaveForm({ group, name: "" });
  };

  const startMapping = () => {
    if (!window.confirm(
      "Start a new mapping session? This shuts down localization/navigation and launches mapping mode — the robot must be driven around to build the map. Save it here when done.",
    )) return;
    sendCmd("build_map");
    toast.info("Mapping started — drive the robot around the space, then save the map.");
  };

  const deleteMap = (group, map) => {
    if (!window.confirm(`Delete map "${map}" and its routes? This cannot be undone.`)) return;
    sendCmd("delete_map", { group, map });
    toast.success(`Deleted "${map}"`);
  };

  const commitRename = () => {
    if (!renaming) return;
    const next = renaming.value.trim();
    if (next && next !== renaming.map) {
      sendCmd("rename_map", { group: renaming.group, map_old: renaming.map, map_new: next });
      toast.success(`Renamed to "${next}"`);
    }
    setRenaming(null);
  };

  const createGroup = () => {
    const g = newGroup.trim();
    if (!g) return;
    sendCmd("create_group", { group: g });
    toast.success(`Created group "${g}"`);
    setNewGroup("");
  };

  const deleteGroup = (group) => {
    if (!window.confirm(`Delete group "${group}" and everything in it?`)) return;
    sendCmd("delete_group", { group });
    toast.success(`Deleted group "${group}"`);
  };

  const groupNames = groups.map((g) => g.name);

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <ToastContainer position="bottom-right" theme="dark" />
      <SectionHeader
        eyebrow="Environments"
        title="Maps"
        description="Save, switch, rename and organise the robot's maps. Switching a map reloads it on the robot immediately."
        action={
          <div className="flex items-center gap-3">
            <StatusBadge status={connected ? "connected" : "disconnected"} pulse={connected} label={connected ? "live" : "offline"} />
            <button onClick={refresh} disabled={!connected} className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray hover:border-themeBlue hover:text-themeBlue disabled:opacity-40">
              Refresh
            </button>
          </div>
        }
      />

      <div className="grid grid-cols-1 gap-3 lg:grid-cols-2">
        <DashboardCard className="p-4 font-[RobotoMono]">
          <p className="mb-1 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">Build a new map</p>
          <p className="mb-3 text-xs text-themeTextGray">
            Launches mapping mode and stops navigation. Drive the robot around the space, then save below.
          </p>
          <button onClick={startMapping} disabled={!connected} className="rounded-lg border border-statusYellow px-3 py-1.5 text-sm font-semibold text-statusYellow transition-colors hover:bg-statusYellow hover:text-black disabled:opacity-40">
            Start mapping
          </button>
        </DashboardCard>

        <DashboardCard className="p-4 font-[RobotoMono]">
          <p className="mb-3 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">Save current map</p>
          <div className="grid grid-cols-1 gap-2 sm:grid-cols-[1fr_1fr_auto]">
            <input list="map-groups" className={inputClass} value={saveForm.group} onChange={(e) => setSaveForm((p) => ({ ...p, group: e.target.value }))} placeholder="Group" />
            <datalist id="map-groups">
              {groupNames.map((g) => <option key={g} value={g} />)}
            </datalist>
            <input className={inputClass} value={saveForm.name} onChange={(e) => setSaveForm((p) => ({ ...p, name: e.target.value }))} placeholder="Map name" />
            <button onClick={saveMap} disabled={!connected} className="rounded-lg border border-themeBlue bg-themeBlue/10 px-4 py-2 text-sm font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:opacity-40">
              Save
            </button>
          </div>
        </DashboardCard>
      </div>

      <DashboardCard className="p-4">
        <div className="mb-3 flex items-center justify-between">
          <p className="text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">Saved maps</p>
          <div className="flex gap-2">
            <input className={`${inputClass} py-1`} value={newGroup} onChange={(e) => setNewGroup(e.target.value)} placeholder="New group" />
            <button onClick={createGroup} disabled={!connected} className="rounded-lg border border-borderSubtle px-3 py-1 text-xs text-themeTextGray hover:border-themeBlue hover:text-themeBlue disabled:opacity-40">
              Add group
            </button>
          </div>
        </div>

        {groups.length === 0 ? (
          <EmptyState title="No maps yet" description="Build and save a map above to see it here." />
        ) : (
          <div className="space-y-4">
            {groups.map((group) => (
              <div key={group.name}>
                <div className="mb-1.5 flex items-center justify-between">
                  <p className="font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">{group.name}</p>
                  <button onClick={() => deleteGroup(group.name)} className="text-[10px] text-themeTextGray hover:text-statusRed">delete group</button>
                </div>
                {group.maps.length === 0 ? (
                  <p className="px-2 text-xs text-themeTextGray/60">No maps in this group.</p>
                ) : (
                  <div className="space-y-1.5">
                    {group.maps.map((m) => {
                      const isActive = active.group === group.name && active.map === m.name;
                      const isRenaming = renaming?.group === group.name && renaming?.map === m.name;
                      return (
                        <div key={m.name} className={`flex flex-wrap items-center gap-3 rounded-lg border px-3 py-2 ${isActive ? "border-themeBlue/50 bg-themeBlue/5" : "border-borderSubtle bg-bgSurface"}`}>
                          {isRenaming ? (
                            <input
                              autoFocus
                              className={`${inputClass} flex-1 py-1`}
                              value={renaming.value}
                              onChange={(e) => setRenaming((p) => ({ ...p, value: e.target.value }))}
                              onKeyDown={(e) => { if (e.key === "Enter") commitRename(); if (e.key === "Escape") setRenaming(null); }}
                            />
                          ) : (
                            <span className="min-w-0 flex-1 truncate text-sm font-semibold text-textWhiteHover">
                              {m.name}
                              {isActive && <span className="ml-2 rounded border border-themeBlue/40 px-1.5 py-0.5 text-[10px] text-themeBlue">active</span>}
                              <span className="ml-2 font-[RobotoMono] text-[11px] text-themeTextGray">{m.routes.length} route(s)</span>
                            </span>
                          )}
                          <div className="flex shrink-0 items-center gap-1.5 font-[RobotoMono] text-xs">
                            {isRenaming ? (
                              <>
                                <button onClick={commitRename} className="rounded-lg border border-themeBlue px-2 py-1 text-themeBlue hover:bg-themeBlue hover:text-white">Save</button>
                                <button onClick={() => setRenaming(null)} className="rounded-lg border border-borderSubtle px-2 py-1 text-themeTextGray">Cancel</button>
                              </>
                            ) : (
                              <>
                                <button onClick={() => switchMap(group.name, m.name)} disabled={!connected || isActive} className="rounded-lg border border-themeBlue px-2 py-1 text-themeBlue hover:bg-themeBlue hover:text-white disabled:opacity-40">
                                  {isActive ? "Loaded" : "Switch"}
                                </button>
                                <button onClick={() => setRenaming({ group: group.name, map: m.name, value: m.name })} className="rounded-lg border border-borderSubtle px-2 py-1 text-themeTextGray hover:border-themeBlue hover:text-themeBlue">Rename</button>
                                <button onClick={() => deleteMap(group.name, m.name)} aria-label={`Delete ${m.name}`} className="px-1 text-themeTextGray hover:text-statusRed">×</button>
                              </>
                            )}
                          </div>
                        </div>
                      );
                    })}
                  </div>
                )}
              </div>
            ))}
          </div>
        )}
      </DashboardCard>
    </div>
  );
};

export default MapsPage;
