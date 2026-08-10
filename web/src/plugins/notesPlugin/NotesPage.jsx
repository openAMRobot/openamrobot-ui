import React, { useEffect, useState } from "react";

import { DashboardCard, EmptyState, SectionHeader } from "../../shared/ui/Dashboard";

const STORAGE_KEY = "openamrPluginNotes";

const load = () => {
  try {
    const stored = JSON.parse(localStorage.getItem(STORAGE_KEY) || "[]");
    return Array.isArray(stored) ? stored : [];
  } catch {
    return [];
  }
};

const persist = (notes) => localStorage.setItem(STORAGE_KEY, JSON.stringify(notes));

const inputClass =
  "w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

/**
 * Example plugin page — a real, working localStorage-backed scratchpad, not
 * a stub. Exists to prove web/src/shared/plugins/registerPlugin.js actually
 * works end to end: this file is never imported by pages/registry.js,
 * Header.jsx, or pages/index.jsx directly — see ./index.js's manifest and
 * web/src/index.js, where installing this plugin is the only line of core
 * app code touched to make "/notes" show up in nav, routing, and the help
 * widget's default content.
 */
const NotesPage = () => {
  const [notes, setNotes] = useState(load);
  const [selectedId, setSelectedId] = useState(null);

  useEffect(() => persist(notes), [notes]);

  const selected = notes.find((n) => n.id === selectedId) || null;

  const createNote = () => {
    const note = { id: `${Date.now()}`, title: "Untitled note", body: "", updatedAt: Date.now() };
    setNotes((prev) => [note, ...prev]);
    setSelectedId(note.id);
  };

  const updateSelected = (fields) => {
    if (!selected) return;
    setNotes((prev) =>
      prev.map((n) => (n.id === selected.id ? { ...n, ...fields, updatedAt: Date.now() } : n)),
    );
  };

  const deleteNote = (id) => {
    setNotes((prev) => prev.filter((n) => n.id !== id));
    if (selectedId === id) setSelectedId(null);
  };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <SectionHeader
        eyebrow="Example plugin"
        title="Notes"
        description="A small scratchpad, saved locally in this browser. Demonstrates the plugin registry — this whole page was added without editing Header.jsx or pages/index.jsx."
      />

      <div className="grid gap-4 lg:grid-cols-[280px_1fr]">
        <DashboardCard className="p-0">
          <div className="flex items-center justify-between border-b border-borderSubtle/30 px-3 py-2">
            <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
              Notes
            </p>
            <button
              onClick={createNote}
              className="rounded-lg border border-themeBlue px-2 py-1 text-xs text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
            >
              + New
            </button>
          </div>
          {notes.length === 0 ? (
            <EmptyState title="No notes yet" description="Create one to get started." />
          ) : (
            <ul className="divide-y divide-borderSubtle/30">
              {notes.map((n) => (
                <li key={n.id}>
                  <button
                    onClick={() => setSelectedId(n.id)}
                    className={`flex w-full items-center justify-between gap-2 px-3 py-2.5 text-left text-sm ${
                      selectedId === n.id
                        ? "bg-themeBlue/10 text-themeBlue"
                        : "text-textWhiteHover hover:bg-bgSurface"
                    }`}
                  >
                    <span className="truncate">{n.title || "Untitled note"}</span>
                    <span
                      onClick={(e) => {
                        e.stopPropagation();
                        deleteNote(n.id);
                      }}
                      role="button"
                      tabIndex={-1}
                      aria-label={`Delete ${n.title}`}
                      className="shrink-0 text-themeTextGray hover:text-statusRed"
                    >
                      ×
                    </span>
                  </button>
                </li>
              ))}
            </ul>
          )}
        </DashboardCard>

        <DashboardCard className="p-4">
          {selected ? (
            <div className="flex h-full flex-col gap-3">
              <input
                className={`${inputClass} text-base font-semibold`}
                value={selected.title}
                onChange={(e) => updateSelected({ title: e.target.value })}
                placeholder="Title"
              />
              <textarea
                className={`${inputClass} min-h-[240px] flex-1 resize-none font-[RobotoMono]`}
                value={selected.body}
                onChange={(e) => updateSelected({ body: e.target.value })}
                placeholder="Write something…"
              />
              <p className="text-[11px] text-themeTextGray/70">
                Saved locally · last edited {new Date(selected.updatedAt).toLocaleString()}
              </p>
            </div>
          ) : (
            <EmptyState
              title="No note selected"
              description="Pick a note on the left, or create a new one."
            />
          )}
        </DashboardCard>
      </div>
    </div>
  );
};

export default NotesPage;
