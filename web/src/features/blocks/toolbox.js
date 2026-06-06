export const openAmrToolbox = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "Program",
      colour: "#0e9fbc",
      contents: [
        { kind: "block", type: "openamr_start" },
        { kind: "block", type: "openamr_repeat" },
        { kind: "block", type: "openamr_log" },
      ],
    },
    {
      kind: "category",
      name: "Navigation",
      colour: "#087ea4",
      contents: [
        { kind: "block", type: "openamr_navigate" },
        { kind: "block", type: "openamr_navigate_named" },
        { kind: "block", type: "openamr_wait_nav_complete" },
        { kind: "block", type: "openamr_patrol" },
      ],
    },
    {
      kind: "category",
      name: "Motion",
      colour: "#3f7c9b",
      contents: [
        { kind: "block", type: "openamr_wait" },
        { kind: "block", type: "openamr_set_speed" },
        { kind: "block", type: "openamr_drive_for" },
        { kind: "block", type: "openamr_rotate_for" },
        { kind: "block", type: "openamr_stop_movement" },
        { kind: "block", type: "openamr_stop" },
      ],
    },
    {
      kind: "category",
      name: "Docking",
      colour: "#22a06b",
      contents: [
        { kind: "block", type: "openamr_dock" },
        { kind: "block", type: "openamr_undock" },
      ],
    },
    {
      kind: "category",
      name: "Robot State",
      colour: "#ca8a04",
      contents: [
        { kind: "block", type: "openamr_battery_below" },
        { kind: "block", type: "openamr_set_mode" },
      ],
    },
  ],
};
