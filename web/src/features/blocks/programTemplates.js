const block = (type, id, fields = {}, extra = {}) => ({
  type,
  id,
  ...(Object.keys(fields).length > 0 ? { fields } : {}),
  ...extra,
});

const chainBlocks = (blocks) => {
  const clonedBlocks = blocks.map((item) => ({ ...item }));

  clonedBlocks.forEach((item, index) => {
    if (clonedBlocks[index + 1]) {
      item.next = { block: clonedBlocks[index + 1] };
    }
  });

  return clonedBlocks[0];
};

const workspaceFromBlocks = (blocks) => ({
  blocks: {
    languageVersion: 0,
    blocks: [
      {
        ...chainBlocks([block("openamr_start", "template_start"), ...blocks]),
        x: 48,
        y: 48,
      },
    ],
  },
});

export const programTemplates = [
  {
    id: "safe_motion_test",
    name: "Safe Motion Test",
    description: "Moves forward slowly for one second, then stops.",
    createWorkspace: () =>
      workspaceFromBlocks([
        block("openamr_drive_for", "safe_drive", {
          LINEAR: 0.05,
          SECONDS: 1,
        }),
        block("openamr_stop_movement", "safe_stop"),
      ]),
  },
  {
    id: "navigate_and_wait",
    name: "Navigate And Wait",
    description: "Sends one map goal and waits for navigation to finish.",
    createWorkspace: () =>
      workspaceFromBlocks([
        block("openamr_navigate", "navigate_goal", {
          X: 1,
          Y: 0,
          YAW: 0,
        }),
        block("openamr_wait_nav_complete", "navigate_wait", {
          TIMEOUT: 60,
        }),
      ]),
  },
  {
    id: "docking_sequence",
    name: "Docking Sequence",
    description: "Navigates to Charging Station, waits, then docks.",
    createWorkspace: () =>
      workspaceFromBlocks([
        block("openamr_navigate_named", "dock_location", {
          LOCATION: "Charging Station",
        }),
        block("openamr_wait_nav_complete", "dock_wait", {
          TIMEOUT: 60,
        }),
        block("openamr_dock", "dock_robot"),
      ]),
  },
  {
    id: "patrol_route",
    name: "Patrol Route",
    description: "Moves between two points twice with a short wait.",
    createWorkspace: () =>
      workspaceFromBlocks([
        block("openamr_patrol", "patrol_route", {
          AX: 0,
          AY: 0,
          AYAW: 0,
          BX: 2,
          BY: 0,
          BYAW: 3.14,
          TIMES: 2,
          WAIT: 1,
        }),
      ]),
  },
  {
    id: "low_battery_dock",
    name: "Low Battery Dock",
    description: "If battery is below 20 percent, go to dock and dock.",
    createWorkspace: () =>
      workspaceFromBlocks([
        block(
          "openamr_battery_below",
          "battery_check",
          { PERCENT: 20 },
          {
            inputs: {
              DO: {
                block: chainBlocks([
                  block("openamr_log", "battery_log", {
                    MESSAGE: "Battery low, docking",
                  }),
                  block("openamr_navigate_named", "battery_dock_location", {
                    LOCATION: "Charging Station",
                  }),
                  block("openamr_wait_nav_complete", "battery_dock_wait", {
                    TIMEOUT: 60,
                  }),
                  block("openamr_dock", "battery_dock_robot"),
                ]),
              },
            },
          },
        ),
      ]),
  },
];

export const defaultProgramTemplateId = programTemplates[0].id;
