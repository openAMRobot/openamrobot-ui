export const testStructure = {
  active_files: {
    group: "floor_1",
    map: "room_1_2",
    route: "room_1_2_route_3.csv",
  },
  structure: [
    {
      floor_1: [
        {
          room_1_1: [
            "room_1_1_route_1.csv",
            "room_1_1_route_2.csv",
            "room_1_1_route_3.csv",
            "room_1_1_route_4.csv",
          ],
        },
        {
          room_1_2: [
            "room_1_2_route_1.csv",
            "room_1_2_route_2.csv",
            "room_1_2_route_3.csv",
            "room_1_2_route_4.csv",
          ],
        },
        { room_1_3: [] },
      ],
    },
    {
      floor_2: [
        { room_2_1: [] },
        { room_2_2: [] },
        { room_2_3: [] },
        { room_2_4: [] },
        { room_2_5: [] },
      ],
    },
    { floor_3: [{ room_3_1: [] }] },
    {
      floor_4: [
        { room_4_1: [] },
        { room_4_2: ["room_4_1_route_1.csv", "room_4_1_route_2.csv"] },
      ],
    },
  ],
};
