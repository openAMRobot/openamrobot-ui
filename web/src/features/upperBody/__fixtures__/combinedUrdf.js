// A small stand-in for the combined model's structure (PR #14 names), not its
// geometry: base_footprint on the floor, a prismatic lift_joint and an arm
// mount. With lift_joint at 0 the shoulder is 1.000 m above the floor.
export const COMBINED_URDF = `<?xml version="1.0"?>
<robot name="openamrobot">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <link name="lift_column_base_link">
    <collision name="placeholder_lift_column_fixed_stage"><geometry><box size="0.24 0.06 0.53"/></geometry></collision>
  </link>
  <link name="lift_carriage_link">
    <visual><geometry><box size="0.2 0.05 0.35"/></geometry><material name="placeholder_geometry"/></visual>
  </link>
  <link name="openarm_left_base_link"/>
  <link name="openarm_left_link1"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.098" rpy="0 0 0"/>
  </joint>
  <joint name="lift_column_base_joint" type="fixed">
    <parent link="base_link"/><child link="lift_column_base_link"/><origin xyz="0.060 0 0.2" rpy="0 0 0"/>
  </joint>
  <joint name="lift_joint" type="prismatic">
    <parent link="lift_column_base_link"/><child link="lift_carriage_link"/>
    <origin xyz="0 0 0.536" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0.35" effort="0" velocity="0"/>
  </joint>
  <joint name="openarm_left_base_link_mount_joint" type="fixed">
    <parent link="lift_carriage_link"/><child link="openarm_left_base_link"/><origin xyz="0.18 0.031 0.166" rpy="0 0 0"/>
  </joint>
  <joint name="openarm_left_joint1" type="revolute">
    <parent link="openarm_left_base_link"/><child link="openarm_left_link1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>`;

export const PLATFORM_ONLY_URDF = `<?xml version="1.0"?>
<robot name="robo_urdf">
  <link name="base_footprint"/><link name="base_link"/>
  <joint name="base_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/></joint>
</robot>`;
