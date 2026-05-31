import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('openamr_ui_package')

    cost_com_conf=PathJoinSubstitution([pkg_share, 'param', 'move_base', 'costmap_common_params.yaml'])
    local_cost_com_conf=PathJoinSubstitution([pkg_share, 'param', 'move_base', 'local_costmap_params.yaml'])
    global_cost_com_conf=PathJoinSubstitution([pkg_share, 'param', 'move_base', 'global_costmap_params.yaml'])
    move_base_conf=PathJoinSubstitution([pkg_share, 'param', 'move_base', 'move_base_params.yaml'])
    base_local_planner_conf=PathJoinSubstitution([pkg_share, 'param', 'move_base', 'base_local_planner_default_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            #parameters=[
            #    cost_com_conf,
            #    local_cost_com_conf,
            #    global_cost_com_conf,
            #    move_base_conf,
            #    base_local_planner_conf,
            #],
            remappings=[
                ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
                ('odom', LaunchConfiguration('odom_topic')),
            ]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            #parameters=[
            #    global_cost_com_conf,
            #    move_base_conf,
            #]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            #parameters=[
            #    move_base_conf,
            #]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator'
                ]
            }]
        )
    ])