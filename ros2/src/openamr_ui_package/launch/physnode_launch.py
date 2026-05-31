from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    folder = Node(
        package='openamr_ui_package',
        executable='handler',
        name='folders_handler',
        output='screen',
    )
    nav = Node(
        package='openamr_ui_package',
        executable='nav',
        name='waypoint_nav',
        output='screen',
    )
    return LaunchDescription([folder, nav])
