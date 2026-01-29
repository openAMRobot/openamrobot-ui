import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('openamr_ui_package')
    config_file_path = os.path.join(share, 'config', 'config.yaml')

    folder = Node(
        package='openamr_ui_package',  # Package name
        executable='handler',          # Refers to the entry point defined in setup.py
        name='folders_handler',        # Node name in ROS2
        output='screen'                # Set output to screen for logs
    )
    nav = Node(
        package='openamr_ui_package',
        name='waypoint_nav',
        executable='nav',
        output='screen'
    )
    Nodes = LaunchDescription([folder,nav])
    return Nodes