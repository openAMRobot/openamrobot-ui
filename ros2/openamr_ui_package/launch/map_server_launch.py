# Import des modules nécessaires pour créer un fichier de lancement
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Launch file for the map server in ROS2.
# The map server provides an active map file to the AMR.
# TODO: Currently, it's called by folder handlers.py as a thread, thi is unorthodox. We need to check if we can include it in ui_launch.py instead.

def generate_launch_description():
    package_share_dir = get_package_share_directory('openamr_ui_package')
    config_file_path = os.path.join(package_share_dir, 'maps/Welcome', 'Start_ros.yaml')
    map_file=os.path.join(package_share_dir, 'maps/Welcome', 'Start.yaml')

    map_server=Node(
            package='nav2_map_server',  # Package name
            executable='map_server',    # Node executable name
            name='map_server',          # Node name
            parameters=[config_file_path,{'yaml_filename': map_file}],
        )
    
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    
    return LaunchDescription([
        map_server, start_lifecycle_manager_cmd
    ])