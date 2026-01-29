from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


#Launch file for the web related components (Flask app, camera feed, ros communication)
def generate_launch_description():

    config= PathJoinSubstitution([FindPackageShare('openamr_ui_package'), 'param', 'config.yaml'])

    # Flask app node
    flask_node = Node(
        package='openamr_ui_package',
        executable='flask',
        name='flask_app',
        output='screen',
        parameters=[config]  # Parameters for Flask app
    )

    # Camera feed node using web_video_server
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='camera',
        output='screen',
        parameters=[config]
    )


    # ROS bridge node for communication, allows for the Flask app to communicate with ROS topics
    
    rosbridge=Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            parameters=[config]
        )

    Nodes = LaunchDescription([
        flask_node,
        web_video_server_node,
        rosbridge])
    
    # Retourne la description du lancement avec tous les nœuds et inclusions
    return Nodes