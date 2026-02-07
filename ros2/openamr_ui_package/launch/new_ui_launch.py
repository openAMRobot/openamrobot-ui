from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


# Launch file for the web related components (Flask app, camera feed, ros communication)
def generate_launch_description():

    config = PathJoinSubstitution([FindPackageShare("openamr_ui_package"), "param", "config.yaml"])

    actions = []

    # Flask app node (serves React build from openamr_ui_package/static/app)
    actions.append(
        Node(
            package="openamr_ui_package",
            executable="flask",
            name="flask_app",
            output="screen",
            parameters=[config],
        )
    )

    # ROS bridge node (optional)
    try:
        get_package_share_directory("rosbridge_server")

        actions.append(
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[config],
            )
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg="[openamr_ui_package] rosbridge_server not installed -> UI control will NOT work. Install: sudo apt-get install -y ros-jazzy-rosbridge-server"
            )
        )

    # Camera feed node using web_video_server (optional)
    try:
        get_package_share_directory("web_video_server")

        actions.append(
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="camera",
                output="screen",
                parameters=[config],
            )
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg="[openamr_ui_package] web_video_server not installed -> camera streaming disabled. Install: sudo apt-get install -y ros-jazzy-web-video-server"
            )
        )

    return LaunchDescription(actions)
