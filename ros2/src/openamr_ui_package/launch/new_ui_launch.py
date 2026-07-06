import os

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def _load_env_file(path):
    """Parse simple KEY=VALUE lines from a .env file. Missing file -> {}."""
    env_vars = {}
    if not os.path.isfile(path):
        return env_vars
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            env_vars[key.strip()] = value.strip()
    return env_vars



# Launch file for the web related components (Flask app, camera feed, ros communication)
def generate_launch_description():

    config = PathJoinSubstitution([FindPackageShare("openamr_ui_package"), "param", "config.yaml"])

    # Package-scoped secrets (ANTHROPIC_API_KEY, etc.), gitignored. See .env.example.
    # realpath (not abspath) matters here: ros2 launch loads this file through
    # the installed share/ copy, which is a symlink back to this source file
    # under --symlink-install. abspath() doesn't follow symlinks, so it would
    # resolve to the install directory instead of here, where .env actually is.
    package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    flask_env = _load_env_file(os.path.join(package_root, ".env"))

    actions = []

    if not flask_env:
        actions.append(
            LogInfo(
                msg="[openamr_ui_package] No .env found next to the package "
                "(copy .env.example to .env and fill in ANTHROPIC_API_KEY) -> "
                "/api/voice-plan will return 500 until set."
            )
        )

    # Map QoS relay: bridges TRANSIENT_LOCAL map_server → VOLATILE for rosbridge
    actions.append(
        Node(
            package="openamr_ui_package",
            executable="map_relay",
            name="map_volatile_relay",
            namespace="ui",
            output="screen",
        )
    )

    # Nav relays: amcl_pose + navigate_to_pose action status TRANSIENT_LOCAL → VOLATILE
    actions.append(
        Node(
            package="openamr_ui_package",
            executable="nav_relay",
            name="nav_relays",
            namespace="ui",
            output="screen",
        )
    )

    # Flask app node (serves React build from openamr_ui_package/static/app)
    actions.append(
        Node(
            package="openamr_ui_package",
            executable="flask",
            name="flask_app",
            namespace="ui",
            output="screen",
            parameters=[config],
            additional_env=flask_env,
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

    # rosapi provides /rosapi/* services used by roslibjs helpers such as getTopics().
    try:
        get_package_share_directory("rosapi")

        actions.append(
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                output="screen",
            )
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg="[openamr_ui_package] rosapi not installed -> /rosapi/* browser helpers unavailable. Install: sudo apt-get install -y ros-jazzy-rosapi"
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
