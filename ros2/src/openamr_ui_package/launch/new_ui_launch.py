import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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


# PROPOSED (contract change request, awaiting the software lead).
# What a rosbridge client may do. Without these globs rosbridge accepts any
# topic, service or action, so a browser tab still running an older UI could
# publish /cmd_vel and bypass the platform command chain (twist_mux, stop
# gate, velocity_smoother, collision_monitor). Exact names only, no wildcard:
# /cmd_vel and every cmd_vel_* input stay unreachable; manual drive enters
# only on /ui/cmd_vel. Parameter names and parsing: rosbridge_suite 2.7.1
# rosbridge_server/scripts/rosbridge_websocket.py (PROTOCOL_PARAMETERS,
# parse_glob_string); rosbridge appends "/rosapi/*" to services_glob itself.
# test/test_rosbridge_allowlist.py checks this against the web app's names.
ROSBRIDGE_ALLOWLIST_UNUSED = {
    "topics_pub_glob": [
        "/ui/cmd_vel",
        "/goal_pose",
        "/initialpose",
        "/dock_trigger",
        "/undock_robot",
        "/ui_operation",
        "/ui_message",
        "/WP_req",
        "/nav_data_req",
        "/new_way_point",
    ],
    "topics_sub_glob": [
        "/safety/stop_lock",
        "/clock",
        "/odom",
        "/ui/map",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/scan_filtered",
        "/plan",
        "/tf",
        "/tf_static",
        "/ui/amcl_pose",
        "/ui/navigate_to_pose/status",
        "/navigate_to_pose/_action/feedback",
        "/dock_trigger_status",
        "/battery_status",
        "/charge_station_connected",
        "/ui_message",
        "/nav_data_resp",
        "/WayPoints_topic",
        "/joint_states",
        "/diagnostics",
        "/rosout",
    ],
    "services_glob": [
        "/safety/stop_lock_engage",
        "/safety/stop_lock_release",
        "/navigate_to_pose/_action/cancel_goal",
        "/navigate_through_poses/_action/cancel_goal",
        "/compute_path_to_pose/_action/cancel_goal",
        "/dock_robot/_action/cancel_goal",
        "/undock_robot/_action/cancel_goal",
        "/compute_path_to_pose/_action/send_goal",
        "/compute_path_to_pose/_action/get_result",
        "/reinitialize_global_localization",
        "/ui_cmd_vel_relay_node/get_parameters",
        "/map_server/get_state",
        "/amcl/get_state",
        "/controller_server/get_state",
        "/planner_server/get_state",
        "/bt_navigator/get_state",
        "/map_server/change_state",
        "/amcl/change_state",
        "/controller_server/change_state",
        "/planner_server/change_state",
        "/bt_navigator/change_state",
        "/controller_server/get_parameters",
        "/controller_server/set_parameters",
        "/planner_server/get_parameters",
        "/planner_server/set_parameters",
        "/global_costmap/global_costmap/get_parameters",
        "/global_costmap/global_costmap/set_parameters",
        "/local_costmap/local_costmap/get_parameters",
        "/local_costmap/local_costmap/set_parameters",
    ],
    # The web app sends no native rosbridge action goals.
    "actions_glob": [],
}

# rosapi_node: the web app reads and writes no parameter through
# /rosapi/get_param or /rosapi/set_param (rosbridge always allows /rosapi/*),
# so none is reachable there (rosapi/src/rosapi/glob_helper.py any_match).
ROSAPI_ALLOWLIST = {
    "params_glob": [],
}


def _glob_params(allowlist):
    """Render each glob list in the "[a, b]" string form rosbridge and rosapi parse."""
    return {name: "[" + ", ".join(globs) + "]" for name, globs in allowlist.items()}


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

    # use_sim_time is passed in by ui.launch.py and handed to every node below,
    # so the UI's ROS side runs on the same clock as the platform (/clock in
    # simulation, wall time on the robot). The browser picks its stamping mode
    # from the relay node's own use_sim_time, not from this value. Default
    # false keeps the behaviour of launching this file directly unchanged:
    # before this argument existed, no node here set use_sim_time.
    use_sim_time = LaunchConfiguration("use_sim_time")
    actions.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock for every UI node (ui.launch.py passes its own value).",
        )
    )
    sim_time = {"use_sim_time": use_sim_time}

    if not flask_env and not os.environ.get("ANTHROPIC_API_KEY"):
        actions.append(
            LogInfo(
                msg="[openamr_ui_package] ANTHROPIC_API_KEY is not configured. "
                "Set it in the process environment or copy .env.example to "
                ".env next to the package; /api/voice-plan will return 500 "
                "until it is set."
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
            parameters=[sim_time],
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
            parameters=[sim_time],
        )
    )

    # Battery node (battery.py): opt-in. Publishes Float32 on /battery_status,
    # the topic the web app reads. With battery_port empty it publishes a
    # SIMULATED value for development and never opens a serial device. On 2.0
    # the battery state comes from the base controller; the serial path is
    # Gate A legacy. Example:
    #   ros2 launch openamr_ui_bringup ui.launch.py start_battery_monitor:=true
    actions.append(
        DeclareLaunchArgument(
            "start_battery_monitor",
            default_value="false",
            description="Start battery.py (simulated value unless battery_port is set).",
        )
    )
    actions.append(
        DeclareLaunchArgument(
            "battery_port",
            default_value="",
            description=(
                "Gate A legacy battery serial device. "
                "Empty: simulated value, no device opened."
            ),
        )
    )
    actions.append(
        Node(
            package="openamr_ui_package",
            executable="battery",
            name="battery_monitor",
            output="screen",
            parameters=[{"port": LaunchConfiguration("battery_port")}, sim_time],
            condition=IfCondition(LaunchConfiguration("start_battery_monitor")),
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
            parameters=[config, sim_time],
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
                parameters=[config, sim_time],
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
                parameters=[sim_time, _glob_params(ROSAPI_ALLOWLIST)],
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
                parameters=[config, sim_time],
            )
        )
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg="[openamr_ui_package] web_video_server not installed -> camera streaming disabled. Install: sudo apt-get install -y ros-jazzy-web-video-server"
            )
        )

    return LaunchDescription(actions)


ROSBRIDGE_ALLOWLIST = {}
