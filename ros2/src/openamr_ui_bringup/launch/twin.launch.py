"""Operator UI on the digital twin: the twin, a restricted rosbridge and the UI backend.

  ros2 launch openamr_ui_bringup twin.launch.py                  # platform twin (Gazebo + Nav2)
  ros2 launch openamr_ui_bringup twin.launch.py stack:=build1    # Build 1 scene (upperbody-sw)
  ros2 launch openamr_ui_bringup twin.launch.py stack:=none      # attach to a twin already running

Then open http://127.0.0.1:5050 on the same machine.

stack
  platform  openamrobot_bringup/bringup.launch.py sim:=true use_docking:=false (openamr-platform-sw).
            With platform-sw#90 in the workspace this is Gazebo, Nav2, twist_mux, the stop gate,
            velocity_smoother, collision_monitor, the /ui/cmd_vel relay and goal_pose_forwarder_node.
            Without #90 there is no stop gate: Stop (software) and Resume cannot work.
  build1    openamrobot_build1_scene/build1.launch.py (openamr-upperbody-sw), arguments passed
            through build1_args. The Baseline 1 acceptance harness is not started here.
  none      start nothing of the twin; use when the twin runs in another terminal or harness.

The rosbridge allowlist is config/twin_bridge_allowlist.yaml. The bridge binds bridge_address
(default 127.0.0.1) and the Flask app binds ui_address (default 127.0.0.1). The camera stream,
the battery simulator, folders_handler and the legacy navigation launches are not started.

Simulation evidence only. Not hardware, physical or safety acceptance.
"""
import os

import yaml

ALLOWLIST_FILE = 'twin_bridge_allowlist.yaml'


def load_allowlist(path):
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    for key in ('read_topics', 'write_topics', 'manual_drive_topics', 'services', 'actions'):
        data.setdefault(key, [])
        if not isinstance(data[key], list):
            raise ValueError(f'{key} must be a list')
    return data


def glob_string(names):
    """rosbridge_suite 2.7.1 glob parameter: "['a', 'b']"; "[]" matches nothing."""
    for name in names:
        if any(c in name for c in "*?[]',"):
            raise ValueError(f'exact names only: {name!r}')
    return '[' + ', '.join(f"'{name}'" for name in names) + ']'


def bridge_parameters(allowlist, manual_drive=False):
    """The rosbridge_websocket glob parameters for the allowlist."""
    write = list(allowlist['write_topics'])
    if manual_drive:
        write += allowlist['manual_drive_topics']
    for name in write:
        if name.lstrip('/').startswith('cmd_vel'):
            raise ValueError(f'the UI never writes {name}')
    return {
        'topics_glob': '[]',
        'topics_sub_glob': glob_string(allowlist['read_topics']),
        'topics_pub_glob': glob_string(write),
        'services_glob': glob_string(allowlist['services']),
        'actions_glob': glob_string(allowlist['actions']),
    }


def rosapi_parameters():
    # rosapi answers /rosapi/* (allowed by rosbridge itself). params_glob "[]": no
    # parameter of any node can be read, set or deleted through it.
    return {'params_glob': '[]'}


def _truthy(value):
    return str(value).lower() in ('true', '1', 'yes')


def _launch_setup(context):
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription, LogInfo, Shutdown
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def value(name):
        return LaunchConfiguration(name).perform(context)

    stack = value('stack')
    manual_drive = _truthy(value('manual_drive'))
    share = get_package_share_directory('openamr_ui_bringup')
    allowlist = load_allowlist(os.path.join(share, 'config', ALLOWLIST_FILE))
    sim_time = {'use_sim_time': True}
    actions = []

    if stack == 'platform':
        bringup = get_package_share_directory('openamrobot_bringup')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'sim': 'true', 'use_docking': 'false', 'use_rviz': 'false',
                'gui': value('gui'), 'use_camera': 'false',
            }.items()))
    elif stack == 'build1':
        scene = get_package_share_directory('openamrobot_build1_scene')
        extra = dict(item.split(':=', 1) for item in value('build1_args').split() if ':=' in item)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scene, 'launch', 'build1.launch.py')),
            launch_arguments={'nav2': 'true', **extra}.items()))
    elif stack != 'none':
        return [LogInfo(msg=f'stack must be platform, build1 or none, not {stack!r}'),
                Shutdown(reason='bad stack argument')]
    if manual_drive:
        actions.append(LogInfo(msg='[twin] manual_drive:=true: /ui/cmd_vel is writable through the '
                                   'bridge. Only with platform-sw#90 on the stack.'))

    actions += [
        Node(package='openamr_ui_package', executable='map_relay', name='map_volatile_relay',
             namespace='ui', output='screen', parameters=[sim_time]),
        Node(package='openamr_ui_package', executable='nav_relay', name='nav_relays',
             namespace='ui', output='screen', parameters=[sim_time]),
        Node(package='openamr_ui_package', executable='flask', name='flask_app', namespace='ui',
             output='screen',
             parameters=[{'appAddress': value('ui_address'), 'portApp': int(value('ui_port'))},
                         sim_time]),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket', output='screen',
             parameters=[{'address': value('bridge_address'), 'port': int(value('bridge_port')),
                          **bridge_parameters(allowlist, manual_drive)}, sim_time]),
        Node(package='rosapi', executable='rosapi_node', name='rosapi', output='screen',
             parameters=[rosapi_parameters(), sim_time]),
    ]
    return actions


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    return LaunchDescription([
        DeclareLaunchArgument('stack', default_value='platform',
                              description='platform | build1 | none'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Gazebo window for stack:=platform'),
        DeclareLaunchArgument('build1_args', default_value='',
                              description='extra build1.launch.py arguments, "a:=b c:=d"'),
        DeclareLaunchArgument('bridge_address', default_value='127.0.0.1',
                              description='rosbridge bind address'),
        DeclareLaunchArgument('bridge_port', default_value='9090'),
        DeclareLaunchArgument('ui_address', default_value='127.0.0.1',
                              description='Flask bind address'),
        DeclareLaunchArgument('ui_port', default_value='5050'),
        DeclareLaunchArgument('manual_drive', default_value='false',
                              description='allow /ui/cmd_vel through the bridge (needs #90)'),
        OpaqueFunction(function=_launch_setup),
    ])
