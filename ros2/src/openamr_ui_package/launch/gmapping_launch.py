from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Launch file for the gmapping node in ROS2.
# This is used for creating a map of the environment using SLAM (Simultaneous Localization and Mapping).
# It is called by the mapping launch file.

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('set_map_frame', default_value='map'),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',#Need to check cause slam_gmapping is deprecated in ROS2
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'map_frame': LaunchConfiguration('set_map_frame'),
                'map_update_interval': 0.01,
                'maxUrange': 4.0,
                'maxRange': 5.0,
                'sigma': 0.05,
                'kernelSize': 3,
                'lstep': 0.05,
                'astep': 0.05,
                'iterations': 5,
                'lsigma': 0.075,
                'ogain': 3.0,
                'lskip': 0,
                'minimumScore': 30,
                'srr': 0.01,
                'srt': 0.02,
                'str': 0.01,
                'stt': 0.02,
                'linearUpdate': 0.05,
                'angularUpdate': 0.0436,
                'temporalUpdate': -1.0,
                'resampleThreshold': 0.5,
                'particles': 8,
                'xmin': -1.0,
                'ymin': -1.0,
                'xmax': 1.0,
                'ymax': 1.0,
                'delta': 0.05,
                'llsamplerange': 0.01,
                'llsamplestep': 0.01,
                'lasamplerange': 0.005,
                'lasamplestep': 0.005,
            }],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic'))
            ]
        )
    ])