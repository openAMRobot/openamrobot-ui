from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch file for the AMCL (Adaptive Monte Carlo Localization) node in ROS2.
    # Called by navigation launch
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'min_particles': 500,
                'max_particles': 3000,
                'kld_err': 0.02,
                'kld_z': 0.99,
                'update_min_d': 0.20,
                'update_min_a': 0.20,
                'resample_interval': 1,
                'transform_tolerance': 0.5,
                'recovery_alpha_slow': 0.001,
                'recovery_alpha_fast': 0.1,
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_a': LaunchConfiguration('initial_pose_a'),
                'gui_publish_rate': 50.0,
                'laser_max_range': 7.0,
                'laser_max_beams': 180,
                'laser_z_hit': 0.5,
                'laser_z_short': 0.05,
                'laser_z_max': 0.05,
                'laser_z_rand': 0.5,
                'laser_sigma_hit': 0.2,
                'laser_lambda_short': 0.1,
                'laser_likelihood_max_dist': 2.0,
                'laser_model_type': 'likelihood_field',
                'odom_model_type': 'diff',
                'odom_alpha1': 0.2,
                'odom_alpha2': 0.2,
                'odom_alpha3': 0.2,
                'odom_alpha4': 0.2,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
            }],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic'))
            ]
        )
    ])