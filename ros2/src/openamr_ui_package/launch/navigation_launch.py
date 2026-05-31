import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# Launch file for the navigation stack in ROS2.
# I find it pretty weird that none of the linorobot2 packages are called here.
#TODO: Check if this is the correct procedure: Maybe we have to call linorobot2_bringup and that those nodes are part of their own packages
# (Though, if this works without linorobot, this is better, it means we didn't ever need the package in the first place)

def generate_launch_description():
    pkg_share = get_package_share_directory('openamr_ui_package')

    move_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'move_base_launch.py')
        )
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'amcl_launch.py')
        )
    )

    return LaunchDescription([
        move_base_launch,
        amcl_launch
    ])