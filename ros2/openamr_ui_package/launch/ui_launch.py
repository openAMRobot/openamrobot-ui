import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Fetches the package share directory for 'openamr_ui_package' (found in the install folder)
    
    package_share_dir = get_package_share_directory('openamr_ui_package')
    

    new_ui_launch_path = os.path.join(package_share_dir, 'launch', 'new_ui_launch.py') #Launch file for web related packages
    phys_launch_path = os.path.join(package_share_dir, 'launch', 'physnode_launch.py') #Launch file for physical ROS2 nodes

    #Tells ROS2 to launch those two files
    new_ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(new_ui_launch_path)
    )

    phys_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(phys_launch_path)
    )

    Nodes = LaunchDescription([new_ui_launch, phys_launch])

    # Retourne la description du lancement
    return Nodes