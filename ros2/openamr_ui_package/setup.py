import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'openamr_ui_package'


def package_files(directory):
    """
    Collect all files under 'directory' so they can be installed by ament_python.

    Returns a list of tuples in the form:
      (install_destination_folder, [file_path])
    """
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)

            # Example:
            #   path = "www/assets"
            #   install_path = "share/openamr_ui_package/www/assets"
            install_path = os.path.join('share', package_name, path)

            paths.append((install_path, [file_path]))
    return paths


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*')),
        (os.path.join('share', package_name, 'param', 'move_base'),
         glob('param/move_base/*')),
        (os.path.join('share', package_name, 'param'),
         glob('param/*[!move]')),
        (os.path.join('share', package_name, 'maps', 'Welcome'),
         glob('Welcome/*')),
        (os.path.join('share', package_name, 'paths', 'Welcome', 'Start'),
         glob('Start/*')),
    ] + package_files('www'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BotshareAI',
    maintainer_email='alex@botshare.ai',
    description='ROS2 Conversion for the OpenAMR platform',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'handler=openamr_ui_package.folders_handler:main',
            'nav=openamr_ui_package.waypoint_nav:main',
            'flask=openamr_ui_package.flask_app:main',
            'battery=openamr_ui_package.battery:main',
        ],
    },
)
