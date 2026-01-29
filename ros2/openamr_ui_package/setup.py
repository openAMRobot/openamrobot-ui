import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'openamr_ui_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'param','move_base'), glob('param/move_base/*')),
        (os.path.join('share', package_name, 'param'), glob('param/*[!move]')),
        (os.path.join('share', package_name, 'maps','Welcome'), glob('Welcome/*')),
        (os.path.join('share', package_name, 'paths/Welcome/Start'), glob('Start/*')),
                
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='darkadius',
    maintainer_email='tobbalya@gmail.com',
    description='ROS2 Conversion for the OpenAMR platform',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['handler=openamr_ui_package.folders_handler:main',
                            'nav=openamr_ui_package.waypoint_nav:main',
                            'flask=openamr_ui_package.flask_app:main',
                            'battery=openamr_ui_package.battery:main'
        ],
    },
)
