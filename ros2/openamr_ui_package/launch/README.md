# About

The launch folder holds all of the packages launch files. A launch file is a special ROS2 file tasked
to launch nodes in specific ways, such as loading parameters from a config file, or creating copies
of the node.

# Content

- **ui_launch.py** is the main launch file. It’s goal is to load parameters and use them to launch the
other launch files, namely **new_ui_launch.py, folders_handler_launch.py** and
**waypoint_nav_launch.py**
- **new_ui_launch.py** launches the different web components : the host Flask server, the camera feed
and the rosbridge communication server.
- **mapping_launch.py** launches the different map building components, together, they allow the
AMR and its LIDAR.
- **navigation_launch.py** launches the different navigation components, together, they allow the
AMR to follow along routes and waypoints.
- **The other files** are either node launchers called by the **launch files above** or during the program’s
execution

# Further steps

To learn how to create launch files, consult the ROS [tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
