# About

**openamr_ui_package** is the main ROS package. It handles the UI’s execution as well as its
communication with the different ROS nodes.
There are 4 nodes as of now:

- **handler :** Handler communicates with the UI, it receives commands and sends data such as the
map, the waypoints or the camera feed.
- **nav :** Nav’s goal is to send waypoints for the AMR to follow. It communicates with it thanks to the
action server found in **openamr_msgs**.
- **flask :** Flask hosts the flask server which the UI uses. It also opens the sftware automatically upon
launch.
- **battery :** Battery tracks the current charge level of the AMR and sends commands to move to the
docking station if the level goes below a certain treshold.
Aside from these nodes, the package also handles the launch and ressource (maps, routes, config)
files.
