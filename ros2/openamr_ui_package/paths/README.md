# About

This folder hosts the routes as **csv** files. Each route consist of a list of waypoints : a waypoint is
composed of **10 floats** , the first 3 indicates the point’s **position** (with x,y,z coordinates), the next 4
are its **orientation** (in quaternions), the next 3 are it’s **covariance** (required by nav2 for navigation),
the last one is it’s **purpose**

# Installing routes

In this folder, only the placeholder route is present. Additional routes are created by using the
**Create Route** button in the UI. However, it is possible to download and install preexisting routes:

1: In the ROS2 **install** folder, locate the package’s share directory (which should be named
**openamr_ui_package** ) and go to the **paths** folder.

2: Create a folder with the same name as the group of the desired map. Create another folder inside
with the name of the name inside the group folder.

3: Copy the **route’s csv** file inside this new folder.


