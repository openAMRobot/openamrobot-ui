# About


**open_amr_msgs** is a subpackage for **openamr_ui_package**. Every custom ROS2 interface is stored here in their respective folders.

These interfaces allow ROS2 to send data and instructions to the robot or the UI.
As the software progresses, the need for additional interfaces may come up. Bellow is detailed how to add them.

# Adding interfaces


**1. Choosing the right type :**

There are three types of interfaces present in ROS2
    
-**Messages** transmit data to nodes, such as a desired color. They are found in the **msg** folder and use the **.msg** extension
       
-**Services** gives the node simple instructions, such as changing the color of a point. They are found in the **srv** folder and use the **.srv** extension
       
-**Actions** are complex, goal based, sets of instructions that generally take some time, such as moving to a red colored point. They are found in the **action** folder and use the **.action** extension.
       
**2. Checking dependencies:** Interfaces use data types which come from various libraries. It is absolutely fundamental than these libraries are imported in **package.xml** using the ```<depend>``` tag
    
**3. Creating the interface:** Templates for each type can be found in their respective folders

**4. Importing the interface:** In **CmakeLists.txt** , add the path to the newly created interface under line 13 (```rosidl_generate_interfaces(${PROJECT_NAME}```) following the convention here: ```{folder}/{name}.{extension}```
The new interface should be ready

# Further steps


For more in depth knowledge of what these interfaces can do and how to implement them, the ROS2 foundation’s [tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) are a great starting point.


