#!/usr/bin/env python3
import webbrowser
import rclpy
from rclpy.node import Node
from rclpy.parameter import get_parameter_value #TODO: Implement params
import os
import shutil
import json
import time
import subprocess
import yaml
import csv
from time import sleep 
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from openamr_ui_msgs.msg import ArrayPoseStampedWithCovariance
from nav2_msgs.srv import LoadMap

#TODO:Convert all comments bellow function defs to docstrings
class UIFoldersHandler(Node):
    # Initialization of node
    def __init__(self):
        super().__init__('ui_folders') #Node named ui_folders
        os.system("pkill -f /map_server") #Kill a map server previously launch to prevent duplicates TODO: Use nodechecker to disable lifecycle manager launches (Because that node does not like pkill unlike map_server)

        self.WPs = [] #Holds the waypoints sent by the UI as a string
        self.waypoints = [] #Holds the waypoint read from the current route file as a tupple
        self.position = 0

        
        self.get_logger().info("------------ UI folders handler started ------------")
        #Parameters for the webrowser
        config_file= os.path.join(get_package_share_directory('openamr_ui_package'), 'param', 'config.yaml')
        #Fetches the ip and port of the Flask app for the web browser opening
        with open(config_file, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
        self.local_ip= data["flask_app"]["ros__parameters"]["appAddress"] #IP address of the Flask app
        self.local_port= data["flask_app"]["ros__parameters"]["portApp"] #Port of the Flask app
        #Subcription creation
        self.odomsub= self.create_subscription(Odometry,"/odom", self.odom_callback,10) #Odometry for AMR position, name of the topic is suposed to depend of YAML (TODO later)
        self.uiopsub=self.create_subscription(String,"ui_operation", self.ui_callback,10) #Subscriber to the messages that the UI can send
        self.waysub=self.create_subscription(PoseWithCovarianceStamped,"/new_way_point", self.new_way_point_callback,10) #Subscription to the waypoint handler specifically for WP creation
        self.navsub=self.create_subscription(Empty,"/nav_data_req", self.nav_data_callback,10) #Subscription to the UI to get the navigation data
        self.reqsub=self.create_subscription(Empty,"WP_req",  self.WP_req_callback,10) #Subscription to get the waypoints from the current route file
        #Publisher creation
        self.ui_pub = self.create_publisher(String,'ui_message', 1) #Publisher to send messages to the UI
        self.poseArray_publisher = self.create_publisher(ArrayPoseStampedWithCovariance,"WayPoints_topic", 1) #Publisher to send the waypoints to the UI 
        self.set_pose = self.create_publisher( PoseWithCovarianceStamped,'initialpose', 1) #Publisher to set the initial pose of the robot
        self.nav_data_pub = self.create_publisher(String,'nav_data_resp', 1) #Publisher to send the navigation data to the UI
        #Ressource folders registration
        package_share_dir = get_package_share_directory('openamr_ui_package') #Main package share directory
        self.maps_folder = os.path.join(package_share_dir, 'maps') #Folder where the maps are stored
        self.routs_folder = os.path.join(package_share_dir, 'paths') #Folder where the routes are stored
        self.current_files = os.path.join(package_share_dir, 'param/current_map_route.yaml') #Config file for active map and route
        #Templated launch files, will depend on parameters once the config problems are fixed
        self.mappingCmd = "openamr_ui_package mapping_launch.py" #Lidar related launch file (used for map bulding) TODO: Something is bugged here, either it's the command or the launch file itself
        self.navigationCmd = "openamr_ui_package navigation_launch.py" #Linorobot navigation related launch file TODO: Finish refurbishing the AMR so we can test this
        self.dict_cmd = None #Holds the command received from the UI
        
        #Starts both the map server and the lifecycle manager
        run_map_server_command = f"ros2 launch openamr_ui_package map_server_launch.py" 
        run_map_cycle = f"ros2 run nav2_util lifecycle_bringup map_server" 
        self.get_logger().info("--------------------------------------------")      
        self.get_logger().info(run_map_server_command)      
        self.get_logger().info("--------------------------------------------")
        self.get_logger().info(run_map_cycle)      
        self.get_logger().info("--------------------------------------------") 
        #TODO: Store all subprocesses in variavbles to be able to shut them down     
        subprocess.Popen(run_map_server_command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        subprocess.Popen(run_map_cycle, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        # --------
        #Change map client
        sleep(1)
        self.change_map_cli= self.create_client(LoadMap,'/map_server/load_map') #Map_server service to change the map
        self.get_logger().info('Connecting to map_server... at')
        self.change_map_cli.wait_for_service(300) #Tries to connect to the service for 5 minutes and then timeouts
        self.get_logger().info('CONNECTED to map_server')
        webbrowser.open(f"http://{self.local_ip}:{self.local_port}") #Automatically open the web browser to the Flask app
        #TODO: Right now the ROS2 map_server loads the same map every time because of ROS2 restrictions. Call a change map function with the map found in current_map_route.yaml to fix this.
        #self.change_map(self.get_cur_files()["map_file"], True, True) #Changes the map to the one found in the current_map_route.yaml file
        #TODO: The command above is incomplete: The dict_cmd is currently Null so the software cannot search for paths.
    #---------------------[METHODS]-----------------------------
        
    def odom_callback(self, data:Odometry):
        self.position = data.pose.pose #Stores the pose given by the Odometry topic (Coords + Orientation)
        #TODO: Since this variable is only used to create a PoseWithCoveriance msg, check if there is an easier solutions to transmit it

    def nav_data_callback(self, data:Empty):
        time.sleep(0.5)
        msg=String()
        msg.data=self.get_paths()
        self.nav_data_pub.publish(msg)

    def new_way_point_callback(self, data:PoseWithCovarianceStamped):
        #Registers a new waypoint
        #That is a lot of strs TODO: check if we could loop that
        line =  f"{data.pose.pose.position.x},"
        line += f"{data.pose.pose.position.y},"
        line += f"{data.pose.pose.position.z},"
        line += f"{data.pose.pose.orientation.x},"
        line += f"{data.pose.pose.orientation.y},"
        line += f"{data.pose.pose.orientation.z},"
        line += f"{data.pose.pose.orientation.w},"
        line += f"{data.pose.covariance[0]},"
        line += f"{data.pose.covariance[1]},"
        line += f"{data.pose.covariance[2]}" 
        #Prevents two identical waypoints to be saved (Kind of redundant since precise waypoint editing is not implemented yet)
        if(line not in self.WPs):
            self.WPs.append(line)
            msg=String()
            if (len(self.WPs) == 1):
                msg.data="1 waypoint added to the route"
            else:
                msg.data=f"{str(len(self.WPs))} waypoints added to the route"
            self.ui_pub.publish(msg)
        else:
            self.get_logger().info("Waypoint already exists in the route")
        
    def convert_PoseArray(self, waypoints:list):
        #Unused function, keeping it for the time being if the custom message does not work
        poses = PoseArray()
        poses.header.frame_id = 'map'
        poses.poses = [pose.pose.pose for pose, purpose in waypoints]
        return poses

    def convert_PoseWithCovArray_to_PoseArrayCov(self, waypoints:list):
        #Convert the waypoints into an array
        poses = ArrayPoseStampedWithCovariance()
        for pose_arg, purpose in waypoints:       
            poses.poses.append(pose_arg)
        return poses

    def WP_req_callback(self, data:Empty):
        time.sleep(0.5)
        #Sends the waypoints to the UI when the user clicks on a new page
        self.read_wp()  
        self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints)) 
 
    def read_wp(self):
        #Collects the waypoint from the csv file
        route_file = os.path.expanduser(self.get_cur_files()["route_file"]) #Gets the current route file path from the YAML file
        #Resets the waypoint list
        del self.waypoints[:]
        if len(route_file) != 0:
            #share_dir=get_package_share_directory('openamr_ui_package')
            with open(route_file, 'r') as file:
                reader = csv.reader(file, delimiter=',') #Separated the waypoints in a list
                for line in reader:
                    current_pose = PoseWithCovarianceStamped()
                    current_pose.header.frame_id = 'map'
                    current_pose.pose.pose.position.x = float(line[0])
                    current_pose.pose.pose.position.y = float(line[1])
                    current_pose.pose.pose.position.z = float(line[2])
                    current_pose.pose.pose.orientation.x = float(line[3])
                    current_pose.pose.pose.orientation.y = float(line[4])
                    current_pose.pose.pose.orientation.z = float(line[5])
                    current_pose.pose.pose.orientation.w = float(line[6])
                    current_pose.pose.covariance[0] = float(line[7])
                    current_pose.pose.covariance[1] = float(line[8])
                    current_pose.pose.covariance[2] = float(line[9])

                    self.waypoints.append((current_pose, float(line[10]))) #Formats the waypoints into real Pose msgs

        if self.waypoints == []:
            Empty_msg = String(data="The waypoint queue is empty.")
            self.ui_pub.publish(Empty_msg)
    
    def get_paths(self):
        #Fetches the different components of the map and route folders and returns them in a JSON format
        files = {}
        for group in os.listdir(self.routs_folder):
            files[group] = []
            for i in os.listdir(f"{self.routs_folder}/{group}"):
                files[group].append({i: os.listdir(f"{self.routs_folder}/{group}/{i}")})
        
        data = self.get_cur_files()  
        if len(data["route_file"]) != 0: #Everything is present
            data = os.path.expanduser(data["route_file"]).split("/")[-3:]
            group, map, route = data[0], data[1], data[2].split(".")[0]
        elif len(data["map_file"]) != 0: #Case of no route file
            data = os.path.expanduser(data["map_file"]).split("/")[-2:]
            group, map, route = data[0], data[1].split(".")[0], "Null"
        else: #Worse case
            group, map, route = "Null", "Null", "Null"
        
        response = {"structure":[], "active_files":{"group":group,"map":map,"route":route}} #Sends the active files to the UI

        for i, j in files.items():
            response["structure"].append({i: j})
        response = str(response).replace('\'', '\"')
        return response

# ----- UI CALLBACKS -----
    #MAP SECTION
    def build_map_func(self):
        #Starts the Lidar nodes to create a new map
        try:
            msg=String()
            msg.data="Mapping..."
            self.poseArray_publisher.publish(ArrayPoseStampedWithCovariance()) #what, why
            self.ui_pub.publish(msg)
            #TODO: Use Pkill
            os.system("ros2 lifecycle set /amcl shutdown")
            os.system("ros2 lifecycle set /move_base shutdown")
            os.system("ros2 lifecycle set /map_server shutdown")
            time.sleep(1)
            #TODO: Fix the bugs with the launch files
            subprocess.Popen(f"ros2 launch {self.mappingCmd}", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)
            time.sleep(3)
            
            msg.data="Move the robot along the perimeter of the room and in the center using the control buttons and return robot to start position"
            self.ui_pub.publish(msg)
        except Exception as e:
            self.get_logger().info(f"Error in build_map_func: {e}")

    def save_map_func(self):
        #Saves changes made to the map
        try:            
            self.ui_pub.publish("Saving map...")
            map_path_to_save = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_folder_path_to_save = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            #TODO: Check if the command works (We can't use it until build_map_func is fixed)
            map_save_command = "ros2 run map_server map_saver -f "
            cmd = f"{map_save_command}{map_path_to_save}"

            # self.ui_pub.publish(cmd)  
            self.get_logger().info(cmd)                  
            os.system(cmd)
            
            #This whole section exists because the program currently cannot edit an existing map. This assumes that the map was just created and thus has no routes to speak of. TODO:Once editing is implemented, modify everything there to account for it
            os.mkdir(route_folder_path_to_save) 
            self.set_cur_route("")
            self.WP_req_callback(Empty())
            self.ui_pub.publish("Map saved")                

            # 1. save new current map to file
            map_yaml_file = f"{map_path_to_save}.yaml"
            self.set_cur_map(map_path_to_save)
            self.get_logger().info(f"Current map was setted to {map_yaml_file}")  

            # 2. save curent position
            current = self.position

            # 3. reload (do we need to?) TODO: Check relevancy of this whole section
            #--------------------------
            os.system("ros2 lifecycle set /slam_gmapping 5")
            os.system("ros2 lifecycle set /map_server 5")

            self.ui_pub.publish("Localization...")
            time.sleep(1)
            #Note that this is the only point where navigation commands are used, meaning that users can't pilot their robot until they save a map at each launch???
            subprocess.Popen(f"ros2 launch {self.navigationCmd}", stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)
            
            self.get_logger().info("running saved map")  
            # self.ui_pub.publish("running saved map")               
            command = f"ros2 launch openamr_ui_package map_server.launch map_file:={map_yaml_file}"

            subprocess.Popen(command, stdout=subprocess.PIPE,
                            shell=True, preexec_fn=os.setsid)
            #-------------------------------
            time.sleep(3)

            # 3. initial pose -> saved pos
            current_pose = PoseWithCovarianceStamped()
            current_pose.header.frame_id = "map"
            current_pose.pose.pose = current  
            # self.ui_pub.publish("current_pose")  

            # self.set_robot_pose.publish(current) 
            self.set_pose.publish(current_pose)
            self.ui_pub.publish("You can set points or follow the route")


            self.nav_data_pub.publish(self.get_paths())

        except Exception as e:
            self.get_logger().info(f"Error in save_map_func: {e}")

    def change_map_func(self):
        #Receives a map from the UI and calls the change_map function.
        path_to_new_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
        self.change_map(path_to_new_map)


    def create_group_func(self):
        #Creates a new group folder
        try:
            os.mkdir(f"{self.routs_folder}/{self.dict_cmd['group']}") #Inside the routes folder 
            os.mkdir(f"{self.maps_folder}/{self.dict_cmd['group']}") #Inside the maps folder
            self.nav_data_pub.publish(self.get_paths()) #Refreshes the UI with the new group
        except Exception as e:
            self.get_logger().info(f"Error in create_group_func: {e}")

    def rename_map_func(self):
        #Changes the name of the map and updates the YAML file accordingly
        try:
            #Map names
            old_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"
            new_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}" 
            #Route names
            old_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"  
            new_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}"
            #ROS map names (Deprecated structure, needs changing)
            old_ros_folder_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}_ros"  
            new_ros_folder_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}_ros" 

            map_cur_path = None
            ros_cur_path = None
            with open(f"{old_map_file}.yaml", 'r') as file:
                map_cur_path = yaml.load(file, Loader=yaml.FullLoader)
            
            with open(f"{old_ros_folder_file}.yaml", 'r') as file:
                ros_cur_path = yaml.load(file, Loader=yaml.FullLoader)

            map_cur_path["image"] = f"{new_map_file}.png" #Modifies the image path in the map's YAML file
            ros_cur_path["yaml_filename"] = f"{self.dict_cmd['map_new']}.yaml" #Again, probably useless and deprecated and shamefull
            #Changes application
            with open(f"{old_ros_folder_file}.yaml", 'w') as file:
                yaml.dump(ros_cur_path, file) 

            with open(f"{old_map_file}.yaml", 'w') as file:
                yaml.dump(map_cur_path, file) 
            #Renaming
            os.rename(f"{old_map_file}.yaml", f"{new_map_file}.yaml") 
            os.rename(f"{old_map_file}.png", f"{new_map_file}.png")
            os.rename(f"{old_ros_folder_file}.yaml", f"{new_ros_folder_file}.yaml")
            os.rename(old_route_folder_file, new_route_folder_file)
            #Updates the current map and route to the new name. (I'd like to point the fact that theoretically, you can rename a map that isn't the active one...)
            data = self.get_cur_files()
            if os.path.expanduser(data["map_file"]) == f"{old_map_file}.yaml":
                self.set_cur_map(new_map_file)
                self.set_cur_route(f"{new_route_folder_file}/{data['route_file'].split('/')[-1].split('.')[0]}")
            
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            self.get_logger().info(f"Error in rename_map_func: {e}")

    def delete_map_func(self):
        #TODO: when the default map is implemented, make it immune to this function
        try:
            #Stores the path to the map and route to delete
            map_to_delete = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_map_folder_to_delete = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"   
            #Performs deletion
            os.remove(f"{map_to_delete}.yaml")
            os.remove(f"{map_to_delete}_ros.yaml")
            os.remove(f"{map_to_delete}.png")
            shutil.rmtree(route_map_folder_to_delete)
            #Fetches all the maps in the same group first
            maps_in_group = os.listdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            if len(maps_in_group) != 0:
                #Map found
                path_to_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}"
                self.set_cur_map(path_to_map)                
                self.get_logger().info(f"Curr map: {path_to_map}")
                self.change_map(path_to_map,True)

                routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}")
                self.get_logger().info(f"routes_on_map: {routes_on_map}")
                
                if len(routes_on_map) != 0:
                    path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}"
                    self.set_cur_route(path_to_route) 
                    self.get_logger().info(f"Curr route: {path_to_route}")
                else:
                    self.set_cur_route("")
                    self.ui_pub.publish("No routes on the map")

            else:
                #TODO: Implement a method for when maps aren't detected (Probably goes back to the default map)
                self.set_cur_route("")
                self.set_cur_map("")
                self.ui_pub.publish("No maps in the group")
            self.WP_req_callback(Empty())         
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            self.get_logger().info(f"Error in delete_map_func: {e}")

    def delete_group_func(self):
        #TODO: When the default group is implemented, make it immune to this function

        #Deletes the group and all the maps and routes in it
        try:
            group_to_delete = f"{self.maps_folder}/{self.dict_cmd['group']}"
            route_group_folder_to_delete = f"{self.routs_folder}/{self.dict_cmd['group']}"
            # Deletes the group folder and all its contents
            shutil.rmtree(group_to_delete)
            shutil.rmtree(route_group_folder_to_delete)
            # Fetches the first map and route in the first group to set them as current TODO: Once the default group is implemented, this should be changed to point to the default group and map
            groups_in_folder = os.listdir(f"{self.routs_folder}") #Detects groups 
            self.get_logger().info(f"groups_in_folder: {groups_in_folder}")
            
            if len(groups_in_folder) != 0: 
                path_to_map = f"{self.maps_folder}/{groups_in_folder[0].split('.')[0]}" #Creates the path to the first group found

                maps_in_group = os.listdir(path_to_map) #Detects maps in the group
                if len(maps_in_group) != 0:
                    path_to_map = f"{path_to_map}/{maps_in_group[0].split('.')[0]}" #Creates the path to the first map found in the group
                    self.set_cur_map(path_to_map) #Config file update
                    self.change_map(path_to_map, True) #Service call           
                    self.get_logger().info(f"Curr map: {path_to_map}")

                    routes_on_map = os.listdir(f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}") #Detects routes for the map
                    self.get_logger().info(f"routes_on_map: {routes_on_map}")
                    
                    if len(routes_on_map) != 0:
                        path_to_route = f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}" #Creates the path to the first route found in the map
                        self.set_cur_route(path_to_route) #Config file update
                        self.get_logger().info(f"Curr route: {path_to_route}")
                    else:
                        self.set_cur_route("") #Empty route
                        self.ui_pub.publish("No routes on the map")

                else:
                    self.set_cur_map("") #Empty map
                    self.set_cur_route("")
                    self.ui_pub.publish("No maps in the group")
            else:
                self.set_cur_map("")
                self.set_cur_route("")
                self.ui_pub.publish("No maps in the group")
            self.WP_req_callback(Empty()) 

            self.nav_data_pub.publish(self.get_paths()) #UI update
        except Exception as e:
            self.get_logger().info(f"Error in delete_group_func: {e}")
    
    #ROUTE SECTION

    def clear_route_func(self):
        #Resets the current Waypoint list
        try:
            del self.WPs[:]
            self.ui_pub.publish("Waypoints cleared, please set new points on the map")
        except Exception as e:
            self.get_logger().info(f"Error in clear_route_func: {e}")

    def save_route_func(self):
        #Saves the current Waypoint list to a CSV file
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}"

            with open(f"{path_to_route}.csv", 'w') as file:
                for WP in self.WPs: #Here, waypoints are strings, not PoseWithCovarianceStamped
                    pos = WP + ", 1"
                    file.write(pos + '\n')
            self.ui_pub.publish(f"{len(self.WPs)} waypoints saved")
            self.get_logger().info(f"{len(self.WPs)} waypoints saved")

            self.set_cur_route(path_to_route) #Config file update


            del self.WPs[:]
            self.nav_data_pub.publish(self.get_paths()) #UI update
        except Exception as e:
            self.get_logger().info(f"Error in save_route_func: {e}")

    def edit_route_func(self):
        #Reads the current route file and adds the waypoints to the WPs list TODO: This function only allows adding waypoints to the current route, not edit them. Needs to be implemented later
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route'].split('.')[0]}"
            self.set_cur_route(path_to_route) #Config file update  

            self.read_wp() #Collects the waypoints from the current route file
            for point, purpose in self.waypoints: #TODO: Establish what "purpose" is supposed to do
                if purpose == 2: continue
                line =  f"{point.pose.pose.position.x},{point.pose.pose.position.y},{point.pose.pose.position.z},{point.pose.pose.orientation.x},{point.pose.pose.orientation.y},{point.pose.pose.orientation.z},{point.pose.pose.orientation.w},{point.pose.covariance[0]},{point.pose.covariance[1]},{point.pose.covariance[2]}"

                if(line not in self.WPs):
                    self.WPs.append(line)
            
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            self.get_logger().info(f"Error in edit_route_func: {e}")

    def delete_route_func(self):
        #Deletes the current route file and sets the first route on the map as the current route
        try:
            file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}.csv" #Path to the route file to delete
            
            os.remove(file) #Route deletion
            routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}") #Collects the routes on the map
            if len(routes_on_map) != 0:
                path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}" #Creates the path to the first route found in the map
                self.set_cur_route(path_to_route) #Config file update
            else:
                self.set_cur_route("") #Empty route
                self.ui_pub.publish("No routes on the map")

            self.WP_req_callback(Empty())
            self.nav_data_pub.publish(self.get_paths())  
               
        except Exception as e:
            self.get_logger().info(f"Error in delete_route_func: {e}")

    def change_route_func(self):
        #Changes the current route to the one given in the UI command
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}" 
            self.set_cur_route(path_to_route) 
            self.read_wp() #Collects the waypoints from the new route file     
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints)) 
            self.nav_data_pub.publish(self.get_paths()) #UI update
        except Exception as e:
            self.get_logger().info(f"Error in change_route_func: {e}")

    def rename_route_func(self):
        #Changes the name of the route and updates the YAML file accordingly
        try:
            old_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_old']}.csv" 
            new_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_new']}.csv" 

            os.rename(old_file, new_file)
            self.set_cur_route(new_file.split(".")[0]) 
            self.nav_data_pub.publish(self.get_paths())
        except Exception as e:
            self.get_logger().info(f"Error in rename_route_func: {e}")
   
    def ui_callback(self, data:String):
        #Receives commands from the UI and executes the corresponding function
        self.get_logger().info(f"COMMAND RECEIVED: {data} ")

        try:
            command = data.data.split("/") 

            if len(command) > 1:  
                self.dict_cmd = json.loads(command[1])

            if command[0] == "build_map":
                self.get_logger().info("build_map")
                self.build_map_func()                
                                    
            elif command[0] == "save_map":  
                self.get_logger().info("build_map") 
                self.save_map_func()

            elif command[0] == "change_map":
                self.get_logger().info("change_map")
                self.change_map_func() 

            elif command[0] == "create_group":
                self.get_logger().info("create_group") 
                self.create_group_func()    

            elif command[0] == "rename_map":
                self.get_logger().info("rename_map") 
                self.rename_map_func() 

            elif command[0] == "delete_map":
                self.get_logger().info("delete_map")
                self.delete_map_func() 

            elif command[0] == "delete_group":
                self.get_logger().info("delete_group")
                self.delete_group_func()  

            elif command[0] == "clear_route": 
                self.get_logger().info("clear_route")   
                self.clear_route_func()  

            elif command[0] == "save_route":
                self.get_logger().info("save_route")
                self.save_route_func()

            elif command[0] == "edit_route":
                self.edit_route_func()
                self.get_logger().info("edit_route")

            elif command[0] == "delete_route":
                self.get_logger().info("delete_route")
                self.delete_route_func() 

            elif command[0] == "change_route":
                self.get_logger().info("change_route")
                self.change_route_func()

            elif command[0] == "rename_route":
                self.get_logger().info("rename_route")
                self.rename_route_func()

            elif command[0] == "rename_group":
                self.get_logger().info("rename_group")
                self.rename_group_func()

        except Exception as e:
            self.get_logger().error(f"Error in ui_callback: {e}")
    
# -----------------------------------------------------------
    def change_map(self, map_name:str, yaml:bool=False, manual:bool=False):
        self.get_logger().info(f"\n ==========[CHANGING MAP TO {map_name}]======== \n")
        #Changes the current map to the one given in the argument
        if not yaml: #If the map comes without the .yaml extension, allows for backward compatibility
            map_yaml_file = f"{map_name}.yaml"
            
        else:
            map_yaml_file = map_name #For when the map is given with the .yaml extension

        self.set_cur_map(map_yaml_file) #Updates the current map in the YAML file
        #If the change does not come from the UI, we have to manually provide the group and the name for the program to fetch the routes
        if manual:
            List= map_name.split("/")
            self.dict_cmd['group']= List[len(List)-2]
            self.dict_cmd['map']= List[len(List)-1].split(".")[0] #Gets the map name without the .yaml extension

        routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}")
        
        if len(routes_on_map) != 0:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}"
            self.set_cur_route(path_to_route)
        else:
            self.set_cur_route("")
            self.ui_pub.publish("No routes on the map")
        self.WP_req_callback(Empty())
        
        
        msg_change=LoadMap.Request() #Service message to change the map

        msg_change.map_url=map_yaml_file
        self.change_map_cli.call_async(msg_change)
        msg_path=self.get_paths()
        self.nav_data_pub.publish(msg_path)

    def set_cur_map(self, map_name:str):
        #Updates the YAML file with the current map path
        data = self.get_cur_files() #Opens the YAML file and fetches the right key
        if len(map_name) == 0:
            data["map_file"] = ""
        else:
            data["map_file"] = f"{map_name}" #Assumes the map is given with the .yaml extension
            
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file) #Writes the path
     
    def set_cur_route(self, route_name:str):
        #Updates the YAML file with the current route path
        data = self.get_cur_files() #Opens the YAML file and fetches the right key
        if len(route_name)  == 0:
            data["route_file"] = ""
        else:
            data["route_file"] = f"{route_name}.csv" #Path including extension   
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)  #Writes the path
  
    def get_cur_files(self):
        #Opens the map route YAML and fetches the right key
        with open(self.current_files, 'r') as file: #Note: make your that the file is present in the package share directory
            data = yaml.load(file, Loader=yaml.FullLoader)  
        return data
 
def main():
    #Main function to start the ROS2 node
    rclpy.init()
    controller = UIFoldersHandler()        
    rclpy.spin(controller)
    #TODO: Implement node shutdown as well a subprocesses kill


if (__name__ == "__main__"):
    main() #ROS2 requires an actual main function to be given in the setup file