#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient,ActionServer #TODO: Check if ActionServer is needed (normally not, it should be launched by the move_base package)
from ament_index_python.packages import get_package_share_directory
import os
import csv 
import time
import yaml

from std_srvs.srv import Empty as Emp #Is this for real needed?
#Note: This action was added to the openamr_ui_msgs package, despite being part of the move_base package. This is because the github repo for the jazzy branch of move_base does not include move_base_msgs
#If someone can confirm that this is normal and won't cause issues, then removing it and using the proper move_base action is better.
from openamr_ui_msgs.action import MoveBase
#from move_base_msgs.action import MoveBase #TODO: Check if we can import the action with this line instead of the previous one
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance, Quaternion

from std_msgs.msg import Empty, String, Bool, Float32
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry, Path

class WayPointMover(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('Way_points_handler')
        self.waypoints = []
        self.current_wp = 0
        self.on_the_route = False
        self.break_mission = False
        package_share_dir = get_package_share_directory('openamr_ui_package') #Hopefully in ros2_ws/install
        self.current_files = os.path.join(package_share_dir, 'param/curent_map_route.yaml') #Should not be changed

        self.charge_connected = False
        self.current_pos = PoseWithCovariance()
        self.home_pose = PoseWithCovariance()

        # Topics subscribers
        self.uiopsub=self.create_subscription(String,"ui_operation", self.ui_operation_callback,10) #Subscriber to the messages that the UI can send
        self.odomsub= self.create_subscription(Odometry,"/odom", self.odom_callback,10) #Odometry for AMR position, name of the topic is suposed to depend of YAML (TODO later)
        self.needCheckCharger = False #Usually dependent on param TODO: Add them back in once the config launch works
        if self.needCheckCharger:
            self.isChargeSub=self.create_subscription(Bool,"charge_station_connected",  self.charge_station_calback,10) #TODO: Modify this according to developements on autodocking

        # Topics publishers
        self.ui_message_pub = self.create_publisher(String,'ui_message', 1) #Publisher to send messages to the UI
        self.poseArray_publisher = self.create_publisher(PoseArray,"/WPs_topic", 1) #Publisher to send the waypoints to the UI
        
        # Actions 
        self.move_base_client = ActionClient(self,MoveBase,'move_base') #Subscribes to the move_base action server: needed to send poses goals to the robot.

        self.get_logger().info('Connecting to move_base...')
        self.move_base_client.wait_for_server() #Wont work for now
        self.get_logger().info('CONNECTED to move_base')
        
        self.get_logger().info("------------ Way points handler started ------------")       
        

    def charge_station_calback(self, data:Bool):
        #Used if the robot can charge autonomously TODO: Modify the callback to include behavior with charge level reading.
        self.charge_connected = data.data
        
        self.ui_message_pub.publish(f"Robot connected to charge station: {self.charge_connected}")
        self.get_logger().info(f"Robot connected to charge station: {self.charge_connected}")  

        if self.charge_connected:        
            self.home_pose.pose.position = self.current_pos.pose.position
            self.home_pose.pose.orientation = self.current_pos.pose.orientation

       
    def get_cur_files(self):
        # Reads the path of the active map and route from the YAML file TODO: This function is also present in folders_handler.py, so let's make it a submodule instead
        with open(self.current_files, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)  
        return data

    def read_wp(self):
        # Reads the waypoints from the route file TODO: Same as above
        route_file = self.get_cur_files()["route_file"]

        del self.waypoints[:]
        with open(route_file, 'r') as file:
            reader = csv.reader(file, delimiter=',')
            for line in reader:
                current_pose = PoseWithCovarianceStamped()
                current_pose.pose.pose.position.x = float(line[0])
                current_pose.pose.pose.position.y = float(line[1])
                current_pose.pose.pose.position.z = float(line[2])  # instead z coord use type of point
                current_pose.pose.pose.orientation.x = float(line[3])
                current_pose.pose.pose.orientation.y = float(line[4])
                current_pose.pose.pose.orientation.z = float(line[5])
                current_pose.pose.pose.orientation.w = float(line[6])
                current_pose.pose.covariance[0] = float(line[7])
                current_pose.pose.covariance[1] = float(line[8])
                current_pose.pose.covariance[2] = float(line[9])
                point_type = float(line[7])
                purpouse = float(line[10])

                self.waypoints.append((current_pose, point_type, purpouse)) 

        if self.waypoints == []:
            self.ui_message_pub.publish("The waypoint queue is empty.")


    def odom_callback(self, data:Odometry):
        # Callback to get the current position of the robot
        self.current_pos.pose = data.pose.pose
 
    def follow_func(self):
        # Function to follow all the waypoints of a single route
        if self.needCheckCharger and not self.charge_connected:
            self.ui_message_pub.publish(f"Robot can't start. Please connect to charge station")
            self.get_logger().info(f"Robot can't start. Please connect to charge station")  
            return
        
        if self.on_the_route:
            self.ui_message_pub.publish(f"The robot on the route already")
            self.get_logger().info(f"The robot on the route already") 
            return
        
        self.break_mission = False
        self.on_the_route = True
        self.current_wp = 0 #Seems pointless, it only is called to be incremented.

        self.read_wp()
        points_amount = len(self.waypoints)

        for index, (waypoint, point_type, purpouse) in enumerate(self.waypoints): #TODO: Confirm what the point_type and purpouse are for, and if they are needed in the future
            
            if self.break_mission:
                self.ui_message_pub.publish(f"break_mission")
                break
            self.ui_message_pub.publish("Following to "+str(index % points_amount+1)+" waypoint...")

            goal = MoveBase.Goal() #This is the central part of the code
            goal.target.header.frame_id =  "map" 
            goal.target.pose.position = waypoint.pose.pose.position
            goal.target.pose.orientation = waypoint.pose.pose.orientation

            self.move_base_client.send_goal_async(goal) #This function sends the goal to the move_base action server, meaning that the robot should move to the waypoint
            self.move_base_client._get_result() # Waits for the result of the action, meaning that the robot should have reached the waypoint before continuing

            self.current_wp = index+1

            if not self.break_mission:
                self.ui_message_pub.publish(f"Doing some action #{purpouse} on {index+1} point") #Curious, folders_handler.py always wants purpose = 2, why is it here
            time.sleep(0.1)  
               
        time.sleep(1)        
        self.ui_message_pub.publish("Current route was successfully completed")
        self.on_the_route = False

    def next_wp_func(self): 
        # Function to follow the next waypoint in the route
        self.get_logger().info("Following to next waypoint...") 
        self.ui_message_pub.publish("Following to next waypoint...") 

        self.read_wp() #Updates the waypoints list in case it was changed
            
        self.current_wp +=1

        if(len(self.waypoints) == 0): 
            self.ui_message_pub.publish("The waypoint queue is empty.")
            return
        
        if(self.current_wp > len(self.waypoints)):  
            self.current_wp = 1
        
        self.ui_message_pub.publish("Following to "+str(self.current_wp)+" waypoint...")
        point, pointType, purpouse = self.waypoints[self.current_wp-1]
        #Sending goal to the move_base action server
        goal = MoveBase.Goal()
        goal.target.header.frame_id = "map" 
        goal.target.pose.position = point.pose.pose.position
        goal.target.pose.orientation = point.pose.pose.orientation

        self.move_base_client.send_goal_async(goal)
        self.move_base_client._get_result()  
        time.sleep(0.1)    
        
        self.ui_message_pub.publish(f"Point #{self.current_wp} successfully reached")
        self.ui_message_pub.publish(f"Doing some action #{purpouse} on {self.current_wp} point")

    def previous_wp_func(self):
        # Function to follow the previous waypoint in the route (Proposal: Fuse both functions into one, and use a parameter to determine if it is next or previous)
        self.get_logger().info("Following to previous waypoint...")
        self.ui_message_pub.publish("Following to previous waypoint...") 

        self.read_wp()
            
        self.current_wp -=1

        if(len(self.waypoints) == 0): 
            self.ui_message_pub.publish("The waypoint queue is empty.")
            return
        
        if(self.current_wp > len(self.waypoints)):  
            self.current_wp = 1
        
        self.ui_message_pub.publish("Following to "+str(self.current_wp)+" waypoint...")
        point, pointType, purpouse = self.waypoints[self.current_wp-1]

        goal = MoveBase.Goal()
        goal.target.header.frame_id = "map" 
        goal.target.pose.position = point.pose.pose.position
        goal.target.pose.orientation = point.pose.pose.orientation

        self.move_base_client.send_goal_async(goal)
        self.move_base_client._get_result()  
        time.sleep(0.1)    
        
        self.ui_message_pub.publish(f"Point #{self.current_wp} successfully reached")
        self.ui_message_pub.publish(f"Doing some action #{purpouse} on {self.current_wp} point")

    def home_func(self):
        # Sends the robot to the initial position
        self.get_logger().info("Following to home position")
        self.ui_message_pub.publish("Following to home position")
                
        goal = MoveBase.Goal()
        goal.target.header.frame_id =  "map" 
        goal.target.pose.position = self.home_pose.pose.position
        new_orientation = self.chang_point_dir(self.home_pose.pose.orientation)
        goal.target.pose.orientation.x =  new_orientation[0]
        goal.target.pose.orientation.y =  new_orientation[1]
        goal.target.pose.orientation.z =  new_orientation[2]
        goal.target.pose.orientation.w =  new_orientation[3]

        self.move_base_client.send_goal_async(goal) 
        self.move_base_client._get_result() 
        
    def stop_func(self):
        # Should be a flag to stop the current route
        self.get_logger().info("Canceling current route")
        self.ui_message_pub.publish(f"Canceling current route")

        self.break_mission = True
        #TODO
        '''try:
            if self.move_base_client.__getstate__() == GoalStatus.ACTIVE: #TODO find the ros2 equivalent to the goal status
                self.move_base_client.cancel_goal()

            if self.move_base_client.get_state() == actionlib.GoalStatus.PREEMPTED:
                print("Goal was canceled.")
        except Exception as e:  
            self.get_logger().info(f"ERROR in stop_func {e}")  '''

    def function(self, typeFunc):
        #Placeholder function for USer defined goals. This should look like a scratch interface where the user can use building blocks to make simple functions with logic statements and sensor control
        #The robot would then be able to execute this function when it reaches specific waypoints.
        self.ui_message_pub.publish(f"Function {typeFunc} will be done")
        # TODO: This whole segment (Weak priority tho)
        
    def ui_operation_callback(self, data:String):
        #Receives commands from the UI and executes the corresponding function
        print(f"ui_operation_callback : {data.data}")
        
        if(data.data == "follow_route") or (data.data == "start"):
            self.follow_func()                 
                                    
        elif data.data == "next_point":  
            self.next_wp_func()              
                                    
        elif data.data == "previous_point":  
            self.previous_wp_func()              
                                    
        elif data.data == "home":  
            self.home_func()           
                                    
        elif data.data == "stop": 
            self.stop_func()     
               
        else :
            command = data.data.split("_")
            if command[0] == "function":
                self.function(command[1])


def main():
    rclpy.init()
    controller = WayPointMover()
    rclpy.spin(controller)
        

if(__name__ == "__main__"):
   main()