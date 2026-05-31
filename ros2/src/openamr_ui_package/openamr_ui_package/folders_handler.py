#!/usr/bin/env python3
import webbrowser
import rclpy
from rclpy.node import Node
import os
import shutil
import json
import time
import subprocess
import yaml
import csv
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from openamr_ui_msgs.msg import ArrayPoseStampedWithCovariance
from nav2_msgs.srv import LoadMap


class UIFoldersHandler(Node):
    def __init__(self):
        super().__init__('ui_folders')
        self.WPs = []
        self.waypoints = []
        self.position = 0

        self.get_logger().info("------------ UI folders handler started ------------")

        config_file = os.path.join(get_package_share_directory('openamr_ui_package'), 'param', 'config.yaml')
        with open(config_file, 'r') as file:
            data = yaml.safe_load(file) or {}
        self.local_ip = data["flask_app"]["ros__parameters"]["appAddress"]
        self.local_port = data["flask_app"]["ros__parameters"]["portApp"]

        self.odomsub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.uiopsub = self.create_subscription(String, "ui_operation", self.ui_callback, 10)
        self.waysub = self.create_subscription(PoseWithCovarianceStamped, "/new_way_point", self.new_way_point_callback, 10)
        self.navsub = self.create_subscription(Empty, "/nav_data_req", self.nav_data_callback, 10)
        self.reqsub = self.create_subscription(Empty, "WP_req", self.WP_req_callback, 10)

        self.ui_pub = self.create_publisher(String, 'ui_message', 1)
        self.poseArray_publisher = self.create_publisher(ArrayPoseStampedWithCovariance, "/WayPoints_topic", 1)
        self.set_pose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.nav_data_pub = self.create_publisher(String, 'nav_data_resp', 1)

        package_share_dir = get_package_share_directory('openamr_ui_package')
        self.maps_folder = os.path.join(package_share_dir, 'maps')
        self.routs_folder = os.path.join(package_share_dir, 'paths')
        self.current_files = os.path.join(package_share_dir, 'param/current_map_route.yaml')

        try:
            with open(self.current_files, 'r') as file:
                cur_data = yaml.safe_load(file) or {}

            needs_update = False
            if cur_data:
                map_path = cur_data.get("map_file", "")
                route_path = cur_data.get("route_file", "")

                if not map_path or "darkadius" in map_path or not os.path.exists(map_path):
                    local_welcome_map = os.path.join(self.maps_folder, "Welcome/Start.yaml")
                    if os.path.exists(local_welcome_map):
                        cur_data["map_file"] = local_welcome_map
                        needs_update = True

                if not route_path or "darkadius" in route_path or not os.path.exists(route_path):
                    local_welcome_route = os.path.join(self.routs_folder, "Welcome/Start/Building.csv")
                    if os.path.exists(local_welcome_route):
                        cur_data["route_file"] = local_welcome_route
                        needs_update = True
                    else:
                        cur_data["route_file"] = ""
                        needs_update = True

            if needs_update:
                with open(self.current_files, 'w') as file:
                    yaml.dump(cur_data, file)
                self.get_logger().info("Automatically corrected current_map_route.yaml paths.")
        except Exception as e:
            self.get_logger().error(f"Error checking current_map_route.yaml: {e}")

        self.mappingCmd = "openamr_ui_package mapping_launch.py"
        self.navigationCmd = "openamr_ui_package navigation_launch.py"
        self.dict_cmd = None

        # Connect to the map_server already managed by Nav2's lifecycle manager.
        self.change_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.get_logger().info('Checking map_server availability...')
        if self.change_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('CONNECTED to map_server')
        else:
            self.get_logger().warn('map_server/load_map is not available yet; map changes will retry later.')
        webbrowser.open(f"http://{self.local_ip}:{self.local_port}")

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _pub(self, text: str):
        self.ui_pub.publish(String(data=text))

    def _pub_nav_data(self):
        self.nav_data_pub.publish(String(data=self.get_paths()))

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, data: Odometry):
        self.position = data.pose.pose

    def nav_data_callback(self, data: Empty):
        time.sleep(0.5)
        self._pub_nav_data()

    def new_way_point_callback(self, data: PoseWithCovarianceStamped):
        line  = f"{data.pose.pose.position.x},"
        line += f"{data.pose.pose.position.y},"
        line += f"{data.pose.pose.position.z},"
        line += f"{data.pose.pose.orientation.x},"
        line += f"{data.pose.pose.orientation.y},"
        line += f"{data.pose.pose.orientation.z},"
        line += f"{data.pose.pose.orientation.w},"
        line += f"{data.pose.covariance[0]},"
        line += f"{data.pose.covariance[1]},"
        line += f"{data.pose.covariance[2]}"
        if line not in self.WPs:
            self.WPs.append(line)
            count = len(self.WPs)
            self._pub("1 waypoint added to the route" if count == 1 else f"{count} waypoints added to the route")
        else:
            self.get_logger().info("Waypoint already exists in the route")

    def convert_PoseArray(self, waypoints: list):
        poses = PoseArray()
        poses.header.frame_id = 'map'
        poses.poses = [pose.pose.pose for pose, purpose in waypoints]
        return poses

    def convert_PoseWithCovArray_to_PoseArrayCov(self, waypoints: list):
        poses = ArrayPoseStampedWithCovariance()
        for pose_arg, purpose in waypoints:
            poses.poses.append(pose_arg)
        return poses

    def WP_req_callback(self, data: Empty):
        time.sleep(0.5)
        self.read_wp()
        self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))

    def read_wp(self):
        route_file = os.path.expanduser(self.get_cur_files().get("route_file", ""))
        del self.waypoints[:]
        if route_file:
            if not os.path.isfile(route_file):
                self.get_logger().warn(f"Route file does not exist: {route_file}")
            else:
                with open(route_file, 'r') as file:
                    reader = csv.reader(file, delimiter=',')
                    for row_number, line in enumerate(reader, start=1):
                        if len(line) < 11:
                            self.get_logger().warn(f"Skipping malformed waypoint row {row_number} in {route_file}")
                            continue
                        try:
                            values = [float(value) for value in line[:11]]
                        except ValueError:
                            self.get_logger().warn(f"Skipping non-numeric waypoint row {row_number} in {route_file}")
                            continue

                        current_pose = PoseWithCovarianceStamped()
                        current_pose.header.frame_id = 'map'
                        current_pose.pose.pose.position.x = values[0]
                        current_pose.pose.pose.position.y = values[1]
                        current_pose.pose.pose.position.z = values[2]
                        current_pose.pose.pose.orientation.x = values[3]
                        current_pose.pose.pose.orientation.y = values[4]
                        current_pose.pose.pose.orientation.z = values[5]
                        current_pose.pose.pose.orientation.w = values[6]
                        current_pose.pose.covariance[0] = values[7]
                        current_pose.pose.covariance[1] = values[8]
                        current_pose.pose.covariance[2] = values[9]
                        self.waypoints.append((current_pose, values[10]))

        if not self.waypoints:
            self._pub("The waypoint queue is empty.")

    def get_paths(self):
        files = {}
        if not os.path.isdir(self.routs_folder):
            self.get_logger().warn(f"Routes folder does not exist: {self.routs_folder}")
            return json.dumps({"structure": [], "active_files": {"group": "Null", "map": "Null", "route": "Null"}})

        for group in os.listdir(self.routs_folder):
            group_path = os.path.join(self.routs_folder, group)
            if not os.path.isdir(group_path):
                continue
            files[group] = []
            for map_name in os.listdir(group_path):
                map_path = os.path.join(group_path, map_name)
                if os.path.isdir(map_path):
                    files[group].append({map_name: os.listdir(map_path)})

        data = self.get_cur_files()
        route_file = data.get("route_file", "")
        map_file = data.get("map_file", "")
        if route_file:
            route_parts = os.path.expanduser(route_file).split("/")[-3:]
            if len(route_parts) == 3:
                group, map_name, route = route_parts[0], route_parts[1], route_parts[2].split(".")[0]
            else:
                group, map_name, route = "Null", "Null", "Null"
        elif map_file:
            map_parts = os.path.expanduser(map_file).split("/")[-2:]
            if len(map_parts) == 2:
                group, map_name, route = map_parts[0], map_parts[1].split(".")[0], "Null"
            else:
                group, map_name, route = "Null", "Null", "Null"
        else:
            group, map_name, route = "Null", "Null", "Null"

        response = {"structure": [], "active_files": {"group": group, "map": map_name, "route": route}}
        for i, j in files.items():
            response["structure"].append({i: j})
        return json.dumps(response)

    # ── Map commands ─────────────────────────────────────────────────────────

    def build_map_func(self):
        try:
            self._pub("Mapping...")
            self.poseArray_publisher.publish(ArrayPoseStampedWithCovariance())
            os.system("ros2 lifecycle set /amcl shutdown")
            os.system("ros2 lifecycle set /move_base shutdown")
            os.system("ros2 lifecycle set /map_server shutdown")
            time.sleep(1)
            subprocess.Popen(f"ros2 launch {self.mappingCmd}", stdout=subprocess.PIPE,
                             shell=True, preexec_fn=os.setsid)
            time.sleep(3)
            self._pub("Move the robot along the perimeter of the room and return to start position")
        except Exception as e:
            self.get_logger().info(f"Error in build_map_func: {e}")

    def save_map_func(self):
        try:
            self._pub("Saving map...")
            map_path_to_save = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_folder_path_to_save = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            map_save_command = "ros2 run map_server map_saver -f "
            cmd = f"{map_save_command}{map_path_to_save}"
            self.get_logger().info(cmd)
            os.system(cmd)

            os.mkdir(route_folder_path_to_save)
            self.set_cur_route("")
            self.WP_req_callback(Empty())
            self._pub("Map saved")

            map_yaml_file = f"{map_path_to_save}.yaml"
            self.set_cur_map(map_path_to_save)

            current = self.position

            os.system("ros2 lifecycle set /slam_gmapping 5")
            os.system("ros2 lifecycle set /map_server 5")

            self._pub("Localization...")
            time.sleep(1)
            subprocess.Popen(f"ros2 launch {self.navigationCmd}", stdout=subprocess.PIPE,
                             shell=True, preexec_fn=os.setsid)

            # Nav2 owns /map_server. Starting a UI-owned map_server here creates
            # duplicate /map_server nodes and breaks AMCL lifecycle management.
            self.change_map(map_yaml_file, yaml=True)
            time.sleep(1)

            current_pose = PoseWithCovarianceStamped()
            current_pose.header.frame_id = "map"
            current_pose.pose.pose = current
            self.set_pose.publish(current_pose)
            self._pub("You can set points or follow the route")
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in save_map_func: {e}")

    def change_map_func(self):
        path_to_new_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
        self.change_map(path_to_new_map)

    def create_group_func(self):
        try:
            os.mkdir(f"{self.routs_folder}/{self.dict_cmd['group']}")
            os.mkdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in create_group_func: {e}")

    def rename_map_func(self):
        try:
            old_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"
            new_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}"
            old_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"
            new_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}"
            old_ros_folder_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}_ros"
            new_ros_folder_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}_ros"

            with open(f"{old_map_file}.yaml", 'r') as file:
                map_cur_path = yaml.safe_load(file) or {}
            with open(f"{old_ros_folder_file}.yaml", 'r') as file:
                ros_cur_path = yaml.safe_load(file) or {}

            map_cur_path["image"] = f"{new_map_file}.png"
            if "map_server" in ros_cur_path and "ros__parameters" in ros_cur_path["map_server"]:
                ros_cur_path["map_server"]["ros__parameters"]["yaml_filename"] = f"{self.dict_cmd['map_new']}.yaml"
            else:
                ros_cur_path["yaml_filename"] = f"{self.dict_cmd['map_new']}.yaml"

            with open(f"{old_ros_folder_file}.yaml", 'w') as file:
                yaml.dump(ros_cur_path, file)
            with open(f"{old_map_file}.yaml", 'w') as file:
                yaml.dump(map_cur_path, file)

            os.rename(f"{old_map_file}.yaml", f"{new_map_file}.yaml")
            os.rename(f"{old_map_file}.png", f"{new_map_file}.png")
            os.rename(f"{old_ros_folder_file}.yaml", f"{new_ros_folder_file}.yaml")
            os.rename(old_route_folder_file, new_route_folder_file)

            data = self.get_cur_files()
            if os.path.expanduser(data["map_file"]) == f"{old_map_file}.yaml":
                self.set_cur_map(new_map_file)
                self.set_cur_route(f"{new_route_folder_file}/{data['route_file'].split('/')[-1].split('.')[0]}")

            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in rename_map_func: {e}")

    def delete_map_func(self):
        try:
            map_to_delete = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            route_map_folder_to_delete = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
            os.remove(f"{map_to_delete}.yaml")
            os.remove(f"{map_to_delete}_ros.yaml")
            os.remove(f"{map_to_delete}.png")
            shutil.rmtree(route_map_folder_to_delete)

            maps_in_group = os.listdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            if maps_in_group:
                path_to_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}"
                self.set_cur_map(path_to_map)
                self.change_map(path_to_map, True)

                routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}")
                if routes_on_map:
                    path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}"
                    self.set_cur_route(path_to_route)
                else:
                    self.set_cur_route("")
                    self._pub("No routes on the map")
            else:
                self.set_cur_route("")
                self.set_cur_map("")
                self._pub("No maps in the group")

            self.WP_req_callback(Empty())
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in delete_map_func: {e}")

    def delete_group_func(self):
        try:
            shutil.rmtree(f"{self.maps_folder}/{self.dict_cmd['group']}")
            shutil.rmtree(f"{self.routs_folder}/{self.dict_cmd['group']}")

            groups_in_folder = os.listdir(self.routs_folder)
            if groups_in_folder:
                path_to_map = f"{self.maps_folder}/{groups_in_folder[0].split('.')[0]}"
                maps_in_group = os.listdir(path_to_map)
                if maps_in_group:
                    path_to_map = f"{path_to_map}/{maps_in_group[0].split('.')[0]}"
                    self.set_cur_map(path_to_map)
                    self.change_map(path_to_map, True)

                    routes_on_map = os.listdir(f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}")
                    if routes_on_map:
                        path_to_route = f"{self.routs_folder}/{groups_in_folder[0].split('.')[0]}/{maps_in_group[0].split('.')[0]}/{routes_on_map[0].split('.')[0]}"
                        self.set_cur_route(path_to_route)
                    else:
                        self.set_cur_route("")
                        self._pub("No routes on the map")
                else:
                    self.set_cur_map("")
                    self.set_cur_route("")
                    self._pub("No maps in the group")
            else:
                self.set_cur_map("")
                self.set_cur_route("")
                self._pub("No maps in the group")

            self.WP_req_callback(Empty())
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in delete_group_func: {e}")

    # ── Route commands ────────────────────────────────────────────────────────

    def clear_route_func(self):
        try:
            del self.WPs[:]
            self._pub("Waypoints cleared, please set new points on the map")
        except Exception as e:
            self.get_logger().info(f"Error in clear_route_func: {e}")

    def save_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}"
            with open(f"{path_to_route}.csv", 'w') as file:
                for WP in self.WPs:
                    file.write(WP + ", 1\n")
            self._pub(f"{len(self.WPs)} waypoints saved")
            self.set_cur_route(path_to_route)
            del self.WPs[:]
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in save_route_func: {e}")

    def edit_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route'].split('.')[0]}"
            self.set_cur_route(path_to_route)
            self.read_wp()
            for point, purpose in self.waypoints:
                if purpose == 2:
                    continue
                line = f"{point.pose.pose.position.x},{point.pose.pose.position.y},{point.pose.pose.position.z},{point.pose.pose.orientation.x},{point.pose.pose.orientation.y},{point.pose.pose.orientation.z},{point.pose.pose.orientation.w},{point.pose.covariance[0]},{point.pose.covariance[1]},{point.pose.covariance[2]}"
                if line not in self.WPs:
                    self.WPs.append(line)
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in edit_route_func: {e}")

    def delete_route_func(self):
        try:
            file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}.csv"
            os.remove(file)
            routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}")
            if routes_on_map:
                path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}"
                self.set_cur_route(path_to_route)
            else:
                self.set_cur_route("")
                self._pub("No routes on the map")
            self.WP_req_callback(Empty())
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in delete_route_func: {e}")

    def change_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}"
            self.set_cur_route(path_to_route)
            self.read_wp()
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in change_route_func: {e}")

    def rename_route_func(self):
        try:
            old_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_old']}.csv"
            new_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_new']}.csv"
            os.rename(old_file, new_file)
            self.set_cur_route(new_file.split(".")[0])
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in rename_route_func: {e}")

    def rename_group_func(self):
        try:
            old_maps = f"{self.maps_folder}/{self.dict_cmd['group_old']}"
            new_maps = f"{self.maps_folder}/{self.dict_cmd['group_new']}"
            old_routes = f"{self.routs_folder}/{self.dict_cmd['group_old']}"
            new_routes = f"{self.routs_folder}/{self.dict_cmd['group_new']}"
            os.rename(old_maps, new_maps)
            os.rename(old_routes, new_routes)

            data = self.get_cur_files()
            if self.dict_cmd['group_old'] in data.get("map_file", ""):
                data["map_file"] = data["map_file"].replace(old_maps, new_maps)
            if self.dict_cmd['group_old'] in data.get("route_file", ""):
                data["route_file"] = data["route_file"].replace(old_routes, new_routes)
            with open(self.current_files, 'w') as file:
                yaml.dump(data, file)

            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in rename_group_func: {e}")

    # ── UI command dispatcher ─────────────────────────────────────────────────

    def ui_callback(self, data: String):
        self.get_logger().info(f"COMMAND RECEIVED: {data}")
        try:
            command = data.data.split("/")
            if len(command) > 1:
                self.dict_cmd = json.loads(command[1])

            dispatch = {
                "build_map":    self.build_map_func,
                "save_map":     self.save_map_func,
                "change_map":   self.change_map_func,
                "create_group": self.create_group_func,
                "rename_map":   self.rename_map_func,
                "delete_map":   self.delete_map_func,
                "delete_group": self.delete_group_func,
                "clear_route":  self.clear_route_func,
                "save_route":   self.save_route_func,
                "edit_route":   self.edit_route_func,
                "delete_route": self.delete_route_func,
                "change_route": self.change_route_func,
                "rename_route": self.rename_route_func,
                "rename_group": self.rename_group_func,
            }
            fn = dispatch.get(command[0])
            if fn:
                self.get_logger().info(command[0])
                fn()
            else:
                self.get_logger().warn(f"Unknown UI command: {command[0]}")
        except Exception as e:
            self.get_logger().error(f"Error in ui_callback: {e}")

    # ── Map service helpers ───────────────────────────────────────────────────

    def change_map(self, map_name: str, yaml: bool = False, manual: bool = False):
        self.get_logger().info(f"\n ==========[CHANGING MAP TO {map_name}]======== \n")
        map_yaml_file = map_name if yaml else f"{map_name}.yaml"
        self.set_cur_map(map_yaml_file)

        if manual:
            parts = map_name.split("/")
            self.dict_cmd['group'] = parts[-2]
            self.dict_cmd['map'] = parts[-1].split(".")[0]

        route_dir = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
        routes_on_map = os.listdir(route_dir) if os.path.isdir(route_dir) else []
        if routes_on_map:
            path_to_route = f"{route_dir}/{routes_on_map[0].split('.')[0]}"
            self.set_cur_route(path_to_route)
        else:
            self.set_cur_route("")
            self._pub("No routes on the map")

        self.WP_req_callback(Empty())

        req = LoadMap.Request()
        req.map_url = map_yaml_file
        if self.change_map_cli.service_is_ready() or self.change_map_cli.wait_for_service(timeout_sec=1.0):
            self.change_map_cli.call_async(req)
        else:
            self.get_logger().warn("Cannot load map because /map_server/load_map is unavailable.")
            self._pub("Map server is unavailable; map selection was saved but not loaded.")
        self._pub_nav_data()

    def set_cur_map(self, map_name: str):
        data = self.get_cur_files()
        data["map_file"] = map_name
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)

    def set_cur_route(self, route_name: str):
        data = self.get_cur_files()
        data["route_file"] = f"{route_name}.csv" if route_name else ""
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)

    def get_cur_files(self):
        with open(self.current_files, 'r') as file:
            return yaml.safe_load(file) or {"map_file": "", "route_file": ""}


def main():
    rclpy.init()
    controller = UIFoldersHandler()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
