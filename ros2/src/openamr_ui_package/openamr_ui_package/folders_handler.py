#!/usr/bin/env python3
import webbrowser
import rclpy
from rclpy.node import Node
import os
import shutil
import json
import time
import yaml
import csv
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav2_msgs.srv import LoadMap, SaveMap
from std_msgs.msg import String, Empty
from openamr_ui_msgs.msg import ArrayPoseStampedWithCovariance
from openamr_ui_package import map_files

# Mapping and navigation belong to the platform. This node never launches
# them and never changes the lifecycle state of platform nodes; it only
# calls the platform's map services (openAMRobot/openamrobot-ui#32).
MAPPING_NOT_STARTED_BY_UI = (
    'Mapping is not started by this UI. Start mapping on the robot platform, '
    'drive the robot around the area, then save the map here.'
)


class UIFoldersHandler(Node):
    def __init__(self):
        super().__init__('ui_folders')
        self.WPs = []
        self.waypoints = []

        self.get_logger().info("------------ UI folders handler started ------------")

        config_file = os.path.join(get_package_share_directory('openamr_ui_package'), 'param', 'config.yaml')
        with open(config_file, 'r') as file:
            data = yaml.safe_load(file) or {}
        self.local_ip = data["ui/flask_app"]["ros__parameters"]["appAddress"]
        self.local_port = data["ui/flask_app"]["ros__parameters"]["portApp"]

        self.uiopsub = self.create_subscription(String, "ui_operation", self.ui_callback, 10)
        self.waysub = self.create_subscription(PoseWithCovarianceStamped, "/new_way_point", self.new_way_point_callback, 10)
        self.navsub = self.create_subscription(Empty, "/nav_data_req", self.nav_data_callback, 10)
        self.reqsub = self.create_subscription(Empty, "WP_req", self.WP_req_callback, 10)

        self.ui_pub = self.create_publisher(String, 'ui_message', 1)
        self.poseArray_publisher = self.create_publisher(ArrayPoseStampedWithCovariance, "/WayPoints_topic", 1)
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

        self.dict_cmd = None

        # Connect to the map_server already managed by Nav2's lifecycle manager.
        self.change_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.get_logger().info('Checking map_server availability...')
        if self.change_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('CONNECTED to map_server')
        else:
            self.get_logger().warn('map_server/load_map is not available yet; map changes will retry later.')

        # Platform map saver (nav2 map_saver_server). The service name is a
        # parameter because the platform owns it; the default is the Nav2
        # standard name.
        self.declare_parameter('map_save_service', '/map_saver/save_map')
        save_service = self.get_parameter('map_save_service').get_parameter_value().string_value
        self.save_map_cli = self.create_client(SaveMap, save_service)
        webbrowser.open(f"http://{self.local_ip}:{self.local_port}")

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _pub(self, text: str):
        self.ui_pub.publish(String(data=text))

    def _pub_nav_data(self):
        self.nav_data_pub.publish(String(data=self.get_paths()))

    # ── Callbacks ─────────────────────────────────────────────────────────────

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
        # The platform starts and stops mapping. The UI launches nothing and
        # leaves AMCL, map_server and the controller in their current state.
        self._pub(MAPPING_NOT_STARTED_BY_UI)

    def save_map_func(self):
        group = self.dict_cmd['group']
        map_name = self.dict_cmd['map']
        map_path_to_save = f'{self.maps_folder}/{group}/{map_name}'
        if not self.save_map_cli.service_is_ready():
            self.get_logger().warn(
                f'Map save service {self.save_map_cli.srv_name} is unavailable.')
            self._pub('The platform map save service is not available, so the map was not saved. '
                      'Start the platform map saver and try again.')
            return

        self._pub('Saving map...')
        req = SaveMap.Request()
        req.map_topic = '/map'
        req.map_url = map_path_to_save
        req.image_format = 'png'
        req.map_mode = 'trinary'
        future = self.save_map_cli.call_async(req)
        future.add_done_callback(
            lambda done: self._on_map_saved(done, group, map_name, map_path_to_save))

    def _on_map_saved(self, future, group, map_name, map_path_to_save):
        try:
            response = future.result()
        except Exception as e:
            response = None
            self.get_logger().error(f'Map save call failed: {e}')
        if response is None or not response.result:
            self._pub('The platform could not save the map.')
            return
        try:
            route_folder_path_to_save = f'{self.routs_folder}/{group}/{map_name}'
            os.makedirs(route_folder_path_to_save, exist_ok=True)
            self.dict_cmd = {'group': group, 'map': map_name}
            self.set_cur_route('')
            self.WP_req_callback(Empty())
            self._pub('Map saved')
            # Load the saved map into the platform map_server through its
            # load_map service. Localization stays with the platform.
            self.change_map(f'{map_path_to_save}.yaml', yaml=True)
        except Exception as e:
            self.get_logger().info(f'Error after saving map: {e}')

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

    # Map files (<name>.yaml, its image, an optional <name>_ros.yaml) are
    # renamed and deleted by map_files.py, which refuses an image outside the
    # map's folder and keeps an image another map also uses.

    def _first_map_in(self, group_dir):
        """Return the first map name in group_dir, ignoring images and _ros files."""
        names = sorted(f[:-len('.yaml')] for f in os.listdir(group_dir)
                       if f.endswith('.yaml') and not f.endswith('_ros.yaml'))
        return names[0] if names else None

    def rename_map_func(self):
        try:
            group = self.dict_cmd['group']
            old, new = self.dict_cmd['map_old'], self.dict_cmd['map_new']
            old_map_file = f'{self.maps_folder}/{group}/{old}'
            new_map_file = f'{self.maps_folder}/{group}/{new}'
            old_route_folder_file = f'{self.routs_folder}/{group}/{old}'
            new_route_folder_file = f'{self.routs_folder}/{group}/{new}'
            try:
                notes = map_files.rename_map(old_map_file, new_map_file)
            except map_files.MapFileError as refused:
                self._pub(f'Nothing was renamed: {refused}.')
                return
            for note in notes:
                self._pub(note)
            if os.path.isdir(old_route_folder_file):
                os.rename(old_route_folder_file, new_route_folder_file)

            data = self.get_cur_files()
            if os.path.expanduser(data.get('map_file', '')) == f'{old_map_file}.yaml':
                self.set_cur_map(f'{new_map_file}.yaml')
                route = data.get('route_file', '')
                route_name = route.split('/')[-1].split('.')[0]
                self.set_cur_route(f'{new_route_folder_file}/{route_name}' if route else '')

            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f'Error in rename_map_func: {e}')
            self._pub(f'The map could not be renamed: {e}')

    def delete_map_func(self):
        try:
            group = self.dict_cmd['group']
            map_to_delete = f"{self.maps_folder}/{group}/{self.dict_cmd['map']}"
            route_map_folder_to_delete = (
                f"{self.routs_folder}/{group}/{self.dict_cmd['map']}")
            try:
                notes = map_files.delete_map(map_to_delete)
            except map_files.MapFileError as refused:
                self._pub(f'Nothing was deleted: {refused}.')
                return
            for note in notes:
                self._pub(note)
            if os.path.isdir(route_map_folder_to_delete):
                shutil.rmtree(route_map_folder_to_delete)

            first = self._first_map_in(f'{self.maps_folder}/{group}')
            if first:
                path_to_map = f'{self.maps_folder}/{group}/{first}'
                self.set_cur_map(path_to_map)
                self.dict_cmd = {'group': group, 'map': first}
                self.change_map(path_to_map)
            else:
                self.set_cur_route('')
                self.set_cur_map('')
                self._pub('No maps in the group')

            self.WP_req_callback(Empty())
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f'Error in delete_map_func: {e}')
            self._pub(f'The map could not be deleted: {e}')

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
            future = self.change_map_cli.call_async(req)
            future.add_done_callback(lambda done: self._on_map_loaded(done, map_yaml_file))
        else:
            self.get_logger().warn("Cannot load map because /map_server/load_map is unavailable.")
            self._pub("Map server is unavailable; map selection was saved but not loaded.")
        self._pub_nav_data()

    def _on_map_loaded(self, future, map_yaml_file):
        try:
            response = future.result()
        except Exception as e:
            self._pub(f'The platform map server could not load {map_yaml_file}: {e}')
            return
        if response is None or response.result != LoadMap.Response.RESULT_SUCCESS:
            code = getattr(response, 'result', None)
            self._pub(f'The platform map server could not load {map_yaml_file} '
                      f'(load_map result {code}).')
        else:
            self._pub('Map loaded')

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
