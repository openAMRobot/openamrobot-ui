import os
import unittest
from unittest import mock

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)


def _description_installed():
    try:
        get_package_share_directory("openamrobot_description")
        return True
    except PackageNotFoundError:
        return False


class RobotDescriptionApiTest(unittest.TestCase):
    """The Robot Description page loads openamrobot_description from platform-sw."""

    def setUp(self):
        import openamr_ui_package.flask_app as flask_app

        self.flask_app = flask_app
        flask_app._robot_urdf_cache.update({"mtime": None, "xml": None})
        flask_app.app.config.update(TESTING=True)
        self.client = flask_app.app.test_client()

    def test_no_vendored_copy_is_installed(self):
        share = get_package_share_directory("openamr_ui_package")
        self.assertFalse(os.path.exists(os.path.join(share, "robot_description")))

    def test_missing_package_reports_unavailable(self):
        def missing(name):
            raise PackageNotFoundError(name)

        with mock.patch.object(self.flask_app, "get_package_share_directory", missing):
            manifest = self.client.get("/api/robot-description/manifest").get_json()
            self.assertFalse(manifest["available"])
            self.assertEqual(manifest["package"], "openamrobot_description")
            self.assertEqual(self.client.get("/api/robot-description/urdf").status_code, 404)
            self.assertEqual(
                self.client.get(
                    "/api/robot-description/assets/meshes/visual/base_link.STL"
                ).status_code,
                404,
            )

    @unittest.skipUnless(_description_installed(), "openamrobot_description not sourced")
    def test_serves_platform_sw_description(self):
        manifest = self.client.get("/api/robot-description/manifest").get_json()
        self.assertTrue(manifest["available"])

        response = self.client.get("/api/robot-description/urdf")
        self.assertEqual(response.status_code, 200)
        urdf = response.get_data(as_text=True)
        # REP 105 root and the 2.0 sensor frames from platform-sw PR #55.
        for link in (
            "base_footprint", "base_link", "lidar_link", "base_camera_link",
            "camera_link", "imu_link", "us_front_left", "us_front_right",
            "us_rear_left", "us_rear_right", "tof_front", "tof_rear",
        ):
            self.assertIn(f'<link name="{link}"', urdf)

        mesh = self.client.get("/api/robot-description/assets/meshes/visual/base_link.STL")
        self.assertEqual(mesh.status_code, 200)
        self.assertGreater(len(mesh.get_data()), 0)

    @unittest.skipUnless(_description_installed(), "openamrobot_description not sourced")
    def test_asset_path_traversal_is_rejected(self):
        # Deep enough to reach / from any share directory, plain and
        # percent-encoded; /etc/passwd exists on every Linux host.
        for path in (
            "../" * 30 + "etc/passwd",
            "%2e%2e/" * 30 + "etc/passwd",
            "urdf/" + "%2e%2e/" * 30 + "etc/passwd",
        ):
            response = self.client.get(f"/api/robot-description/assets/{path}")
            self.assertEqual(response.status_code, 404, path)


class BatteryMonitorTest(unittest.TestCase):
    """battery.py must never open a serial device unless a port is named."""

    def _make(self, args=None):
        import rclpy
        from std_msgs.msg import Float32  # noqa: F401
        import openamr_ui_package.battery as battery

        rclpy.init(args=args)
        self.addCleanup(rclpy.shutdown)
        node = battery.BatteryMonitor()
        self.addCleanup(node.destroy_node)
        return battery, node

    def test_no_port_never_opens_serial_and_publishes_simulated(self):
        with mock.patch("serial.Serial") as serial_ctor:
            _, node = self._make()
            serial_ctor.assert_not_called()
        published = []
        node.batterypub = mock.Mock(publish=lambda msg: published.append(msg.data))
        node.read_battery_status()
        self.assertEqual(len(published), 1)
        self.assertLess(published[0], 100.0)

    def test_named_port_is_the_only_device_opened(self):
        with mock.patch("serial.Serial") as serial_ctor:
            self._make(args=["--ros-args", "-p", "port:=/dev/fake_battery"])
            serial_ctor.assert_called_once_with("/dev/fake_battery", 9600, timeout=1)

    def test_entry_point_is_registered(self):
        from importlib.metadata import entry_points

        scripts = {
            ep.name: ep.value
            for ep in entry_points(group="console_scripts")
            if ep.value.startswith("openamr_ui_package.")
        }
        self.assertEqual(scripts.get("battery"), "openamr_ui_package.battery:main")


if __name__ == "__main__":
    unittest.main()
