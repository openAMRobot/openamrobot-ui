# Battery percentage publisher for the UI (Float32 on battery_status).
#
# On OpenAMRobot 2.0 the battery state comes from the base controller (Daly
# BMS on CAN2 via the STM32); the serial path in this node is Gate A legacy.
#
# The serial port is a parameter with no default: the node never opens a
# device unless one is named. /dev/ttyUSB0 is commonly the RPLIDAR on the
# robot, and opening it here would reconfigure and read the lidar's line.
# Without a port, or if the named port cannot be opened, the node publishes
# a simulated, slowly draining value for development only. The UI launch
# file starts this node only when start_battery_monitor:=true.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        port = self.declare_parameter('port', '').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 9600).get_parameter_value().integer_value
        self.serial_port = None
        if not port:
            self.get_logger().warn(
                "No battery serial port set (parameter 'port'). Publishing a "
                "SIMULATED battery value on battery_status for development only.")
        else:
            try:
                self.serial_port = serial.Serial(port, baud, timeout=1)
                self.get_logger().info(f"Connected to battery monitor on {port}")
            except Exception as e:
                self.get_logger().warn(
                    f"Could not open serial port {port} ({e}). Publishing a "
                    "SIMULATED battery value on battery_status for development only.")

        self.timer = self.create_timer(1.0, self.read_battery_status)
        self.batterypub = self.create_publisher(Float32, 'battery_status', 10)
        self.dummy_battery = 100.0

    def publish_battery_status(self, status):
        try:
            num = float(status)
            self.batterypub.publish(Float32(data=num))
        except ValueError:
            self.get_logger().warn(f"Could not convert battery reading '{status}' to float. Skipping.")

    def read_battery_status(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.publish_battery_status(line)
                    self.get_logger().info(f'Battery Status: {line}')
            except Exception as e:
                self.get_logger().error(f'Error reading battery status: {e}')
        else:
            # Fallback to simulated slow-draining battery
            self.dummy_battery = max(0.0, self.dummy_battery - 0.01)
            self.publish_battery_status(self.dummy_battery)

def main():
    rclpy.init()
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()