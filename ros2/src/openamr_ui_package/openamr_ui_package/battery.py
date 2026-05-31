import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.serial_port = None
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Successfully connected to battery monitor on /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().warn(f"Could not open serial port /dev/ttyUSB0 ({e}). Falling back to dummy battery monitor.")

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