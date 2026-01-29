import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.timer = self.create_timer(1.0, self.read_battery_status)
        self.batterypub = self.create_publisher(Float32, 'battery_status', 10)

    def publish_battery_status(self, status:float):
        num = float(status)
        self.batterypub.publish(Float32(data=num))

    def read_battery_status(self):
        line = 0.0
        if self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()

                self.publish_battery_status(line)
                if line:
                    self.get_logger().info(f'Battery Status: {line}')
            except Exception as e:
                self.get_logger().error(f'Error reading battery status: {e}')
        else:
            self.get_logger().error('Serial port is not open')

def main():
    rclpy.init()
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()