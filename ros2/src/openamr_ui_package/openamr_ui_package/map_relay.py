#!/usr/bin/env python3
"""
Relay /map from TRANSIENT_LOCAL to /ui/map as VOLATILE so rosbridge
receives the latched map. Republishes every 2 s to cover late-connecting clients.
Nav2 nodes (TRANSIENT_LOCAL subscribers) are unaffected — they only connect to
the map_server's TRANSIENT_LOCAL publisher, not this VOLATILE relay.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid


class MapVolatileRelay(Node):
    def __init__(self):
        super().__init__('map_volatile_relay')

        tl_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        vol_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._map: OccupancyGrid | None = None

        self._sub = self.create_subscription(OccupancyGrid, '/map', self._cb, tl_qos)
        self._pub = self.create_publisher(OccupancyGrid, '/ui/map', vol_qos)
        self._timer = self.create_timer(2.0, self._republish)

        self.get_logger().info('map_volatile_relay started: /map -> /ui/map')

    def _cb(self, msg: OccupancyGrid) -> None:
        self._map = msg
        self._pub.publish(msg)

    def _republish(self) -> None:
        if self._map is not None:
            self._pub.publish(self._map)


def main() -> None:
    rclpy.init()
    node = MapVolatileRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
