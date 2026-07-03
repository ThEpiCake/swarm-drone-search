#!/usr/bin/env python3
"""Convert a single-beam LaserScan stream into sensor_msgs/Range."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    def __init__(self) -> None:
        super().__init__("scan_to_range")

        input_topic = self.declare_parameter("input_topic", "range/down_scan").value
        output_topic = self.declare_parameter("output_topic", "range/down").value
        default_fov = self.declare_parameter("default_field_of_view_rad", 0.01).value
        radiation_type = int(self.declare_parameter(
            "radiation_type", int(Range.INFRARED)).value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._default_fov = float(default_fov)
        self._radiation_type = radiation_type
        self._pub = self.create_publisher(Range, output_topic, qos)
        self._sub = self.create_subscription(LaserScan, input_topic, self._on_scan, qos)

    def _on_scan(self, msg: LaserScan) -> None:
        reading = math.inf
        if msg.ranges:
            center = len(msg.ranges) // 2
            candidates = [msg.ranges[center], *msg.ranges]
            for value in candidates:
                if math.isfinite(value) and value > 0.0:
                    reading = float(value)
                    break

        out = Range()
        out.header = msg.header
        out.radiation_type = self._radiation_type
        out.field_of_view = max(
            self._default_fov,
            abs(float(msg.angle_max) - float(msg.angle_min)),
        )
        out.min_range = float(msg.range_min)
        out.max_range = float(msg.range_max)
        out.range = reading
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanToRange()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
