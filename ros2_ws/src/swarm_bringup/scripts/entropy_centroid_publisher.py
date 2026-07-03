#!/usr/bin/env python3
"""Entropy centroid publisher for Stage D entropic APF navigation.

Subscribes to the lidar_mapper's 2D OccupancyGrid and computes the centroid
of UNKNOWN cells (-1) within a local search radius R_sense around the drone.
Publishes the result as geometry_msgs/PointStamped (x=North, y=East) on
the 'entropy_centroid' topic, which apf_controller consumes as its attraction
goal.

All UNKNOWN cells have maximum binary entropy H=1 (uncertain occupancy), so
the centroid is simply the unweighted mean of their positions — equivalent to
the thesis entropy-weighted centroid (eq. 4.13) in the discrete binary-grid case.

Local minimum escape: if no UNKNOWN cells exist within R_sense, the radius is
expanded to R_max so the drone is attracted toward the nearest unexplored zone
rather than stalling.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleLocalPosition


class EntropyCentroidPublisher(Node):
    def __init__(self):
        super().__init__('entropy_centroid_publisher')

        self.r_sense = self.declare_parameter('r_sense_m', 3.0).value
        self.r_max   = self.declare_parameter('r_max_m',   8.0).value
        self.r_min   = self.declare_parameter('r_min_m',   1.5).value  # ignore cells too close (symmetry)

        self._map: OccupancyGrid | None = None
        self._pos: VehicleLocalPosition | None = None
        self._has_pos = False

        # Match lidar_mapper's transient-local reliable publisher
        qos_map = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_be = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, 'occupancy_map', self._on_map, qos_map)
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1',
            self._on_pos, qos_be)

        self.pub = self.create_publisher(PointStamped, 'entropy_centroid', 10)
        self.timer = self.create_timer(0.2, self._publish)  # 5 Hz

        self.get_logger().info(
            f'EntropyCentroidPublisher ready — r_sense={self.r_sense} m  r_max={self.r_max} m')

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_pos(self, msg: VehicleLocalPosition):
        self._pos = msg
        self._has_pos = msg.xy_valid

    def _publish(self):
        if self._map is None or not self._has_pos:
            return

        g = self._map
        res       = g.info.resolution
        origin_e  = g.info.origin.position.x   # East  origin (ROS x → East in our NED map)
        origin_n  = g.info.origin.position.y   # North origin (ROS y → North in our NED map)
        width     = g.info.width
        height    = g.info.height
        drone_n   = self._pos.x
        drone_e   = self._pos.y

        # Find the nearest UNKNOWN cell beyond r_min.
        # Using "nearest" instead of centroid avoids the symmetry problem where
        # uniformly distributed unknown cells cancel to the drone's own position.
        r_min2 = self.r_min * self.r_min
        r_max2 = self.r_max * self.r_max

        best_d2   = float('inf')
        best_n    = None
        best_e    = None

        for row in range(height):
            cell_n = origin_n + (row + 0.5) * res
            dn = cell_n - drone_n
            if dn * dn > r_max2:
                continue
            for col in range(width):
                if g.data[row * width + col] != -1:
                    continue   # not UNKNOWN
                cell_e = origin_e + (col + 0.5) * res
                de = cell_e - drone_e
                d2 = dn * dn + de * de
                if d2 < r_min2:
                    continue   # too close — skip to avoid symmetry cancellation
                if d2 < best_d2:
                    best_d2 = d2
                    best_n  = cell_n
                    best_e  = cell_e

        if best_n is not None:
            msg = PointStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map_ned'
            msg.point.x = best_n   # North (NED x)
            msg.point.y = best_e   # East  (NED y)
            msg.point.z = 0.0
            self.pub.publish(msg)
            return

        self.get_logger().info(
            'No UNKNOWN cells found in entire map — exploration complete',
            throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = EntropyCentroidPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
