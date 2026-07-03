#!/usr/bin/env python3
"""Filter LiDAR rays that geometrically hit the floor or ceiling.

When the drone pitches/rolls during flight the RPLIDAR (nominally horizontal)
tilts with it.  Rays angled downward hit the floor and get registered as
valid obstacles at wrong positions, corrupting the SLAM map and voxel map.

Algorithm (mirrors search_mission_controller lidar_ray_hits_floor_or_ceiling):
    ray_world = rotate(att_q, body_ray)        # body->NED rotation
    if ray_world.z > 0.05:                     # ray points toward floor (NED z+ down)
        floor_hit = AGL / ray_world.z
        if |range - floor_hit| < tolerance:    → replace with range_max
    if ray_world.z < -0.05:                    # ray points toward ceiling
        ceiling_hit = ceil_m / -ray_world.z
        if |range - ceiling_hit| < tolerance:  → replace with range_max

Subscribes:
    lidar/scan                   sensor_msgs/LaserScan  — raw 360° LiDAR
    px4_odom_topic               px4_msgs/VehicleOdometry — attitude quaternion + altitude

Publishes:
    lidar/scan_filtered          sensor_msgs/LaserScan  — cleaned scan (floor/ceiling rays → range_max)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleOdometry


def _rotate_vec(qw: float, qx: float, qy: float, qz: float,
                vx: float, vy: float, vz: float) -> tuple[float, float, float]:
    """Rotate vector (vx,vy,vz) by quaternion q (active rotation, body→world)."""
    # v' = v + 2*qw*(q×v) + 2*(q×(q×v)),  q = [qx,qy,qz]
    cx = qy * vz - qz * vy
    cy = qz * vx - qx * vz
    cz = qx * vy - qy * vx
    c2x = qy * cz - qz * cy
    c2y = qz * cx - qx * cz
    c2z = qx * cy - qy * cx
    return (vx + 2.0 * (qw * cx + c2x),
            vy + 2.0 * (qw * cy + c2y),
            vz + 2.0 * (qw * cz + c2z))


class LidarTiltFilter(Node):
    def __init__(self):
        super().__init__("lidar_tilt_filter")

        self.declare_parameter("match_tol_min",    0.12)   # m — minimum hit-match tolerance
        self.declare_parameter("match_tol_factor", 0.08)   # fraction of range added to tol
        self.declare_parameter("ceiling_m_fallback", 3.5)  # m — ceiling height when no rangefinder
        self.declare_parameter("px4_odom_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("output_frame_id", "")

        self._tol_min  = float(self.get_parameter("match_tol_min").value)
        self._tol_f    = float(self.get_parameter("match_tol_factor").value)
        self._ceil_m   = float(self.get_parameter("ceiling_m_fallback").value)
        self._px4_odom_topic = str(self.get_parameter("px4_odom_topic").value)
        self._output_frame_id = str(self.get_parameter("output_frame_id").value).strip()

        self._px4_odom: VehicleOdometry | None = None
        self._filt_rays = 0
        self._total_rays = 0

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(
            VehicleOdometry, self._px4_odom_topic,
            self._on_odom, qos_be)
        self.create_subscription(
            LaserScan, "lidar/scan",
            self._on_scan, qos_be)
        self._pub = self.create_publisher(LaserScan, "lidar/scan_filtered", qos_rel)

        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f"lidar_tilt_filter ready — tol={self._tol_min:.2f}m + {self._tol_f:.2f}×range, "
            f"ceiling_fallback={self._ceil_m:.1f}m, odom={self._px4_odom_topic}, "
            f"output_frame={self._output_frame_id or '<input>'}"
        )

    def _on_odom(self, msg: VehicleOdometry):
        self._px4_odom = msg

    def _on_scan(self, msg: LaserScan):
        if self._px4_odom is None:
            if self._output_frame_id:
                out = LaserScan()
                # Wall-clock stamp (see main path): the Gazebo bridge stamps scans
                # in sim time, but the whole stack runs use_sim_time=false, so SLAM's
                # scan→odom TF sync fails against the wall-time TF. Re-stamp to now().
                out.header.stamp = self.get_clock().now().to_msg()
                out.header.frame_id = self._output_frame_id
                out.angle_min = msg.angle_min
                out.angle_max = msg.angle_max
                out.angle_increment = msg.angle_increment
                out.time_increment = msg.time_increment
                out.scan_time = msg.scan_time
                out.range_min = msg.range_min
                out.range_max = msg.range_max
                out.ranges = msg.ranges
                out.intensities = msg.intensities
                self._pub.publish(out)
            else:
                self._pub.publish(msg)
            return

        q = self._px4_odom.q          # [qw, qx, qy, qz] body→NED
        qw = float(q[0]); qx = float(q[1])
        qy = float(q[2]); qz = float(q[3])

        # AGL: NED z is negative above ground
        agl = max(0.0, -float(self._px4_odom.position[2]))

        new_ranges = list(msg.ranges)
        angle = msg.angle_min

        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                # Body-frame ray direction (LiDAR is horizontal in body, FRD convention)
                bx = math.cos(angle)
                by = math.sin(angle)

                # Rotate to NED world frame
                wx, wy, wz = _rotate_vec(qw, qx, qy, qz, bx, by, 0.0)

                tol = max(self._tol_min, self._tol_f * r)

                if wz > 0.05 and agl > 0.05:
                    # Ray aimed downward → check floor hit
                    floor_hit = agl / wz
                    if abs(r - floor_hit) <= tol:
                        new_ranges[i] = msg.range_max
                        self._filt_rays += 1

                elif wz < -0.05:
                    # Ray aimed upward → check ceiling hit
                    ceiling_hit = self._ceil_m / (-wz)
                    if abs(r - ceiling_hit) <= tol:
                        new_ranges[i] = msg.range_max
                        self._filt_rays += 1

                self._total_rays += 1
            angle += msg.angle_increment

        out = LaserScan()
        # Re-stamp to wall clock so slam_toolbox (use_sim_time=false) can sync the
        # scan against odom_publisher's wall-time TF. The Gazebo bridge emits sim
        # time, which slam_toolbox silently dropped — the root cause of "map frame
        # does not exist" and the dead-reckoning drift between the two drones.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._output_frame_id or msg.header.frame_id
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = new_ranges
        out.intensities = msg.intensities
        self._pub.publish(out)

    def _log_stats(self):
        if self._total_rays > 0:
            pct = 100.0 * self._filt_rays / self._total_rays
            self.get_logger().info(
                f"lidar_tilt_filter: filtered {self._filt_rays}/{self._total_rays} "
                f"rays ({pct:.1f}%) in last 10 s"
            )
            self._filt_rays = 0
            self._total_rays = 0


def main(args=None):
    rclpy.init(args=args)
    node = LidarTiltFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
