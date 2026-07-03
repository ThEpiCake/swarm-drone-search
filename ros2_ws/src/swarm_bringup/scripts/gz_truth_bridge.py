#!/usr/bin/env python3
"""Gazebo ground-truth pose → PX4 VehicleOdometry bridge (SITL only).

Reads /model/<drone>/odometry_with_covariance from Gazebo Harmonic
(bridged via ros_gz_bridge as nav_msgs/Odometry) and publishes EV yaw
to /fmu/in/vehicle_visual_odometry at 30 Hz.

This gives EKF2 a yaw reference before arming, fixing the SITL drift
caused by the Gazebo Harmonic magnetometer bug (PR #2460).

Drop-in source behind the EV interface:
  SITL:     gz_truth_bridge.py  +  EKF2_EV_CTRL=8  (yaw from EV, GPS pos)
  Hardware: slam_to_px4.py      +  EKF2_EV_CTRL=15 (full 6-DOF from VIO)
Zero changes to the controller or voxel_mapper when switching.

ENU → NED conversions (same as slam_to_px4.py):
  heading_NED = pi/2 - yaw_ENU
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

NAN = float("nan")


class GzTruthBridge(Node):
    def __init__(self):
        super().__init__("gz_truth_bridge")

        self.declare_parameter(
            "odom_topic",
            "/model/x500_swarm_0/odometry_with_covariance")
        self.declare_parameter("publish_rate_hz", 30.0)

        topic = self.get_parameter("odom_topic").value
        rate  = float(self.get_parameter("publish_rate_hz").value)

        self._heading = None   # None until first Gazebo frame arrives
        self._ts_us   = 0

        self._pub = self.create_publisher(
            VehicleOdometry, "/fmu/in/vehicle_visual_odometry", 10)

        self.create_subscription(Odometry, topic, self._on_odom, 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"gz_truth_bridge ready  odom={topic}  rate={rate:.0f} Hz\n"
            "  EV interface: yaw only (EKF2_EV_CTRL=8) — GPS holds XY/Z position."
        )

    # ── Gazebo odometry callback ──────────────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Gazebo Harmonic uses ENU: yaw = counter-clockwise from East
        yaw_enu = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        # NED heading: clockwise from North
        self._heading = math.pi / 2.0 - yaw_enu

        s = msg.header.stamp
        self._ts_us = s.sec * 1_000_000 + s.nanosec // 1_000

        if not hasattr(self, '_first_logged'):
            self._first_logged = True
            self.get_logger().info(
                f"First odometry: yaw_ENU={math.degrees(yaw_enu):.1f}°  "
                f"heading_NED={math.degrees(self._heading):.1f}°  "
                f"ts={self._ts_us} μs")

    # ── 30 Hz publish timer ───────────────────────────────────────────────────
    def _publish(self):
        if self._heading is None:
            return   # no Gazebo frame yet

        h   = self._heading
        hw  = math.cos(h / 2.0)
        hz  = math.sin(h / 2.0)

        msg = VehicleOdometry()
        msg.timestamp        = self._ts_us
        msg.timestamp_sample = self._ts_us

        msg.pose_frame  = VehicleOdometry.POSE_FRAME_NED
        # EKF2_EV_CTRL=8: position bits (0,1) not set → EKF2 ignores position.
        msg.position[0] = NAN
        msg.position[1] = NAN
        msg.position[2] = NAN
        msg.position_variance[0] = 1.0
        msg.position_variance[1] = 1.0
        msg.position_variance[2] = 1.0

        # Yaw-only quaternion in NED: [cos(h/2), 0, 0, sin(h/2)]
        msg.q[0] = hw
        msg.q[1] = 0.0
        msg.q[2] = 0.0
        msg.q[3] = hz
        msg.orientation_variance[0] = 0.1   # roll  — not from EV, loose
        msg.orientation_variance[1] = 0.1   # pitch — not from EV, loose
        msg.orientation_variance[2] = 0.02  # yaw   — ground truth, tight (~1.1°)

        msg.velocity_frame     = VehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity[0]        = NAN
        msg.velocity[1]        = NAN
        msg.velocity[2]        = NAN
        msg.velocity_variance[0] = 0.1
        msg.velocity_variance[1] = 0.1
        msg.velocity_variance[2] = 0.1

        msg.reset_counter = 0
        msg.quality       = 100

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GzTruthBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
