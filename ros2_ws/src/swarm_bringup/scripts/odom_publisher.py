#!/usr/bin/env python3
"""Publish PX4 NED position as ROS 2 ENU nav_msgs/Odometry and TF odom→base_frame.

slam_toolbox needs a standard ROS 2 TF chain (odom→base_frame) and optionally
/odom for dead-reckoning between scan matches.
PX4 works in NED; ROS standard is ENU.

NED→ENU: x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned, yaw_enu=π/2−heading_ned

Parameters
----------
child_frame_id : TF child frame (default "rplidar_link")
frame_prefix   : prepended to all frame IDs, e.g. "d1_" for drone 1 (default "")
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        self.declare_parameter("child_frame_id", "rplidar_link")
        self.declare_parameter("frame_prefix", "")
        prefix = self.get_parameter("frame_prefix").value
        self.odom_frame_  = f"{prefix}odom"
        self.child_frame_ = f"{prefix}{self.get_parameter('child_frame_id').value}"
        odom_topic = f"/{prefix}odom" if prefix else "/odom"

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub_ = self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position_v1",
            self._on_pos,
            qos,
        )
        self.odom_pub_ = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_br_ = TransformBroadcaster(self)

        # Velocity-integrated XY odometry — avoids the circular dependency where
        # EKF position (derived from SLAM external-vision) feeds back into SLAM
        # via odom_publisher, causing EKF to fight its own corrections.
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._last_msg_ns: int | None = None

        self.get_logger().info(
            f"odom_publisher: odom_frame={self.odom_frame_}  child={self.child_frame_}"
        )

    def _on_pos(self, msg: VehicleLocalPosition):
        if not (msg.z_valid and math.isfinite(msg.heading)):
            return

        # Integrate NED velocity → ENU position for SLAM odometry.
        # Using velocity integration (not EKF position) breaks the circular loop:
        #   EKF-pos → odom_publisher TF → slam_to_px4 EV → EKF-pos
        if msg.v_xy_valid:
            now_ns = int(msg.timestamp) * 1_000  # µs → ns
            if self._last_msg_ns is not None:
                dt = (now_ns - self._last_msg_ns) * 1e-9
                if 0.002 < dt < 0.5:
                    self._odom_x += float(msg.vy) * dt  # vy_ned (East) → ENU x
                    self._odom_y += float(msg.vx) * dt  # vx_ned (North) → ENU y
            self._last_msg_ns = now_ns

        x_enu = self._odom_x
        y_enu = self._odom_y
        z_enu = float(-msg.z)
        yaw_enu = math.pi / 2.0 - float(msg.heading)
        qz = math.sin(yaw_enu / 2.0)
        qw = math.cos(yaw_enu / 2.0)

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame_
        odom.child_frame_id  = self.child_frame_
        odom.pose.pose.position.x    = x_enu
        odom.pose.pose.position.y    = y_enu
        odom.pose.pose.position.z    = z_enu
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = float(msg.vy)
        odom.twist.twist.linear.y    = float(msg.vx)
        odom.twist.twist.linear.z    = float(-msg.vz)
        self.odom_pub_.publish(odom)

        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = self.odom_frame_
        t.child_frame_id  = self.child_frame_
        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_br_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
