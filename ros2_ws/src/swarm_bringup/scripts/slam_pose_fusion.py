#!/usr/bin/env python3
"""Fuse 2D SLAM XY/yaw with PX4 Z/roll/pitch for stable voxel-map projection.

Inputs:
  - TF map -> lidar_2d_link from slam_toolbox
  - PX4 VehicleOdometry for altitude, roll/pitch, and velocity

Output:
  - nav_msgs/Odometry in NED-style coordinates on `slam/odom_ned`

This keeps the 3D depth projection physically consistent while replacing the
drifting XY/yaw components with the scan-matched SLAM estimate.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros


def quat_to_rpy_ned(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_to_quat_ned(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SlamPoseFusion(Node):
    def __init__(self):
        super().__init__("slam_pose_fusion")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "lidar_2d_link")
        self.declare_parameter("px4_odom_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("tf_timeout_s", 0.08)
        self.declare_parameter("yaw_residual_gate_rad", 0.55)
        self.declare_parameter("yaw_residual_jump_gate_rad", 0.25)
        self.declare_parameter("yaw_drift_on_frames", 2)
        self.declare_parameter("yaw_drift_off_frames", 15)
        self.declare_parameter("max_slam_yaw_rate_dps", 90.0)
        self.declare_parameter("publish_px4_fallback", True)
        # Spawn offset: translate local SLAM frame to a shared global frame.
        # Set to the drone's spawn position relative to drone-0's spawn.
        # Default 0.0 (drone-0, or single-drone mode).
        self.declare_parameter("spawn_north_m", 0.0)
        self.declare_parameter("spawn_east_m",  0.0)

        self.map_frame_ = self.get_parameter("map_frame").value
        self.base_frame_ = self.get_parameter("base_frame").value
        self.tf_timeout_ = float(self.get_parameter("tf_timeout_s").value)
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        px4_odom_topic = self.get_parameter("px4_odom_topic").value
        self._yaw_residual_gate = float(self.get_parameter("yaw_residual_gate_rad").value)
        self._yaw_residual_jump_gate = float(
            self.get_parameter("yaw_residual_jump_gate_rad").value)
        self._yaw_drift_on_frames = int(self.get_parameter("yaw_drift_on_frames").value)
        self._yaw_drift_off_frames = int(self.get_parameter("yaw_drift_off_frames").value)
        self._max_slam_yaw_step = (
            math.radians(float(self.get_parameter("max_slam_yaw_rate_dps").value)) /
            max(rate_hz, 1.0)
        )
        self._publish_px4_fallback = bool(self.get_parameter("publish_px4_fallback").value)

        self._spawn_north_m = float(self.get_parameter("spawn_north_m").value)
        self._spawn_east_m  = float(self.get_parameter("spawn_east_m").value)

        self.tf_buffer_ = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.px4_odom_: VehicleOdometry | None = None
        self.create_subscription(VehicleOdometry, px4_odom_topic, self._on_px4_odom, qos_px4)
        self.pub_ = self.create_publisher(Odometry, "slam/odom_ned", 10)
        self.timer_ = self.create_timer(1.0 / rate_hz, self._on_timer)

        self._tf_fail_count: int = 0
        self._tf_warn_interval: int = 60  # warn every N timer ticks (~4 s at 15 Hz)

        # Velocity drift guard: compare SLAM-derived position velocity with PX4 velocity.
        # If SLAM moves faster than PX4 says the drone is flying, the scan matcher
        # has drifted — fall back to PX4 dead-reckoning until SLAM recovers.
        self._rate_hz: float = rate_hz
        self._prev_slam_n: float | None = None
        self._prev_slam_e: float | None = None
        self._drift_frames: int = 0
        self._slam_drift_active: bool = False
        self._kDriftVelThresh: float = 0.8   # m/s above PX4 speed → probable drift
        self._kDriftOnFrames:  int   = 3     # frames before engaging fallback (~0.2 s)
        self._kDriftOffFrames: int   = 10    # frames of clean data before re-enabling SLAM
        self._yaw_residual_bias: float | None = None
        self._prev_yaw_residual: float | None = None
        self._prev_slam_yaw: float | None = None
        self._yaw_bad_frames: int = 0
        self._yaw_good_frames: int = 0
        self._slam_yaw_drift_active: bool = False

        self.get_logger().info(
            f"slam_pose_fusion ready — TF {self.map_frame_}->{self.base_frame_}, "
            f"PX4 odom {px4_odom_topic}, out slam/odom_ned @ {rate_hz:.0f} Hz, "
            f"yaw_gate={self._yaw_residual_gate:.2f} rad, "
            f"spawn_offset=({self._spawn_north_m:.2f}N, {self._spawn_east_m:.2f}E)"
        )

    def _on_px4_odom(self, msg: VehicleOdometry):
        self.px4_odom_ = msg

    def _slam_yaw_usable(self, slam_yaw: float, px4_yaw: float) -> bool:
        slam_yaw_jump = 0.0
        if self._prev_slam_yaw is not None:
            slam_yaw_jump = abs(wrap_pi(slam_yaw - self._prev_slam_yaw))
        self._prev_slam_yaw = slam_yaw

        residual = wrap_pi(slam_yaw - px4_yaw)
        if self._yaw_residual_bias is None:
            self._yaw_residual_bias = residual
        residual_err = wrap_pi(residual - self._yaw_residual_bias)
        residual_jump = 0.0
        if self._prev_yaw_residual is not None:
            residual_jump = abs(wrap_pi(residual - self._prev_yaw_residual))
        self._prev_yaw_residual = residual

        bad = (
            abs(residual_err) > self._yaw_residual_gate or
            residual_jump > self._yaw_residual_jump_gate or
            slam_yaw_jump > self._max_slam_yaw_step
        )
        if bad:
            self._yaw_bad_frames += 1
            self._yaw_good_frames = 0
            if (self._yaw_bad_frames >= self._yaw_drift_on_frames and
                    not self._slam_yaw_drift_active):
                self._slam_yaw_drift_active = True
                self.get_logger().warn(
                    f"SLAM yaw rejected: err={residual_err:.2f} rad "
                    f"residual_jump={residual_jump:.2f} rad "
                    f"slam_jump={slam_yaw_jump:.2f} rad — using PX4 yaw/XY fallback"
                )
        else:
            self._yaw_good_frames += 1
            self._yaw_bad_frames = 0
            if (self._slam_yaw_drift_active and
                    self._yaw_good_frames >= self._yaw_drift_off_frames):
                self._slam_yaw_drift_active = False
                self._yaw_good_frames = 0
                self.get_logger().info("SLAM yaw residual cleared — resuming SLAM pose")

        return not self._slam_yaw_drift_active

    def _on_timer(self):
        if self.px4_odom_ is None:
            return

        px4_q = self.px4_odom_.q
        roll, pitch, px4_yaw = quat_to_rpy_ned(px4_q[0], px4_q[1], px4_q[2], px4_q[3])
        use_slam = True
        try:
            tf_msg = self.tf_buffer_.lookup_transform(
                self.map_frame_,
                self.base_frame_,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_),
            )
            self._tf_fail_count = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            use_slam = False
            self._tf_fail_count += 1
            if self._tf_fail_count % self._tf_warn_interval == 1:
                self.get_logger().warn(
                    f"SLAM TF {self.map_frame_}->{self.base_frame_} unavailable "
                    f"({e}), using PX4 NED position as fallback"
                )

        if use_slam:
            x_enu = float(tf_msg.transform.translation.x)
            y_enu = float(tf_msg.transform.translation.y)
            n_ned = y_enu
            e_ned = x_enu
            q = tf_msg.transform.rotation
            yaw_enu = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            yaw_ned = math.pi / 2.0 - yaw_enu

            # ── Velocity drift guard ──────────────────────────────────────────
            # SLAM-derived velocity: position delta per timer tick.
            if self._prev_slam_n is not None:
                dt = 1.0 / self._rate_hz
                slam_vn = (n_ned - self._prev_slam_n) / dt
                slam_ve = (e_ned - self._prev_slam_e) / dt
                slam_spd = math.hypot(slam_vn, slam_ve)
                px4_spd  = math.hypot(float(self.px4_odom_.velocity[0]),
                                      float(self.px4_odom_.velocity[1]))
                # Drone physically cannot fly faster than ~1 m/s in this project;
                # if SLAM position change implies much higher speed, it has drifted.
                if slam_spd > px4_spd + self._kDriftVelThresh:
                    self._drift_frames += 1
                    if self._drift_frames >= self._kDriftOnFrames and not self._slam_drift_active:
                        self._slam_drift_active = True
                        self.get_logger().warn(
                            f"SLAM drift detected: SLAM_vel={slam_spd:.2f} m/s, "
                            f"PX4_vel={px4_spd:.2f} m/s — falling back to PX4 NED"
                        )
                else:
                    if self._slam_drift_active:
                        self._drift_frames -= 1
                        if self._drift_frames <= 0:
                            self._slam_drift_active = False
                            self._drift_frames = 0
                            self.get_logger().info("SLAM drift cleared — resuming SLAM XY")
                    else:
                        self._drift_frames = 0

            self._prev_slam_n = n_ned
            self._prev_slam_e = e_ned

            if self._slam_drift_active or not self._slam_yaw_usable(yaw_ned, px4_yaw):
                use_slam = False
                n_ned = float(self.px4_odom_.position[0])
                e_ned = float(self.px4_odom_.position[1])
        else:
            # Fallback: use PX4 EKF NED position (position[0]=N, position[1]=E)
            n_ned = float(self.px4_odom_.position[0])
            e_ned = float(self.px4_odom_.position[1])
            yaw_ned = 0.0  # will be overwritten by px4_yaw below

        d_ned = float(self.px4_odom_.position[2])

        if not use_slam and not self._publish_px4_fallback:
            return

        # SLAM toolbox starts each drone map at its own local origin, so SLAM TF
        # needs the Gazebo spawn offset. PX4 fallback is already in the shared
        # Gazebo/world frame, so do not offset it again.
        if use_slam:
            n_ned += self._spawn_north_m
            e_ned += self._spawn_east_m

        yaw_ned = yaw_ned if use_slam else px4_yaw
        qw, qx, qy, qz = rpy_to_quat_ned(roll, pitch, yaw_ned)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"
        msg.child_frame_id = self.base_frame_
        msg.pose.pose.position.x = n_ned
        msg.pose.pose.position.y = e_ned
        msg.pose.pose.position.z = d_ned
        msg.pose.pose.orientation.w = qw
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.twist.twist.linear.x = float(self.px4_odom_.velocity[0])
        msg.twist.twist.linear.y = float(self.px4_odom_.velocity[1])
        msg.twist.twist.linear.z = float(self.px4_odom_.velocity[2])

        for idx in (0, 7, 14):
            msg.pose.covariance[idx] = 0.05
            msg.twist.covariance[idx] = 0.10
        msg.pose.covariance[35] = 0.03
        msg.twist.covariance[35] = 0.05

        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
