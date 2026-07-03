#!/usr/bin/env python3
"""Bridge RTAB-Map Odometry → PX4 VehicleVisualOdometry for EKF2 full 6-DOF fusion.

Primary input: nav_msgs/Odometry on /rtabmap/odom (ENU frame from RTAB-Map).
Fallback:      TF map→base_link when no Odometry received (slam_toolbox compat).

Publishes px4_msgs/VehicleOdometry on /fmu/in/vehicle_visual_odometry.
Designed to work with EKF2_EV_CTRL=15 (XY+Z position + velocity + yaw from EV).

ENU → NED conversion:
  x_NED = y_ENU (North = East in ENU... wait: N=y_ENU, E=x_ENU, D=-z_ENU)
  position_NED = [y_ENU, x_ENU, -z_ENU]
  velocity_NED = [vy_ENU, vx_ENU, -vz_ENU]
  heading_NED  = pi/2 - yaw_ENU  (East-North vs North-East convention)
"""

import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped  # noqa: F401
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
import tf2_ros


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw_ned(qw: float, qx: float, qy: float, qz: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class SlamToPX4(Node):
    def __init__(self):
        super().__init__("slam_to_px4")

        self.declare_parameter("odom_topic",       "/rtabmap/odom")
        self.declare_parameter("map_frame",        "map")
        self.declare_parameter("base_frame",       "base_link")
        self.declare_parameter("publish_rate_hz",  20.0)
        self.declare_parameter("jump_gate_m",      3.0)
        self.declare_parameter("odom_timeout_s",   1.0)  # fall back to TF after this
        self.declare_parameter("output_topic",     "/fmu/in/vehicle_visual_odometry")
        self.declare_parameter("range_topic",      "range/down")
        self.declare_parameter("px4_odom_topic",   "/fmu/out/vehicle_odometry")
        self.declare_parameter("yaw_residual_gate_rad", 0.55)
        self.declare_parameter("yaw_residual_jump_gate_rad", 0.25)
        self.declare_parameter("yaw_drift_on_frames", 2)
        self.declare_parameter("yaw_drift_off_frames", 15)
        self.declare_parameter("max_slam_yaw_rate_dps", 90.0)

        odom_topic      = self.get_parameter("odom_topic").value
        self.map_frame_ = self.get_parameter("map_frame").value
        self.base_frame_= self.get_parameter("base_frame").value
        rate            = float(self.get_parameter("publish_rate_hz").value)
        self.jump_gate_ = float(self.get_parameter("jump_gate_m").value)
        self.odom_tout_ = float(self.get_parameter("odom_timeout_s").value)
        output_topic    = self.get_parameter("output_topic").value
        range_topic     = self.get_parameter("range_topic").value
        px4_odom_topic  = self.get_parameter("px4_odom_topic").value
        self._yaw_residual_gate = float(self.get_parameter("yaw_residual_gate_rad").value)
        self._yaw_residual_jump_gate = float(
            self.get_parameter("yaw_residual_jump_gate_rad").value)
        self._yaw_drift_on_frames = int(self.get_parameter("yaw_drift_on_frames").value)
        self._yaw_drift_off_frames = int(self.get_parameter("yaw_drift_off_frames").value)
        self._max_slam_yaw_step = (
            math.radians(float(self.get_parameter("max_slam_yaw_rate_dps").value)) /
            max(rate, 1.0)
        )

        # TF fallback (slam_toolbox / any tf-publishing SLAM)
        self.tf_buffer_  = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self.tf_listener_= tf2_ros.TransformListener(self.tf_buffer_, self)

        self.pub_ = self.create_publisher(VehicleOdometry, output_topic, 10)

        px4_qos = QoSProfile(depth=1)
        px4_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.px4_odom_: VehicleOdometry | None = None
        self.create_subscription(VehicleOdometry, px4_odom_topic, self._on_px4_odom, px4_qos)

        # RTAB-Map Odometry subscriber
        self.odom_msg_: Odometry | None = None
        self.odom_time_: float = 0.0
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        # Downward rangefinder — provides clean EV Z independent of PX4 altitude,
        # avoiding the feedback loop: EKF altitude → odom_publisher → EV Z → EKF.
        self._range_z: float = float("nan")  # ENU altitude (metres AGL)
        range_qos = QoSProfile(depth=1)
        range_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Range, range_topic, self._on_range, range_qos)

        self.timer_ = self.create_timer(1.0 / rate, self._on_timer)

        self._last_n:   float | None = None
        self._last_e:   float | None = None
        self._reset_ctr: int = 0

        # Freeze detection: if SLAM position doesn't change for _kFreezeFrames
        # consecutive frames, the SLAM tracker has likely lost features.
        # Suspend EV publication so PX4 falls back to IMU dead-reckoning,
        # which correctly registers the physical velocity and prevents windup.
        self._freeze_n:      float | None = None
        self._freeze_e:      float | None = None
        self._frozen_frames: int   = 0
        self._is_frozen:     bool  = False
        # 1e-4 m = 0.1 mm: real hover has IMU noise > this; algorithmic freeze is exactly 0
        self._kFreezeDelta:  float = 1e-4
        self._kFreezeFrames: int   = 5       # frames at 20 Hz = 0.25 s
        self._yaw_residual_bias: float | None = None
        self._prev_yaw_residual: float | None = None
        self._prev_slam_yaw: float | None = None
        self._yaw_bad_frames: int = 0
        self._yaw_good_frames: int = 0
        self._slam_yaw_drift_active: bool = False

        # Last-good pose hold: when yaw gating rejects SLAM briefly, continue
        # publishing the last accepted pose with higher position uncertainty
        # instead of dropping EV entirely.  EKF2 without any EV input for >2 s
        # falls back to pure IMU, which drifts and can cause attitude failure.
        self._last_good_n: float | None = None
        self._last_good_e: float | None = None
        self._last_good_d: float | None = None
        self._last_good_yaw: float | None = None
        self._gated_frames: int = 0
        self._max_gated_hold_frames: int = 15  # 0.75 s at 20 Hz

        self.get_logger().info(
            f"slam_to_px4 ready — odom={odom_topic}  TF={self.map_frame_}→{self.base_frame_} "
            f"range={range_topic}  px4_odom={px4_odom_topic}  out={output_topic} "
            f"@ {rate:.0f} Hz  yaw_gate={self._yaw_residual_gate:.2f} rad "
            "(6-DOF EKF2_EV_CTRL=15)"
        )

    def _on_px4_odom(self, msg: VehicleOdometry):
        self.px4_odom_ = msg

    # ── Downward rangefinder callback ─────────────────────────────────────────
    def _on_range(self, msg: Range):
        r = float(msg.range)
        if math.isfinite(r) and msg.min_range <= r <= msg.max_range:
            self._range_z = r  # metres above ground (ENU z)

    # ── RTAB-Map Odometry callback ────────────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        self.odom_msg_ = msg
        self.odom_time_ = self.get_clock().now().nanoseconds * 1e-9

    # ── 20 Hz publish timer ───────────────────────────────────────────────────
    def _on_timer(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.odom_msg_ and (now_s - self.odom_time_) < self.odom_tout_:
            self._publish_from_odom(self.odom_msg_)
        else:
            self._publish_from_tf()

    def _publish_from_odom(self, odom: Odometry):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        v = odom.twist.twist.linear

        # ENU → NED.  Z from downward rangefinder — independent of PX4 altitude,
        # so no circular feedback loop (EKF alt → odom_publisher → EV Z → EKF).
        n_ned = float(p.y)
        e_ned = float(p.x)
        d_ned = -self._range_z   # rangefinder ENU z → NED d (positive-down)
        vn    = float(v.y)
        ve    = float(v.x)
        vd    = float("nan")     # no vZ from EV

        if not self._jump_check(n_ned, e_ned):
            return
        if self._freeze_check(n_ned, e_ned):
            self._try_publish_held()
            return

        # ENU yaw → NED heading (level-drone orientation — roll/pitch from IMU)
        yaw_enu = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        heading_ned = math.pi / 2.0 - yaw_enu
        if not self._yaw_check(heading_ned):
            self._try_publish_held()
            return

        stamp: Time = odom.header.stamp
        ts = stamp.sec * 1_000_000 + stamp.nanosec // 1_000

        self._gated_frames = 0
        self._last_good_n, self._last_good_e = n_ned, e_ned
        self._last_good_d, self._last_good_yaw = d_ned, heading_ned
        msg = self._make_msg(ts, n_ned, e_ned, d_ned, heading_ned, vn, ve, vd)
        self.pub_.publish(msg)

    def _publish_from_tf(self):
        try:
            t = self.tf_buffer_.lookup_transform(
                self.map_frame_, self.base_frame_,
                rclpy.time.Time(), timeout=Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        x_enu = t.transform.translation.x
        y_enu = t.transform.translation.y
        n_ned = float(y_enu)
        e_ned = float(x_enu)
        # Z from downward rangefinder (not TF z, which comes from PX4 altitude via
        # odom_publisher and would create a feedback loop with EKF2_HGT_REF=3).
        d_ned = -self._range_z

        if not self._jump_check(n_ned, e_ned):
            return
        if self._freeze_check(n_ned, e_ned):
            self._try_publish_held()
            return

        q = t.transform.rotation
        yaw_enu = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        heading_ned = math.pi / 2.0 - yaw_enu
        if not self._yaw_check(heading_ned):
            self._try_publish_held()
            return

        stamp: Time = t.header.stamp
        ts = stamp.sec * 1_000_000 + stamp.nanosec // 1_000

        self._gated_frames = 0
        self._last_good_n, self._last_good_e = n_ned, e_ned
        self._last_good_d, self._last_good_yaw = d_ned, heading_ned
        msg = self._make_msg(ts, n_ned, e_ned, d_ned, heading_ned,
                              float("nan"), float("nan"), float("nan"))
        self.pub_.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _yaw_check(self, heading_ned: float) -> bool:
        slam_yaw_jump = 0.0
        if self._prev_slam_yaw is not None:
            slam_yaw_jump = abs(wrap_pi(heading_ned - self._prev_slam_yaw))
        self._prev_slam_yaw = heading_ned

        if self.px4_odom_ is None:
            return True

        q = self.px4_odom_.q
        px4_yaw = quat_to_yaw_ned(q[0], q[1], q[2], q[3])
        residual = wrap_pi(heading_ned - px4_yaw)
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
                    f"SLAM EV yaw rejected: err={residual_err:.2f} rad "
                    f"residual_jump={residual_jump:.2f} rad "
                    f"slam_jump={slam_yaw_jump:.2f} rad — suspending EV publication"
                )
        else:
            self._yaw_good_frames += 1
            self._yaw_bad_frames = 0
            if (self._slam_yaw_drift_active and
                    self._yaw_good_frames >= self._yaw_drift_off_frames):
                self._slam_yaw_drift_active = False
                self._yaw_good_frames = 0
                self.get_logger().info("SLAM EV yaw residual cleared — resuming EV publication")

        return not self._slam_yaw_drift_active

    def _jump_check(self, n: float, e: float) -> bool:
        if self._last_n is not None:
            jump = math.hypot(n - self._last_n, e - self._last_e)
            if jump > self.jump_gate_:
                self._reset_ctr = min(255, self._reset_ctr + 1)
                self.get_logger().warn(
                    f"Position jump {jump:.2f} m — skipping (reset_counter={self._reset_ctr})"
                )
                self._last_n, self._last_e = n, e
                return False
        self._last_n, self._last_e = n, e
        return True

    def _freeze_check(self, n: float, e: float) -> bool:
        """Return True (and suppress publish) when SLAM position has been frozen.

        A delta smaller than _kFreezeDelta between consecutive frames is treated
        as algorithmic freeze (not physical hover noise). After _kFreezeFrames
        consecutive frozen frames, EV publication is suspended so PX4 falls back
        to IMU dead-reckoning, which correctly senses physical acceleration and
        prevents velocity-controller integral windup.
        State update always happens (step 6 in spec) so recovery is immediate.
        """
        if self._freeze_n is not None:
            delta = math.hypot(n - self._freeze_n, e - self._freeze_e)
            if delta < self._kFreezeDelta:
                self._frozen_frames += 1
            else:
                self._frozen_frames = 0
                if self._is_frozen:
                    self._is_frozen = False
                    self.get_logger().info(
                        f"SLAM pose unfrozen (delta={delta:.6f} m) — "
                        "resuming visual odometry to PX4 EKF."
                    )
        # Always update last known pose (spec step 6)
        self._freeze_n = n
        self._freeze_e = e

        if self._frozen_frames >= self._kFreezeFrames:
            if not self._is_frozen:
                self._is_frozen = True
            # Throttle: warn once per second (~20 Hz timer)
            if self._frozen_frames % 20 == self._kFreezeFrames % 20:
                self.get_logger().warn(
                    f"[WARN] SLAM pose frozen! Suspending visual odometry to PX4 EKF. "
                    f"(pos=({n:.4f},{e:.4f}) frozen_frames={self._frozen_frames})"
                )
            return True
        return False

    def _try_publish_held(self) -> None:
        """Publish last-good SLAM pose with raised XY uncertainty.

        Called when gating rejects the current SLAM update.  Keeps PX4 EKF2
        anchored to the last trusted position instead of falling back to pure IMU
        (which accumulates drift within ~2 s and can cause attitude failure).
        Publishing stops after _max_gated_hold_frames consecutive rejections
        so a true SLAM failure does eventually trigger the IMU-only fallback.
        """
        self._gated_frames += 1
        if self._last_good_n is None or self._gated_frames > self._max_gated_hold_frames:
            return
        t = self.get_clock().now()
        ts = t.nanoseconds // 1000
        msg = self._make_msg(
            ts,
            self._last_good_n, self._last_good_e, self._last_good_d,
            self._last_good_yaw,
            float("nan"), float("nan"), float("nan"),
        )
        # 3× higher XY uncertainty signals "stale anchor" to EKF2 — trusts IMU
        # more for velocity while still preventing position drift.
        msg.position_variance[0] = 0.60
        msg.position_variance[1] = 0.60
        self.pub_.publish(msg)

    def _make_msg(self, ts: int, n: float, e: float, d: float, heading: float,
                  vn: float, ve: float, vd: float) -> VehicleOdometry:
        msg = VehicleOdometry()
        msg.timestamp        = ts
        msg.timestamp_sample = ts

        msg.pose_frame  = VehicleOdometry.POSE_FRAME_NED
        msg.position[0] = n
        msg.position[1] = e
        msg.position[2] = d  # NED d from downward rangefinder (EKF2_HGT_REF=3)

        msg.position_variance[0] = 0.20   # wider XY uncertainty → IMU has more weight
        msg.position_variance[1] = 0.20   # for velocity estimation between SLAM updates
        msg.position_variance[2] = 0.10

        # Level-drone quaternion with NED yaw (roll/pitch from IMU)
        hw = math.cos(heading / 2.0)
        hz = math.sin(heading / 2.0)
        msg.q[0] = hw; msg.q[1] = 0.0; msg.q[2] = 0.0; msg.q[3] = hz
        msg.orientation_variance[0] = 0.05
        msg.orientation_variance[1] = 0.05
        msg.orientation_variance[2] = 0.02   # yaw from VIO — tighter

        msg.velocity_frame  = VehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity[0]     = vn
        msg.velocity[1]     = ve
        msg.velocity[2]     = vd
        msg.velocity_variance[0] = 0.10
        msg.velocity_variance[1] = 0.10
        msg.velocity_variance[2] = 0.10

        msg.reset_counter = self._reset_ctr & 0xFF
        msg.quality = 100
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SlamToPX4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
