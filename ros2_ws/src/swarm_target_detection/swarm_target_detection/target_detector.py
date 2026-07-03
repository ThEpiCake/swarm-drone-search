"""HSV red-color-blob target detector for a single drone's RGB-D camera.

Subscribes to the bridged OakD-Lite RGB/depth streams (see
swarm_sim_bringup/launch/single_drone_sim.launch.py) plus the drone's
odometry, finds the largest contour matching the red Coca-Cola can's color
in an HSV mask (gated by pixel area and red-purity, since no model weights
fine-tuned on the can exist yet — a generic COCO-pretrained YOLO cannot
recognize a flat-shaded synthetic cylinder as "bottle": verified by direct
inference tests, best guess was "kite" at ~0.19 confidence, far under any
usable threshold), back-projects red pixels through the depth image into the
world frame, and publishes swarm_msgs/TargetFound.

TODO: once fine-tuned weights exist, replace _find_red_blob with a YOLO
inference call gated on the trained class + this same red-mask check.
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from swarm_msgs.msg import TargetFound


def _quat_rotate(q, v):
    """Rotate vector v=(x,y,z) by quaternion q=(w,x,y,z): body frame -> world frame."""
    qw, qx, qy, qz = q
    vx, vy, vz = v
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx
    return (
        vx + 2.0 * (qw * uvx + uuvx),
        vy + 2.0 * (qw * uvy + uuvy),
        vz + 2.0 * (qw * uvz + uuvz),
    )


def _pitch_about_body_y(v, pitch_rad):
    """Rotate a body-frame vector by a positive downward pitch around +Y."""
    x, y, z = v
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    return (
        c * x - s * z,
        y,
        s * x + c * z,
    )


class TargetDetector(Node):

    def __init__(self):
        super().__init__('target_detector')

        # ── Parameters ────────────────────────────────────────────────────
        self._drone_id = self.declare_parameter('drone_id', 0).value
        self._red_fraction_min = self.declare_parameter('red_pixel_fraction_min', 0.15).value
        self._min_blob_area_px = self.declare_parameter('min_blob_area_px', 40).value
        self._max_blob_area_fraction = self.declare_parameter(
            'max_blob_area_fraction', 0.25).value
        self._inference_period_s = self.declare_parameter('inference_period_s', 0.33).value
        self._republish_period_s = self.declare_parameter('republish_period_s', 0.5).value
        self._depth_min_m = float(self.declare_parameter('target_depth_min_m', 0.20).value)
        self._depth_max_m = float(self.declare_parameter('target_depth_max_m', 12.0).value)
        self._max_depth_spread_m = float(
            self.declare_parameter('target_max_depth_spread_m', 0.45).value)
        self._cluster_radius_m = float(self.declare_parameter('target_cluster_radius_m', 4.0).value)
        self._cluster_alt_radius_m = float(
            self.declare_parameter('target_cluster_alt_radius_m', 2.0).value)
        self._min_cluster_confirmations = int(
            self.declare_parameter('target_min_cluster_confirmations', 3).value)
        self._cluster_weight_limit = float(
            self.declare_parameter('target_cluster_weight_limit', 12.0).value)
        self._target_altitude_min_m = float(
            self.declare_parameter('target_altitude_min_m', 1.0).value)
        self._target_altitude_max_m = float(
            self.declare_parameter('target_altitude_max_m', 6.8).value)
        # Fixed OakD-Lite mount offset in the FRD body frame — see the
        # rgbd_joint pose in swarm_sim_bringup/models/x500_swarm/model.sdf.
        # The camera is pitched downward by 15 degrees; _localize applies
        # that fixed rotation after the optical->body axis remap.
        offset = self.declare_parameter('camera_offset_frd_m', [0.12, 0.03, 0.06]).value
        self._cam_offset = tuple(float(v) for v in offset)
        self._camera_pitch_rad = float(
            self.declare_parameter('camera_pitch_rad', math.pi / 12.0).value)

        self._bridge = CvBridge()
        self._latest_depth = None        # np.ndarray, float32, metres
        self._latest_camera_info = None  # sensor_msgs/CameraInfo
        self._latest_odom = None         # px4_msgs/VehicleOdometry
        self._last_inference_t = 0.0
        self._last_publish_t = 0.0
        self._target_clusters = []

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── Subscriptions — relative names so namespace=px4_<i> maps them
        # onto /px4_<i>/camera/... and /px4_<i>/fmu/out/..., matching the
        # convention established in swarm_control/offboard_control.cpp.
        self.create_subscription(Image, 'camera/image_raw', self._on_rgb, sensor_qos)
        self.create_subscription(Image, 'camera/depth/image_raw', self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, 'camera/depth/camera_info',
                                 self._on_camera_info, sensor_qos)
        self.create_subscription(VehicleOdometry, 'fmu/out/vehicle_odometry',
                                 self._on_odometry, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────
        # Reliable QoS for "found" events per the documented convention
        # (best-effort for high-rate sensor streams, reliable for commands
        # & found/obstacle announcements).
        self._found_pub = self.create_publisher(TargetFound, 'target_found', reliable_qos)
        self._debug_pub = self.create_publisher(Image, 'camera/detections_debug', sensor_qos)

        self.get_logger().info('target_detector ready (HSV red-blob detector)')

    # ── Caching callbacks for the lower-rate / lookup inputs ─────────────
    def _on_depth(self, msg: Image):
        self._latest_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def _on_camera_info(self, msg: CameraInfo):
        self._latest_camera_info = msg

    def _on_odometry(self, msg: VehicleOdometry):
        self._latest_odom = msg

    # ── Main detection path, throttled to ~inference_period_s ────────────
    def _on_rgb(self, msg: Image):
        now = self._stamp_to_seconds(msg.header.stamp)
        if now - self._last_inference_t < self._inference_period_s:
            return
        self._last_inference_t = now

        if (self._latest_depth is None or self._latest_camera_info is None
                or self._latest_odom is None):
            return  # not enough context yet to localize a detection

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        det = self._find_red_blob(frame)
        if det is not None:
            self._publish_debug_image(frame, det)
            self._try_publish_target(frame, msg.header, det)

    def _find_red_blob(self, frame):
        """Largest contour in the HSV red mask that passes the area/purity gates."""
        mask = self._red_mask(frame)
        max_area_px = self._max_blob_area_fraction * mask.size
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if not (self._min_blob_area_px <= area <= max_area_px):
                continue
            x, y, w, h = cv2.boundingRect(c)
            x1, y1, x2, y2 = x, y, x + w, y + h
            red_fraction = self._red_pixel_fraction(frame[y1:y2, x1:x2])
            if red_fraction < self._red_fraction_min:
                continue
            if best is None or area > best['area']:
                best = {'bbox': (x1, y1, x2, y2), 'conf': red_fraction,
                        'red_fraction': red_fraction, 'area': area}
        return best

    @staticmethod
    def _red_mask(bgr) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Red wraps around hue 0/180 in OpenCV's 8-bit HSV — combine both bands.
        # Narrowed to saturated Coca-Cola red. Wider bands (hue 0-10/170-180,
        # sat>=90) also matched the pink/magenta floor mats in the world, which
        # produced phantom targets. Higher min saturation+value and tighter hue
        # reject pastel/magenta while keeping the deep-red can.
        lower1, upper1 = np.array([0, 120, 80]), np.array([8, 255, 255])
        lower2, upper2 = np.array([174, 120, 80]), np.array([180, 255, 255])
        return cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

    def _red_pixel_fraction(self, bgr_crop) -> float:
        if bgr_crop.size == 0:
            return 0.0
        mask = self._red_mask(bgr_crop)
        return float(np.count_nonzero(mask)) / mask.size

    def _try_publish_target(self, frame, header, det):
        now = self._stamp_to_seconds(header.stamp)
        if now - self._last_publish_t < self._republish_period_s:
            return

        position = self._localize(frame, det['bbox'])
        if position is None:
            return
        position, confirmations = self._stabilize_position(position)
        if confirmations < self._min_cluster_confirmations:
            self.get_logger().debug(
                f'Target candidate buffered ({confirmations}/{self._min_cluster_confirmations}) '
                f'@ world=({position.x:.2f}, {position.y:.2f}, {position.z:.2f})')
            return

        msg = TargetFound()
        msg.header = header
        msg.header.frame_id = 'odom'  # drone-local NED origin — each drone's own map frame
        msg.drone_id = self._drone_id
        msg.position_world = position
        msg.confidence = det['conf'] * det['red_fraction']
        x1, y1, x2, y2 = det['bbox']
        msg.still_image = self._bridge.cv2_to_imgmsg(frame[y1:y2, x1:x2], encoding='bgr8')

        self._found_pub.publish(msg)
        self._last_publish_t = now
        self.get_logger().info(
            f'TargetFound @ world=({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) '
            f'confidence={msg.confidence:.2f}')

    def _localize(self, frame, bbox):
        """Back-project red pixels through the depth image into the world frame."""
        depth_img = self._latest_depth
        info = self._latest_camera_info
        odom = self._latest_odom

        x1, y1, x2, y2 = bbox
        rgb_h, rgb_w = frame.shape[:2]
        depth_h, depth_w = depth_img.shape[:2]

        crop = frame[y1:y2, x1:x2]
        red_mask = self._red_mask(crop)
        if red_mask.size == 0:
            return None

        kernel = np.ones((3, 3), dtype=np.uint8)
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        ys, xs = np.nonzero(red_mask)
        if len(xs) < 6:
            return None

        # Detection runs on RGB while depth is lower resolution. Scale the red
        # support pixels into the depth image, then use a median inlier depth
        # instead of one noisy centre pixel.
        u = np.clip(((xs + x1) * depth_w / rgb_w).astype(np.int32), 0, depth_w - 1)
        v = np.clip(((ys + y1) * depth_h / rgb_h).astype(np.int32), 0, depth_h - 1)
        depths = depth_img[v, u].astype(np.float32)
        valid = (
            np.isfinite(depths) &
            (depths >= self._depth_min_m) &
            (depths <= self._depth_max_m)
        )
        if np.count_nonzero(valid) < 6:
            return None

        u = u[valid].astype(np.float32)
        v = v[valid].astype(np.float32)
        depths = depths[valid]
        median_depth = float(np.median(depths))
        inlier = np.abs(depths - median_depth) <= self._max_depth_spread_m
        if np.count_nonzero(inlier) < 6:
            return None

        u = u[inlier]
        v = v[inlier]
        depths = depths[inlier]
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]
        # Pinhole back-projection in the optical frame (x right, y down, z forward).
        x_opt = float(np.median((u - cx) * depths / fx))
        y_opt = float(np.median((v - cy) * depths / fy))
        z_opt = float(np.median(depths))

        # Optical frame -> body FRD for the camera's neutral axes, then rotate
        # by the fixed downward mount pitch of the RGB-D payload.
        ox, oy, oz = self._cam_offset
        body_frd_neutral = (z_opt, x_opt, y_opt)
        body_frd_rotated = _pitch_about_body_y(body_frd_neutral, self._camera_pitch_rad)
        body_frd = (
            body_frd_rotated[0] + ox,
            body_frd_rotated[1] + oy,
            body_frd_rotated[2] + oz,
        )

        q = (odom.q[0], odom.q[1], odom.q[2], odom.q[3])
        wx, wy, wz = _quat_rotate(q, body_frd)

        p = Point()
        p.x = float(odom.position[0] + wx)
        p.y = float(odom.position[1] + wy)
        p.z = float(odom.position[2] + wz)
        altitude_m = -p.z
        if not (self._target_altitude_min_m <= altitude_m <= self._target_altitude_max_m):
            return None
        return p

    def _stabilize_position(self, position: Point) -> tuple[Point, int]:
        for cluster in self._target_clusters:
            dist = math.hypot(position.x - cluster['x'], position.y - cluster['y'])
            alt_dist = abs(position.z - cluster['z'])
            if dist > self._cluster_radius_m or alt_dist > self._cluster_alt_radius_m:
                continue

            old_weight = min(cluster['count'], self._cluster_weight_limit)
            denom = old_weight + 1.0
            cluster['x'] = (cluster['x'] * old_weight + position.x) / denom
            cluster['y'] = (cluster['y'] * old_weight + position.y) / denom
            cluster['z'] = (cluster['z'] * old_weight + position.z) / denom
            cluster['count'] += 1.0

            stable = Point()
            stable.x = float(cluster['x'])
            stable.y = float(cluster['y'])
            stable.z = float(cluster['z'])
            return stable, int(cluster['count'])

        self._target_clusters.append({
            'x': position.x,
            'y': position.y,
            'z': position.z,
            'count': 1.0,
        })
        return position, 1

    def _publish_debug_image(self, frame, det):
        annotated = frame.copy()
        x1, y1, x2, y2 = det['bbox']
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
        label = f"target red={det['red_fraction']:.2f} area={det['area']:.0f}px"
        cv2.putText(annotated, label, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        self._debug_pub.publish(self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
