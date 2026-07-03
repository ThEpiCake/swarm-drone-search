#!/usr/bin/env python3
"""Merge voxel point clouds and map summaries from two drones.

In the current D2 launch, slam_pose_fusion normalizes both drone voxel maps into
the shared Gazebo/PX4 world frame before they reach this node.  The configured
offsets therefore stay zero in normal experiments; the parameters remain only as
a compatibility hook for older local-frame runs.

This node publishes the union of occupied and free voxel clouds so both mappers
can plan over the team exploration map, and combines per-layer MapUpdateSummary
counts so mission controllers can make layer-completion decisions from shared
team coverage.

Parameters
----------
drone0_ns        : ROS namespace for drone 0 (default "px4_0")
drone1_ns        : ROS namespace for drone 1 (default "px4_1")
drone1_north_m   : world-frame North spawn offset of drone 1 vs drone 0 [m]
drone1_east_m    : world-frame East spawn offset of drone 1 vs drone 0 [m]
publish_rate_hz  : how often to republish the merged cloud
"""

import struct
import threading

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from swarm_msgs.msg import MapUpdateSummary


def _cloud_to_numpy_xyz(msg: PointCloud2) -> np.ndarray:
    """Return Nx3 float32 array (x=North, y=East, z=Down)."""
    n = msg.width * msg.height
    if n == 0 or len(msg.data) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    stride = max(1, msg.point_step // 4)
    raw = np.frombuffer(bytes(msg.data), dtype=np.float32)
    if raw.size < n * stride or stride < 3:
        return np.zeros((0, 3), dtype=np.float32)
    return raw[: n * stride].reshape(n, stride)[:, :3].astype(np.float32, copy=True)


def _cloud_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Return Nx4 float32 array (x=North, y=East, z=Down, intensity)."""
    n = msg.width * msg.height
    if n == 0 or len(msg.data) == 0:
        return np.zeros((0, 4), dtype=np.float32)
    stride = max(1, msg.point_step // 4)
    raw = np.frombuffer(bytes(msg.data), dtype=np.float32)
    if raw.size < n * stride or stride < 3:
        return np.zeros((0, 4), dtype=np.float32)
    arr = raw[: n * stride].reshape(n, stride)
    pts = np.zeros((n, 4), dtype=np.float32)
    pts[:, :3] = arr[:, :3]
    if stride >= 4:
        pts[:, 3] = arr[:, 3]
    return pts


def _numpy_to_cloud(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Build a PointCloud2 (XYZL, float32) from Nx4 ndarray."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(pts)
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(pts)
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = pts.astype(np.float32).tobytes()
    return msg


def _numpy_xyz_to_cloud(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Build a PointCloud2 (XYZ, float32) from Nx3 ndarray."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(pts)
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(pts)
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = pts.astype(np.float32).tobytes()
    return msg


class MapMerger(Node):
    def __init__(self):
        super().__init__("map_merger")

        d0_ns  = self.declare_parameter("drone0_ns", "px4_0").value
        d1_ns  = self.declare_parameter("drone1_ns", "px4_1").value
        self._d1_dn = float(self.declare_parameter("drone1_north_m", 0.0).value)
        self._d1_de = float(self.declare_parameter("drone1_east_m",  0.0).value)
        rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        self._lock = threading.Lock()
        self._cloud0: np.ndarray = np.zeros((0, 4), dtype=np.float32)
        self._cloud1: np.ndarray = np.zeros((0, 4), dtype=np.float32)
        self._free0: np.ndarray = np.zeros((0, 3), dtype=np.float32)
        self._free1: np.ndarray = np.zeros((0, 3), dtype=np.float32)
        self._summaries_by_drone: dict[int, dict[float, MapUpdateSummary]] = {
            0: {},
            1: {},
        }
        self._latest_summary_by_drone: dict[int, MapUpdateSummary] = {}

        be = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        rel = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(PointCloud2, f"/{d0_ns}/voxel_map", self._cb0, be)
        self.create_subscription(PointCloud2, f"/{d1_ns}/voxel_map", self._cb1, be)
        self.create_subscription(PointCloud2, f"/{d0_ns}/free_voxel_map", self._free_cb0, be)
        self.create_subscription(PointCloud2, f"/{d1_ns}/free_voxel_map", self._free_cb1, be)
        self.create_subscription(
            MapUpdateSummary, f"/{d0_ns}/map_update_summary", self._summary_cb(0), rel
        )
        self.create_subscription(
            MapUpdateSummary, f"/{d1_ns}/map_update_summary", self._summary_cb(1), rel
        )
        self._pub = self.create_publisher(PointCloud2, "/merged_voxel_map", be)
        self._free_pub = self.create_publisher(PointCloud2, "/merged_free_voxel_map", be)
        self._summary_pub = self.create_publisher(
            MapUpdateSummary, "/shared_map_update_summary", rel
        )
        self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f"map_merger ready — drone1 offset N={self._d1_dn:.1f} E={self._d1_de:.1f} m, "
            f"shared coverage from free voxels"
        )

    def _cb0(self, msg: PointCloud2):
        pts = _cloud_to_numpy(msg)
        with self._lock:
            self._cloud0 = pts

    def _cb1(self, msg: PointCloud2):
        pts = _cloud_to_numpy(msg)
        if len(pts) > 0:
            pts = pts.copy()
            pts[:, 0] += self._d1_dn   # North offset
            pts[:, 1] += self._d1_de   # East offset
        with self._lock:
            self._cloud1 = pts

    def _free_cb0(self, msg: PointCloud2):
        pts = _cloud_to_numpy_xyz(msg)
        with self._lock:
            self._free0 = pts

    def _free_cb1(self, msg: PointCloud2):
        pts = _cloud_to_numpy_xyz(msg)
        if len(pts) > 0:
            pts = pts.copy()
            pts[:, 0] += self._d1_dn
            pts[:, 1] += self._d1_de
        with self._lock:
            self._free1 = pts

    def _summary_cb(self, drone_id: int):
        def cb(msg: MapUpdateSummary):
            if msg.layer_total_cells == 0:
                return
            # Quantise altitude to centimetres so 2.000 and 1.999 stay together.
            layer_key = round(float(msg.layer_altitude_m), 2)
            with self._lock:
                self._summaries_by_drone.setdefault(drone_id, {})[layer_key] = msg
                self._latest_summary_by_drone[drone_id] = msg

        return cb

    def _publish(self):
        with self._lock:
            c0 = self._cloud0.copy()
            c1 = self._cloud1.copy()
            f0 = self._free0.copy()
            f1 = self._free1.copy()

        if len(c0) == 0 and len(c1) == 0:
            self._publish_merged_free(f0, f1)
            self._publish_shared_summaries()
            return

        merged = np.concatenate([c0, c1], axis=0) if len(c0) and len(c1) else (c0 if len(c0) else c1)

        # Downsample if too large for the dashboard (keep ≤ 20000 pts)
        if len(merged) > 20_000:
            step = len(merged) // 20_000 + 1
            merged = merged[::step]

        cloud_msg = _numpy_to_cloud(merged, "map_ned", self.get_clock().now().to_msg())
        self._pub.publish(cloud_msg)
        self._publish_merged_free(f0, f1)
        self._publish_shared_summaries()

    def _publish_merged_free(self, f0: np.ndarray, f1: np.ndarray):
        if self._free_pub.get_subscription_count() == 0:
            return
        if len(f0) == 0 and len(f1) == 0:
            return

        merged = np.concatenate([f0, f1], axis=0) if len(f0) and len(f1) else (f0 if len(f0) else f1)
        if len(merged) > 60_000:
            step = len(merged) // 60_000 + 1
            merged = merged[::step]

        msg = _numpy_xyz_to_cloud(merged, "map_ned", self.get_clock().now().to_msg())
        self._free_pub.publish(msg)

    def _publish_shared_summaries(self):
        with self._lock:
            summaries_by_drone = {
                drone_id: dict(layer_map)
                for drone_id, layer_map in self._summaries_by_drone.items()
            }
            latest = dict(self._latest_summary_by_drone)

        layer_keys = sorted({k for layer_map in summaries_by_drone.values() for k in layer_map})
        if not layer_keys:
            return

        per_layer: dict[float, tuple[int, int]] = {}
        for key in layer_keys:
            layer_msgs = [
                layer_map[key]
                for layer_map in summaries_by_drone.values()
                if key in layer_map
            ]
            if not layer_msgs:
                continue
            # Each voxel mapper already computes layer coverage on top of the merged
            # shared-map overlay, so a single drone's (observed, total) pair already
            # reflects the team's union for that layer. Combining them with
            # max(total)+max(observed) mixed one drone's observed with another's
            # total: when a drone climbing between layers publishes a transient,
            # wide-but-barely-scanned summary at the OTHER drone's altitude (snapped
            # onto its layer), the scanner's observed got divided by the transiter's
            # larger total → e.g. 989/2000≈50% instead of 989/1000≈99%, stalling the
            # layer until timeout. Pick the single drone with the highest coverage
            # fraction (the one actually scanning that layer) and use ITS consistent
            # pair, so a transient low-coverage pass cannot drag the layer down.
            best = max(
                (m for m in layer_msgs if int(m.layer_total_cells) > 0),
                key=lambda m: int(m.layer_observed_cells) / int(m.layer_total_cells),
                default=None,
            )
            if best is not None:
                observed = int(best.layer_observed_cells)
                total = int(best.layer_total_cells)
                per_layer[key] = (min(observed, total), total)

        if not per_layer:
            return

        global_observed = sum(obs for obs, _ in per_layer.values())
        global_total = sum(total for _, total in per_layer.values())
        global_cov = float(global_observed) / float(global_total) if global_total else 0.0

        entropy_values = [float(m.entropy_mean) for m in latest.values()]
        entropy_mean = float(np.mean(entropy_values)) if entropy_values else 0.0
        voxels_updated = sum(int(m.voxels_updated) for m in latest.values())
        frontier_clusters = sum(int(m.frontier_clusters) for m in latest.values())
        reachable_clusters = sum(int(m.reachable_frontier_clusters) for m in latest.values())
        reachable_cells = sum(int(m.reachable_frontier_cells) for m in latest.values())
        route_available = any(bool(m.frontier_route_available) for m in latest.values())
        stamp = self.get_clock().now().to_msg()

        for key, (observed, total) in per_layer.items():
            msg = MapUpdateSummary()
            msg.header.stamp = stamp
            msg.header.frame_id = "map_ned"
            msg.drone_id = 255
            msg.entropy_mean = entropy_mean
            msg.coverage_fraction = global_cov
            msg.voxels_updated = voxels_updated
            msg.layer_coverage_fraction = float(observed) / float(total)
            msg.layer_altitude_m = float(key)
            msg.layer_observed_cells = observed
            msg.layer_total_cells = total
            msg.frontier_clusters = frontier_clusters
            msg.reachable_frontier_clusters = reachable_clusters
            msg.reachable_frontier_cells = reachable_cells
            msg.frontier_route_available = route_available
            self._summary_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
