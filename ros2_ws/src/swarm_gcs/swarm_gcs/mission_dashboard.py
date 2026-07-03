"""Mission dashboard — single or dual drone VFH search.

Tkinter + Matplotlib 3-D scene. Shows:
  - shared 3-D voxel map (merged when two drones are active)
  - per-drone path, pose, heading, frontier/entropy goals
  - mission state, telemetry, link freshness, and result per drone
  - a shared event log

Parameters
----------
drone_ns         : namespace for drone 0           (default "px4_0")
drone1_ns        : namespace for drone 1           (default "" = single-drone)
num_drones       : 1 or 2                          (default 1)
drone1_north_m   : drone-1 global North offset     (default 0.0)
drone1_east_m    : drone-1 global East offset      (default 0.0)
"""

import math
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
import warnings
from collections import deque
import importlib.util
from pathlib import Path as FsPath
from tkinter import ttk

import matplotlib

matplotlib.use("TkAgg")
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.projections import projection_registry
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
from swarm_msgs.msg import MapUpdateSummary, TargetFound
from visualization_msgs.msg import MarkerArray


def _load_axes3d():
    mplot3d_dir = FsPath(matplotlib.__file__).resolve().parent.parent / "mpl_toolkits" / "mplot3d"
    spec = importlib.util.spec_from_file_location(
        "mpl_toolkits.mplot3d",
        mplot3d_dir / "__init__.py",
        submodule_search_locations=[str(mplot3d_dir)],
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load mpl_toolkits.mplot3d from {mplot3d_dir}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["mpl_toolkits.mplot3d"] = module
    spec.loader.exec_module(module)
    return module.Axes3D


Axes3D = _load_axes3d()  # noqa: F401
projection_registry.register(Axes3D)


def _parse_cloud(msg: PointCloud2) -> np.ndarray:
    """Return Nx3 float32 (North, East, Alt-above-ground) from PointCloud2."""
    n_pts = msg.width * msg.height
    if n_pts == 0 or len(msg.data) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    n_floats = max(3, msg.point_step // 4)  # 3 floats (xyz) or 4+ (xyzi etc.)
    raw = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, n_floats)
    return np.column_stack([raw[:, 0], raw[:, 1], -raw[:, 2]]).astype(np.float32)


def _yaw_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _compass(deg: float) -> str:
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return dirs[int((deg + 22.5) / 45.0) % 8]


def _freshness_text(age_s: float) -> str:
    if not math.isfinite(age_s):
        return "waiting"
    if age_s < 1.5:
        return f"live {age_s:0.1f}s"
    if age_s < 5.0:
        return f"stale {age_s:0.1f}s"
    return f"lost {age_s:0.1f}s"


class SharedState:
    _MAX_VOXELS = 15_000

    def __init__(self, label: str):
        self._label = label
        self._lock = threading.Lock()
        self._seen = set()

        self.voxels = np.zeros((0, 3), dtype=np.float32)
        self.free_voxels = np.zeros((0, 3), dtype=np.float32)
        self.path_nea = np.zeros((0, 3), dtype=np.float32)
        self.slam_occ = None
        self.slam_extent = None

        self.bounds_n = [math.inf, -math.inf]
        self.bounds_e = [math.inf, -math.inf]
        self.bounds_a = [math.inf, -math.inf]

        self.drone_n = None
        self.drone_e = None
        self.drone_agl = None
        self.drone_hdg = 0.0
        self.pose_source = "-"
        self.pose_priority = 0

        self.frontier_goal = None
        self.committed_goal = None
        self.entropy_goal = None
        self.frontier_waypoints_ne = np.zeros((0, 2), dtype=np.float32)
        self.active_waypoints_ne = np.zeros((0, 2), dtype=np.float32)
        self.coverage_pct = 0.0
        self.layer_coverage_pct = 0.0
        self.layer_altitude_m = None
        self.entropy_mean = 0.0
        self.mission_state = "PRIMING"
        self.mission_result = ""
        self.log_lines = deque(maxlen=500)
        self.target_north: float | None = None
        self.target_east:  float | None = None
        self.targets: list[dict[str, float]] = []
        self.target_merge_radius_m = 4.0

        # VFH unicycle vectors (NED radians)
        self.vfh_raw_yaw: float | None = None   # raw VFH histogram heading
        self.vfh_cmd_yaw: float | None = None   # smoothed command heading
        self._raw_vfh_pending: float = 0.0      # staging area until cmd_yaw arrives

        self.last_cloud_t = 0.0
        self.last_free_cloud_t = 0.0
        self.last_path_t = 0.0
        self.last_map_t = 0.0
        self.last_pose_t = 0.0
        self.last_summary_t = 0.0
        self.last_frontier_t = 0.0
        self.last_frontier_path_t = 0.0
        self.last_active_path_t = 0.0
        self.last_entropy_t = 0.0
        self.last_state_t = 0.0
        self.last_result_t = 0.0
        self.last_vfh_t = 0.0

    def _log_unlocked(self, msg: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_lines.appendleft(f"[{stamp}] [{self._label}] {msg}")

    def _touch_unlocked(self, key: str, label: str):
        now_t = time.monotonic()
        setattr(self, f"last_{key}_t", now_t)
        if key not in self._seen:
            self._seen.add(key)
            self._log_unlocked(f"{label} connected")
        return now_t

    def _expand_bounds_unlocked(self, north, east, alt):
        north = np.asarray(north, dtype=np.float32).reshape(-1)
        east = np.asarray(east, dtype=np.float32).reshape(-1)
        alt = np.asarray(alt, dtype=np.float32).reshape(-1)
        if north.size > 0:
            self.bounds_n[0] = min(self.bounds_n[0], float(np.min(north)))
            self.bounds_n[1] = max(self.bounds_n[1], float(np.max(north)))
        if east.size > 0:
            self.bounds_e[0] = min(self.bounds_e[0], float(np.min(east)))
            self.bounds_e[1] = max(self.bounds_e[1], float(np.max(east)))
        if alt.size > 0:
            self.bounds_a[0] = min(self.bounds_a[0], float(np.min(alt)))
            self.bounds_a[1] = max(self.bounds_a[1], float(np.max(alt)))

    def update_cloud(self, msg: PointCloud2):
        pts = _parse_cloud(msg)
        if len(pts) > self._MAX_VOXELS:
            step = len(pts) // self._MAX_VOXELS + 1
            pts = pts[::step][:self._MAX_VOXELS]
        with self._lock:
            self.voxels = pts
            self._touch_unlocked("cloud", "Voxel map")
            if len(pts) > 0:
                self._expand_bounds_unlocked(pts[:, 0], pts[:, 1], pts[:, 2])

    def update_free_cloud(self, msg: PointCloud2):
        pts = _parse_cloud(msg)
        if len(pts) > self._MAX_VOXELS:
            step = len(pts) // self._MAX_VOXELS + 1
            pts = pts[::step][:self._MAX_VOXELS]
        with self._lock:
            self.free_voxels = pts
            self._touch_unlocked("free_cloud", "Free voxel map")

    def update_path(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y, -p.pose.position.z) for p in msg.poses]
        with self._lock:
            if pts:
                self.path_nea = np.array(pts, dtype=np.float32)
                self._expand_bounds_unlocked(
                    self.path_nea[:, 0], self.path_nea[:, 1], self.path_nea[:, 2]
                )
            self._touch_unlocked("path", "Drone path")

    def update_slam_map(self, msg: OccupancyGrid):
        if msg.info.width == 0 or msg.info.height == 0 or len(msg.data) == 0:
            return
        grid = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        extent = (
            msg.info.origin.position.x,
            msg.info.origin.position.x + msg.info.width * msg.info.resolution,
            msg.info.origin.position.y,
            msg.info.origin.position.y + msg.info.height * msg.info.resolution,
        )
        with self._lock:
            self.slam_occ = grid
            self.slam_extent = extent
            self._touch_unlocked("map", "SLAM map")
            self.bounds_e[0] = min(self.bounds_e[0], float(extent[0]))
            self.bounds_e[1] = max(self.bounds_e[1], float(extent[1]))
            self.bounds_n[0] = min(self.bounds_n[0], float(extent[2]))
            self.bounds_n[1] = max(self.bounds_n[1], float(extent[3]))

    def update_position(self, n, e, agl, hdg, source: str, priority: int):
        now_t = time.monotonic()
        with self._lock:
            fresh_pose = self.last_pose_t > 0.0 and (now_t - self.last_pose_t) < 0.75
            if fresh_pose and priority < self.pose_priority:
                return
            old_source = self.pose_source
            self.drone_n = n
            self.drone_e = e
            self.drone_agl = agl
            self.drone_hdg = hdg
            self.pose_source = source
            self.pose_priority = priority
            setattr(self, "last_pose_t", now_t)
            if "pose" not in self._seen:
                self._seen.add("pose")
                self._log_unlocked(f"{source} pose connected")
            elif source != old_source:
                self._log_unlocked(f"Pose source -> {source}")
            self._expand_bounds_unlocked([n], [e], [0.0 if agl is None else agl])

    def update_telemetry(self, cov, ent, layer_cov=None, layer_alt=None):
        with self._lock:
            self.coverage_pct = cov * 100.0
            if layer_cov is not None and math.isfinite(float(layer_cov)):
                self.layer_coverage_pct = float(layer_cov) * 100.0
            if layer_alt is not None and math.isfinite(float(layer_alt)):
                self.layer_altitude_m = float(layer_alt)
            self.entropy_mean = ent
            self._touch_unlocked("summary", "Map summary")

    def update_frontier(self, n, e, a):
        with self._lock:
            self.frontier_goal = (n, e, a)
            self._touch_unlocked("frontier", "Frontier goal")
            self._expand_bounds_unlocked([n], [e], [a])

    def update_committed_goal(self, n, e, a):
        with self._lock:
            self.committed_goal = (n, e, a)

    def update_entropy_centroid(self, n, e, a):
        with self._lock:
            self.entropy_goal = (n, e, a)
            self._touch_unlocked("entropy", "Entropy centroid")
            self._expand_bounds_unlocked([n], [e], [a])

    def update_waypoints(self, pts, source="frontier"):
        with self._lock:
            arr = np.array(pts, dtype=np.float32) if pts else np.zeros((0, 2), dtype=np.float32)
            now_t = time.monotonic()
            if source == "active":
                self.active_waypoints_ne = arr
                self.last_active_path_t = now_t
                if not pts:
                    self.committed_goal = None
            else:
                self.frontier_waypoints_ne = arr
                self.last_frontier_path_t = now_t
                if not pts:
                    self.frontier_goal = None

    def update_state(self, text: str):
        with self._lock:
            if text != self.mission_state:
                self.mission_state = text
                self._log_unlocked(f"State -> {text}")
            self._touch_unlocked("state", "Mission state")

    def update_vfh(self, raw_yaw: float, cmd_yaw: float):
        with self._lock:
            self.vfh_raw_yaw = raw_yaw
            self.vfh_cmd_yaw = cmd_yaw
            self._touch_unlocked("vfh", "VFH vectors")

    def update_result(self, text: str):
        with self._lock:
            if text != self.mission_result:
                self.mission_result = text
                self._log_unlocked(f"RESULT: {text}")
            self._touch_unlocked("result", "Mission result")

    def update_target(self, north: float, east: float):
        with self._lock:
            self.target_north = north
            self.target_east  = east
            for target in self.targets:
                if math.hypot(north - target["north"], east - target["east"]) <= self.target_merge_radius_m:
                    count = target["count"]
                    old_weight = min(count, 8.0)
                    denom = old_weight + 1.0
                    target["north"] = (target["north"] * old_weight + north) / denom
                    target["east"] = (target["east"] * old_weight + east) / denom
                    target["count"] = count + 1.0
                    target["last_seen"] = time.monotonic()
                    return

            self.targets.append({
                "north": north,
                "east": east,
                "count": 1.0,
                "last_seen": time.monotonic(),
            })
            self._log_unlocked(
                f"TARGET {len(self.targets)} DETECTED at N={north:.2f} E={east:.2f}"
            )

    def log(self, msg: str):
        with self._lock:
            self._log_unlocked(msg)

    def clear_log(self):
        with self._lock:
            self.log_lines.clear()

    def snapshot(self):
        def _norm(bounds, default):
            lo, hi = bounds
            if math.isfinite(lo) and math.isfinite(hi):
                return (lo, hi)
            return default

        def _age(now_t: float, stamp_t: float) -> float:
            return math.inf if stamp_t <= 0.0 else max(0.0, now_t - stamp_t)

        with self._lock:
            now_t = time.monotonic()
            active_count = len(self.active_waypoints_ne)
            frontier_count = len(self.frontier_waypoints_ne)
            use_active_path = active_count > 0
            selected_waypoints = (
                self.active_waypoints_ne if use_active_path else self.frontier_waypoints_ne
            )
            selected_source = "active" if use_active_path else "frontier"
            return {
                "voxels": self.voxels.copy(),
                "free_voxels": self.free_voxels.copy(),
                "path_nea": self.path_nea.copy(),
                "slam_occ": None if self.slam_occ is None else self.slam_occ.copy(),
                "slam_extent": self.slam_extent,
                "bounds_n": _norm(self.bounds_n, (-1.0, 1.0)),
                "bounds_e": _norm(self.bounds_e, (-1.0, 1.0)),
                "bounds_a": _norm(self.bounds_a, (0.0, 1.0)),
                "drone_n": self.drone_n,
                "drone_e": self.drone_e,
                "drone_agl": self.drone_agl,
                "drone_hdg": self.drone_hdg,
                "pose_source": self.pose_source,
                "frontier_goal": self.frontier_goal,
                "committed_goal": self.committed_goal,
                "entropy_goal": self.entropy_goal,
                "waypoints_ne": selected_waypoints.copy(),
                "waypoints_source": selected_source,
                "active_waypoint_count": active_count,
                "frontier_waypoint_count": frontier_count,
                "coverage_pct": self.coverage_pct,
                "layer_coverage_pct": self.layer_coverage_pct,
                "layer_altitude_m": self.layer_altitude_m,
                "entropy_mean": self.entropy_mean,
                "mission_state": self.mission_state,
                "mission_result": self.mission_result,
                "log_lines": list(self.log_lines),
                "target_north": self.target_north,
                "target_east":  self.target_east,
                "targets": [dict(target) for target in self.targets],
                "age_cloud_s": _age(now_t, self.last_cloud_t),
                "age_free_cloud_s": _age(now_t, self.last_free_cloud_t),
                "age_pose_s": _age(now_t, self.last_pose_t),
                "age_state_s": _age(now_t, self.last_state_t),
                "age_frontier_s": _age(now_t, self.last_frontier_t),
                "age_frontier_path_s": _age(now_t, self.last_frontier_path_t),
                "age_active_path_s": _age(now_t, self.last_active_path_t),
                "age_entropy_s": _age(now_t, self.last_entropy_t),
                "age_summary_s": _age(now_t, self.last_summary_t),
                "vfh_raw_yaw": self.vfh_raw_yaw,
                "vfh_cmd_yaw": self.vfh_cmd_yaw,
                "age_vfh_s": _age(now_t, self.last_vfh_t),
            }


_state = SharedState("D0")
_state1 = SharedState("D1")

_merged_lock = threading.Lock()
_merged_voxels = np.zeros((0, 3), dtype=np.float32)
_merged_stamp_t = 0.0
_merged_seen = False
_shared_summary_lock = threading.Lock()
_shared_layer_cov: dict[float, float] = {}
_shared_cov_pct = 0.0
_shared_summary_stamp_t = 0.0


class DashboardNode(Node):
    def __init__(self):
        super().__init__("mission_dashboard")

        ns = self.declare_parameter("drone_ns", "px4_0").value
        ns1 = self.declare_parameter("drone1_ns", "").value
        num_drones = int(self.declare_parameter("num_drones", 1).value)
        self._d1_north = float(self.declare_parameter("drone1_north_m", 0.0).value)
        self._d1_east = float(self.declare_parameter("drone1_east_m", 0.0).value)
        self._gazebo_model0 = self.declare_parameter("gazebo_model0", "x500_swarm_0").value
        self._gazebo_model1 = self.declare_parameter("gazebo_model1", "x500_swarm_1").value

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
        tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._dual = num_drones >= 2 and bool(ns1)
        self._cmd_pubs = [
            self.create_publisher(String, f"/{ns}/dashboard/cmd", 1),
        ]
        if self._dual:
            self._cmd_pubs.append(self.create_publisher(String, f"/{ns1}/dashboard/cmd", 1))

        # Drone 0 subscriptions
        self.create_subscription(PointCloud2, f"/{ns}/voxel_map", self._cloud_cb, be)
        self.create_subscription(PointCloud2, f"/{ns}/free_voxel_map", self._free_cloud_cb, be)
        self.create_subscription(Path, f"/{ns}/drone_path", self._path_cb, be)
        self.create_subscription(OccupancyGrid, f"/{ns}/map", self._slam_map_cb, tl)
        self.create_subscription(MapUpdateSummary, f"/{ns}/map_update_summary", self._summary_cb, rel)
        self.create_subscription(PointStamped, f"/{ns}/frontier_goal", self._frontier_cb, rel)
        self.create_subscription(PointStamped, f"/{ns}/committed_goal", self._committed_goal_cb, rel)
        self.create_subscription(PointStamped, f"/{ns}/entropy_centroid", self._centroid_cb, rel)
        self.create_subscription(Path, f"/{ns}/frontier_path", self._frontier_path_cb, tl)
        self.create_subscription(Path, f"/{ns}/active_path", self._active_path_cb, tl)
        self.create_subscription(Odometry, f"/{ns}/slam/odom_ned", self._nav_pose_cb, be)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._position_cb, be
        )
        self.create_subscription(String, f"/{ns}/mission/state", self._state_cb, tl)
        self.create_subscription(String, f"/{ns}/mission/result", self._result_cb, tl)
        self.create_subscription(TargetFound, f"/{ns}/target_found", self._target_cb, rel)
        self.create_subscription(Float32, f"/{ns}/telemetry/raw_vfh_yaw", self._raw_vfh_cb, be)
        self.create_subscription(Float32, f"/{ns}/telemetry/cmd_yaw",     self._cmd_yaw_cb, be)

        if self._dual:
            self.create_subscription(PointCloud2, f"/{ns1}/voxel_map", self._cloud1_cb, be)
            self.create_subscription(PointCloud2, f"/{ns1}/free_voxel_map", self._free_cloud1_cb, be)
            self.create_subscription(Path, f"/{ns1}/drone_path", self._path1_cb, be)
            self.create_subscription(OccupancyGrid, f"/{ns1}/map", self._slam_map1_cb, tl)
            self.create_subscription(MapUpdateSummary, f"/{ns1}/map_update_summary", self._summary1_cb, rel)
            self.create_subscription(PointStamped, f"/{ns1}/frontier_goal", self._frontier1_cb, rel)
            self.create_subscription(PointStamped, f"/{ns1}/committed_goal", self._committed_goal1_cb, rel)
            self.create_subscription(PointStamped, f"/{ns1}/entropy_centroid", self._centroid1_cb, rel)
            self.create_subscription(Path, f"/{ns1}/frontier_path", self._frontier_path1_cb, tl)
            self.create_subscription(Path, f"/{ns1}/active_path", self._active_path1_cb, tl)
            self.create_subscription(Odometry, f"/{ns1}/slam/odom_ned", self._nav_pose1_cb, be)
            self.create_subscription(
                VehicleLocalPosition,
                f"/{ns1}/fmu/out/vehicle_local_position_v1",
                self._position1_cb,
                be,
            )
            self.create_subscription(String, f"/{ns1}/mission/state", self._state1_cb, tl)
            self.create_subscription(String, f"/{ns1}/mission/result", self._result1_cb, tl)
            self.create_subscription(TargetFound, f"/{ns1}/target_found", self._target1_cb, rel)
            self.create_subscription(Float32, f"/{ns1}/telemetry/raw_vfh_yaw", self._raw_vfh1_cb, be)
            self.create_subscription(Float32, f"/{ns1}/telemetry/cmd_yaw",     self._cmd_yaw1_cb, be)
            self.create_subscription(PointCloud2, "/merged_voxel_map", self._merged_cb, be)
            self.create_subscription(
                MapUpdateSummary,
                "/shared_map_update_summary",
                self._shared_summary_cb,
                rel,
            )

        _state.log("Dashboard started, waiting for telemetry...")

    def send_cmd(self, text: str):
        msg = String()
        msg.data = text
        for pub in self._cmd_pubs:
            pub.publish(msg)
        _state.log(f"Command broadcast: {text}")

    def follow_gazebo(self, drone_label: str):
        if drone_label == "D0":
            model = self._gazebo_model0
        elif drone_label == "D1":
            model = self._gazebo_model1
        else:
            _state.log("Gazebo follow: choose D0 or D1")
            return

        threading.Thread(
            target=self._follow_gazebo_worker,
            args=(drone_label, model),
            daemon=True,
        ).start()

    def _follow_gazebo_worker(self, drone_label: str, model: str):
        cmd = [
            "gz",
            "service",
            "-s",
            "/gui/follow",
            "--reqtype",
            "gz.msgs.StringMsg",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1200",
            "--req",
            f'data: "{model}"',
        ]
        try:
            res = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            _state.log(f"Gazebo follow {drone_label} failed: {exc}")
            return

        text = (res.stdout + " " + res.stderr).strip()
        if res.returncode == 0 and "false" not in text.lower():
            _state.log(f"Gazebo follow -> {drone_label} ({model})")
        else:
            detail = f": {text}" if text else ""
            _state.log(f"Gazebo follow {drone_label} was not accepted{detail}")

    def _update_nav_pose(self, store: SharedState, msg: Odometry):
        q = msg.pose.pose.orientation
        store.update_position(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(-msg.pose.pose.position.z),
            _yaw_from_xyzw(float(q.x), float(q.y), float(q.z), float(q.w)),
            "SLAM",
            2,
        )

    def _update_px4_pose(self, store: SharedState, msg: VehicleLocalPosition):
        if not msg.xy_valid:
            return
        store.update_position(
            float(msg.x),
            float(msg.y),
            float(-msg.z) if msg.z_valid else None,
            float(msg.heading) if math.isfinite(msg.heading) else 0.0,
            "PX4",
            1,
        )

    def _cloud_cb(self, msg): _state.update_cloud(msg)
    def _free_cloud_cb(self, msg): _state.update_free_cloud(msg)
    def _path_cb(self, msg): _state.update_path(msg)
    def _slam_map_cb(self, msg): _state.update_slam_map(msg)
    def _summary_cb(self, msg):
        _state.update_telemetry(
            msg.coverage_fraction,
            msg.entropy_mean,
            msg.layer_coverage_fraction,
            msg.layer_altitude_m,
        )
    def _frontier_cb(self, msg): _state.update_frontier(msg.point.x, msg.point.y, -msg.point.z)
    def _committed_goal_cb(self, msg): _state.update_committed_goal(msg.point.x, msg.point.y, -msg.point.z)
    def _centroid_cb(self, msg): _state.update_entropy_centroid(msg.point.x, msg.point.y, -msg.point.z)
    def _frontier_path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        _state.update_waypoints(pts, "frontier")
    def _active_path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        _state.update_waypoints(pts, "active")
    def _nav_pose_cb(self, msg): self._update_nav_pose(_state, msg)
    def _position_cb(self, msg): self._update_px4_pose(_state, msg)
    def _state_cb(self, msg): _state.update_state(msg.data)
    def _result_cb(self, msg): _state.update_result(msg.data)
    def _target_cb(self, msg: TargetFound): _state.update_target(msg.position_world.x, msg.position_world.y)
    def _raw_vfh_cb(self,  msg): _state._raw_vfh_pending  = float(msg.data)
    def _cmd_yaw_cb(self,  msg): _state.update_vfh(_state._raw_vfh_pending, float(msg.data))

    def _cloud1_cb(self, msg): _state1.update_cloud(msg)
    def _free_cloud1_cb(self, msg): _state1.update_free_cloud(msg)
    def _path1_cb(self, msg): _state1.update_path(msg)
    def _slam_map1_cb(self, msg): _state1.update_slam_map(msg)
    def _summary1_cb(self, msg):
        _state1.update_telemetry(
            msg.coverage_fraction,
            msg.entropy_mean,
            msg.layer_coverage_fraction,
            msg.layer_altitude_m,
        )
    def _frontier1_cb(self, msg): _state1.update_frontier(msg.point.x, msg.point.y, -msg.point.z)
    def _committed_goal1_cb(self, msg): _state1.update_committed_goal(msg.point.x, msg.point.y, -msg.point.z)
    def _centroid1_cb(self, msg): _state1.update_entropy_centroid(msg.point.x, msg.point.y, -msg.point.z)
    def _frontier_path1_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        _state1.update_waypoints(pts, "frontier")
    def _active_path1_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        _state1.update_waypoints(pts, "active")
    def _nav_pose1_cb(self, msg): self._update_nav_pose(_state1, msg)
    def _position1_cb(self, msg): self._update_px4_pose(_state1, msg)
    def _state1_cb(self, msg): _state1.update_state(msg.data)
    def _result1_cb(self, msg): _state1.update_result(msg.data)
    def _target1_cb(self, msg: TargetFound): _state1.update_target(msg.position_world.x, msg.position_world.y)
    def _raw_vfh1_cb(self, msg): _state1._raw_vfh_pending = float(msg.data)
    def _cmd_yaw1_cb(self, msg): _state1.update_vfh(_state1._raw_vfh_pending, float(msg.data))

    def _merged_cb(self, msg: PointCloud2):
        global _merged_voxels, _merged_stamp_t, _merged_seen
        pts = _parse_cloud(msg)
        with _merged_lock:
            _merged_voxels = pts
            _merged_stamp_t = time.monotonic()
        if not _merged_seen:
            _merged_seen = True
            _state.log("Shared merged map connected")

    def _shared_summary_cb(self, msg: MapUpdateSummary):
        global _shared_cov_pct, _shared_summary_stamp_t
        if msg.layer_total_cells == 0:
            return
        layer_alt = round(float(msg.layer_altitude_m), 2)
        with _shared_summary_lock:
            _shared_cov_pct = float(msg.coverage_fraction) * 100.0
            _shared_layer_cov[layer_alt] = float(msg.layer_coverage_fraction) * 100.0
            _shared_summary_stamp_t = time.monotonic()


class DashboardWindow:
    def __init__(self, root: tk.Tk, node: DashboardNode):
        self.root = root
        self.node = node
        self._dual = node._dual
        self._closed = False
        self._refresh_after_id = None

        root.title("Swarm Mission Dashboard" + (" — 2 Drones" if self._dual else ""))
        root.geometry("1680x940")
        root.configure(bg="#0d0d1a")
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#0d0d1a")
        style.configure("TLabelframe", background="#0d0d1a", foreground="#ccccff")
        style.configure("TLabelframe.Label", background="#0d0d1a", foreground="#ccccff")
        style.configure("TLabel", background="#0d0d1a", foreground="#aaddff")
        style.configure("D0.TLabel", background="#0d0d1a", foreground="#00eeff")
        style.configure("D1.TLabel", background="#0d0d1a", foreground="#ffaa00")
        style.configure("TButton", padding=6)

        main = ttk.Frame(root)
        main.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.rowconfigure(0, weight=1)
        left.rowconfigure(1, weight=0)
        left.columnconfigure(0, weight=1)

        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(3, weight=1)
        right.columnconfigure(0, weight=1)

        # Persist user-rotated view between redraws (overrides _style_ax defaults).
        self._view_elev = 28.0
        self._view_azim = -58.0
        self.map_focus_var = tk.StringVar(value="All")
        self.gz_focus_var = tk.StringVar(value="D0")
        self.show_d0_var = tk.BooleanVar(value=True)
        self.show_d1_var = tk.BooleanVar(value=True)

        self._build_plot(left)
        self._build_map_controls(left)
        self._build_status_d0(right)
        if self._dual:
            self._build_status_d1(right)
        self._build_commands(right)
        self._build_log(right)

        self._schedule_refresh()

    def _schedule_refresh(self):
        if not self._closed:
            self._refresh_after_id = self.root.after(1000, self.refresh)

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._refresh_after_id is not None:
            try:
                self.root.after_cancel(self._refresh_after_id)
            except tk.TclError:
                pass
            self._refresh_after_id = None
        try:
            self.root.quit()
            self.root.destroy()
        except tk.TclError:
            pass

    def _build_plot(self, parent):
        self.fig = Figure(facecolor="#0d0d1a")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._style_ax()
        self.fig.subplots_adjust(left=0.02, right=0.98, bottom=0.04, top=0.97)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _build_map_controls(self, parent):
        frame = ttk.LabelFrame(parent, text="Map View")
        frame.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        for col in range(4):
            frame.columnconfigure(col, weight=1)

        if self._dual:
            ttk.Checkbutton(frame, text="D0", variable=self.show_d0_var).grid(
                row=0, column=0, sticky="w", padx=8, pady=(8, 2)
            )
            ttk.Checkbutton(frame, text="D1", variable=self.show_d1_var).grid(
                row=0, column=1, sticky="w", padx=8, pady=(8, 2)
            )
        else:
            ttk.Label(frame, text="D0").grid(row=0, column=0, sticky="w", padx=8, pady=(8, 2))

        ttk.Label(frame, text="Map focus").grid(row=1, column=0, sticky="w", padx=8, pady=(6, 2))
        map_combo = ttk.Combobox(
            frame,
            textvariable=self.map_focus_var,
            values=("All", "D0", "D1") if self._dual else ("All", "D0"),
            state="readonly",
            width=8,
        )
        map_combo.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 8))

        if self._dual:
            ttk.Label(frame, text="Gazebo focus").grid(row=1, column=2, sticky="w", padx=8, pady=(6, 2))
            gz_combo = ttk.Combobox(
                frame,
                textvariable=self.gz_focus_var,
                values=("D0", "D1"),
                state="readonly",
                width=8,
            )
            gz_combo.grid(row=2, column=2, sticky="ew", padx=8, pady=(0, 8))
            gz_combo.bind("<<ComboboxSelected>>", self._gazebo_focus_changed)
            ttk.Button(frame, text="Follow", command=self._gazebo_focus_changed).grid(
                row=2, column=3, sticky="ew", padx=8, pady=(0, 8)
            )

        ttk.Button(frame, text="Reset Map View", command=self._reset_view).grid(
            row=3, column=0, columnspan=4, sticky="ew", padx=8, pady=(0, 8)
        )

    def _style_ax(self):
        ax = self.ax
        ax.set_facecolor("#080818")
        ax.set_xlabel("East (m)", color="#aaaacc", labelpad=10)
        ax.set_ylabel("North (m)", color="#aaaacc", labelpad=10)
        ax.set_zlabel("Altitude (m)", color="#aaaacc", labelpad=10)
        ax.set_title("Shared 3D Swarm Map" if self._dual else "3D Voxel Map", color="#ccccff")
        ax.tick_params(colors="#aaaacc", labelsize=8)
        ax.grid(True, color="#222244", alpha=0.35)
        ax.xaxis.set_pane_color((0.03, 0.03, 0.08, 1.0))
        ax.yaxis.set_pane_color((0.03, 0.03, 0.08, 1.0))
        ax.zaxis.set_pane_color((0.04, 0.04, 0.10, 1.0))
        ax.view_init(elev=28, azim=-58)

    def _build_status_d0(self, parent):
        lbl = "Drone 0 (Cyan)" if self._dual else "Status"
        frame = ttk.LabelFrame(parent, text=lbl)
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        frame.columnconfigure(0, weight=1)

        self.state_var = tk.StringVar(value="Mode: PRIMING")
        self.pos_var = tk.StringVar(value="Position: -")
        self.alt_var = tk.StringVar(value="Altitude: -")
        self.hdg_var = tk.StringVar(value="Heading: -")
        self.goal_var = tk.StringVar(value="Goal: -")
        self.ent_var = tk.StringVar(value="Entropy: -")
        self.vox_var = tk.StringVar(value="Voxels: 0")
        self.link_var = tk.StringVar(value="Links: waiting")
        self.res_var = tk.StringVar(value="Result: -")

        for i, var in enumerate([
            self.state_var,
            self.pos_var,
            self.alt_var,
            self.hdg_var,
            self.goal_var,
            self.ent_var,
            self.vox_var,
            self.link_var,
            self.res_var,
        ]):
            ttk.Label(frame, textvariable=var, style="D0.TLabel").grid(
                row=i, column=0, sticky="w", padx=8, pady=2
            )

        self.vfh_var = tk.StringVar(value="VFH: -")
        ttk.Label(frame, textvariable=self.vfh_var, style="D0.TLabel").grid(
            row=9, column=0, sticky="w", padx=8, pady=2
        )
        self.cov_var = tk.StringVar(value="Coverage: 0.0 %")
        ttk.Label(frame, textvariable=self.cov_var, style="D0.TLabel").grid(
            row=10, column=0, sticky="w", padx=8, pady=(4, 2)
        )
        self.layer_cov_var = tk.StringVar(value="Layer coverage: 0.0 %")
        ttk.Label(frame, textvariable=self.layer_cov_var, style="D0.TLabel").grid(
            row=11, column=0, sticky="w", padx=8, pady=2
        )
        self.progress = ttk.Progressbar(
            frame, orient=tk.HORIZONTAL, mode="determinate", maximum=100.0
        )
        self.progress.grid(row=12, column=0, sticky="ew", padx=8, pady=(0, 6))

    def _build_status_d1(self, parent):
        frame = ttk.LabelFrame(parent, text="Drone 1 (Amber)")
        frame.grid(row=1, column=0, sticky="ew", pady=(0, 4))
        frame.columnconfigure(0, weight=1)

        self.state1_var = tk.StringVar(value="Mode: PRIMING")
        self.pos1_var = tk.StringVar(value="Position: -")
        self.alt1_var = tk.StringVar(value="Altitude: -")
        self.hdg1_var = tk.StringVar(value="Heading: -")
        self.goal1_var = tk.StringVar(value="Goal: -")
        self.ent1_var = tk.StringVar(value="Entropy: -")
        self.vox1_var = tk.StringVar(value="Voxels: 0")
        self.link1_var = tk.StringVar(value="Links: waiting")
        self.res1_var = tk.StringVar(value="Result: -")

        for i, var in enumerate([
            self.state1_var,
            self.pos1_var,
            self.alt1_var,
            self.hdg1_var,
            self.goal1_var,
            self.ent1_var,
            self.vox1_var,
            self.link1_var,
            self.res1_var,
        ]):
            ttk.Label(frame, textvariable=var, style="D1.TLabel").grid(
                row=i, column=0, sticky="w", padx=8, pady=2
            )

        self.vfh1_var = tk.StringVar(value="VFH: -")
        ttk.Label(frame, textvariable=self.vfh1_var, style="D1.TLabel").grid(
            row=9, column=0, sticky="w", padx=8, pady=2
        )
        self.cov1_var = tk.StringVar(value="Coverage: 0.0 %")
        ttk.Label(frame, textvariable=self.cov1_var, style="D1.TLabel").grid(
            row=10, column=0, sticky="w", padx=8, pady=(4, 2)
        )
        self.layer_cov1_var = tk.StringVar(value="Layer coverage: 0.0 %")
        ttk.Label(frame, textvariable=self.layer_cov1_var, style="D1.TLabel").grid(
            row=11, column=0, sticky="w", padx=8, pady=2
        )
        self.progress1 = ttk.Progressbar(
            frame, orient=tk.HORIZONTAL, mode="determinate", maximum=100.0
        )
        self.progress1.grid(row=12, column=0, sticky="ew", padx=8, pady=(0, 6))

        self.merged_vox_var = tk.StringVar(value="Merged voxels: 0")
        self.shared_var = tk.StringVar(value="Shared map: waiting")
        ttk.Label(frame, textvariable=self.merged_vox_var).grid(
            row=13, column=0, sticky="w", padx=8, pady=(0, 2)
        )
        ttk.Label(frame, textvariable=self.shared_var).grid(
            row=14, column=0, sticky="w", padx=8, pady=(0, 4)
        )

    def _reset_view(self):
        self._view_elev = 28.0
        self._view_azim = -58.0
        self.ax.view_init(elev=self._view_elev, azim=self._view_azim)
        self.canvas.draw_idle()

    def _gazebo_focus_changed(self, *_):
        focus = self.gz_focus_var.get()
        if focus in ("D0", "D1"):
            self.node.follow_gazebo(focus)

    def _build_commands(self, parent):
        frame = ttk.LabelFrame(parent, text="Commands")
        frame.grid(row=2, column=0, sticky="ew", pady=(0, 4))
        frame.columnconfigure(0, weight=1)
        row = 0
        ttk.Button(frame, text="Return Home", command=lambda: self.node.send_cmd("RETURN")).grid(
            row=row, column=0, sticky="ew", padx=8, pady=(8, 4)
        )
        row += 1
        ttk.Button(frame, text="LAND", command=lambda: self.node.send_cmd("LAND")).grid(
            row=row, column=0, sticky="ew", padx=8, pady=4
        )
        row += 1
        ttk.Button(
            frame,
            text="EMERGENCY DISARM",
            command=lambda: self.node.send_cmd("DISARM"),
        ).grid(row=row, column=0, sticky="ew", padx=8, pady=4)
        row += 1
        ttk.Button(frame, text="Reset View", command=self._reset_view).grid(
            row=row, column=0, sticky="ew", padx=8, pady=4
        )
        row += 1
        ttk.Button(frame, text="Clear Log", command=self._clear_logs).grid(
            row=row, column=0, sticky="ew", padx=8, pady=(4, 8)
        )

    def _build_log(self, parent):
        frame = ttk.LabelFrame(parent, text="Log")
        frame.grid(row=3, column=0, sticky="nsew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        self.log_text = tk.Text(
            frame,
            bg="#060612",
            fg="#99cc99",
            insertbackground="#99cc99",
            relief=tk.FLAT,
            font=("Courier", 9),
        )
        self.log_text.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        sb = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.log_text.yview)
        sb.grid(row=0, column=1, sticky="ns", pady=6)
        self.log_text.configure(yscrollcommand=sb.set)
        # Color tags for log severity
        self.log_text.tag_config("target",  foreground="#FF8C00", font=("Courier", 9, "bold"))
        self.log_text.tag_config("warning", foreground="#FFD700")
        self.log_text.tag_config("error",   foreground="#FF4444", font=("Courier", 9, "bold"))
        self.log_text.tag_config("state",   foreground="#4FC3F7")
        self.log_text.tag_config("info",    foreground="#99cc99")

    def _clear_logs(self):
        _state.clear_log()
        _state1.clear_log()

    def _globalize_snapshot(self, data, north_offset: float, east_offset: float):
        out = dict(data)

        def _shift_cloud(cloud):
            shifted = cloud.copy()
            if len(shifted) > 0:
                shifted[:, 0] += north_offset
                shifted[:, 1] += east_offset
            return shifted

        def _shift_goal(goal):
            if goal is None:
                return None
            return (goal[0] + north_offset, goal[1] + east_offset, goal[2])

        out["voxels"] = _shift_cloud(data["voxels"])
        out["free_voxels"] = _shift_cloud(data["free_voxels"])
        out["path_nea"] = _shift_cloud(data["path_nea"])
        out["frontier_goal"] = _shift_goal(data["frontier_goal"])
        out["committed_goal"] = _shift_goal(data["committed_goal"])
        out["entropy_goal"] = _shift_goal(data["entropy_goal"])
        out["targets"] = [
            {
                **target,
                "north": target.get("north", 0.0) + north_offset,
                "east": target.get("east", 0.0) + east_offset,
            }
            for target in data.get("targets", [])
        ]
        wps = data.get("waypoints_ne", np.zeros((0, 2), dtype=np.float32))
        if len(wps) > 0:
            shifted_wps = wps.copy()
            shifted_wps[:, 0] += north_offset
            shifted_wps[:, 1] += east_offset
            out["waypoints_ne"] = shifted_wps
        else:
            out["waypoints_ne"] = wps
        out["drone_n"] = None if data["drone_n"] is None else data["drone_n"] + north_offset
        out["drone_e"] = None if data["drone_e"] is None else data["drone_e"] + east_offset
        out["bounds_n"] = (data["bounds_n"][0] + north_offset, data["bounds_n"][1] + north_offset)
        out["bounds_e"] = (data["bounds_e"][0] + east_offset, data["bounds_e"][1] + east_offset)
        if data["slam_extent"] is not None:
            e0, e1, n0, n1 = data["slam_extent"]
            out["slam_extent"] = (e0 + east_offset, e1 + east_offset, n0 + north_offset, n1 + north_offset)
        return out

    def _draw_drone(self, data, color, path_color, label, endpoint_color):
        path = data["path_nea"]
        dn = data["drone_n"]
        de = data["drone_e"]
        da = data["drone_agl"]
        hdg = data["drone_hdg"]
        fg = data["frontier_goal"]
        cg = data.get("committed_goal")
        eg = data["entropy_goal"]

        if len(path) > 1:
            self.ax.plot(path[:, 1], path[:, 0], path[:, 2], color=path_color, linewidth=1.5, label=f"Path {label}")

        if dn is not None and de is not None:
            alt = 0.0 if da is None else da
            self.ax.scatter([de], [dn], [alt], c=color, s=90, edgecolors="white", linewidths=0.8, depthshade=False)
            self.ax.plot([de, de], [dn, dn], [0.0, alt], color=color, alpha=0.25, linewidth=1.0)
            self.ax.quiver(
                de,
                dn,
                alt,
                math.sin(hdg),
                math.cos(hdg),
                0.0,
                length=1.6,
                normalize=True,
                color=color,
                linewidth=1.2,
            )
            self.ax.text(de + 0.3, dn + 0.3, alt + 0.1, label, color=color, fontsize=8)

        if dn is not None and de is not None:
            alt = 0.0 if da is None else da
            # VFH raw heading — yellow dashed arrow
            raw_y = data.get("vfh_raw_yaw")
            cmd_y = data.get("vfh_cmd_yaw")
            vfh_age = data.get("age_vfh_s", math.inf)
            if raw_y is not None and vfh_age < 2.0:
                self.ax.quiver(
                    de, dn, alt,
                    math.sin(raw_y), math.cos(raw_y), 0.0,
                    length=1.5, normalize=True,
                    color="#ffff00", linewidth=1.0, alpha=0.75,
                    arrow_length_ratio=0.25,
                )
            # VFH cmd heading — cyan solid arrow
            if cmd_y is not None and vfh_age < 2.0:
                self.ax.quiver(
                    de, dn, alt,
                    math.sin(cmd_y), math.cos(cmd_y), 0.0,
                    length=2.0, normalize=True,
                    color="#00ffff", linewidth=1.4, alpha=0.90,
                    arrow_length_ratio=0.20,
                )

        # Draw controller active path when available; otherwise show mapper frontier path.
        wps = data.get("waypoints_ne", np.zeros((0, 2), dtype=np.float32))
        wp_source = data.get("waypoints_source", "frontier")
        wp_style = "-" if wp_source == "active" else "--"
        wp_alpha = 0.85 if wp_source == "active" else 0.60
        alt_wp = 0.0 if da is None else da
        if len(wps) >= 1:
            self.ax.scatter([wps[0, 1]], [wps[0, 0]], [alt_wp],
                            c="#ffffff", s=80, marker="s", edgecolors=color,
                            linewidths=0.9, depthshade=False, alpha=0.95)
        if len(wps) >= 2:
            self.ax.plot(wps[:, 1], wps[:, 0], [alt_wp] * len(wps),
                         color=color, linewidth=1.2, linestyle=wp_style, alpha=wp_alpha)
            self.ax.scatter(wps[1:, 1], wps[1:, 0], [alt_wp] * (len(wps) - 1),
                            c=color, s=30, marker="o", edgecolors="white", linewidths=0.4,
                            depthshade=False, alpha=wp_alpha)

        # Frontier endpoint is the mapper's route destination, not always the active
        # waypoint the controller is following right now.
        if fg is not None:
            self.ax.scatter([fg[1]], [fg[0]], [fg[2]], c=endpoint_color, s=150, marker="X",
                            edgecolors="black", linewidths=0.5, depthshade=False)

        # Single star = committed goal (what the controller is actually flying toward now).
        if cg is not None:
            self.ax.scatter([cg[1]], [cg[0]], [cg[2]], c=color, s=260, marker="*",
                            edgecolors="white", linewidths=0.8, depthshade=False)
        if eg is not None and (fg is None or abs(eg[0] - fg[0]) > 0.5 or abs(eg[1] - fg[1]) > 0.5):
            self.ax.scatter([eg[1]], [eg[0]], [eg[2]], c=color, s=60, marker="D",
                            edgecolors="white", linewidths=0.5, depthshade=False)

    def _redraw_plot(self, data0, data1=None):
        # Save whatever angle the user has rotated to before clearing.
        self._view_elev = self.ax.elev
        self._view_azim = self.ax.azim
        self.ax.cla()
        self._style_ax()
        # Restore the user's view (overrides the default in _style_ax).
        self.ax.view_init(elev=self._view_elev, azim=self._view_azim)

        show_d0 = True if not self._dual else bool(self.show_d0_var.get())
        show_d1 = bool(self._dual and data1 is not None and self.show_d1_var.get())

        vox = np.zeros((0, 3), dtype=np.float32)
        if self._dual:
            if show_d0 and show_d1:
                with _merged_lock:
                    vox = _merged_voxels.copy()
            if len(vox) == 0 and data1 is not None:
                clouds = []
                if show_d0 and len(data0["voxels"]):
                    clouds.append(data0["voxels"])
                if show_d1 and len(data1["voxels"]):
                    clouds.append(data1["voxels"])
                vox = np.vstack(clouds) if clouds else vox
        else:
            vox = data0["voxels"]

        if len(vox) > 0:
            colors = vox[:, 2]
            self.ax.scatter(
                vox[:, 1],
                vox[:, 0],
                vox[:, 2],
                c=colors,
                cmap="inferno",
                s=4,
                alpha=0.70,
                linewidths=0,
                depthshade=False,
            )
        else:
            self.ax.text2D(0.05, 0.95, "Waiting for voxel_map / merged_voxel_map ...", transform=self.ax.transAxes, color="#ccccff")

        free_clouds = []
        if show_d0:
            free_clouds.append(data0.get("free_voxels", np.zeros((0, 3), dtype=np.float32)))
        if show_d1:
            free_clouds.append(data1.get("free_voxels", np.zeros((0, 3), dtype=np.float32)))
        free_vox = np.vstack([c for c in free_clouds if len(c)]) if any(len(c) for c in free_clouds) else np.zeros((0, 3), dtype=np.float32)
        if len(free_vox) > 0:
            self.ax.scatter(
                free_vox[:, 1],
                free_vox[:, 0],
                free_vox[:, 2],
                c="#00ff00",
                s=6,
                alpha=0.10,
                linewidths=0,
                depthshade=False,
            )

        if show_d0:
            self._draw_drone(data0, "#00eeff", "#4488ff", "D0", "#ffcc33")
        if show_d1:
            self._draw_drone(data1, "#ffaa00", "#ff6600", "D1", "#ff44aa")

        # Draw all clustered targets reported by the active drone dashboards.
        target_points = []
        target_snaps = []
        if show_d0:
            target_snaps.append(data0)
        if show_d1:
            target_snaps.append(data1)
        for snap in target_snaps:
            for target in snap.get("targets", []):
                tn = target.get("north")
                te = target.get("east")
                if tn is None or te is None:
                    continue
                if any(math.hypot(tn - old_n, te - old_e) <= 4.0 for old_n, old_e in target_points):
                    continue
                target_points.append((tn, te))

        alt_t = data0["bounds_a"][1] if math.isfinite(data0["bounds_a"][1]) else 1.0
        for idx, (tn, te) in enumerate(target_points, start=1):
            self.ax.scatter([te], [tn], [0.0], c="#FF4500", s=300, marker="*",
                            edgecolors="white", linewidths=0.8, depthshade=False,
                            zorder=10, label="Target" if idx == 1 else None)
            self.ax.plot([te, te], [tn, tn], [0.0, alt_t],
                         color="#FF4500", alpha=0.35, linewidth=1.5, linestyle="--")
            self.ax.text(te + 0.4, tn + 0.4, 0.0 + 0.2, f"TARGET {idx}",
                         color="#FF4500", fontsize=8, fontweight="bold")

        bounds_sources = []
        if show_d0:
            bounds_sources.append(data0)
        if show_d1:
            bounds_sources.append(data1)
        if not bounds_sources:
            bounds_sources = [data0]

        e_min, e_max = bounds_sources[0]["bounds_e"]
        n_min, n_max = bounds_sources[0]["bounds_n"]
        a_min, a_max = bounds_sources[0]["bounds_a"]
        for b in bounds_sources[1:]:
            e_min = min(e_min, b["bounds_e"][0])
            e_max = max(e_max, b["bounds_e"][1])
            n_min = min(n_min, b["bounds_n"][0])
            n_max = max(n_max, b["bounds_n"][1])
            a_min = min(a_min, b["bounds_a"][0])
            a_max = max(a_max, b["bounds_a"][1])
        if len(vox) > 0:
            e_min = min(e_min, float(np.min(vox[:, 1])))
            e_max = max(e_max, float(np.max(vox[:, 1])))
            n_min = min(n_min, float(np.min(vox[:, 0])))
            n_max = max(n_max, float(np.max(vox[:, 0])))
            a_min = min(a_min, float(np.min(vox[:, 2])))
            a_max = max(a_max, float(np.max(vox[:, 2])))

        pad_xy = 2.0
        pad_a = 0.8
        if e_max - e_min < 1.0:
            e_min -= 0.5
            e_max += 0.5
        if n_max - n_min < 1.0:
            n_min -= 0.5
            n_max += 0.5
        if a_max - a_min < 1.0:
            a_min = min(a_min, 0.0)
            a_max += 0.5

        focus = self.map_focus_var.get() if self._dual else "All"
        focus_data = data0 if focus == "D0" else data1 if focus == "D1" else None
        if focus_data is not None:
            dn = focus_data.get("drone_n")
            de = focus_data.get("drone_e")
            da = focus_data.get("drone_agl")
            if dn is not None and de is not None:
                radius = 8.0
                e_min = de - radius
                e_max = de + radius
                n_min = dn - radius
                n_max = dn + radius
                if da is not None:
                    a_min = max(0.0, da - 3.0)
                    a_max = da + 4.0

        self.ax.set_xlim(e_min - pad_xy, e_max + pad_xy)
        self.ax.set_ylim(n_min - pad_xy, n_max + pad_xy)
        self.ax.set_zlim(max(0.0, a_min - pad_a), a_max + pad_a)
        if hasattr(self.ax, "set_box_aspect"):
            self.ax.set_box_aspect((
                max(1.0, e_max - e_min + 2.0 * pad_xy),
                max(1.0, n_max - n_min + 2.0 * pad_xy),
                max(1.0, a_max - a_min + 2.0 * pad_a),
            ))
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc="upper right", fontsize=7, facecolor="#0d0d1a", labelcolor="white")
        self.canvas.draw_idle()

    def _update_status(
        self,
        data,
        state_var,
        pos_var,
        alt_var,
        hdg_var,
        goal_var,
        ent_var,
        vox_var,
        link_var,
        res_var,
        vfh_var,
        cov_var,
        layer_cov_var,
        progress,
        shared_layers=None,
        team_global_cov_pct=None,
    ):
        dn = data["drone_n"]
        de = data["drone_e"]
        agl = data["drone_agl"]
        hdg = data["drone_hdg"]
        fg = data["frontier_goal"]
        cg = data.get("committed_goal")
        eg = data["entropy_goal"]

        state_var.set(f"Mode: {data['mission_state']}")
        if dn is not None and de is not None:
            pos_var.set(f"Position [{data['pose_source']}]: N={dn:+6.1f} m   E={de:+6.1f} m")
        else:
            pos_var.set(f"Position [{data['pose_source']}]: -")
        alt_var.set(f"Altitude: {agl:.2f} m AGL" if agl is not None else "Altitude: -")
        hdg_deg = math.degrees(hdg) % 360.0
        hdg_var.set(f"Heading: {hdg_deg:5.1f}° ({_compass(hdg_deg)})")

        if cg is not None and fg is not None:
            goal_var.set(
                f"Active ★: N={cg[0]:+.1f} E={cg[1]:+.1f} A={cg[2]:.1f}  |  "
                f"Endpoint X: N={fg[0]:+.1f} E={fg[1]:+.1f} A={fg[2]:.1f}"
            )
        elif cg is not None:
            goal_var.set(f"Active ★: N={cg[0]:+.1f}  E={cg[1]:+.1f}  A={cg[2]:.1f}")
        elif fg is not None:
            goal_var.set(f"Endpoint X: N={fg[0]:+.1f}  E={fg[1]:+.1f}  A={fg[2]:.1f}")
        elif eg is not None:
            goal_var.set(f"Goal [entropy]: N={eg[0]:+.1f}  E={eg[1]:+.1f}  A={eg[2]:.1f}")
        else:
            goal_var.set("Goal: -")

        ent_var.set(f"Entropy: {data['entropy_mean']:.4f}")
        vox_var.set(f"Voxels: {len(data['voxels']):,}")
        link_var.set(
            "Links: "
            f"pose {_freshness_text(data['age_pose_s'])} | "
            f"vox {_freshness_text(data['age_cloud_s'])} | "
            f"state {_freshness_text(data['age_state_s'])} | "
            f"path {data.get('waypoints_source', '-')}: {len(data.get('waypoints_ne', []))}"
        )
        res_var.set(f"Result: {data['mission_result'] or '-'}")

        raw_y = data.get("vfh_raw_yaw")
        cmd_y = data.get("vfh_cmd_yaw")
        vfh_age = data.get("age_vfh_s", math.inf)
        if raw_y is not None and cmd_y is not None and vfh_age < 2.0:
            raw_deg = math.degrees(raw_y) % 360.0
            cmd_deg = math.degrees(cmd_y) % 360.0
            err_deg = abs((math.degrees(raw_y - cmd_y) + 180) % 360 - 180)
            vfh_var.set(f"VFH: Raw={raw_deg:.0f}°  Cmd={cmd_deg:.0f}°  Err={err_deg:.0f}°")
        else:
            vfh_var.set("VFH: -")

        display_coverage_pct = data["coverage_pct"]
        if team_global_cov_pct is not None and math.isfinite(team_global_cov_pct):
            display_coverage_pct = float(team_global_cov_pct)
            cov_var.set(f"Coverage: team {display_coverage_pct:.1f} %")
        else:
            cov_var.set(f"Coverage: {display_coverage_pct:.1f} %")
        layer_alt = data.get("layer_altitude_m")
        team_cov = None
        if layer_alt is not None and shared_layers:
            rounded_alt = round(float(layer_alt), 2)
            team_cov = shared_layers.get(rounded_alt)
        if layer_alt is None:
            layer_cov_var.set(f"Layer coverage: {data['layer_coverage_pct']:.1f} %")
        elif team_cov is not None:
            layer_cov_var.set(
                f"Layer coverage: local {data['layer_coverage_pct']:.1f} % | "
                f"team {team_cov:.1f} % @ {layer_alt:.1f} m"
            )
        else:
            layer_cov_var.set(
                f"Layer coverage: {data['layer_coverage_pct']:.1f} % @ {layer_alt:.1f} m"
            )
        progress["value"] = display_coverage_pct

    def refresh(self):
        if self._closed:
            return
        data0 = self._globalize_snapshot(_state.snapshot(), 0.0, 0.0)
        data1 = (
            self._globalize_snapshot(_state1.snapshot(), self.node._d1_north, self.node._d1_east)
            if self._dual
            else None
        )
        with _shared_summary_lock:
            shared_layers = dict(_shared_layer_cov)
            shared_cov_pct = _shared_cov_pct
            shared_summary_age = (
                math.inf if _shared_summary_stamp_t <= 0.0
                else max(0.0, time.monotonic() - _shared_summary_stamp_t)
            )
        team_global_cov_pct = shared_cov_pct if self._dual else None

        self._update_status(
            data0,
            self.state_var,
            self.pos_var,
            self.alt_var,
            self.hdg_var,
            self.goal_var,
            self.ent_var,
            self.vox_var,
            self.link_var,
            self.res_var,
            self.vfh_var,
            self.cov_var,
            self.layer_cov_var,
            self.progress,
            shared_layers,
            team_global_cov_pct,
        )

        if self._dual and data1 is not None:
            self._update_status(
                data1,
                self.state1_var,
                self.pos1_var,
                self.alt1_var,
                self.hdg1_var,
                self.goal1_var,
                self.ent1_var,
                self.vox1_var,
                self.link1_var,
                self.res1_var,
                self.vfh1_var,
                self.cov1_var,
                self.layer_cov1_var,
                self.progress1,
                shared_layers,
                team_global_cov_pct,
            )
            with _merged_lock:
                n_merged = len(_merged_voxels)
                merged_age = math.inf if _merged_stamp_t <= 0.0 else max(0.0, time.monotonic() - _merged_stamp_t)
            self.merged_vox_var.set(f"Merged voxels: {n_merged:,}")
            self.shared_var.set(
                f"Shared map: {_freshness_text(merged_age)} | "
                f"team cov {shared_cov_pct:.1f}% | summary {_freshness_text(shared_summary_age)}"
            )

        all_lines = list(data0["log_lines"])
        if self._dual and data1 is not None:
            all_lines.extend(data1["log_lines"])
            all_lines = sorted(all_lines, reverse=True)[:500]
        self.log_text.delete("1.0", tk.END)
        for line in all_lines:
            lu = line.upper()
            if any(k in lu for k in ("TARGET", "FOUND", "DETECT", "ORBIT")):
                tag = "target"
            elif any(k in lu for k in ("ERROR", "EMERGENCY", "CRASH")):
                tag = "error"
            elif any(k in lu for k in ("WARN", "LOST", "STUCK", "DRIFT")):
                tag = "warning"
            elif "STATE ->" in line:
                tag = "state"
            else:
                tag = "info"
            self.log_text.insert(tk.END, line + "\n", tag)

        self._redraw_plot(data0, data1)
        self._schedule_refresh()


def main():
    rclpy.init()
    node = DashboardNode()

    warnings.filterwarnings("ignore", message="Unable to import Axes3D.*", category=UserWarning)

    def _spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    ros_thread = threading.Thread(target=_spin, daemon=True)
    ros_thread.start()

    root = tk.Tk()
    app = DashboardWindow(root, node)
    shutdown_requested = threading.Event()

    def _request_shutdown(_signum=None, _frame=None):
        shutdown_requested.set()

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    def _poll_shutdown():
        if shutdown_requested.is_set():
            app.close()
            return
        root.after(100, _poll_shutdown)

    root.after(100, _poll_shutdown)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        del app
        sys.exit(0)


if __name__ == "__main__":
    main()
