"""Stage D2 — dual-agent collaborative VFH indoor search (thesis §4.3).

Two drones explore the same room with shared visualization and complementary
scan layers. A map_merger node combines both clouds into a single global view
for the dashboard. Drone 0 and drone 1 spawn symmetrically around the original
takeoff point on the East axis.

Pipeline (per drone):
  dual_drone_sim   → Gazebo + 2× PX4 SITL (4022) + 2× sensor bridges
  voxel_mapper     → depth → 3-D voxel map → frontier_goal
  slam_toolbox     → 2-D scan matching for XY/yaw stability
  slam_pose_fusion → fuse SLAM XY/yaw with PX4 Z/velocity
  slam_to_px4      → feed SLAM pose back into PX4 EKF2
  search_mission_controller → VFH → PX4 velocity setpoints

Global nodes:
  map_merger       → /px4_0/voxel_map + /px4_1/voxel_map → /merged_voxel_map
  mission_dashboard → shows both drones and merged map

Usage:
  ros2 launch swarm_bringup d2_dual_agent_search.launch.py
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_px4_root() -> str:
    here = Path(__file__).resolve()
    for p in here.parents:
        candidate = p / "PX4-Autopilot"
        if candidate.exists():
            return str(candidate)
    return str(Path.cwd().resolve() / "PX4-Autopilot")


# ── World geometry ─────────────────────────────────────────────────────────────
# Gazebo spawns around the original single-drone start:
#   drone 0: East=-0.75, drone 1: East=+0.75.
# Each mapper/controller runs in its own local spawn frame, so room bounds are
# shifted by -spawn_offset for that drone.
_D0_SPAWN_NORTH = 0.0
_D0_SPAWN_EAST  = -0.75
_D1_SPAWN_NORTH = 0.0
_D1_SPAWN_EAST  = 0.75
_D1_NORTH_OFFSET = _D1_SPAWN_NORTH - _D0_SPAWN_NORTH
_D1_EAST_OFFSET  = _D1_SPAWN_EAST  - _D0_SPAWN_EAST
_D0_LOCAL_NORTH_SHIFT = 0.0
_D0_LOCAL_EAST_SHIFT  = 0.0
_D1_LOCAL_NORTH_SHIFT = 0.0
_D1_LOCAL_EAST_SHIFT  = 0.0
# PX4/Gazebo odometry is already expressed in the shared world frame. SLAM TF is
# local per drone, so slam_pose_fusion applies the spawn offset only when real
# SLAM is used, not while publishing PX4 fallback.
_D0_HOME_NORTH_LOCAL = -0.8
_D0_HOME_EAST_LOCAL = -1.0
_D1_HOME_NORTH_LOCAL = 0.8
_D1_HOME_EAST_LOCAL = 1.0
_D1_HOME_NORTH_SHARED = _D1_NORTH_OFFSET + _D1_HOME_NORTH_LOCAL
_D1_HOME_EAST_SHARED = _D1_EAST_OFFSET + _D1_HOME_EAST_LOCAL

# ── Scan box ───────────────────────────────────────────────────────────────────
_LAYER_HEIGHT = 1.00
_SCAN_START   = 1.0
_CEILING_AGL  = 10.0
_CEILING_CLEARANCE = 0.80
_BOX_HEIGHT   = _CEILING_AGL - _CEILING_CLEARANCE
_NUM_LAYERS   = int(math.floor((_BOX_HEIGHT - _SCAN_START) / _LAYER_HEIGHT)) + 1
_BOX_HEIGHT   = _SCAN_START + (_NUM_LAYERS - 1) * _LAYER_HEIGHT
_DRONE_RADIUS = 0.25
_SAFETY_MARGIN = 0.10
_SCAN_XY_CLEARANCE = 1.0
_SCAN_NORTH_MIN = -8.0 + _SCAN_XY_CLEARANCE
_SCAN_NORTH_MAX =  8.0 - _SCAN_XY_CLEARANCE
_SCAN_EAST_MIN  = -8.0 + _SCAN_XY_CLEARANCE
_SCAN_EAST_MAX  = 30.0 - _SCAN_XY_CLEARANCE

_CAMERA_OFFSET = [0.12, 0.03, 0.06]
_MAP_MIN = [-9.0, -9.0, -11.0]
_MAP_MAX = [ 9.0, 31.0,   0.5]


def _shift_rect(rect: list[float], dn: float, de: float) -> list[float]:
    return [rect[0] + dn, rect[1] + dn, rect[2] + de, rect[3] + de]


def _shift_bounds(bounds: list[float], dn: float, de: float) -> list[float]:
    return [bounds[0] + dn, bounds[1] + de, bounds[2]]


def _controller_params(extra: dict | None = None) -> dict:
    """Common search_mission_controller parameters — identical to single-agent."""
    p = {
        "takeoff_lookaround_enabled": True,
        "takeoff_yaw_rate_dps":  10.0,
        "takeoff_map_settle_s":   3.0,
        "layer_settle_s":         2.0,
        "target_altitude_m":     _BOX_HEIGHT,
        "climb_rate_mps":         0.40,
        "ceiling_clearance_m":    _CEILING_CLEARANCE,
        "ceiling_headroom_tolerance_m": 0.20,
        "floor_clearance_m":      0.45,
        "floor_recovery_climb_m": 0.40,
        "attraction_gain":        1.10,
        "repulsion_gain":         3.0,
        "drone_radius_m":        _DRONE_RADIUS,
        "safety_margin_m":       _SAFETY_MARGIN,
        "voxel_alt_band_m":       1.0,
        "max_accel_mps2":         1.0,
        "kd_vel_damp":            0.20,
        "lidar_repulsion_rho0_m": 2.0,
        "lidar_repulsion_eta":    2.0,
        "forward_stop_m":         0.70,
        "forward_resume_margin_m": 0.20,
        "emergency_stop_range_m": 0.42,
        "holonomic_vfh":          True,
        "align_threshold_rad":    1.0,
        "vfh_fov_half_deg":       0.0,
        "yaw_slew_alpha":         0.08,
        "yaw_rate_limit_dps":     30.0,
        "camera_yaw_track_goal":  True,
        "camera_yaw_slowdown_rad": 1.30,
        "camera_yaw_stop_rad":    2.20,
        "camera_yaw_min_speed_scale": 0.65,
        "narrow_passage_slowdown_enabled": True,
        "narrow_passage_clearance_m": 0.85,
        "narrow_passage_resume_clearance_m": 1.35,
        "narrow_passage_min_speed_scale": 0.45,
        # Speed cap disabled: it stalled the drone in passable gaps
        # (2026-06-30). Wall safety comes from the approach guard
        # below, which has its own flag and must stay enabled.
        "nearest_obstacle_slowdown_enabled": False,
        "nearest_obstacle_guard_enabled": True,
        "nearest_obstacle_slowdown_m": 1.05,
        "nearest_obstacle_resume_m": 1.65,
        "nearest_obstacle_min_speed_scale": 0.14,
        "nearest_obstacle_approach_guard_m": 1.10,
        "nearest_obstacle_centering_enabled": False,
        "nearest_obstacle_centering_m": 1.25,
        "nearest_obstacle_centering_max_mps": 0.15,
        "hold_scan_yaw_rate_dps": 10.0,
        "frontier_goal_yaw_capture_radius_m": 1.2,
        "frontier_arrival_look_enabled": True,
        "frontier_arrival_look_s": 6.0,
        "frontier_arrival_look_half_angle_deg": 90.0,
        "nav_pose_timeout_s":     2.0,
        "transit_timeout_s":      20.0,
        "north_min_m":            _SCAN_NORTH_MIN,
        "north_max_m":            _SCAN_NORTH_MAX,
        "east_min_m":             _SCAN_EAST_MIN,
        "east_max_m":             _SCAN_EAST_MAX,
        "scan_alt_start_m":       _SCAN_START,
        "scan_layer_step_m":      _LAYER_HEIGHT,
        "return_altitude_m":      _SCAN_START,
        "layer_complete_frac":    0.83,
        "layer_complete_stable_s": 8.0,
        "layer_complete_max_reachable_frontier_cells": 120,
        "layer_stagnation_min_frac": 0.85,
        "layer_timeout_min_frac":  0.85,
        "min_layer_dwell_s":      60.0,
        "no_frontier_layer_grace_s": 45.0,
        "layer_dwell_s":          300.0,
        "layer_stagnation_s":     120.0,
        "goal_progress_timeout_s": 10.0,
        "active_goal_blocked_range_m": 0.55,
        "route_blocked_range_m": 0.90,
        "route_blocked_half_angle_deg": 18.0,
        "route_conflict_replan_enabled": True,
        "route_conflict_range_m": 1.25,
        "route_conflict_angle_deg": 55.0,
        "route_conflict_replan_s": 0.8,
        "blocked_waypoint_reject_radius_m": 0.95,
        "blocked_waypoint_reject_s": 10.0,
        "local_apf_repulsion_enabled": True,
        "local_apf_repulsion_range_m": 1.35,
        "local_apf_repulsion_gain": 0.08,
        "local_apf_repulsion_max_mps": 0.20,
        "waypoint_lookahead_radius_m": 1.80,
        "waypoint_lookahead_cos": 0.20,
        "scan_enabled":           True,
        "scan_complete_frac":     0.75,
        "search_timeout_s":      2700.0,
        "goal_reach_radius_m":    0.5,
        "arm_timeout_s":         180.0,
        "target_hold_s":          10.0,
        "target_revisit_radius_m": 6.00,
        "target_confirm_radius_m": 2.50,
    }
    if extra:
        p.update(extra)
    return p


def _voxel_params(local_north_shift: float = 0.0,
                  local_east_shift: float = 0.0) -> dict:
    """Voxel mapper parameters — identical to single-agent."""
    return {
        "voxel_size_m":        0.20,
        "camera_offset_frd_m": _CAMERA_OFFSET,
        "camera_pitch_rad":    math.pi / 12.0,
        "map_bounds_min_m":    _shift_bounds(_MAP_MIN, local_north_shift, local_east_shift),
        "map_bounds_max_m":    _shift_bounds(_MAP_MAX, local_north_shift, local_east_shift),
        "pixel_stride":        8,
        "max_range_m":         8.0,
        "publish_rate_hz":     2.0,
        "use_slam_pose":       True,
        # In dual-drone runs the mapper must wait for slam_pose_fusion so every
        # pose is normalized to the shared Gazebo/world frame before mapping.
        "require_slam_pose":   True,
        "integrate_lidar_into_map": False,
        # Collaborative exploration: each drone keeps its local log-odds map for
        # sensor updates, but plans frontiers over the merged team map overlay.
        # Both maps are already in the shared world frame; do not apply spawn
        # offsets again in map_merger.
        "shared_map_enabled":   True,
        "shared_occupied_topic": "/merged_voxel_map",
        "shared_free_topic":    "/merged_free_voxel_map",
        "slam_pose_timeout_s": 1.0,
        "home_north_m":        0.0,
        "home_east_m":         0.0,
        "log_odds_hit":        0.60,
        "log_odds_miss":      -0.45,
        "occupied_probability": 0.65,
        "slice_half_thickness_m": 0.60,
        "frontier_cluster_radius_m": 1.20,
        "frontier_min_goal_distance_m": 0.60,
        "frontier_clearance_cells": 2,
        "path_clearance_cells":     0,
        "obstacle_inflate_cells":   3,
        "astar_clearance_penalty_weight": 42.0,
        "path_shortcut_max_segment_m": 2.50,
        "path_shortcut_near_clearance_m": 1.20,
        "path_shortcut_near_segment_m": 0.60,
        "path_shortcut_mid_clearance_m": 1.80,
        "path_shortcut_mid_segment_m": 1.20,
        "path_shortcut_line_clearance_m": 0.85,
        "path_smoothing_enabled": True,
        "path_smoothing_iterations": 2,
        "path_smoothing_spacing_m": 0.35,
        "path_smoothing_clearance_m": 0.85,
        "path_smoothing_max_points": 120,
        "local_start_clearance_cells": 5,
        # Keep the 2D route mask near the flight body. From 4 m and up,
        # mezzanine/ceiling voxels can otherwise disconnect the Room A/B
        # corridor in the planner even when the physical passage is open.
        "obstacle_band_half_m":     0.70,
        "obstacle_band_high_alt_threshold_m": 3.5,
        "obstacle_band_high_half_m": 0.45,
        "occupied_retain_probability": 0.75,
        "occupied_miss_scale":      0.07,
        "frontier_view_standoff_min_m": 1.00,
        "frontier_view_standoff_max_m": 2.20,
        "frontier_view_standoff_samples": 3,
        "frontier_view_angle_samples": 8,
        "frontier_max_clusters_scored": 24,
        "frontier_candidate_clearance_m": 0.60,
        "frontier_min_obstacle_clearance_m": 0.65,
        "frontier_world_boundary_clearance_m": 1.00,
        "frontier_candidate_vertical_half_m": 0.45,
        "frontier_gain_u_samples": 7,
        "frontier_gain_v_samples": 4,
        "frontier_cluster_weight": 2.0,
        "frontier_info_gain_weight": 5.2,
        "frontier_distance_weight": 0.28,
        "frontier_progress_weight": 2.0,
        "frontier_yaw_weight": 0.4,
        # 10.0: tested 3.5 to let info_gain lead — worse in practice (21 emergency
        # hovers vs 0, lower coverage; wall-adjacent viewpoints reveal less because
        # the wall occludes the frustum). High clearance keeps open, high-exposure
        # viewpoints. Reverted to 10.0.
        "frontier_clearance_weight": 10.0,
        "frontier_awareness_weight": 0.3,
        "world_area_mask_enabled": True,
        "world_zone_a":        _shift_rect([-8.5,  8.5, -8.5,  8.5], local_north_shift, local_east_shift),
        "world_zone_corridor": _shift_rect([-2.5,  2.5,  7.5, 14.5], local_north_shift, local_east_shift),
        "world_zone_b":        _shift_rect([-8.5,  8.5, 13.5, 30.5], local_north_shift, local_east_shift),
        "coverage_use_reachable_component": False,
        "coverage_reachability_guard_enabled": True,
        "coverage_denominator_drop_ratio": 0.45,
        "coverage_min_guard_total_cells": 500,
    }


def generate_launch_description():
    sim_launch = (
        Path(get_package_share_directory("swarm_sim_bringup")) /
        "launch" / "dual_drone_sim.launch.py"
    )
    bringup_share = get_package_share_directory("swarm_bringup")
    slam_params_d0 = str(Path(bringup_share) / "config" / "slam_toolbox_params.yaml")
    slam_params_d1 = str(Path(bringup_share) / "config" / "slam_toolbox_params_d1.yaml")

    px4_root       = LaunchConfiguration("px4_root")
    world          = LaunchConfiguration("world")
    show_dashboard = LaunchConfiguration("show_dashboard")
    use_slam       = LaunchConfiguration("use_slam")
    v_max_mps = LaunchConfiguration("v_max_mps")
    v_max_target_mps = LaunchConfiguration("v_max_target_mps")
    v_max_ramp_step_mps = LaunchConfiguration("v_max_ramp_step_mps")
    disable_vfh = LaunchConfiguration("disable_vfh")

    # ── 1. Simulator (two drones) ─────────────────────────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={
            "world": world,
            "px4_root": px4_root,
        }.items(),
    )

    # ── 2a. Drone-0 SLAM → PX4 EV bridge ──────────────────────────────────────
    slam_to_px4_d0 = TimerAction(period=10.0, actions=[Node(
        package="swarm_bringup",
        executable="slam_to_px4.py",
        name="slam_to_px4",
        namespace="px4_0",
        output="screen",
        parameters=[{
            "map_frame": "map",
            "base_frame": "lidar_2d_link",
            "publish_rate_hz": 20.0,
            "px4_odom_topic": "/fmu/out/vehicle_odometry",
            "output_topic": "/fmu/in/vehicle_visual_odometry",
        }],
        condition=IfCondition(use_slam),
    )])

    # ── 2b. Drone-1 SLAM → PX4 EV bridge ──────────────────────────────────────
    slam_to_px4_d1 = TimerAction(period=15.0, actions=[Node(
        package="swarm_bringup",
        executable="slam_to_px4.py",
        name="slam_to_px4",
        namespace="px4_1",
        output="screen",
        parameters=[{
            "map_frame": "d1_map",
            "base_frame": "d1_lidar_2d_link",
            "publish_rate_hz": 20.0,
            "px4_odom_topic": "/px4_1/fmu/out/vehicle_odometry",
            "output_topic": "/px4_1/fmu/in/vehicle_visual_odometry",
        }],
        condition=IfCondition(use_slam),
    )])

    # ── 3a. Drone-0 voxel_mapper ──────────────────────────────────────────────
    voxel_d0 = TimerAction(period=10.2, actions=[Node(
        package="swarm_perception",
        executable="voxel_mapper",
        name="voxel_mapper",
        namespace="px4_0",
        output="screen",
        parameters=[{
            "drone_id": 0,
            **_voxel_params(_D0_LOCAL_NORTH_SHIFT, _D0_LOCAL_EAST_SHIFT),
            "home_north_m": _D0_HOME_NORTH_LOCAL,
            "home_east_m": _D0_HOME_EAST_LOCAL,
            "peer_frontier_goal_topic": "/px4_1/frontier_goal",
            "peer_frontier_exclusion_radius_m": 4.0,
            "peer_frontier_penalty": 3.0,
        }],
        remappings=[("fmu/out/vehicle_odometry", "/fmu/out/vehicle_odometry")],
    )])

    # ── 3b. Drone-1 voxel_mapper ──────────────────────────────────────────────
    voxel_d1 = TimerAction(period=15.2, actions=[Node(
        package="swarm_perception",
        executable="voxel_mapper",
        name="voxel_mapper",
        namespace="px4_1",
        output="screen",
        parameters=[{
            "drone_id": 1,
            **_voxel_params(_D1_LOCAL_NORTH_SHIFT, _D1_LOCAL_EAST_SHIFT),
            "home_north_m": _D1_HOME_NORTH_LOCAL,
            "home_east_m": _D1_HOME_EAST_LOCAL,
            "peer_frontier_goal_topic": "/px4_0/frontier_goal",
            "peer_frontier_exclusion_radius_m": 4.0,
            "peer_frontier_penalty": 3.0,
        }],
        # Drone 1's PX4 instance publishes under /px4_1/fmu/... and the node is
        # already in px4_1 namespace, so relative topic resolution works correctly —
        # no explicit remapping needed here.
    )])

    # ── 3c/3d. Per-drone HSV red-can target detectors ─────────────────────────
    # One detector per drone, each in its own namespace. Each detector localizes
    # the can in that drone's local NED frame (odom.position origin = its spawn)
    # and publishes TargetFound on /px4_<i>/target_found — consumed by the same
    # drone's controller, which works in the identical local frame, so no spawn
    # offset is needed. Without these nodes the dual-drone runs have no detector
    # at all, which is why a drone could fly over the can and never mark it.
    _detector_params = {
        "camera_pitch_rad":   math.pi / 12.0,   # 15° downward
        "camera_offset_frd_m": _CAMERA_OFFSET,
        "red_pixel_fraction_min": 0.30,
        "min_blob_area_px": 60,
        "target_depth_min_m": 0.20,
        "target_depth_max_m": 7.0,
        "target_max_depth_spread_m": 0.45,
        "target_cluster_radius_m": 6.0,
        "target_cluster_alt_radius_m": 3.0,
        "target_min_cluster_confirmations": 3,
        "target_cluster_weight_limit": 12.0,
        "target_altitude_min_m": 1.0,
        "target_altitude_max_m": 6.8,
    }

    target_detector_d0 = TimerAction(period=10.5, actions=[Node(
        package="swarm_target_detection",
        executable="target_detector",
        name="target_detector",
        namespace="px4_0",
        output="screen",
        parameters=[{"drone_id": 0, **_detector_params}],
        # Drone 0's PX4 instance is un-namespaced at /fmu/...
        remappings=[("fmu/out/vehicle_odometry", "/fmu/out/vehicle_odometry")],
    )])

    target_detector_d1 = TimerAction(period=15.5, actions=[Node(
        package="swarm_target_detection",
        executable="target_detector",
        name="target_detector",
        namespace="px4_1",
        output="screen",
        parameters=[{"drone_id": 1, **_detector_params}],
        # In namespace px4_1 the relative fmu/out/vehicle_odometry resolves to
        # /px4_1/fmu/out/vehicle_odometry — no explicit remapping needed.
    )])

    # ── 4a. Drone-0 LiDAR tilt filter ────────────────────────────────────────
    lidar_tilt_filter_d0 = TimerAction(period=7.2, actions=[Node(
        package="swarm_bringup",
        executable="lidar_tilt_filter.py",
        name="lidar_tilt_filter",
        namespace="px4_0",
        output="screen",
        parameters=[{
            "match_tol_min":      0.12,
            "match_tol_factor":   0.08,
            "ceiling_m_fallback": _CEILING_AGL,
            "px4_odom_topic":     "/fmu/out/vehicle_odometry",
            "output_frame_id":     "lidar_2d_link",
        }],
        condition=IfCondition(use_slam),
    )])

    # ── 4b. Drone-1 LiDAR tilt filter ────────────────────────────────────────
    lidar_tilt_filter_d1 = TimerAction(period=12.2, actions=[Node(
        package="swarm_bringup",
        executable="lidar_tilt_filter.py",
        name="lidar_tilt_filter",
        namespace="px4_1",
        output="screen",
        parameters=[{
            "match_tol_min":      0.12,
            "match_tol_factor":   0.08,
            "ceiling_m_fallback": _CEILING_AGL,
            "px4_odom_topic":     "/px4_1/fmu/out/vehicle_odometry",
            "output_frame_id":     "d1_lidar_2d_link",
        }],
        condition=IfCondition(use_slam),
    )])

    # ── 4c. Drone-0 SLAM ─────────────────────────────────────────────────────
    odom_d0 = TimerAction(period=7.5, actions=[Node(
        package="swarm_bringup",
        executable="odom_publisher.py",
        name="odom_publisher",
        namespace="px4_0",
        output="screen",
        parameters=[{
            "child_frame_id": "lidar_2d_link",
            "frame_prefix":   "",
        }],
        remappings=[
            ("fmu/out/vehicle_local_position_v1", "/fmu/out/vehicle_local_position_v1"),
        ],
        condition=IfCondition(use_slam),
    )])

    slam_d0 = TimerAction(period=8.5, actions=[Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="px4_0",
        output="screen",
        parameters=[slam_params_d0],
        condition=IfCondition(use_slam),
    )])

    slam_d0_autostart = TimerAction(period=10.0, actions=[Node(
        package="swarm_bringup",
        executable="lifecycle_autostart.py",
        name="slam_toolbox_autostart",
        output="screen",
        parameters=[{
            "target_node": "/px4_0/slam_toolbox",
            "timeout_s": 60.0,
            "retry_period_s": 0.5,
        }],
        condition=IfCondition(use_slam),
    )])

    slam_fuse_d0 = TimerAction(period=9.5, actions=[Node(
        package="swarm_bringup",
        executable="slam_pose_fusion.py",
        name="slam_pose_fusion",
        namespace="px4_0",
        output="screen",
        parameters=[{
            "map_frame":      "map",
            "base_frame":     "lidar_2d_link",
            "px4_odom_topic": "/fmu/out/vehicle_odometry",
            "publish_rate_hz": 15.0,
            "yaw_residual_gate_rad": 0.55,
            "yaw_residual_jump_gate_rad": 0.25,
            "max_slam_yaw_rate_dps": 90.0,
            "yaw_drift_on_frames": 2,
            "yaw_drift_off_frames": 15,
            "spawn_north_m": _D0_SPAWN_NORTH,
            "spawn_east_m": _D0_SPAWN_EAST,
        }],
        condition=IfCondition(use_slam),
    )])

    # ── 4d. Drone-1 SLAM ─────────────────────────────────────────────────────
    odom_d1 = TimerAction(period=12.5, actions=[Node(
        package="swarm_bringup",
        executable="odom_publisher.py",
        name="odom_publisher",
        namespace="px4_1",
        output="screen",
        parameters=[{
            "child_frame_id": "lidar_2d_link",
            "frame_prefix":   "d1_",
        }],
        remappings=[
            ("fmu/out/vehicle_local_position_v1", "/px4_1/fmu/out/vehicle_local_position_v1"),
        ],
        condition=IfCondition(use_slam),
    )])

    slam_d1 = TimerAction(period=13.5, actions=[Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="px4_1",
        output="screen",
        parameters=[slam_params_d1],
        condition=IfCondition(use_slam),
    )])

    slam_d1_autostart = TimerAction(period=15.0, actions=[Node(
        package="swarm_bringup",
        executable="lifecycle_autostart.py",
        name="slam_toolbox_autostart",
        namespace="px4_1",
        output="screen",
        parameters=[{
            "target_node": "/px4_1/slam_toolbox",
            "timeout_s": 60.0,
            "retry_period_s": 0.5,
        }],
        condition=IfCondition(use_slam),
    )])

    slam_fuse_d1 = TimerAction(period=14.5, actions=[Node(
        package="swarm_bringup",
        executable="slam_pose_fusion.py",
        name="slam_pose_fusion",
        namespace="px4_1",
        output="screen",
        parameters=[{
            "map_frame":      "d1_map",
            "base_frame":     "d1_lidar_2d_link",
            "px4_odom_topic": "/px4_1/fmu/out/vehicle_odometry",
            "publish_rate_hz": 15.0,
            "yaw_residual_gate_rad": 0.55,
            "yaw_residual_jump_gate_rad": 0.25,
            "max_slam_yaw_rate_dps": 90.0,
            "yaw_drift_on_frames": 2,
            "yaw_drift_off_frames": 15,
            "spawn_north_m":  _D1_SPAWN_NORTH,
            "spawn_east_m":   _D1_SPAWN_EAST,
        }],
        condition=IfCondition(use_slam),
    )])

    # ── 5a. Drone-0 controller ────────────────────────────────────────────────
    controller_speed_params = {
        "v_max_mps": ParameterValue(v_max_mps, value_type=float),
        "v_max_target_mps": ParameterValue(v_max_target_mps, value_type=float),
        "v_max_ramp_step_mps": ParameterValue(v_max_ramp_step_mps, value_type=float),
        "disable_vfh": ParameterValue(disable_vfh, value_type=bool),
    }

    ctrl_d0_params = {
        **controller_speed_params,
        "north_min_m": _SCAN_NORTH_MIN + _D0_LOCAL_NORTH_SHIFT,
        "north_max_m": _SCAN_NORTH_MAX + _D0_LOCAL_NORTH_SHIFT,
        "east_min_m": _SCAN_EAST_MIN + _D0_LOCAL_EAST_SHIFT,
        "east_max_m": _SCAN_EAST_MAX + _D0_LOCAL_EAST_SHIFT,
        "layer_index_start": 0,
        "layer_index_stride": 2,
        "home_north_m": _D0_HOME_NORTH_LOCAL,
        "home_east_m": _D0_HOME_EAST_LOCAL,
        "shared_summary_topic": "/shared_map_update_summary",
        "peer_avoidance_enabled": True,
        "peer_local_position_topic": "/px4_1/fmu/out/vehicle_local_position_v1",
        "own_spawn_north_m": 0.0,
        "own_spawn_east_m": 0.0,
        "peer_spawn_north_m": 0.0,
        "peer_spawn_east_m": 0.0,
        "peer_avoidance_z_tolerance_m": 1.5,
    }
    ctrl_d1_params = {
        **controller_speed_params,
        "north_min_m": _SCAN_NORTH_MIN + _D1_LOCAL_NORTH_SHIFT,
        "north_max_m": _SCAN_NORTH_MAX + _D1_LOCAL_NORTH_SHIFT,
        "east_min_m":  _SCAN_EAST_MIN + _D1_LOCAL_EAST_SHIFT,
        "east_max_m":  _SCAN_EAST_MAX + _D1_LOCAL_EAST_SHIFT,
        "layer_index_start": 1,
        "layer_index_stride": 2,
        "home_north_m": _D1_HOME_NORTH_LOCAL,
        "home_east_m": _D1_HOME_EAST_LOCAL,
        "shared_summary_topic": "/shared_map_update_summary",
        "peer_avoidance_enabled": True,
        "peer_local_position_topic": "/fmu/out/vehicle_local_position_v1",
        "own_spawn_north_m": 0.0,
        "own_spawn_east_m": 0.0,
        "peer_spawn_north_m": 0.0,
        "peer_spawn_east_m": 0.0,
        "peer_avoidance_z_tolerance_m": 1.5,
    }

    ctrl_d0 = TimerAction(period=11.0, actions=[Node(
        package="swarm_control",
        executable="search_mission_controller",
        name="search_mission_controller",
        namespace="px4_0",
        # When this node exits, shut down the entire launch process.
        # This is key for the automated experiment script.
        on_exit=[Shutdown(reason="px4_0 search_mission_controller exited")],
        output="screen",
        parameters=[_controller_params(ctrl_d0_params)],
        remappings=[
            ("fmu/in/offboard_control_mode",      "/fmu/in/offboard_control_mode"),
            ("fmu/in/trajectory_setpoint",        "/fmu/in/trajectory_setpoint"),
            ("fmu/in/vehicle_command",            "/fmu/in/vehicle_command"),
            ("fmu/out/vehicle_status_v4",         "/fmu/out/vehicle_status_v4"),
            ("fmu/out/vehicle_local_position_v1", "/fmu/out/vehicle_local_position_v1"),
            ("fmu/out/vehicle_attitude",          "/fmu/out/vehicle_attitude"),
        ],
    )])

    # ── 5b. Drone-1 controller ────────────────────────────────────────────────
    ctrl_d1 = TimerAction(period=16.0, actions=[Node(
        package="swarm_control",
        executable="search_mission_controller",
        name="search_mission_controller",
        namespace="px4_1",
        # When this node exits, shut down the entire launch process.
        # This is key for the automated experiment script.
        on_exit=[Shutdown(reason="px4_1 search_mission_controller exited")],
        output="screen",
        parameters=[_controller_params(ctrl_d1_params)],
        remappings=[
            ("fmu/in/offboard_control_mode",      "/px4_1/fmu/in/offboard_control_mode"),
            ("fmu/in/trajectory_setpoint",        "/px4_1/fmu/in/trajectory_setpoint"),
            ("fmu/in/vehicle_command",            "/px4_1/fmu/in/vehicle_command"),
            ("fmu/out/vehicle_status_v4",         "/px4_1/fmu/out/vehicle_status_v4"),
            ("fmu/out/vehicle_local_position_v1", "/px4_1/fmu/out/vehicle_local_position_v1"),
            ("fmu/out/vehicle_attitude",          "/px4_1/fmu/out/vehicle_attitude"),
        ],
    )])

    # ── 6. Map merger ─────────────────────────────────────────────────────────
    map_merger = TimerAction(period=13.0, actions=[Node(
        package="swarm_bringup",
        executable="map_merger.py",
        name="map_merger",
        output="screen",
        parameters=[{
            "drone0_ns":       "px4_0",
            "drone1_ns":       "px4_1",
            # Both voxel maps are already in the shared Gazebo/world frame.
            "drone1_north_m":  0.0,
            "drone1_east_m":   0.0,
            "publish_rate_hz": 2.0,
        }],
    )])

    # ── 7. Dashboard (dual-drone) ─────────────────────────────────────────────
    dashboard = TimerAction(period=15.0, actions=[Node(
        package="swarm_gcs",
        executable="mission_dashboard",
        name="mission_dashboard",
        output="screen",
        parameters=[{
            "drone_ns":   "px4_0",
            "drone1_ns":  "px4_1",
            "num_drones": 2,
            "drone1_north_m": 0.0,
            "drone1_east_m": 0.0,
            "gazebo_model0": "x500_swarm_0",
            "gazebo_model1": "x500_swarm_1",
        }],
        condition=IfCondition(show_dashboard),
    )])

    return LaunchDescription([
        DeclareLaunchArgument("px4_root", default_value=_find_px4_root()),
        DeclareLaunchArgument(
            "world",
            default_value="single_agent_search_room_easy",
            description="Gazebo world SDF name (without .sdf)",
        ),
        DeclareLaunchArgument("show_dashboard", default_value="true"),
        DeclareLaunchArgument("use_slam",       default_value="true"),
        DeclareLaunchArgument(
            "v_max_mps",
            default_value="0.5",
            description="Fixed cruise speed cap for both drones",
        ),
        DeclareLaunchArgument(
            "v_max_target_mps",
            default_value="0.5",
            description="Speed ramp target; keep equal to v_max_mps for fixed-speed sweeps",
        ),
        DeclareLaunchArgument(
            "v_max_ramp_step_mps",
            default_value="0.0",
            description="Speed increase after completed layers; keep 0.0 for fixed-speed sweeps",
        ),
        DeclareLaunchArgument(
            "disable_vfh",
            default_value="false",
            description="Bypass VFH and fly directly toward active waypoints; emergency hover remains enabled",
        ),
        sim,
        slam_to_px4_d0, slam_to_px4_d1,
        voxel_d0, voxel_d1,
        target_detector_d0, target_detector_d1,
        lidar_tilt_filter_d0, lidar_tilt_filter_d1,
        odom_d0, slam_d0, slam_d0_autostart, slam_fuse_d0,
        odom_d1, slam_d1, slam_d1_autostart, slam_fuse_d1,
        ctrl_d0, ctrl_d1,
        map_merger,
        dashboard,
    ])
