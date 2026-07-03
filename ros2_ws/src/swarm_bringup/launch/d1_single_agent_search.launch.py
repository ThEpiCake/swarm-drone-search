"""Stage D — single-agent VFH indoor search (thesis §4.2).

Pipeline:
  1. single_drone_sim.launch.py — Gazebo + PX4 SITL (airframe 4022, GNSS-denied)
                                   + sensor bridges (RGB-D camera, 2-D LiDAR,
                                     range_up, range_down)
  2. voxel_mapper               — depth image → 3D log-odds voxel map
                                   publishes: frontier_goal, entropy_centroid,
                                              map_update_summary, voxel_slice_map
  3. target_detector            — RGB image → HSV red-blob detection → TargetFound
  4. search_mission_controller  — VFH: LiDAR 360° → 72 blocked sectors → command
                                   holonomic XY velocity through nearest open corridor
                                   → PX4 velocity setpoints

VFH law:
  sect_min[s] = min LiDAR range in sector s (5° sectors, 72 total)
  blocked[s]  = sect_min[s] < rho0  (dilated ±3 sectors for drone footprint)
  steer       = center of open valley closest to goal bearing
  speed       = v_max × clamp((min_all − rho0) / (0.5×lidar_rho0 − rho0))
  yaw         = held stable/rate-limited; it is not forced to chase every VFH steer angle

Termination: each scan layer reaches configured layer coverage, or has no
reachable frontier after the dwell guard; timeout is a safety fuse only.
TargetFound triggers a short camera inspection hold, then SCAN resumes.
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
from ament_index_python.packages import get_package_share_directory


def _find_px4_root() -> str:
    here = Path(__file__).resolve()
    for p in here.parents:
        candidate = p / "PX4-Autopilot"
        if candidate.exists():
            return str(candidate)
    return str(Path.cwd().resolve() / "PX4-Autopilot")


def generate_launch_description():
    sim_launch = (Path(get_package_share_directory("swarm_sim_bringup")) /
                  "launch" / "single_drone_sim.launch.py")

    px4_root         = LaunchConfiguration("px4_root")
    world            = LaunchConfiguration("world")
    show_camera_view = LaunchConfiguration("show_camera_view")
    show_dashboard = LaunchConfiguration("show_dashboard")
    show_rviz = LaunchConfiguration("show_rviz")
    use_slam = LaunchConfiguration("use_slam")
    v_max_mps = LaunchConfiguration("v_max_mps")
    v_max_target_mps = LaunchConfiguration("v_max_target_mps")
    v_max_ramp_step_mps = LaunchConfiguration("v_max_ramp_step_mps")
    disable_vfh = LaunchConfiguration("disable_vfh")

    # ── Scan box geometry — single source of truth ────────────────────────────
    # World ceiling bottom is 10.0 m AGL. Keep the top commanded scan layer at
    # 9.0 m AGL, with a slightly looser ceiling guard to avoid false guard loops.
    # 1.0 m layers: LiDAR intersects obstacle sides including short crates (1.4 m),
    # so the voxel map sees them → A* routes around → no leg collision.
    _LAYER_HEIGHT  = 1.00   # m per layer
    _SCAN_START    = 1.0    # m AGL (first layer)
    _CEILING_AGL   = 10.0   # m AGL, bottom face of the Gazebo ceiling slab
    _CEILING_CLEARANCE = 0.80 # m, keeps the 9 m layer but avoids false ceiling guard loops
    _BOX_HEIGHT    = _CEILING_AGL - _CEILING_CLEARANCE
    _NUM_LAYERS    = int(math.floor((_BOX_HEIGHT - _SCAN_START) / _LAYER_HEIGHT)) + 1
    _BOX_HEIGHT    = _SCAN_START + (_NUM_LAYERS - 1) * _LAYER_HEIGHT
    _DRONE_RADIUS  = 0.25   # m (500 mm drone half-span)
    _SAFETY_MARGIN = 0.10   # m -> rho0 = drone_radius(0.25) + 0.10 = 0.35 m VFH block range
    _CAMERA_OFFSET = [0.12, 0.03, 0.06]  # Matches x500_swarm/model.sdf rgbd_link pose.

    # Controller scan bounds stay 1.0 m inside the physical room envelope.
    _SCAN_XY_CLEARANCE = 1.0
    _SCAN_NORTH_MIN = -8.0 + _SCAN_XY_CLEARANCE
    _SCAN_NORTH_MAX =  8.0 - _SCAN_XY_CLEARANCE
    _SCAN_EAST_MIN  = -8.0 + _SCAN_XY_CLEARANCE
    _SCAN_EAST_MAX  = 30.0 - _SCAN_XY_CLEARANCE

    # Voxel map NED bounds — cover both rooms + corridor + 0.5 m margin.
    # NED: x=North (-8→+8 m), y=East (-8→+30 m), z=Down (0→-10.5 m)
    _MAP_MIN = [-9.0, -9.0, -11.0]
    _MAP_MAX = [ 9.0, 31.0,   0.5]

    # ── 1. Simulator ──────────────────────────────────────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={
            "world":             world,
            "drone_id":          "0",
            "drone_model":       "x500_swarm",
            # 4022: optical flow + rangefinder + external vision (slam_to_px4.py), no GPS.
            "px4_sys_autostart": "4022",
            "px4_root":          px4_root,
        }.items(),
    )

    # ── 1b. SLAM TF → PX4 VehicleVisualOdometry (+9 s) ───────────────────────
    # Feeds the SLAM pose back into EKF2 so flight control and mapping share the
    # same XY/yaw reference in GNSS-denied operation.
    slam_to_px4 = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="swarm_bringup",
                executable="slam_to_px4.py",
                name="slam_to_px4",
                output="screen",
                parameters=[{
                    "map_frame": "map",
                    "base_frame": "lidar_2d_link",
                    "publish_rate_hz": 20.0,
                    "jump_gate_m": 0.75,
                    "yaw_residual_gate_rad": 0.55,
                    "yaw_residual_jump_gate_rad": 0.25,
                    "max_slam_yaw_rate_dps": 90.0,
                    "yaw_drift_on_frames": 2,
                    "yaw_drift_off_frames": 15,
                    "output_topic": "/fmu/in/vehicle_visual_odometry",
                }],
                condition=IfCondition(use_slam),
            ),
        ],
    )

    # ── 2. 3-D voxel mapper (+6 s — let sensor bridges settle) ───────────────
    voxel_mapper = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="swarm_perception",
                executable="voxel_mapper",
                name="voxel_mapper",
                namespace="px4_0",
                output="screen",
                parameters=[{
                    "drone_id":            0,
                    "voxel_size_m":        0.20,
                    "camera_offset_frd_m": _CAMERA_OFFSET,
                    "camera_pitch_rad":    math.pi / 12.0,   # 15° downward
                    "map_bounds_min_m":    _MAP_MIN,
                    "map_bounds_max_m":    _MAP_MAX,
                    "pixel_stride":        8,
                    "max_range_m":         8.0,   # was 14.0 — reject far-field LiDAR noise
                    "publish_rate_hz":     2.0,
                    "use_slam_pose":       True,
                    "require_slam_pose":   False,
                    # Avoid double-integrating geometry into the exploration map:
                    # depth camera builds voxel occupancy; LiDAR stays for VFH
                    # safety and the temporary 2-D SLAM pose source.
                    "integrate_lidar_into_map": False,
                    "slam_pose_timeout_s": 1.0,
                    "home_north_m":        0.0,
                    "home_east_m":         0.0,
                    "log_odds_hit":        0.60,  # was 0.85 — requires 2 hits before occupied
                    "log_odds_miss":      -0.45,  # was -0.40 — slightly faster free-space clearing
                    "occupied_probability": 0.65,
                    "slice_half_thickness_m": 0.60,
                    "frontier_cluster_radius_m": 1.20,
                    "frontier_min_goal_distance_m": 0.60,
                    "frontier_clearance_cells": 2,
                    "path_clearance_cells":     0,   # legacy; obstacle_inflate_cells owns route clearance
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
                    "local_start_clearance_cells": 5, # clears a 1.0 m bubble around active pose + Bresenham corridor to home
                    # Keep the 2D route mask near the flight body. From 4 m and up,
                    # mezzanine/ceiling voxels can otherwise disconnect the Room A/B
                    # corridor in the planner even when the physical passage is open.
                    "obstacle_band_half_m":     0.70,
                    "obstacle_band_high_alt_threshold_m": 3.5,
                    "obstacle_band_high_half_m": 0.45,
                    "occupied_retain_probability": 0.75,
                    "occupied_miss_scale":      0.07,     # was 0.10 — occupied cells decay a bit faster
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
                    "frontier_distance_weight": 0.28,  # 0.28: reduced from 0.40 — Room B at 22m path costs 6.2 pts (was 8.8); with zone_bonus=9 net=-2.8 vs Room A -1.4, competitive
                    "frontier_progress_weight": 2.0,   # bounded 0→5 m bonus so SCAN prefers useful outward progress
                    "frontier_yaw_weight": 0.4,
                    # Voronoi clearance bonus: rewards goals at corridor centres (max distance
                    # from all obstacles), penalises goals near pillars/walls.
                    # Weight × clearance_m (capped at 2.0 m) added to frontier score.
                    # 10.0: tested lowering to 3.5 to let info_gain lead, but in
                    # practice it was worse — wall-adjacent viewpoints reveal LESS
                    # (the wall occludes the frustum) and caused 21 emergency hovers
                    # vs 0 at 10.0, with lower coverage. High clearance keeps the drone
                    # at open viewpoints that actually see more. Reverted to 10.0.
                    "frontier_clearance_weight": 10.0,
                    "frontier_awareness_weight": 0.3,  # reduced: high awareness_weight penalises Room B (unknown area) and keeps drone in Room A
                    # World area mask: rejects ray-cast free-space and frontier goals outside
                    # the known physical world (two rooms + corridor). Prevents ghost free
                    # voxels behind exterior walls from becoming navigation goals.
                    # Bounds in NED frame (x=North, y=East), +0.5 m wall margin.
                    "world_area_mask_enabled": True,
                    "world_zone_a":        [-8.5,  8.5, -8.5,  8.5],   # Room A
                    "world_zone_corridor": [-2.5,  2.5,  7.5, 14.5],   # Corridor
                    "world_zone_b":        [-8.5,  8.5, 13.5, 30.5],   # Room B
                    "coverage_use_reachable_component": False,
                    "coverage_reachability_guard_enabled": True,
                    "coverage_denominator_drop_ratio": 0.45,
                    "coverage_min_guard_total_cells": 500,
                }],
                remappings=[
                    # Camera bridge runs in px4_0 namespace → no extra remapping needed.
                    # PX4 odometry topic is un-namespaced — map it explicitly.
                    ("fmu/out/vehicle_odometry", "/fmu/out/vehicle_odometry"),
                    # Use pre-filtered LiDAR scan (floor/ceiling hits removed)
                    ("lidar/scan", "lidar/scan_filtered"),
                ],
            ),
        ],
    )

    rtabmap_params = str(Path(get_package_share_directory("swarm_bringup")) / "config" / "rtabmap_params.yaml")

    # ── 2a. Static TF: lidar_2d_link → rgbd_link ─────────────────────────────
    # From x500_swarm/model.sdf:
    #   lidar_2d_link pose relative to base_link: (0, 0, 0.08, 0, 0, 0)
    #   rgbd_link pose relative to base_link:     (0.12, 0.03, 0.06, 0, 0.261799, 0)
    # → rgbd_link relative to lidar_2d_link: t=(0.12, 0.03, -0.02), pitch=0.2618 rad (15° downward)
    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf_pub",
        namespace="px4_0",
        arguments=[
            "--x",        "0.12",
            "--y",        "0.03",
            "--z",        "-0.02",
            "--roll",     "0",
            "--pitch",    "0.2618",
            "--yaw",      "0",
            "--frame-id", "lidar_2d_link",
            "--child-frame-id", "rgbd_link",
        ],
        condition=IfCondition(use_slam),
    )

    # ── 2b. LiDAR tilt filter (+6.2 s — after sensor bridges, before SLAM) ─────
    # Removes floor/ceiling hits caused by drone pitch/roll so SLAM and the
    # voxel mapper never integrate tilted-ray false obstacles into the map.
    lidar_tilt_filter = TimerAction(
        period=6.2,
        actions=[
            Node(
                package="swarm_bringup",
                executable="lidar_tilt_filter.py",
                name="lidar_tilt_filter",
                namespace="px4_0",
                output="screen",
                parameters=[{
                    "match_tol_min":      0.12,
                    "match_tol_factor":   0.08,
                    "ceiling_m_fallback": _CEILING_AGL,
                    "output_frame_id":     "lidar_2d_link",
                }],
                condition=IfCondition(use_slam),
            ),
        ],
    )

    odom_for_slam = TimerAction(
        period=6.5,
        actions=[
            Node(
                package="swarm_bringup",
                executable="odom_publisher.py",
                name="odom_publisher",
                namespace="px4_0",
                output="screen",
                parameters=[{
                    "child_frame_id": "lidar_2d_link",
                }],
                remappings=[
                    ("fmu/out/vehicle_local_position_v1", "/fmu/out/vehicle_local_position_v1"),
                ],
                condition=IfCondition(use_slam),
            ),
        ],
    )

    slam_params = str(Path(get_package_share_directory("swarm_bringup")) / "config" / "slam_toolbox_params.yaml")

    # ── 2c. slam_toolbox LiDAR SLAM (+7.5 s) ────────────────────────────────
    # Uses filtered LiDAR scan (floor/ceiling hits removed by lidar_tilt_filter).
    # This is the active SITL localization stopgap. The thesis/FUEL direction is
    # still camera/VIO or RTAB-Map 6-DOF pose, not GPS and not simulator truth.
    slam_toolbox = TimerAction(
        period=7.5,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                namespace="px4_0",
                output="screen",
                parameters=[slam_params],
                condition=IfCondition(use_slam),
            ),
        ],
    )

    slam_toolbox_autostart = TimerAction(
        period=9.5,
        actions=[
            Node(
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
            ),
        ],
    )

    slam_pose_fusion = TimerAction(
        period=8.5,
        actions=[
            Node(
                package="swarm_bringup",
                executable="slam_pose_fusion.py",
                name="slam_pose_fusion",
                namespace="px4_0",
                output="screen",
                parameters=[{
                    "map_frame": "map",
                    "base_frame": "lidar_2d_link",
                    "px4_odom_topic": "/fmu/out/vehicle_odometry",
                    "publish_rate_hz": 15.0,
                    "yaw_residual_gate_rad": 0.55,
                    "yaw_residual_jump_gate_rad": 0.25,
                    "max_slam_yaw_rate_dps": 90.0,
                    "yaw_drift_on_frames": 2,
                    "yaw_drift_off_frames": 15,
                }],
                condition=IfCondition(use_slam),
            ),
        ],
    )

    # ── 3. Red-can HSV target detector (+8 s — after camera bridge ready) ────
    target_detector = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="swarm_target_detection",
                executable="target_detector",
                name="target_detector",
                namespace="px4_0",
                output="screen",
                parameters=[{
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
                }],
                remappings=[
                    ("fmu/out/vehicle_odometry", "/fmu/out/vehicle_odometry"),
                ],
            ),
        ],
    )

    # ── 4. VFH search mission controller (+10 s — mapper + detector ready) ───
    controller = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="swarm_control",
                executable="search_mission_controller",
                name="search_mission_controller",
                namespace="px4_0",
                # When this node exits, shut down the entire launch process.
                # This is key for the automated experiment script.
                on_exit=[Shutdown(reason="search_mission_controller exited")],
                output="screen",
                parameters=[{
                    # Initial camera lookaround: after climb, rotate slowly in place
                    # before SCAN so the depth-only voxel map has 360° local context.
                    # Keep this much slower than old 30°/s to avoid SLAM yaw shocks.
                    "takeoff_lookaround_enabled": True,
                    "takeoff_yaw_rate_dps": 10.0,
                    "takeoff_map_settle_s":  3.0,
                    "layer_settle_s":        2.0,

                    # Altitude = top of scan box
                    "target_altitude_m":   _BOX_HEIGHT,
                    "climb_rate_mps":      0.40,
                    "ceiling_clearance_m": _CEILING_CLEARANCE,
                    "ceiling_headroom_tolerance_m": 0.20,
                    "floor_clearance_m":   0.45,
                    "floor_recovery_climb_m": 0.40,

                    # Goal-tracking gains
                    "attraction_gain":     1.10,  # ξ — route waypoint attraction
                    "repulsion_gain":      3.0,   # kept for config compatibility
                    "drone_radius_m":      _DRONE_RADIUS,
                    "safety_margin_m":     _SAFETY_MARGIN,
                    "voxel_alt_band_m":    1.0,   # legacy param; not used by VFH

                    # Velocity & acceleration limits
                    # Speed ramp: starts at v_max_mps, steps up by v_max_ramp_step_mps
                    # after each fully-covered layer, up to v_max_target_mps.
                    # fwd_stop, rho0, emergency_stop all scale automatically.
                    "v_max_mps":            ParameterValue(v_max_mps, value_type=float),
                    "v_max_ramp_step_mps":  ParameterValue(v_max_ramp_step_mps, value_type=float),
                    "v_max_target_mps":     ParameterValue(v_max_target_mps, value_type=float),
                    "max_accel_mps2":       1.0,
                    "kd_vel_damp":          0.20,
                    "waypoint_lookahead_radius_m": 1.80,
                    "waypoint_lookahead_cos": 0.20,

                    # VFH histogram radius / compatibility params.
                    "lidar_repulsion_rho0_m": 2.0,   # VFH visibility horizon (NOT the block threshold)
                    # block_range  = min(2.0, max(rho0_=0.35, fwd_stop=0.38)) = 0.38 m
                    # clear_range  = min(2.0, 0.38+0.10)                      = 0.48 m
                    # kCreepStop   = 0.38 m  (gate < block_range → never fires for valid sectors)
                    # Minimum passable corridor: 2 × block_range = 0.76 m
                    "lidar_repulsion_eta":    2.0,  # legacy param; VFH uses rho0 only
                    # Emergency stop only — last resort if VFH somehow fails.
                    "forward_stop_m":      0.70,
                    "forward_resume_margin_m": 0.20,
                    "emergency_stop_range_m": 0.42,
                    # Legacy unicycle yaw alignment gate. With holonomic_vfh=true,
                    # VFH commands XY velocity directly and does not spin the nose
                    # toward every transient corridor valley.
                    "holonomic_vfh":      True,
                    "disable_vfh": ParameterValue(disable_vfh, value_type=bool),
                    "align_threshold_rad": 1.0,
                    # Full 360° LiDAR VFH. Restricting this to a forward camera-like FOV
                    # hides side/rear obstacles from the sector histogram and can make
                    # the planner choose turns that are not actually clear.
                    "vfh_fov_half_deg":    0.0,
                    # Yaw control (tracks flight direction softly — does NOT gate motion)
                    "yaw_slew_alpha":      0.08,  # lower → slower yaw slew, less oscillation
                    "yaw_rate_limit_dps":  30.0,
                    # Camera-aware yaw for depth-only voxel mapping: keep the
                    # holonomic XY controller, but slowly point the RGB-D camera
                    # toward the active route/frontier so mapping keeps up.
                    "camera_yaw_track_goal": True,
                    "camera_yaw_slowdown_rad": 1.30,
                    "camera_yaw_stop_rad": 2.20,
                    "camera_yaw_min_speed_scale": 0.65,
                    # Narrow passages stay passable, but high-speed sweeps slow down
                    # before wall/shelf clearance gets too small.
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
                    "nav_pose_timeout_s":  2.0,     # allow up to 2 s gap in slam/odom_ned
                    "transit_timeout_s":   20.0,    # hold goal for up to 20 s before giving up

                    # Scan bounding box (NED, metres from spawn)
                    "north_min_m":        _SCAN_NORTH_MIN,
                    "north_max_m":        _SCAN_NORTH_MAX,
                    "east_min_m":         _SCAN_EAST_MIN,
                    "east_max_m":         _SCAN_EAST_MAX,

                    # 3-D ascending scan layers: 1.0 → 9.0 m AGL, one metre below ceiling.
                    "scan_alt_start_m":    _SCAN_START,
                    "scan_layer_step_m":   _LAYER_HEIGHT,
                    "return_altitude_m":   _SCAN_START,
                    "layer_complete_frac": 0.83,
                    "layer_complete_stable_s": 8.0,
                    "layer_complete_max_reachable_frontier_cells": 120,
                    "layer_stagnation_min_frac": 0.85,
                    "layer_timeout_min_frac": 0.85,
                    "min_layer_dwell_s":   60.0,
                    "no_frontier_layer_grace_s": 45.0,
                    "layer_dwell_s":      300.0,  # fallback only; normal advance uses coverage/no-frontier/stagnation
                    "layer_stagnation_s":  120.0,
                    "layer_index_start":    0,
                    "layer_index_stride":   1,
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

                    "scan_enabled":        True,

                    # Mission termination
                    "scan_complete_frac":  0.75,
                    "search_timeout_s": 2700.0,   # safety fuse; timeout runs are not valid completed sweeps

                    # Tolerances
                    "goal_reach_radius_m": 0.5,
                    "arm_timeout_s":      180.0,
                    "target_hold_s":       10.0,
                    "target_revisit_radius_m": 6.00,
                    "target_confirm_radius_m": 2.50,
                }],
                remappings=[
                    # PX4 uXRCE-DDS publishes without namespace prefix.
                    ("fmu/in/offboard_control_mode",      "/fmu/in/offboard_control_mode"),
                    ("fmu/in/trajectory_setpoint",        "/fmu/in/trajectory_setpoint"),
                    ("fmu/in/vehicle_command",            "/fmu/in/vehicle_command"),
                    ("fmu/out/vehicle_status_v4",         "/fmu/out/vehicle_status_v4"),
                    ("fmu/out/vehicle_local_position_v1", "/fmu/out/vehicle_local_position_v1"),
                    ("fmu/out/vehicle_attitude",          "/fmu/out/vehicle_attitude"),
                    # voxel_mapper, target_detector, and controller all run in px4_0 namespace
                    # so intra-namespace topics (voxel_map, entropy_centroid, map_update_summary,
                    # camera/depth/image_raw, target_found) resolve automatically — no remapping.
                ],
            ),
        ],
    )

    # ── 5. Optional lightweight camera viewer ─────────────────────────────────
    camera_view = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="swarm_bringup",
                executable="camera_view.py",
                name="drone_camera_view",
                output="screen",
                parameters=[{
                    "topic":       "/px4_0/camera/image_raw",
                    "window_name": "Drone Camera",
                }],
                condition=IfCondition(show_camera_view),
            ),
        ],
    )

    dashboard = TimerAction(
        period=13.0,
        actions=[
            Node(
                package="swarm_gcs",
                executable="mission_dashboard",
                name="mission_dashboard",
                output="screen",
                parameters=[{
                    "drone_ns": "px4_0",
                }],
                condition=IfCondition(show_dashboard),
            ),
        ],
    )

    rviz_cfg = str(Path(get_package_share_directory("swarm_bringup")) / "config" / "scan_rviz.rviz")
    rviz = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="scan_rviz",
                output="screen",
                arguments=["-d", rviz_cfg],
                condition=IfCondition(show_rviz),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="single_agent_search_room_easy",
            description="Gazebo world SDF name (without .sdf)",
        ),
        DeclareLaunchArgument(
            "px4_root",
            default_value=_find_px4_root(),
            description="Absolute path to PX4-Autopilot root",
        ),
        DeclareLaunchArgument(
            "show_camera_view",
            default_value="false",
            description="Open the lightweight HSV camera viewer",
        ),
        DeclareLaunchArgument(
            "show_dashboard",
            default_value="true",
            description="Open the mission dashboard window",
        ),
        DeclareLaunchArgument(
            "show_rviz",
            default_value="false",
            description="Open RViz2 3D view for the voxel map",
        ),
        DeclareLaunchArgument(
            "use_slam",
            default_value="true",
            description="Run GNSS-denied SLAM pose feedback and feed it into mapping and PX4 EKF2",
        ),
        DeclareLaunchArgument(
            "v_max_mps",
            default_value="0.85",
            description="Fixed cruise speed cap for controlled speed experiments",
        ),
        DeclareLaunchArgument(
            "v_max_target_mps",
            default_value="0.85",
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
        slam_to_px4,
        voxel_mapper,
        lidar_tilt_filter,
        odom_for_slam,
        slam_toolbox,
        slam_toolbox_autostart,
        slam_pose_fusion,
        target_detector,
        controller,
        camera_view,
        dashboard,
        rviz,
    ])
