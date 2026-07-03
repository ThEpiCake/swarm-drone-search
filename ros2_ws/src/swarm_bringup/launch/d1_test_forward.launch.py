"""Forward-movement test for the single drone.

Sequence (automated, no user input needed):
  1. Takeoff to 1.5 m — no 360° rotation (test_skip_rotation=true)
  2. Fly test_forward_dist_m (default 2 m) straight ahead in POSITION mode
  3. Hold 2 s
  4. Drive forward at v_max until LiDAR/depth obstacle range reaches test_wall_stop_m
  5. Hold in place forever — Ctrl-C to stop

Purpose: verify the drone moves STRAIGHT (not sideways), and that wall detection works.
If the drone drifts sideways in Phase 3, the coordinate or yaw transform is wrong.
If it crashes instead of stopping, the depth forward-stop is broken.

Important: this launch must run in a minimal wall world. The cluttered search room
is not a valid straight-line experiment because side obstacles can enter the depth ROI
before the drone reaches the wall directly ahead.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    px4_root = LaunchConfiguration("px4_root")
    world    = LaunchConfiguration("world")

    _SCAN_START    = 1.5    # m AGL — takeoff altitude for test
    _DRONE_RADIUS  = 0.25
    _SAFETY_MARGIN = 0.60   # rho0 = 0.85 m (same as main search launch)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(sim_launch)),
        launch_arguments={
            "world":             world,
            "drone_id":          "0",
            "drone_model":       "x500_swarm",
            "px4_sys_autostart": "4023",
            "px4_root":          px4_root,
        }.items(),
    )

    gz_truth_bridge = TimerAction(
        period=5.0,
        actions=[Node(
            package="swarm_bringup",
            executable="gz_truth_bridge.py",
            name="gz_truth_bridge",
            output="screen",
            parameters=[{
                "odom_topic":      "/model/x500_swarm_0/odometry_with_covariance",
                "publish_rate_hz": 30.0,
            }],
        )],
    )

    controller = TimerAction(
        period=10.0,
        actions=[Node(
            package="swarm_control",
            executable="search_mission_controller",
            name="search_mission_controller",
            namespace="px4_0",
            output="screen",
            parameters=[{
                # ── Test mode ─────────────────────────────────────────────────
                "test_forward_mode":    True,   # enable straight-ahead test
                "test_skip_rotation":   True,   # skip 360° rotation at takeoff
                "test_forward_dist_m":  2.0,    # Phase 1: fly this many metres ahead
                "test_wall_stop_m":     1.0,    # stop when obstacle range < this (m)
                "test_hold_s":          2.0,    # Phase 2: short hold before wall approach
                "test_goal_radius_m":   0.20,
                "test_alt_tolerance_m": 0.15,
                "test_ignore_depth_until_phase3": True,  # isolate straight-flight from depth-stop
                "test_lidar_nearest_stop": True,  # safety fallback if LiDAR frame is misaligned

                # ── Takeoff ───────────────────────────────────────────────────
                "scan_alt_start_m":    _SCAN_START,
                "takeoff_yaw_rate_dps": 30.0,
                "takeoff_map_settle_s":  2.0,   # shorter settle — no scan needed
                "climb_rate_mps":       0.15,
                "ceiling_clearance_m":  0.80,
                "floor_clearance_m":    0.45,
                "floor_recovery_climb_m": 0.40,
                "altitude_tolerance_m": 0.30,

                # ── Obstacle detection ────────────────────────────────────────
                "forward_stop_m":           0.8,
                "forward_resume_margin_m":  0.25,
                "emergency_stop_range_m":   0.30,
                "lidar_forward_angle_deg":  0.0,
                "lidar_forward_half_angle_deg": 60.0,
                "depth_roi_width_frac":     0.25,   # ignore most side clutter
                "depth_percentile":        15.0,    # resist single-pixel / thin-leg spikes
                "depth_min_world_elev_deg":  5.0,   # use only above-horizon rows in the wall test
                "depth_max_world_elev_deg": 20.0,   # ignore steep ceiling rows

                # ── VFH/search params (mostly unused in this straight-line test) ─
                "attraction_gain":     0.3,
                "repulsion_gain":      3.0,
                "drone_radius_m":      _DRONE_RADIUS,
                "safety_margin_m":     _SAFETY_MARGIN,
                "voxel_alt_band_m":    1.0,
                "v_max_mps":           0.3,
                "max_accel_mps2":      0.5,

                # ── Scan box (fallback values, not used in test mode) ─────────
                "scan_layer_step_m":   1.0,
                "return_altitude_m":   _SCAN_START,
                "layer_dwell_s":       30.0,
                "layer_stagnation_s":  20.0,
                "north_min_m":        -7.0,
                "north_max_m":         7.0,
                "east_min_m":         -7.0,
                "east_max_m":         29.0,
                "target_altitude_m":   8.5,
                "scan_complete_frac":  0.90,
                "search_timeout_s":  720.0,
                "goal_reach_radius_m": 0.6,
                "arm_timeout_s":      180.0,
                "target_hold_s":        5.0,
                "align_threshold_rad":  0.52,
                "yaw_slew_alpha":       0.20,
                "transit_timeout_s":   15.0,
                "scan_enabled":        False,   # no layer scan in test mode
            }],
            remappings=[
                ("fmu/in/offboard_control_mode",      "/fmu/in/offboard_control_mode"),
                ("fmu/in/trajectory_setpoint",        "/fmu/in/trajectory_setpoint"),
                ("fmu/in/vehicle_command",            "/fmu/in/vehicle_command"),
                ("fmu/out/vehicle_status_v4",         "/fmu/out/vehicle_status_v4"),
                ("fmu/out/vehicle_local_position_v1", "/fmu/out/vehicle_local_position_v1"),
                ("fmu/out/vehicle_attitude",          "/fmu/out/vehicle_attitude"),
            ],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="forward_stop_test_arena",
            description="Gazebo world name",
        ),
        DeclareLaunchArgument(
            "px4_root",
            default_value=_find_px4_root(),
            description="Path to PX4-Autopilot root",
        ),
        sim,
        gz_truth_bridge,
        controller,
    ])
