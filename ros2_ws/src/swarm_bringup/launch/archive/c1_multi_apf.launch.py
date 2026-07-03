"""C1 launch: 4 independent apf_controller instances converging on one shared
world point — Stage C's "convergence test".

Pair with multi_apf_sim.launch.py (indoor_arena world — bounded 20x20m room with
2 interior pillars, replacing the open-field apf_open_arena so the "indoor
GNSS-denied" premise holds and Stage D's entropy map has a finite space; 4
x500_swarm spawned in a square at world XY (5,0) (0,5) (-5,0) (0,-5)):
  ros2 launch swarm_sim_bringup multi_apf_sim.launch.py
  ros2 launch swarm_bringup c1_multi_apf.launch.py

Why per-drone goal offsets are needed:
  Each PX4 instance's local NED frame is ZERO at its OWN spawn point. To
  converge on shared world point (0,0,3), each drone's LOCAL NED goal must be
  (world_target - its_own_spawn_position) — but the coordinate mapping matters:

  Gazebo ENU  (used in SDF and PX4_GZ_MODEL_POSE): X=East,  Y=North, Z=Up
  PX4 NED local (used in TrajectorySetpoint):       x=North, y=East,  z=Down

  At spawn yaw=0 the x500_swarm front faces Gazebo +X (East). This means:
    range/front → East (NED +y)   range/back → West (NED -y)
    range/left  → North (NED +x)  range/right → South (NED -x)

  Goal conversion (goal_north = ENU_Y_target - spawn_ENU_Y,
                   goal_east  = ENU_X_target - spawn_ENU_X):
      spawn ENU (5,0)  -> local NED goal ( 0, -5)   [go West]
      spawn ENU (0,5)  -> local NED goal (-5,  0)   [go South]
      spawn ENU (-5,0) -> local NED goal ( 0,  5)   [go East]
      spawn ENU (0,-5) -> local NED goal ( 5,  0)   [go North]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (instance id, spawn_x, spawn_y) — must match multi_apf_sim.launch.py's DRONES
DRONES = [
    (1,  5.0,  0.0),
    (2,  0.0,  5.0),
    (3, -5.0,  0.0),
    (4,  0.0, -5.0),
]

SHARED_GOAL_ENU_X = 0.0   # Gazebo ENU East coordinate of convergence point
SHARED_GOAL_ENU_Y = 0.0   # Gazebo ENU North coordinate of convergence point


def _apf_node(idx: int, goal_north: float, goal_east: float) -> Node:
    ns = f"px4_{idx}"
    return Node(
        package="swarm_control",
        executable="apf_controller",
        namespace=ns,
        name="apf_controller",
        output="screen",
        parameters=[{
            "target_altitude_m":     LaunchConfiguration("target_altitude_m"),
            "goal_north_m":          goal_north,
            "goal_east_m":           goal_east,
            "repulsion_threshold_m": LaunchConfiguration("repulsion_threshold_m"),
            "attraction_gain":       LaunchConfiguration("attraction_gain"),
            "repulsion_gain":        LaunchConfiguration("repulsion_gain"),
            "v_max_m":               LaunchConfiguration("v_max_m"),
            "scan_duration_s":       LaunchConfiguration("scan_duration_s"),
            "cruise_duration_s":     LaunchConfiguration("cruise_duration_s"),
            "arm_timeout_s":         LaunchConfiguration("arm_timeout_s"),
        }],
        # The sensor bridge always namespaces range topics under px4_<id>
        # (multi_apf_sim.launch.py's range_bridge), independent of whether the
        # apf_controller node itself is namespaced — same fix as b1_apf_test.
        remappings=[
            ("range/front", f"/{ns}/range/front"),
            ("range/back",  f"/{ns}/range/back"),
            ("range/left",  f"/{ns}/range/left"),
            ("range/right", f"/{ns}/range/right"),
        ],
    )


def generate_launch_description():
    nodes = []
    for idx, sx, sy in DRONES:
        # ENU X=East → NED y=East; ENU Y=North → NED x=North
        goal_north = SHARED_GOAL_ENU_Y - sy   # Gazebo Y (North) → NED x (North)
        goal_east  = SHARED_GOAL_ENU_X - sx   # Gazebo X (East)  → NED y (East)
        nodes.append(_apf_node(idx, goal_north, goal_east))

    return LaunchDescription([
        DeclareLaunchArgument("target_altitude_m",     default_value="3.0"),
        DeclareLaunchArgument("repulsion_threshold_m", default_value="3.5"),
        DeclareLaunchArgument("attraction_gain",       default_value="0.5",
                              description="k_att: scales the unit attraction vector "
                                          "(metres). With radial saturation the "
                                          "free-space cruise force is k_att * 1.0 = "
                                          "0.2 m/tick (well below v_max=0.5 so never "
                                          "saturated). Equilibrium with k_rep=2.0, "
                                          "threshold=2.0 m is at sensor reading ~1.8 m "
                                          "→ min pair separation ~3.6 m."),
        DeclareLaunchArgument("repulsion_gain",        default_value="2.0"),
        DeclareLaunchArgument("scan_duration_s",       default_value="10.0",
                              description="MODE_0 hover-scan window (wall-clock seconds). "
                                          "No APF forces applied during this window — lets "
                                          "PX4 EKF stabilise post-climb and gives sensors "
                                          "time to observe surroundings before navigation."),
        DeclareLaunchArgument("v_max_m",               default_value="1.0",
                              description="Radial saturation bound (thesis eq. 4.7-4.8). "
                                          "F_total is used raw when |F_total| < v_max; "
                                          "only normalized and capped to v_max when "
                                          "above it. Drone naturally decelerates as "
                                          "forces cancel near equilibrium — no EKF "
                                          "spike at the balance point."),
        DeclareLaunchArgument("cruise_duration_s",     default_value="120.0",
                              description="Doubled from 60s — with attraction_gain "
                                          "raised to push the drones together, they "
                                          "need a longer window to reach the center, "
                                          "have repulsion engage, and settle into a "
                                          "dynamic equilibrium before the cruise ends."),
        DeclareLaunchArgument("arm_timeout_s",         default_value="180.0",
                              description="Wall-clock ARM/OFFBOARD timeout. Stage B's "
                                          "10s default assumed real_time_factor~=1.0 "
                                          "(one PX4 instance); under 4-instance CPU "
                                          "contention RTF measured as low as 0.08, so "
                                          "PX4's EKF (which converges in SIM time) needs "
                                          "far more WALL-CLOCK time to become arm-ready"),
        *nodes,
    ])
