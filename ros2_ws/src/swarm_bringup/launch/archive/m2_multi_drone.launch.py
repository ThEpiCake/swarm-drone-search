"""M2 launch: 4 offboard_control nodes, one per drone namespace.

Prerequisites: start_multi_sitl.sh must be running and all preflight checks
must pass (run `ros2 topic echo /px4_1/fmu/out/vehicle_status_v4 | grep pre_flight`).

Each drone flies to its waypoint in its own NED local frame:
  px4_1 → alt 3 m, waypoint (8, 0)
  px4_2 → alt 3 m, waypoint (8, 0)
  px4_3 → alt 3 m, waypoint (8, 0)
  px4_4 → alt 3 m, waypoint (8, 0)

Since each drone's local frame origin is its spawn point, they all fly
8 m forward in parallel — no crossing paths.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def _drone_node(ns: str, alt: float, wp_n: float, wp_e: float) -> Node:
    return Node(
        package="swarm_control",
        executable="offboard_control",
        namespace=ns,
        name="offboard_control",
        output="screen",
        parameters=[{
            "target_altitude_m": alt,
            "waypoint_north_m":  wp_n,
            "waypoint_east_m":   wp_e,
            "position_tol_m":    0.5,
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        _drone_node("px4_1", alt=3.0, wp_n=8.0, wp_e=0.0),
        _drone_node("px4_2", alt=3.0, wp_n=8.0, wp_e=0.0),
        _drone_node("px4_3", alt=3.0, wp_n=8.0, wp_e=0.0),
        _drone_node("px4_4", alt=3.0, wp_n=8.0, wp_e=0.0),
    ])
