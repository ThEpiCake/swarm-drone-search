"""M1 launch: single-drone offboard control.

By default targets PX4 instance 0 (no namespace).
Pass drone_namespace:=px4_1 to target a namespaced instance instead.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_altitude_m", default_value="3.0"),
        DeclareLaunchArgument("waypoint_north_m",  default_value="5.0"),
        DeclareLaunchArgument("waypoint_east_m",   default_value="0.0"),
        DeclareLaunchArgument("drone_namespace",   default_value="",
                              description="ROS2 namespace matching the PX4 instance (e.g. px4_1)"),

        Node(
            package="swarm_control",
            executable="offboard_control",
            namespace=LaunchConfiguration("drone_namespace"),
            name="offboard_control",
            output="screen",
            parameters=[{
                "target_altitude_m": LaunchConfiguration("target_altitude_m"),
                "waypoint_north_m":  LaunchConfiguration("waypoint_north_m"),
                "waypoint_east_m":   LaunchConfiguration("waypoint_east_m"),
            }],
        ),
    ])
