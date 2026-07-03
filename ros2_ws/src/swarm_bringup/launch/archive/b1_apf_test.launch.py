"""B1 launch: single-drone APF obstacle-avoidance controller (Stage B).

Runs the apf_controller node, which arms, takes off, and then runs a
reactive potential-field cruise toward a far goal — repulsion from the
4 horizontal range sensors deflects it away from obstacles in its path.

Pair with the apf_wall_arena world:
  ros2 launch swarm_sim_bringup single_drone_sim.launch.py world:=apf_wall_arena
  ros2 launch swarm_bringup b1_apf_test.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_altitude_m",     default_value="3.0"),
        DeclareLaunchArgument("goal_north_m",          default_value="30.0"),
        DeclareLaunchArgument("repulsion_threshold_m", default_value="2.0"),
        DeclareLaunchArgument("attraction_gain",       default_value="1.0"),
        DeclareLaunchArgument("repulsion_gain",        default_value="2.5"),
        DeclareLaunchArgument("step_m",                default_value="0.15"),
        DeclareLaunchArgument("cruise_duration_s",     default_value="30.0"),
        DeclareLaunchArgument("drone_namespace",       default_value="",
                              description="ROS2 namespace matching the PX4 instance (e.g. px4_1)"),
        DeclareLaunchArgument("range_namespace",       default_value="px4_0",
                              description="Namespace the sensor bridge publishes range topics under "
                                          "(single_drone_sim.launch.py's camera_bridge always uses "
                                          "px4_<drone_id>, independent of the PX4/fmu namespace, which "
                                          "stays unnamespaced for instance 0)"),

        Node(
            package="swarm_control",
            executable="apf_controller",
            namespace=LaunchConfiguration("drone_namespace"),
            name="apf_controller",
            output="screen",
            parameters=[{
                "target_altitude_m":     LaunchConfiguration("target_altitude_m"),
                "goal_north_m":          LaunchConfiguration("goal_north_m"),
                "repulsion_threshold_m": LaunchConfiguration("repulsion_threshold_m"),
                "attraction_gain":       LaunchConfiguration("attraction_gain"),
                "repulsion_gain":        LaunchConfiguration("repulsion_gain"),
                "step_m":                LaunchConfiguration("step_m"),
                "cruise_duration_s":     LaunchConfiguration("cruise_duration_s"),
            }],
            remappings=[
                ("range/front", [LaunchConfiguration("range_namespace"), "/range/front"]),
                ("range/back",  [LaunchConfiguration("range_namespace"), "/range/back"]),
                ("range/left",  [LaunchConfiguration("range_namespace"), "/range/left"]),
                ("range/right", [LaunchConfiguration("range_namespace"), "/range/right"]),
            ],
        ),
    ])
