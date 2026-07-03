"""M3 launch: per-drone target detection + live voxel occupancy/entropy mapping.

Launches `target_detector` (swarm_target_detection) and `voxel_mapper`
(swarm_perception) together, consuming the RGB-D streams bridged by
single_drone_sim.launch.py's camera_bridge (camera/image_raw, camera/depth/...)
plus the drone's fmu/out/vehicle_odometry, and publishing target_found /
map_update_summary + voxel_map respectively.

Topic-namespacing quirk this file works around: PX4 instance 0 publishes its
fmu/... topics with NO namespace prefix (see PX4's rcS — uxrce_dds_ns is only
set for px4_instance != 0), but single_drone_sim.launch.py's camera_bridge
ALWAYS namespaces the camera streams under px4_<id>, including for instance 0.
So for drone_id 0 the nodes must run unnamespaced (to see the flat fmu/...
topics) with the camera subscriptions remapped explicitly to /px4_0/camera/...;
for drone_id >= 1, PX4 *does* namespace fmu/... under px4_<id> too, so a single
uniform node namespace correctly covers both camera and fmu/... topics.

By default targets PX4 instance 0 (matching single_drone_sim.launch.py's
default). Pass drone_id:=1 to target another namespaced instance.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone_id_str = LaunchConfiguration("drone_id").perform(context)

    if drone_id_str == "0":
        node_namespace = ""
        camera_ns = "/px4_0/camera"
        camera_remaps = [
            ("camera/image_raw", f"{camera_ns}/image_raw"),
            ("camera/depth/image_raw", f"{camera_ns}/depth/image_raw"),
            ("camera/depth/camera_info", f"{camera_ns}/depth/camera_info"),
        ]
    else:
        node_namespace = f"px4_{drone_id_str}"
        camera_remaps = []

    common_params = {"drone_id": int(drone_id_str)}

    detector = Node(
        package="swarm_target_detection",
        executable="target_detector",
        namespace=node_namespace,
        name="target_detector",
        output="screen",
        parameters=[common_params],
        remappings=camera_remaps,
    )

    mapper = Node(
        package="swarm_perception",
        executable="voxel_mapper",
        namespace=node_namespace,
        name="voxel_mapper",
        output="screen",
        parameters=[dict(common_params, **{
            "voxel_size_m":    LaunchConfiguration("voxel_size_m"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
        })],
        remappings=camera_remaps,
    )

    return [detector, mapper]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "drone_id", default_value="0",
            description="PX4 instance id (namespacing handled per PX4's own px4_<id> convention)"),
        DeclareLaunchArgument("voxel_size_m", default_value="0.2"),
        DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),

        OpaqueFunction(function=launch_setup),
    ])
