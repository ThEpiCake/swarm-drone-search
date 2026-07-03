"""
Single-drone simulation bringup for the swarm exploration framework.

Launch sequence:
  1. Extend GZ_SIM_RESOURCE_PATH with our custom models directory so Gazebo
     Harmonic resolves x500_swarm and the supporting payload models natively.
  2. Start Gazebo Harmonic with the indoor search room world.
  3. Spawn the Micro XRCE-DDS Agent listening on UDP port 8888.
  4. Start the PX4 SITL instance (x500_swarm), whose built-in uXRCE-DDS client
     auto-connects to the running agent on localhost:8888.
  5. Bridge the forward RGB-D camera plus the upward and downward rangefinders from
     gz-transport into ROS2 under /px4_<drone_id>/camera/... and
     /px4_<drone_id>/range/....

Usage:
  ros2 launch swarm_sim_bringup single_drone_sim.launch.py
  ros2 launch swarm_sim_bringup single_drone_sim.launch.py world:=lakehouse_preview
  ros2 launch swarm_sim_bringup single_drone_sim.launch.py drone_id:=1
  ros2 launch swarm_sim_bringup single_drone_sim.launch.py drone_model:=x500_flow px4_sys_autostart:=4021
"""

import os
from pathlib import Path
from xml.etree import ElementTree

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("swarm_sim_bringup").find("swarm_sim_bringup")
    models_dir = os.path.join(pkg_share, "models")
    worlds_dir = os.path.join(pkg_share, "worlds")

    world_name = LaunchConfiguration("world").perform(context)
    drone_id = LaunchConfiguration("drone_id").perform(context)
    dds_port = LaunchConfiguration("dds_port").perform(context)
    px4_root = LaunchConfiguration("px4_root").perform(context)
    drone_model = LaunchConfiguration("drone_model").perform(context)
    px4_sys_autostart = LaunchConfiguration("px4_sys_autostart").perform(context)

    world_file = os.path.join(worlds_dir, f"{world_name}.sdf")

    # The gz-transport world topic prefix is the SDF's <world name="...">,
    # which does NOT match the SDF *filename* here (e.g. lakehouse.sdf's world
    # is named "terrain_v1"). px4-rc.gzsim auto-detects this at runtime by
    # grepping `gz topic -l` for "/world/<name>/clock" and overrides
    # PX4_GZ_WORLD — so any gz-transport topic we bridge must use this real
    # name too, not the launch "world" argument.
    gz_world_name = ElementTree.parse(world_file).getroot().find("world").get("name")

    # Extend GZ_SIM_RESOURCE_PATH so Gazebo finds our custom models.
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = f"{models_dir}:{existing_gz_path}" if existing_gz_path else models_dir

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=new_gz_path,
    )

    # PX4's server.config loads the system plugins PX4 sensors depend on
    # (Sensors w/ ogre2 render engine, Imu, AirPressure, Magnetometer, NavSat,
    # OpticalFlowSystem, GstCameraSystem, ...). Normally PX4's gz_env.sh sets
    # GZ_SIM_SERVER_CONFIG_PATH to this file, but gz_env.sh is only sourced
    # when PX4 itself starts Gazebo — since we start Gazebo first, we must
    # point at it ourselves or rendering-based sensors (camera, depth_camera,
    # gpu_lidar) silently produce no data.
    px4_server_config = os.path.join(
        px4_root, "src", "modules", "simulation", "gz_bridge", "server.config"
    )
    # libOpticalFlowSystem.so / libGstCameraSystem.so referenced by server.config
    # live here — also normally found via gz_env.sh's GZ_SIM_SYSTEM_PLUGIN_PATH.
    px4_gz_plugins = os.path.join(
        px4_root, "build", "px4_sitl_default", "src", "modules", "simulation", "gz_plugins"
    )
    existing_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    new_plugin_path = (
        f"{px4_gz_plugins}:{existing_plugin_path}" if existing_plugin_path else px4_gz_plugins
    )

    # Gazebo Harmonic — start headless-ready with the Lake House world.
    gz_sim = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", world_file,
        ],
        additional_env={
            "GZ_SIM_RESOURCE_PATH": new_gz_path,
            "GZ_SIM_SERVER_CONFIG_PATH": px4_server_config,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": new_plugin_path,
        },
        output="screen",
        name="gz_sim",
    )

    # Micro XRCE-DDS Agent — bridges PX4 uXRCE-DDS client to ROS2.
    dds_agent = ExecuteProcess(
        cmd=[
            "MicroXRCEAgent", "udp4",
            "-p", dds_port,
        ],
        output="screen",
        name="micro_xrce_dds_agent",
    )

    px4_bin = os.path.join(
        px4_root,
        "build", "px4_sitl_default", "bin", "px4",
    )
    # PX4 SITL — the configured model spawns into the running Gazebo via gz-transport.
    # PX4_UXRCE_DDS_PORT connects the embedded client to our DDS agent.
    px4_sitl = TimerAction(
        period=3.0,  # wait for Gazebo and DDS agent to be ready
        actions=[
            ExecuteProcess(
                cmd=[px4_bin, "-d"],
                additional_env={
                    "GZ_SIM_RESOURCE_PATH": new_gz_path,
                    "PX4_SYS_AUTOSTART": px4_sys_autostart,
                    "PX4_SIM_MODEL": drone_model,
                    "PX4_GZ_WORLD": world_name,
                    "PX4_INSTANCE": drone_id,
                    "UXRCE_DDS_CFG": "1",
                    "PX4_UXRCE_DDS_PORT": dds_port,
                    "PX4_UXRCE_DDS_IP": "127.0.0.1",
                },
                cwd=px4_root,
                output="screen",
                name="px4_sitl",
            )
        ],
    )

    # ── Sensor bridges (Gazebo gz-transport → ROS2) ───────────────────────────
    # PX4's px4-rc.gzsim spawns the model as "<PX4_SIM_MODEL>_<PX4_INSTANCE>",
    # e.g. "x500_swarm_0" — the gz-transport sensor topics are scoped under that
    # exact spawned name, not the bare model name. The OakD-Lite cameras carry
    # no namespace of their own, so ros_gz_bridge's parameter_bridge is given
    # the full gz topic and the ROS-side name is remapped to a clean relative
    # path; launching the bridge under namespace=px4_<drone_id> then yields
    # /px4_<id>/camera/... — matching offboard_control.cpp's relative-name
    # convention so the same nodes work whether namespaced or not.
    gz_model_name = f"{drone_model}_{drone_id}"
    sensor_prefix = f"/world/{gz_world_name}/model/{gz_model_name}/link/rgbd_link/sensor"
    rgb_image_topic = f"{sensor_prefix}/front_rgb/image"
    rgb_info_topic = f"{sensor_prefix}/front_rgb/camera_info"
    # NOTE: Gazebo's depth_camera sensor plugin publishes depth data on
    # "<sensor>/depth_image" and "<sensor>/depth_image/points" — NOT on
    # "<sensor>/image"/"<sensor>/points" (those exist but are never
    # published to; only camera_info is shared with the bridged-but-empty
    # name). Confirmed live via `gz topic -e -t .../front_depth/depth_image`.
    depth_image_topic = f"{sensor_prefix}/front_depth/depth_image"
    depth_info_topic = f"{sensor_prefix}/front_depth/camera_info"
    depth_points_topic = f"{sensor_prefix}/front_depth/depth_image/points"

    range_prefix = f"/world/{gz_world_name}/model/{gz_model_name}/link"
    up_range_topic   = f"{range_prefix}/range_up_link/sensor/range_up/scan"
    down_range_topic = f"{range_prefix}/lidar_sensor_link/sensor/lidar/scan"
    lidar_2d_topic   = f"{range_prefix}/lidar_2d_link/sensor/lidar_2d/scan"

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=f"px4_{drone_id}",
        name="camera_bridge",
        arguments=[
            f"{rgb_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{rgb_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{depth_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{depth_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{depth_points_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            f"{up_range_topic}@sensor_msgs/msg/Range[gz.msgs.LaserScan",
            f"{down_range_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{lidar_2d_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            (rgb_image_topic,   "camera/image_raw"),
            (rgb_info_topic,    "camera/camera_info"),
            (depth_image_topic, "camera/depth/image_raw"),
            (depth_info_topic,  "camera/depth/camera_info"),
            (depth_points_topic,"camera/depth/points"),
            (up_range_topic,    "range/up"),
            (down_range_topic,  "range/down_scan"),
            (lidar_2d_topic,    "lidar/scan"),
        ],
        output="screen",
    )

    down_range_adapter = Node(
        package="swarm_bringup",
        executable="scan_to_range.py",
        namespace=f"px4_{drone_id}",
        name="down_range_adapter",
        output="screen",
        parameters=[{
            "input_topic": "range/down_scan",
            "output_topic": "range/down",
        }],
    )

    # Bridge the drone model's OdometryWithCovariance directly to nav_msgs/Odometry
    # so gz_truth_bridge.py can read ENU pose (position + orientation) before arming.
    # /model/<model>/odometry_with_covariance is a per-model Gazebo topic that
    # includes full 6-DOF pose in the world (ENU) frame — no name-filtering needed.
    gz_odom_topic = f"/model/{gz_model_name}/odometry_with_covariance"
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pose_bridge",
        arguments=[
            f"{gz_odom_topic}@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance",
        ],
        output="screen",
    )

    return [
        set_gz_resource_path,
        gz_sim,
        dds_agent,
        px4_sitl,
        camera_bridge,
        down_range_adapter,
        pose_bridge,
    ]


def generate_launch_description():
    # Discover PX4-Autopilot by walking up from this file regardless of whether
    # it is running from the source tree or the install tree.
    _here = Path(__file__).resolve()
    px4_default_root = next(
        (str(p / "PX4-Autopilot") for p in _here.parents if (p / "PX4-Autopilot").is_dir()),
        str(_here.parents[6] / "PX4-Autopilot"),  # fallback: install-tree offset
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="single_agent_search_room",
            description="World SDF name (without extension) inside worlds/",
        ),
        DeclareLaunchArgument(
            "drone_id",
            default_value="0",
            description="PX4 SITL instance index (0-based); offsets UDP ports automatically",
        ),
        DeclareLaunchArgument(
            "dds_port",
            default_value="8888",
            description="UDP port for the Micro XRCE-DDS Agent",
        ),
        DeclareLaunchArgument(
            "drone_model",
            default_value="x500_swarm",
            description="Gazebo model name to spawn via PX4 SITL",
        ),
        DeclareLaunchArgument(
            "px4_sys_autostart",
            default_value="4022",
            description="PX4 airframe autostart id matching the selected Gazebo model",
        ),
        DeclareLaunchArgument(
            "px4_root",
            default_value=px4_default_root,
            description="Absolute path to the PX4-Autopilot root directory",
        ),
        OpaqueFunction(function=launch_setup),
    ])
