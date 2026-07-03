"""Two-drone simulation bringup for the search-room worlds.

Spawns two x500_swarm drones in the same Gazebo world:
  Drone 0: spawns 0.75 m West of the original start (East=-0.75, North=0)
  Drone 1: spawns 0.75 m East of the original start (East=+0.75, North=0)

One Gazebo instance, one DDS agent (both PX4 instances share it — the uXRCE-DDS
client auto-scopes topics under /px4_<id>/).

Per-instance PX4 state is isolated to /tmp/px4_search_<id>/ to avoid
parameters.bson conflicts (same pattern as multi_apf_sim.launch.py).

Usage:
  ros2 launch swarm_sim_bringup dual_drone_sim.launch.py
"""

import glob
import os
import shutil
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

# (drone_id, Gazebo-ENU-X=East, Gazebo-ENU-Y=North, altitude)
DRONES = [
    (0, -0.75, 0.0, 0.35),   # drone 0 - west of original start
    (1,  0.75, 0.0, 0.35),   # drone 1 - east of original start
]

WORLD_NAME   = "single_agent_search_room_easy"
DRONE_MODEL  = "x500_swarm"
PX4_AUTOSTART = "4022"   # GNSS-denied: optical flow + rangefinder + external vision


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("swarm_sim_bringup").find("swarm_sim_bringup")
    models_dir = os.path.join(pkg_share, "models")
    worlds_dir = os.path.join(pkg_share, "worlds")

    dds_port = LaunchConfiguration("dds_port").perform(context)
    px4_root = LaunchConfiguration("px4_root").perform(context)
    world    = LaunchConfiguration("world").perform(context)

    world_file = os.path.join(worlds_dir, f"{world}.sdf")
    gz_world_name = ElementTree.parse(world_file).getroot().find("world").get("name")

    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = f"{models_dir}:{existing_gz_path}" if existing_gz_path else models_dir

    px4_server_config = os.path.join(
        px4_root, "src", "modules", "simulation", "gz_bridge", "server.config"
    )
    px4_gz_plugins = os.path.join(
        px4_root, "build", "px4_sitl_default", "src", "modules", "simulation", "gz_plugins"
    )
    existing_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    new_plugin_path = (
        f"{px4_gz_plugins}:{existing_plugin_path}" if existing_plugin_path else px4_gz_plugins
    )

    # ── Gazebo (with GUI) ──────────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        additional_env={
            "GZ_SIM_RESOURCE_PATH": new_gz_path,
            "GZ_SIM_SERVER_CONFIG_PATH": px4_server_config,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": new_plugin_path,
        },
        output="screen",
        name="gz_sim",
    )

    # ── Single DDS agent — both PX4 clients connect here ─────────────────────
    dds_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", dds_port],
        output="screen",
        name="micro_xrce_dds_agent",
    )

    # ── Isolate per-instance PX4 state ────────────────────────────────────────
    for drone_id, *_ in DRONES:
        work_dir = f"/tmp/px4_search_{drone_id}"
        shutil.rmtree(work_dir, ignore_errors=True)
        os.makedirs(work_dir, exist_ok=True)
    for lock in glob.glob("/tmp/px4_lock-*"):
        try:
            os.remove(lock)
        except OSError:
            pass

    px4_bin = os.path.join(px4_root, "build", "px4_sitl_default", "bin", "px4")
    px4_etc = os.path.join(px4_root, "build", "px4_sitl_default", "etc")

    actions = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_gz_path),
        gz_sim,
        dds_agent,
    ]

    for n, (drone_id, sx, sy, sz) in enumerate(DRONES):
        work_dir  = f"/tmp/px4_search_{drone_id}"
        pose_str  = f"{sx},{sy},{sz},0,0,0"
        # Stagger starts: drone 0 at t=3s, drone 1 at t=8s
        delay = 3.0 + n * 5.0

        px4_instance = TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=[px4_bin, "-i", str(drone_id), "-d", px4_etc],
                    additional_env={
                        "GZ_SIM_RESOURCE_PATH": new_gz_path,
                        "PX4_SYS_AUTOSTART":    PX4_AUTOSTART,
                        "PX4_SIM_MODEL":         DRONE_MODEL,
                        "PX4_GZ_WORLD":          gz_world_name,
                        "PX4_GZ_MODEL_POSE":     pose_str,
                        "PX4_INSTANCE":          str(drone_id),
                        "UXRCE_DDS_CFG":         "1",
                        "PX4_UXRCE_DDS_PORT":    dds_port,
                        "PX4_UXRCE_DDS_IP":      "127.0.0.1",
                        "GZ_IP":                 "127.0.0.1",
                    },
                    cwd=work_dir,
                    output="screen",
                    name=f"px4_sitl_{drone_id}",
                )
            ],
        )
        actions.append(px4_instance)

        # ── Sensor bridges for this drone ─────────────────────────────────────
        gz_model_name  = f"{DRONE_MODEL}_{drone_id}"
        sensor_prefix  = f"/world/{gz_world_name}/model/{gz_model_name}/link/rgbd_link/sensor"
        range_prefix   = f"/world/{gz_world_name}/model/{gz_model_name}/link"

        rgb_t    = f"{sensor_prefix}/front_rgb/image"
        rgb_ci   = f"{sensor_prefix}/front_rgb/camera_info"
        depth_t  = f"{sensor_prefix}/front_depth/depth_image"
        depth_ci = f"{sensor_prefix}/front_depth/camera_info"
        depth_pt = f"{sensor_prefix}/front_depth/depth_image/points"
        up_t     = f"{range_prefix}/range_up_link/sensor/range_up/scan"
        down_t   = f"{range_prefix}/lidar_sensor_link/sensor/lidar/scan"
        lidar_t  = f"{range_prefix}/lidar_2d_link/sensor/lidar_2d/scan"

        camera_bridge = TimerAction(
            period=delay + 2.0,
            actions=[Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=f"px4_{drone_id}",
                name="camera_bridge",
                arguments=[
                    f"{rgb_t}@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"{rgb_ci}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    f"{depth_t}@sensor_msgs/msg/Image[gz.msgs.Image",
                    f"{depth_ci}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    f"{depth_pt}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    f"{up_t}@sensor_msgs/msg/Range[gz.msgs.LaserScan",
                    f"{down_t}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"{lidar_t}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                ],
                remappings=[
                    (rgb_t,    "camera/image_raw"),
                    (rgb_ci,   "camera/camera_info"),
                    (depth_t,  "camera/depth/image_raw"),
                    (depth_ci, "camera/depth/camera_info"),
                    (depth_pt, "camera/depth/points"),
                    (up_t,     "range/up"),
                    (down_t,   "range/down_scan"),
                    (lidar_t,  "lidar/scan"),
                ],
                output="screen",
            )],
        )
        actions.append(camera_bridge)

        down_adapter = TimerAction(
            period=delay + 2.5,
            actions=[Node(
                package="swarm_bringup",
                executable="scan_to_range.py",
                namespace=f"px4_{drone_id}",
                name="down_range_adapter",
                output="screen",
                parameters=[{
                    "input_topic":  "range/down_scan",
                    "output_topic": "range/down",
                }],
            )],
        )
        actions.append(down_adapter)

        gz_odom_topic = f"/model/{gz_model_name}/odometry_with_covariance"
        pose_bridge = TimerAction(
            period=delay + 2.0,
            actions=[Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"pose_bridge_{drone_id}",
                arguments=[
                    f"{gz_odom_topic}@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance",
                ],
                output="screen",
            )],
        )
        actions.append(pose_bridge)

    # CameraTracking plugin removed from world SDF — PX4's /gui/follow request
    # now times out silently and the camera stays free.  No workaround needed.
    return actions


def generate_launch_description():
    _here = Path(__file__).resolve()
    px4_default_root = next(
        (str(p / "PX4-Autopilot") for p in _here.parents if (p / "PX4-Autopilot").is_dir()),
        str(_here.parents[6] / "PX4-Autopilot"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=WORLD_NAME),
        DeclareLaunchArgument("dds_port", default_value="8888"),
        DeclareLaunchArgument("px4_root", default_value=px4_default_root),
        OpaqueFunction(function=launch_setup),
    ])
