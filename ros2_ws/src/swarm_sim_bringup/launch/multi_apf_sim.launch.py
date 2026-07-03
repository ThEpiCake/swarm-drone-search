"""Multi-drone simulation bringup for Stage C (APF convergence test).

Launches ONE Gazebo Harmonic world (apf_open_arena — flat, textured, no
obstacles besides the drones themselves) plus 4 PX4 SITL instances of
x500_swarm spawned around it in a square, each with its own uXRCE-DDS
client (sharing one Micro XRCE-DDS Agent on :8888, exactly like
start_multi_sitl.sh) and its own range-sensor bridge under /px4_<id>/range/...

This mirrors single_drone_sim.launch.py's proven world+PX4+bridge recipe,
just looped 4x with staggered start times (to avoid spawn races) and
per-instance working directories (for parameters.bson/dataman isolation,
exactly like start_multi_sitl.sh's /tmp/px4_i_<id> dirs).

Spawn layout (world XY, matches Stage C spec — a converging square):
  px4_1 → ( 5,  0)      px4_2 → ( 0,  5)
  px4_3 → (-5,  0)      px4_4 → ( 0, -5)
All 4 then attract toward shared world point (0, 0, 3) — see c1_multi_apf.launch.py
for the per-drone goal-offset math (each drone's local NED origin is its own
spawn point, so the "shared point" goal differs per drone in local coordinates).

Usage:
  ros2 launch swarm_sim_bringup multi_apf_sim.launch.py
  ros2 launch swarm_sim_bringup multi_apf_sim.launch.py px4_root:=/path/to/PX4-Autopilot
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

# (instance id, spawn x, spawn y) — world-frame spawn positions, square layout
DRONES = [
    (1,  5.0,  0.0),
    (2,  0.0,  5.0),
    (3, -5.0,  0.0),
    (4,  0.0, -5.0),
]

WORLD_NAME = "indoor_arena"
DRONE_MODEL = "x500_swarm"
PX4_SYS_AUTOSTART = "4001"  # x500 with GPS — accurate EKF for APF position feedback


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("swarm_sim_bringup").find("swarm_sim_bringup")
    models_dir = os.path.join(pkg_share, "models")
    worlds_dir = os.path.join(pkg_share, "worlds")

    dds_port = LaunchConfiguration("dds_port").perform(context)
    px4_root = LaunchConfiguration("px4_root").perform(context)

    world_file = os.path.join(worlds_dir, f"{WORLD_NAME}.sdf")
    gz_world_name = ElementTree.parse(world_file).getroot().find("world").get("name")

    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path = f"{models_dir}:{existing_gz_path}" if existing_gz_path else models_dir

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=new_gz_path,
    )

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

    gz_sim = ExecuteProcess(
        # -s = server only (no GUI client). 4 simultaneous PX4/EKF/bridge instances
        # already strain the CPU enough that real_time_factor was observed dropping
        # to ~0.03-0.15 with the GUI running — headless keeps more budget for physics.
        cmd=["gz", "sim", "-s", "-r", world_file],
        additional_env={
            "GZ_SIM_RESOURCE_PATH": new_gz_path,
            "GZ_SIM_SERVER_CONFIG_PATH": px4_server_config,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": new_plugin_path,
        },
        output="screen",
        name="gz_sim",
    )

    dds_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", dds_port],
        output="screen",
        name="micro_xrce_dds_agent",
    )

    px4_bin = os.path.join(px4_root, "build", "px4_sitl_default", "bin", "px4")
    px4_etc = os.path.join(px4_root, "build", "px4_sitl_default", "etc")

    # ── Per-instance state isolation (parameters.bson, dataman, logs) ─────────
    # Mirrors start_multi_sitl.sh's /tmp/px4_i_<idx> dirs — without this, all 4
    # instances fight over the same parameters.bson and one comes up corrupted.
    for idx, _, _ in DRONES:
        work_dir = f"/tmp/px4_apf_i_{idx}"
        shutil.rmtree(work_dir, ignore_errors=True)
        os.makedirs(work_dir, exist_ok=True)
    for lock in glob.glob("/tmp/px4_lock-*"):
        os.remove(lock)
    for bson in glob.glob(os.path.join(px4_root, "**", "parameters.bson"), recursive=True):
        os.remove(bson)

    actions = [set_gz_resource_path, gz_sim, dds_agent]

    # Stagger PX4 instance starts so each one's model-spawn into the shared
    # Gazebo world doesn't race the others (same spirit as start_multi_sitl.sh's
    # sequential sleeps, just expressed as TimerAction delays from t=0).
    base_delay = 5.0
    stagger_step = 3.0

    for n, (idx, sx, sy) in enumerate(DRONES):
        work_dir = f"/tmp/px4_apf_i_{idx}"
        pose_str = f"{sx},{sy},0.35,0,0,0"
        delay = base_delay + n * stagger_step

        px4_instance = TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=[px4_bin, "-i", str(idx), "-d", px4_etc],
                    additional_env={
                        "GZ_SIM_RESOURCE_PATH": new_gz_path,
                        "PX4_SYS_AUTOSTART": PX4_SYS_AUTOSTART,
                        "PX4_SIM_MODEL": DRONE_MODEL,
                        "PX4_GZ_WORLD": WORLD_NAME,
                        "PX4_GZ_MODEL_POSE": pose_str,
                        "PX4_INSTANCE": str(idx),
                        "UXRCE_DDS_CFG": "1",
                        "PX4_UXRCE_DDS_PORT": dds_port,
                        "PX4_UXRCE_DDS_IP": "127.0.0.1",
                        "GZ_IP": "127.0.0.1",
                    },
                    cwd=work_dir,
                    output="screen",
                    name=f"px4_sitl_{idx}",
                )
            ],
        )
        actions.append(px4_instance)

        # ── Range-sensor bridge for this drone (matches single_drone_sim.launch.py;
        # camera/depth omitted — Stage C is APF-only, range sensors are the only
        # APF input and the only thing this stage needs bridged). ────────────────
        gz_model_name = f"{DRONE_MODEL}_{idx}"
        range_prefix = f"/world/{gz_world_name}/model/{gz_model_name}/link"
        front_t = f"{range_prefix}/range_front_link/sensor/range_front/scan"
        back_t  = f"{range_prefix}/range_back_link/sensor/range_back/scan"
        left_t  = f"{range_prefix}/range_left_link/sensor/range_left/scan"
        right_t = f"{range_prefix}/range_right_link/sensor/range_right/scan"

        range_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=f"px4_{idx}",
            name="range_bridge",
            arguments=[
                f"{front_t}@sensor_msgs/msg/Range[gz.msgs.LaserScan",
                f"{back_t}@sensor_msgs/msg/Range[gz.msgs.LaserScan",
                f"{left_t}@sensor_msgs/msg/Range[gz.msgs.LaserScan",
                f"{right_t}@sensor_msgs/msg/Range[gz.msgs.LaserScan",
            ],
            remappings=[
                (front_t, "range/front"),
                (back_t,  "range/back"),
                (left_t,  "range/left"),
                (right_t, "range/right"),
            ],
            output="screen",
        )
        actions.append(range_bridge)

    return actions


def generate_launch_description():
    px4_default_root = str(Path(__file__).parents[4] / "PX4-Autopilot")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dds_port",
            default_value="8888",
            description="UDP port for the shared Micro XRCE-DDS Agent (all 4 instances connect here, like start_multi_sitl.sh)",
        ),
        DeclareLaunchArgument(
            "px4_root",
            default_value=px4_default_root,
            description="Absolute path to the PX4-Autopilot root directory",
        ),
        OpaqueFunction(function=launch_setup),
    ])
