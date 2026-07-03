# Drone Swarm Search — ROS 2 / PX4 / Gazebo SITL

**Field Object & Target Search with a Drone Swarm in ROS 2**
Etay Baron · Supervisor: Prof. Amir Shapiro · Ben-Gurion University of the Negev

Autonomous GNSS-denied indoor search with one and two cooperating drones, implemented over
**PX4 SITL + ROS 2 Jazzy + Gazebo Harmonic + uXRCE-DDS** and validated in a controlled
simulation experiment campaign. This repository contains the full autonomy code, simulation
assets, documentation, and experiment automation scripts of the final project (course 26-59,
Department of Mechanical Engineering).

## Demo video

A full autonomous search mission - takeoff, layered scan, live voxel mapping, target
inspection, return and landing - sped up ~19x to under a minute (Gazebo view on top,
live operator dashboard below): [media/drone_swarm_mission_19x.mp4](media/drone_swarm_mission_19x.mp4)

## What the system does

Each drone takes off, performs an initial 360° lookaround, explores a two-room warehouse
world in coverage-driven altitude layers, builds a probabilistic 3D voxel map from its
forward RGB-D depth camera, navigates with frontier-based viewpoint selection +
clearance-penalized A* routes + VFH local avoidance + a braking-distance safety guard,
detects and inspects red-can targets, and returns home. Localization is fully GNSS-denied:
2D LiDAR SLAM fused into the PX4 EKF2 estimator through a gated external-vision bridge.
In the dual configuration the drones share a merged voxel map and split the scan layers.

## Repository layout

```
ros2_ws/
├── docs/                          # technical documentation (single source of truth)
│   ├── CURRENT_STACK_STATUS.md    # architecture, parameters, change log
│   └── STRATEGY_3D_SCAN.md        # math and phase plan
└── src/
    ├── swarm_bringup/             # launch files, runtime parameters, worlds, bridges
    ├── swarm_control/             # search mission controller: FSM + VFH nav (C++)
    ├── swarm_perception/          # 3D log-odds voxel mapper, frontiers, A* (C++)
    ├── swarm_target_detection/    # HSV red-target detector (Python)
    ├── swarm_gcs/                 # PyQt5 mission dashboard (operator UI)
    ├── swarm_sim_bringup/         # Gazebo models and worlds
    ├── swarm_msgs/                # custom message types
    └── px4_msgs/                  # PX4 ROS 2 message definitions
scripts/                           # experiment automation, metrics, plots
px4_config/airframes/              # custom PX4 SITL airframe (4022_gz_x500_swarm)
Mode2_Mode3_Implementation_Plan.md # future hardware VIO/SLAM design notes
```

## Prerequisites

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4-Autopilot (SITL, airframe 4022),
Micro XRCE-DDS Agent, `slam_toolbox`, `ros_gz_bridge`, `colcon`, PyQt5.
PX4-Autopilot and Micro-XRCE-DDS-Agent are built separately alongside this workspace.
Before building PX4 SITL, copy the custom airframe into the PX4 tree:

```bash
cp px4_config/airframes/4022_gz_x500_swarm \
   PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

## Build & run

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Terminal 1 — simulation + all nodes
ros2 launch swarm_bringup d1_single_agent_search.launch.py

# Terminal 2 — mission dashboard (optional)
ros2 run swarm_gcs mission_dashboard
```

## Experiments

Controlled runs are launched through the wrappers under `scripts/`:

```bash
scripts/run_search_experiment.sh 1 1.30 speed_sweep     # single run
DRONE_COUNTS="1 2" SPEEDS="0.70 1.30 1.80 2.00" \
  scripts/run_search_experiment_series.sh final_v1      # full speed sweep
```

Each run produces a structured experiment folder with logs, extracted metrics (CSV),
plots, and a summary. The final report's key result: mission duration is U-shaped in
`v_max` with an optimum at **1.30 m/s**; 1.80–2.00 m/s introduces navigation stress and
failures, especially with two drones.
