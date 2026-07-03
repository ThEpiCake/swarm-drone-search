# Multi-UAV Swarm — Mode 2 (Scan + Navigate + SLAM) & Mode 3 (Return-to-Home)

> Status note:
> This is a planning document for a SLAM/VIO branch.
> It is useful as design input, but it is not the active runtime path in the repo today.
> Do not update this file as if it were the current execution plan; use `ros2_ws/docs/CURRENT_STACK_STATUS.md` for that.

> 2026-06-25 update:
> The active SITL stack now has a working single-drone autonomous scan/search mission:
> GNSS-denied takeoff, initial 360-degree lookaround, depth-camera voxel mapping,
> frontier scoring, A* route following, holonomic VFH local safety, Coca-Cola can
> detection, 10 s target inspection, and return to scanning. The current fixed speed is
> 0.85 m/s with speed ramp disabled.
>
> The next execution plan is:
> 1. Run structured single-drone fixed-speed experiments.
> 2. Add a second drone with shared map/target information and dynamic inter-agent avoidance.
> 3. Repeat speed experiments with two coordinated drones.
> 4. Consolidate the experiment results into the final LaTeX report, send to the supervisor,
>    apply corrections, and submit the final department report.

## Single-Drone Simulation Implementation Plan for Claude Code

> **Project:** Field Object and Target Search with a Drone Swarm in ROS2
> **Author:** Etay Baron · Supervisor: Prof. Amir Shapiro · Ben-Gurion University
> **Stack:** Gazebo Harmonic + PX4 SITL + ROS 2 Jazzy + uXRCE-DDS
> **Scope of THIS document:** ONE drone, full simulation. No GPS. Stereo-VIO (Intel RealSense D435i) for localization + mapping.
> **Goal:** A single drone that takes off stably (GPS-denied), explores an unknown indoor space while building a 3D map, searches for a red Coca-Cola can, and returns home efficiently.

---

## 0. CRITICAL CONTEXT — Read This First

### The root cause of the current sideways drift
The drone currently takes off, drifts sideways/backward, and hits walls. **This is NOT a tuning bug — it is an architectural gap:**

- In GPS-denied mode (airframe 4022), there is **no reliable yaw source**.
- Optical flow gives *velocity*, ToF gives *height*, but **neither gives yaw or absolute position**.
- EKF2 falls back to the **magnetometer**, which is corrupted in Gazebo Harmonic (`cs_mag_field_disturbed=true` — a known sim bug where mag stddev ≈ 2× Earth's field).
- Corrupted yaw → the drone's reference frame is rotated → "forward" becomes "sideways" → drift into walls.

### The fix: Stereo VIO provides full 6-DOF pose INCLUDING yaw
A stereo visual-inertial system (D435i + RTAB-Map) solves **three problems at once**:

| Source | XY position | Yaw | Map |
|---|---|---|---|
| Optical flow + ToF | ❌ velocity only | ❌ | ❌ |
| Magnetometer | ❌ | ⚠️ corrupted in sim | ❌ |
| **Stereo VIO (D435i + RTAB-Map)** | ✅ | ✅ | ✅ |

**This is why we use a depth/stereo camera instead of LiDAR:** it delivers pose (incl. yaw) AND the map from the same sensor. The pose stabilizes flight (kills the drift); the map drives the entropy/APF search.

### Sensor decision: Intel RealSense D435i (available in lab)
- D435i provides **depth directly** (no need to compute disparity ourselves) + an **integrated IMU**.
- Strong existing support: `rtabmap-drone-example`, PX4 VIO docs, many ROS 2 references.
- In simulation we model a D435i-equivalent RGB-D + IMU sensor in Gazebo Harmonic.

### SLAM choice: RTAB-Map (NOT slam_toolbox)
- `slam_toolbox` is **2D only** — incompatible with our 3D voxel formulation.
- RTAB-Map is RGB-D / stereo graph-SLAM with loop closure, outputs **6-DOF pose + 3D occupancy**, and is supported on **ROS 2 Jazzy**.
- ⚠️ Known issue: if RTAB-Map topic rates feel laggy, suspect **DDS QoS**, not RTAB-Map itself.

---

## 1. Target Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      GAZEBO HARMONIC                          │
│   Drone model + D435i-equivalent RGB-D plugin + IMU           │
│   World: indoor room with obstacles + a red can target        │
└───────────────────────┬──────────────────────────────────────┘
                        │ rgb + depth + camera_info + imu
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                  RTAB-Map (Stereo/RGB-D VIO + SLAM)           │
│   IN:  rgb image, depth image, camera_info, imu               │
│   OUT: (1) 6-DOF odometry (incl. YAW)   (2) 3D occupancy grid │
└──────────┬────────────────────────────────┬──────────────────┘
           │ 6-DOF pose                      │ occupancy / cloud
           ▼                                 ▼
┌────────────────────────┐      ┌────────────────────────────────┐
│ VIO→PX4 bridge node    │      │  Probabilistic voxel map        │
│ publishes              │      │  (log-odds, report Eq. 4.2-4.3) │
│ VehicleVisualOdometry  │      │  → entropy (4.11)               │
│ over uXRCE-DDS         │      │  → entropy centroid (4.13)      │
│ EKF2_EV_CTRL = 15      │      │  → F_att (4.14)                 │
└──────────┬─────────────┘      └───────────────┬────────────────┘
           │ stable pose+yaw → EKF2             │ F_att
           ▼                                    ▼
┌─────────────────────────────────────────────────────────────┐
│            MODE CONTROLLER (state machine)                    │
│   Mode 1: Takeoff & stabilize (hover, no drift)               │
│   Mode 2: Explore + map + search   (APF: F_att+F_rep)         │
│   Mode 3: Return-to-home & land    (planned path on known map)│
│   → velocity setpoints (Mode 2) / goto setpoints (Mode 3)     │
│   → PX4 offboard via uXRCE-DDS                                │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
                   PX4 SITL → motors
```

**Key insight:** RTAB-Map feeds TWO consumers from one output —
1. **pose → EKF2** stabilizes flight (fixes the drift)
2. **map → APF** drives the search

This unifies the localization and mapping that the midterm report (§2.4.2, Forsman & Tidén) treats separately. With accurate VIO pose, the map stays clean.

---

## 2. Repository / Workspace Layout (proposed)

```
swarm_ws/
├── src/
│   ├── swarm_bringup/          # launch files, params, worlds
│   │   ├── launch/
│   │   │   ├── sim_mode1.launch.py
│   │   │   ├── sim_mode2.launch.py
│   │   │   ├── sim_mode3.launch.py
│   │   │   └── slam_rtabmap.launch.py
│   │   ├── worlds/
│   │   │   └── indoor_search.sdf
│   │   ├── models/
│   │   │   ├── drone_d435i/     # drone + RGB-D + IMU
│   │   │   └── coke_can/        # red target
│   │   └── params/
│   │       └── ekf2_gps_denied.params
│   ├── vio_px4_bridge/         # RTAB-Map odom → VehicleVisualOdometry (C++)
│   ├── swarm_mapping/          # log-odds voxel grid + entropy (C++)
│   ├── swarm_apf/              # APF controller, Mode 2 (C++)
│   ├── swarm_planner/          # Mode 3 return-to-home planner (C++)
│   ├── swarm_state_machine/    # mode controller (Python)
│   └── swarm_perception/       # YOLO can detection (Python)
└── README.md
```

> **Time-critical loops in C++** (mapping, APF, bridge, planner). **YOLO + state machine in Python.** This matches the locked architecture decisions.

---

## 3. PHASE A — Stable GPS-Denied Flight (Mode 1) — *Day 1 morning*

> **This phase must fully pass before anything else.** A frontier/APF loop on an unstable drone is wasted effort.

### Step 0 — Sensors in Gazebo
- [ ] Remove any LiDAR from the drone model.
- [ ] Add a **D435i-equivalent RGB-D camera plugin** (`gz-sim` sensor) facing forward (with slight downward tilt, ~15°, for floor features).
- [ ] Add/confirm an **IMU sensor** on the airframe.
- [ ] Required published topics (ROS 2): `~/rgb/image`, `~/depth/image`, `~/rgb/camera_info`, `~/imu`.
- [ ] **VERIFY:** `ros2 topic hz` on each — RGB & depth ≥ 15 Hz, IMU ≥ 100 Hz. `ros2 topic echo --once` confirms valid data.

### Step 1 — RTAB-Map standalone (no flight)
- [ ] Create `slam_rtabmap.launch.py` wiring the four topics into RTAB-Map (RGB-D mode, `subscribe_depth=true`, `subscribe_imu=true`, `frame_id` set to the drone base frame).
- [ ] Run RTAB-Map while moving the drone **manually** in Gazebo (drag it / teleport it).
- [ ] **VERIFY (CRITICAL):** In RViz, pose updates and a map starts building. `ros2 topic echo` on RTAB-Map odometry → **yaw changes correctly when you rotate the drone manually.** If yaw is correct here, the drift's root cause is solved.

### Step 2 — Bridge RTAB-Map pose → EKF2
- [ ] Implement `vio_px4_bridge` (C++): subscribe to RTAB-Map odometry, republish as **`VehicleVisualOdometry`** on the uXRCE-DDS topic, with correct **ENU→NED** conversion and proper covariance.
- [ ] Apply EKF2 params (see §7 below): `EKF2_EV_CTRL = 15`, `EKF2_GPS_CTRL = 0`, `EKF2_EV_DELAY = 50`, `EKF2_MAG_TYPE = 0` (now safe — VIO supplies yaw), `EKF2_BARO_CTRL = 0`.
- [ ] **VERIFY:** Arm + take off to 1.5 m and **hover**. The drone must hold position **without sideways drift**. Watch EKF2 console for absence of `mag_field_disturbed` dominance and convergence of EV fusion. This is **Mode 1 PASS in true GPS-denied mode.**

### Step 3 — Single waypoint
- [ ] Send a simple offboard sequence: take off → fly 2 m forward → stop → hold.
- [ ] **VERIFY:** Drone moves in the **commanded direction** (forward, not sideways). Confirms frame + yaw are consistent end-to-end.

> ✅ **Phase A gate:** stable hover + correct-direction waypoint in GPS-denied mode. **Do not proceed to Phase B until this passes.**

---

## 4. PHASE B — Explore + Map + Search (Mode 2) — *Day 1 afternoon → Day 2*

### Step 4 — Probabilistic voxel map from RTAB-Map
- [ ] Implement `swarm_mapping` (C++): consume RTAB-Map occupancy/cloud, maintain a **log-odds 3D voxel grid** per report Eq. (4.2)–(4.3).
- [ ] Classify voxels Free / Occupied / Unscanned via thresholds `b_free < 0.5 < b_occ` (Eq. 4.1).
- [ ] Compute per-voxel **Bernoulli entropy** `H(b)` (Eq. 4.11) and **global entropy** `H(k) = Σ H` (Eq. 5.1).
- [ ] Publish the occupied-voxel set and the entropy field for the controller + RViz.
- [ ] **VERIFY:** RViz shows the voxel map filling in as the drone (manually moved) observes the room; global entropy decreases over time.

### Step 5 — APF exploration controller (Mode 2 core)
- [ ] Implement `swarm_apf` (C++):
  - **Attraction** toward local entropy centroid `p_ent` → `F_att = ξ (p_ent − p_i)` (Eq. 4.13–4.14).
  - **Obstacle repulsion** from occupied voxels within `R_sense` (Eq. 4.15, 4.17).
  - **Agent repulsion** term present but inert for a single drone (Eq. 4.16) — keep the interface for the swarm later.
  - **Saturate** the resultant to `V_max` (Eq. 4.7–4.8).
- [ ] Convert `F_total` → **velocity setpoint** → PX4 offboard (`TrajectorySetpoint` velocity, or `goto` if smoother).
- [ ] **VERIFY:** With the drone armed and flying, it **moves autonomously toward unexplored space**, avoids walls, and global entropy `H(k)` steadily drops. **This is Mode 2 working.**

### Step 6 — Target search (red Coca-Cola can) — runs concurrently
- [ ] Implement `swarm_perception` (Python): run **YOLO** on the RGB stream to detect the red can.
- [ ] On detection, back-project using depth to estimate the can's 3D position; record it and raise a `target_found` flag (latched, Reliable QoS — mission-critical per report §2.1.3).
- [ ] Apply the **velocity–perception trade-off**: detection probability decreases with speed `g(‖v‖)` (report §5.1.3) — cap `V_max` so detection stays reliable.
- [ ] **VERIFY:** Fly the drone past the can; YOLO detects it, a 3D position is logged, and the found-marker appears in RViz.

> ✅ **Phase B gate:** drone autonomously explores, builds the map, entropy drops below `H_thr`, and the can is detected with a recorded location. **This is the Day-2 target.**

---

## 5. PHASE C — Return-to-Home & Land (Mode 3) — *Day 2 → Wednesday*

> **Concept:** By the end of Mode 2 the drone already has a near-complete map. Mode 3 is **planned navigation on a known map** — fly home fast without hitting anything. This is a *new* component not yet in the report; it complements the reactive APF with a **global planner**.

### Design choice — why a planner here, not pure APF
- Mode 2 uses **reactive** APF (it doesn't know the layout ahead).
- Mode 3 has a **known map**, so a **global path planner (A\*/Dijkstra on the occupancy grid)** gives an efficient, obstacle-free route home — avoiding APF local-minima on the way back.
- Use PX4 **`goto_setpoint`** (`/fmu/in/goto_setpoint`, PX4 ≥ 1.15) to track waypoints along the planned path. `goto` is the right primitive for "go to this point" (vs. velocity setpoints used for reactive exploration).

### Step 7 — Global planner on the known map
- [ ] Implement `swarm_planner` (C++): run **A\*** (or Dijkstra) over the occupied/free voxel grid from `swarm_mapping`.
- [ ] Inflate obstacles by `d_safe` (report §5.1.5 obstacle clearance) so the path keeps a safety margin.
- [ ] Plan from current pose → recorded **home** position (captured at takeoff). Output an ordered waypoint list.
- [ ] **VERIFY:** In RViz the planned path goes from the drone to home, routes **around** known obstacles, and respects the inflation margin.

### Step 8 — Execute the return path
- [ ] Feed planned waypoints to PX4 as a sequence of **`goto_setpoint`s**; advance to the next waypoint on arrival within a tolerance.
- [ ] Keep a thin **reactive safety layer** (obstacle repulsion only) active during transit to catch anything the static map missed.
- [ ] On reaching home, command **land**; disarm on touchdown.
- [ ] **VERIFY:** Drone flies the planned route home **without collisions**, faster/straighter than the exploration path, and lands cleanly at the start point.

> ✅ **Phase C gate:** from anywhere in the explored room, the drone plans and flies an efficient, collision-free path home and lands. **This is Mode 3 working.**

---

## 6. The Three-Mode State Machine

- [ ] Implement `swarm_state_machine` (Python) with explicit transitions:

```
ARMED → [Mode 1: TAKEOFF] → reaches hover altitude, pose stable
      → [Mode 2: EXPLORE]  → loop: map + APF + YOLO
                             until H(k) ≤ H_thr  (and/or can found, per mission rule)
      → [Mode 3: RETURN]    → plan path → fly goto waypoints → LAND
      → DISARMED
```

- [ ] Transition Mode 1→2 only after the **pose-stable** check passes.
- [ ] Transition Mode 2→3 on **entropy threshold reached** (report Eq. 5.2) — decide whether "can found" ends the mission or exploration continues to full coverage (recommend: continue to `H_thr` so coverage is provable, then return).
- [ ] Capture **home pose** at the moment of arming/takeoff.
- [ ] **VERIFY:** Full uninterrupted run: arm → takeoff → autonomous explore+search → return → land, with mode transitions logged.

---

## 7. EKF2 Parameters — GPS-Denied with Stereo VIO

Put these in `params/ekf2_gps_denied.params` (airframe 4022). Rationale included so future-you understands each one.

```
# --- Disable GNSS ---
param set-default EKF2_GPS_CTRL 0      # no GPS fusion (indoor)

# --- Enable external vision: position + velocity + YAW ---
param set-default EKF2_EV_CTRL 15      # bit0 XY + bit1 Z + bit2 vel + bit3 YAW
                                       # (bit3 = yaw is the key fix; mag was the only
                                       #  yaw source before and it's corrupted in sim)

# --- Height reference ---
param set-default EKF2_HGT_REF 3       # Vision. (If you keep the LW20 rangefinder
                                       #  in sim, you may use 2 instead — but for a
                                       #  pure-VIO sim, Vision is cleanest.)

# --- Vision latency ---
param set-default EKF2_EV_DELAY 50     # ms. RTAB-Map on the same machine ≈ 30–80 ms.
                                       # Measure with `ros2 topic delay` and refine.

# --- Disable magnetometer (now safe: VIO supplies yaw via EV_CTRL bit3) ---
param set-default EKF2_MAG_TYPE 0      # stop fusing the corrupted Gazebo mag

# --- Disable baro as primary (vision/rangefinder owns height) ---
param set-default EKF2_BARO_CTRL 0
```

> ⚠️ **Ordering rule (learned the hard way):** `EKF2_MAG_TYPE 0` is only safe **after** `EKF2_EV_CTRL` has bit 3 set AND VIO is actually publishing yaw. If you disable mag without a working yaw source, yaw becomes unconstrained gyro drift — worse than the mag noise.

---

## 8. Known Pitfalls & How to Handle Them

- [ ] **RTAB-Map lag** → if topic rates feel slow despite fast processing, tune **DDS QoS** (Best Effort for high-rate image/pose streams; Reliable only for mission events like `target_found`). This matches report §2.1.3.
- [ ] **Frame conversions** → the #1 source of "drift sideways." Be explicit about **ENU (ROS) ↔ NED (PX4)** in the VIO bridge. Verify yaw sign with a manual rotation test (Step 1).
- [ ] **Offboard sequence** → stream `OffboardControlMode` + setpoints at **>2 Hz (use 10–20 Hz)** *before* arming and *before* switching to OFFBOARD, or PX4 rejects offboard.
- [ ] **goto vs velocity** → use **velocity setpoints** for reactive Mode 2; use **`goto_setpoint`** for planned Mode 3 waypoints.
- [ ] **VIO init / featureless walls** → give the camera a slight downward tilt and ensure the world has texture; pure blank walls starve VIO. Add posters/texture to the Gazebo world if tracking is poor.
- [ ] **Single-drone now, swarm later** → keep the **agent-repulsion interface** (Eq. 4.16) and the masterless map-ownership boundaries in place even while inert, so scaling to N drones doesn't require refactoring.

---

## 9. Two-Day Execution Summary

**Day 1 (today)**
- Morning: Phase A — Steps 0–2 (sensors → RTAB-Map standalone → bridge to EKF2). **Target: stable GPS-denied hover.**
- Afternoon: Phase A Step 3 (waypoint) + start Phase B Steps 4–5 (voxel map + APF).

**Day 2**
- Finish Phase B Steps 5–6 (autonomous explore + YOLO can search). **Target: Mode 2 fully working.**
- Begin Phase C Steps 7–8 (planner + return). 

**Wednesday**
- Finish Phase C + Step (state machine). **Target: full run — explore, search, return, land.**
- Course deliverable (Navigation & Robotics Control): demonstrate the running simulation.

---

## 10. Definition of Done

- [ ] Drone takes off and hovers **with zero sideways drift** in GPS-denied mode.
- [ ] Stereo-VIO (D435i-equivalent) provides 6-DOF pose **including correct yaw**.
- [ ] A 3D voxel map builds in real time; global entropy decreases.
- [ ] Drone **autonomously explores** unknown space via APF (entropy attraction + obstacle repulsion).
- [ ] **YOLO detects the red Coca-Cola can** and logs its 3D position.
- [ ] On reaching the entropy threshold, drone **plans a path** on the known map and **returns home efficiently without collisions**.
- [ ] Drone **lands** at the home position; clean state-machine transitions throughout.
- [ ] Architecture preserves swarm-ready interfaces (masterless map ownership, agent-repulsion term) for the next phase.

---

### Working notes for Claude Code
- **Diagnose before changing.** Especially for drift: confirm yaw source and frame conventions *before* editing controllers.
- **Respect the phase gates.** Don't build Phase B on a drone that fails the Phase A hover test.
- **C++ for time-critical loops** (bridge, mapping, APF, planner); **Python for YOLO + state machine**.
- **uXRCE-DDS**, not MAVROS — ignore MAVROS-based examples' transport code; reuse only their EKF2/estimator insights.
- Reference repos worth consulting (read, don't copy blindly): `introlab/rtabmap_ros` (ros2 branch) + `rtabmap-drone-example`; PX4 `goto_setpoint` docs for Mode 3.
