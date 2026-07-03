# 3D Scan-and-Search Strategy

Single- and dual-drone indoor GNSS-denied SITL environment. Goal: high-coverage
3D scan/search, real-time red-target detection, wall-safe navigation, and measured
comparison of maximum speed and drone count.
Current experiment speeds: `0.70, 1.30, 1.80, 2.00 m/s`. Speed ramp is disabled so
speed experiments can be compared cleanly.

Last updated: 2026-07-03.

---

## 1. Mathematical Framework (from thesis §4)

All navigation equations operate in **ℝ³ (NED)**. No planar simplification.

### 1.1 Voxel Map

Workspace E ⊂ ℝ³ discretized into voxels V = {v₁…vₘ}, each with center c_ℓ ∈ E.

Log-odds occupancy belief (thesis eq 4.2–4.3):
```
L_k(v_ℓ) = log( b_k / (1 - b_k) )
L_k(v_ℓ) = L_{k-1}(v_ℓ) + inv_sensor(v_ℓ, z_k) - L₀(v_ℓ)     [additive, efficient]
```
Classification (eq 4.1):  b < 0.3 → Free,  b > 0.7 → Occupied,  else → Unscanned.

### 1.2 Entropy Centroid (exploration goal)

Per thesis eq 4.11–4.14:
```
H(b_k(v_ℓ)) = -b_k·log(b_k) - (1-b_k)·log(1-b_k)       [Bernoulli entropy, max at 0.5]

p_ent(k) = Σ H(b_k(v_ℓ))·c_ℓ  /  (Σ H(b_k(v_ℓ)) + ε)   [entropy-weighted centroid]

F_att(k) = ξ · (p_ent(k) - p(k))    ξ = 0.3, fully 3D vector
```

The original thesis formulation used the entropy centroid as the exploration attractor.
The active implementation now uses frontier viewpoint scoring and A* route planning as
the global navigation layer; entropy remains published as an analysis/visualization signal.

### 1.3 Horizontal obstacle avoidance — VFH (replaces APF, 2026-06-17)

> **Note:** The thesis (§4.15–4.17) describes APF voxel repulsion. That formula is retained
> for the thesis write-up, but the **implementation** switched to VFH on 2026-06-17 because
> APF attraction can cancel repulsion when the goal is behind a wall, causing the drone to
> fly into the wall. VFH cannot produce a wall-pointing velocity — it only selects free sectors.

VFH (Borenstein & Koren 1991) — 72 sectors × 5° = 360° polar histogram:
```
sect_min[s] = min LiDAR range in sector s
blocked[s]  = sect_min[s] < block_range  (dilated, hysteresis via clear_range)

goal_sec = sector of atan2(goal_e − pos_e, goal_n − pos_n)
best_sec = first open valley (≥3 consecutive free sectors) searched outward from goal_sec

steer = center angle of best_sec  (exact goal_ang if best_sec == goal_sec and close)
speed = min(v_max × obstacle_scale, goal_distance × attraction_gain)
```

APF voxel repulsion equations (thesis §4.15–4.17) — preserved for thesis reference only:
```
ρ = ||p - c_ℓ||,   ∇ρ = (p - c_ℓ)/||p - c_ℓ||
F_rep(v_ℓ) = { η·(1/ρ - 1/ρ₀)·(1/ρ²)·∇ρ,   if ρ ≤ ρ₀
             { 0                               otherwise
η = 3.0,  ρ₀ = 0.85 m
```

### 1.4 Velocity Command

Horizontal (XY): `v_cmd.xy = holonomic VFH output`. The quadrotor can translate in
N/E directly while yaw tracks the camera/view direction slowly. This is the important
change that stopped the old yaw-first side-to-side behavior from damaging SLAM.

Vertical (Z) — independent P-controller:
```
v_cmd.z = −clamp(kZKp × (scan_alt − AGL), −kZMax, kZMax)
kZKp = 1.5,  kZMax = 0.40 m/s
```

Send v_cmd.x, v_cmd.y, v_cmd.z to PX4 via `publish_velocity_sp()` (VELOCITY offboard mode).
Ceiling/floor ToF sensors act as hard guards; VFH handles lateral walls.

### 1.5 Coverage Metric (thesis eq 2.5)

```
coverage_fraction = |{v_ℓ : b_k(v_ℓ) ≤ b_free OR b_k(v_ℓ) ≥ b_occ}| / |V_free_total|
```

Mission terminates when global/team `coverage_fraction` reaches the configured
threshold (`scan_complete_frac=0.75` in the current sweep), or on timeout/all
layers complete. A found target no longer terminates the mission immediately: the
drone inspects it and then resumes scanning.

---

## 2. Localization Strategy

**SITL (current):** GNSS-denied — airframe 4022, no GPS.
- `slam_to_px4.py` publishes `VehicleVisualOdometry` from SLAM → EKF2 (EV_CTRL=15)
- EKF2_HGT_REF=3 (vision primary height), EKF2_MAG_TYPE=5 (magnetometer disabled)
- Height: downward ToF (range/down) → `d_ned = -range.range` in EV message
- SLAM Gatekeeper in slam_to_px4.py suspends EV on algorithmic freeze
- Active stopgap pose source is 2-D LiDAR SLAM. This is not GPS and not simulator truth.
- The 3-D occupancy map itself is depth-camera based; LiDAR voxel integration is disabled.

**Indoor hardware path (future gap A):**
- Optical flow (already in model) → EKF3 horizontal velocity  
- Downward ToF → EKF3 altitude  
- Full 6-DOF VIO (D435i stereo + RTAB-Map) — see `Mode2_Mode3_Implementation_Plan.md`
- Equation 2.2 (thesis): `V_metric = Δp·z / (f·Δt)` — optical flow to metric velocity

---

## 3. Implementation Plan

### Phase A — Horizontal navigation ✅ DONE (VFH, 2026-06-17; APF superseded)

**File:** [search_mission_controller.cpp](../src/swarm_control/src/search_mission_controller.cpp)

**2026-06-17:** Horizontal APF (F_att + F_rep voxels) replaced entirely by VFH.
Function: `compute_vfh(goal_n, goal_e) → Force2D{n, e}`.
- 72 sectors × 5°, active block range currently 0.60 m
- Hysteresis: stays blocked until clearance > 0.90 m
- Valley search outward from goal sector; ≥3 consecutive free sectors required
- Speed scales with forward clearance: zero at 0.60 m, full at about 1.60 m
- No local minima — cannot generate a wall-pointing velocity
- Removed: `compute_apf_3d()`, `saturate_omni()`, `rep_mag()`, `slew_velocity()`,
  `occupied_voxels_` member, voxel_map PointCloud2 subscriber

**Previous (2026-06-16, superseded):** 3D APF implemented as `compute_apf_3d()`.
- `Force3D` struct; `saturate_omni()` for 3D radial saturation
- Replaced because APF attraction cancelled repulsion near walls.

Active VFH/navigation params (launch file, as of 2026-07-02):
| Param | Value |
|-------|-------|
| ξ / waypoint attraction | `attraction_gain=1.10` |
| emergency forward stop | `forward_stop_m=0.70 m` |
| lidar_repulsion_rho0_m | 2.0 m visibility horizon/sentinel, not the block threshold |
| v_max_mps | experiment variable |
| v_max_ramp_step_mps | 0.0, ramp disabled |
| Speed formula | obstacle-scaled VFH speed capped by `v_max` and waypoint attraction |
| active-route conflict break | `range=1.25 m`, `angle=55 deg`, `duration=0.8 s` |

### Phase B — Altitude-layer sweep ✅ ACTIVE

The entropy centroid alone can remain at one altitude if the room is uniform in Z,
so the active implementation uses an explicit altitude-layer scheduler:
- Divide scan height into layers using `scan_alt_start_m` and `scan_layer_step_m`.
- Track per-layer coverage from the voxel map and shared team summary.
- Current layer climb threshold: `layer_complete_frac=0.83`.
- Current global mission completion threshold: `scan_complete_frac=0.75`.
- In two-drone runs, each drone follows its assigned layer stride, but the layer
  coverage decision uses team/shared coverage for that altitude.
- Ceiling/headroom checks still block vertical climbs; if a climb is blocked, the
  drone continues searching until it reaches a better XY position.

The controller changes altitude directly with an independent vertical P controller;
frontier/A* remains responsible for choosing useful XY viewpoints at each layer.

### Phase C — Voxel mapper upgrades (partial ✅)

**File:** [voxel_mapper.cpp](../src/swarm_perception/src/voxel_mapper.cpp)

Publishers (all in namespace `px4_0`):
| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `voxel_map` | PointCloud2 | best_effort depth=1 | Occupied voxels (x=N,y=E,z=D NED float32×4 with intensity) |
| `voxel_slice_map` | OccupancyGrid | reliable transient-local | 2D horizontal slice at current altitude |
| `entropy_centroid` | PointStamped | reliable depth=10 | Entropy-weighted goal (x=N,y=E,z=NED_down) |
| `frontier_goal` | PointStamped | reliable depth=10 | Frontier viewpoint goal (x=N,y=E) |
| `frontier_goal_pose` | PoseStamped | reliable depth=10 | Frontier goal with approach heading |
| `frontier_path` | Path | transient-local | Strict A* path to selected frontier viewpoint |
| `return_path` | Path | transient-local | Strict A* return-to-home path |
| `map_update_summary` | MapUpdateSummary | reliable depth=10 | coverage_fraction, entropy_mean |
| `drone_path` | Path | best_effort depth=1 | Full NED path history (up to 10 000 poses) |

Status:
1. ✅ **Occupied voxels**: published as PointCloud2 `voxel_map` at 2 Hz.
2. ✅ **Entropy centroid z**: published 3D in NED. Fixed default z-bounds to [-11.0, 0.5] NED.
3. ✅ **2D slice map**: `voxel_slice_map` (OccupancyGrid). Width=East cells, height=North cells.
4. ✅ **Path history**: `drone_path` published — appends pose each update cycle.
5. ✅ **Frontier/A* route publishing**: `frontier_path` and `return_path`.
6. ✅ **Free voxel cloud**: `free_voxel_map` for the dashboard.
7. ✅ **Coverage fraction per Z-layer**: published in summaries and dashboard.
8. ❌ **Unscanned separate topic**: not yet.

### Phase D — Target detection, inspection, and reporting ✅ DONE (2026-06-25)

**Publisher:** `/mission/result` (std_msgs/String, transient-local reliable, QoS depth=10)

Behavior in `search_mission_controller.cpp`:
1. ✅ `"TARGET FOUND at (N, E)"` — on `TargetFound` message, immediately
2. ✅ Confirm nearby repeated detections before committing target position
3. ✅ Enter `TARGET_INSPECT`: hold XY, yaw toward target for 10 s
4. ✅ Suppress revisits within 1.25 m so the same can is not counted repeatedly
5. ✅ Resume `SCAN` after inspection
6. ✅ `"TARGET NOT FOUND (coverage X%)"` — coverage_fraction ≥ scan_complete_frac_
7. ✅ `"TARGET NOT FOUND (all layers scanned, cov=X%)"` — all layers exhausted
8. ✅ `"TARGET NOT FOUND (timeout Xs)"` — search timeout exceeded

### Phase E — Speed experiments ✅ DONE

Speed ramp exists in `search_mission_controller.cpp`, but it is intentionally
disabled for the controlled experiments:
- Called after each scan layer that reaches `scan_done_frac` (layer_done), not timeout/stagnation
- `v_max_` increments by `v_max_ramp_step_mps` per completed layer, up to `v_max_target_mps`
- All distance parameters scale with v_max automatically:
  - `fwd_stop_m = max(base, v_max * 0.75)`
  - `rho0 = max(base, v_max * 0.60)`
  - `emergency_stop = max(base, v_max * 0.25)`

**Final experiment policy (2026-07-02/2026-07-03):**
```
v_max_mps            = launch/script variable
v_max_ramp_step_mps  = 0.0   # ramp disabled
v_max_target_mps     = same as v_max_mps while ramp is disabled
```

The final report uses these controlled results:
- Best tested operating point: `1.30 m/s`.
- `0.70 m/s`: stable but slower.
- `1.80-2.00 m/s`: navigation stress/failure regime, retained in the analysis as an important limitation.
- Two-drone operation improves mission time only when shared coverage, route-conflict handling, and peer separation remain stable.

Experiment matrix: fixed `v_max_mps` values `0.70, 1.30, 1.80, 2.00 m/s`
were evaluated for one drone and two drones. The independent variables were only
maximum speed and number of drones.

---

## 4. 3D Map Visualization

**Mission dashboard** (PyQt5 + matplotlib 3D, separate process from Gazebo):
```
ros2 run swarm_gcs mission_dashboard
```
| What | How |
|------|-----|
| Occupied voxels (red) | `/px4_0/voxel_map` PointCloud2, scatter3D, up to 15 000 pts |
| Free voxels (green, α=0.20) | `/px4_0/free_voxel_map` PointCloud2, 3 floats/pt, up to 5 000 pts |
| Drone path (blue line) | `/px4_0/drone_path` Path, NED → AGL conversion |
| Drone position (cyan) | `/fmu/out/vehicle_local_position_v1` VehicleLocalPosition |
| Heading arrow (cyan) | quiver from drone position, length 2 m |
| Active waypoint (★) | `/px4_0/committed_goal` PointStamped |
| Active route | `/px4_0/active_path` Path, controller-latched route |
| Mapper candidate route | `/px4_0/frontier_path` Path, fallback display only when no active route is fresh |
| Frontier endpoint (yellow X) | `/px4_0/frontier_goal` PointStamped |
| Entropy centroid (orange ◆) | `/px4_0/entropy_centroid` PointStamped |
| Coverage bar | `/px4_0/map_update_summary` MapUpdateSummary |
| Mission result | `/mission/result` String (transient-local) |

Refresh: 1 Hz. User can rotate and zoom the 3D plot with the mouse.

**RViz2** (not configured):
- `/px4_0/voxel_map` PointCloud2 can be added manually in RViz2 (frame: `odom`)
- No `.rviz` config file exists yet (`ros2_ws/config/scan_rviz.rviz` — open gap)
- Free/unscanned voxels are not published as separate topics (open gap)

---

## 5. Sensor Roles (final assignment)

| Sensor | Role |
|--------|------|
| 2D LiDAR 360° | Horizontal wall detection → forward_range_, left/right sectors |
| Depth camera | 3D voxel map updates (obstacle + free space) |
| RGB camera | Red target HSV detection |
| Downward ToF (range/down) | Floor repulsion + AGL reference for optical flow |
| Upward ToF (range/up) | Ceiling repulsion guard |
| Optical flow | Future: EKF3 localization (GNSS-denied hardware) |

---

## 6. Academic References

| Paper | Key idea we use |
|-------|----------------|
| Borenstein & Koren (1991) | VFH: polar histogram + valley selection — **active horizontal nav** |
| Forsman & Tidén (2023) | Log-odds belief updates + pose-uncertainty smearing (thesis §2.4.2) |
| Hu et al. (2024) | O-APF: tangential sub-targets to escape local minima (thesis §2.4.3) |
| Asaamoning et al. (2021) | NCS framing, DDS QoS for target-found events (thesis §2.4.1) |
| Khatib (1986) | Classical APF: U_att + U_rep, eqs 2.3–2.4 — thesis math reference |
| OctoMap (Hornung 2013) | OcTree voxel compression for memory-efficient 3D maps |

---

## 7. Implementation Order

| Phase | Description | Status |
|-------|-------------|--------|
| A | Horizontal navigation — VFH (replaced APF 2026-06-17) | ✅ Done 2026-06-17 |
| — | Unicycle kinematic model + SLAM Gatekeeper | ✅ Done 2026-06-17 |
| — | GNSS-denied flight (airframe 4022, EKF2+EV) | ✅ Done 2026-06-18 |
| C | Voxel mapper publishers (map, slice, path, frontier, free cloud) | ✅ Done (RViz2 viz still missing) |
| D | Mission result `/mission/result` publisher | ✅ Done 2026-06-16 |
| — | Doorway traversal (lidar_rho0=0.35, decoupled speed formula) | ⚠️ Implemented 2026-06-20, not flight-tested |
| — | Keyhole fix (info gain z-bounded), cross-room frontier scoring | ✅ Done 2026-06-20 |
| — | Holonomic VFH + camera-aware yaw, initial 360° lookaround | ✅ Done 2026-06-24 |
| — | Depth-only voxel mapping, LiDAR kept for VFH and temporary SLAM | ✅ Done 2026-06-24 |
| — | Target inspect/resume behavior | ✅ Done 2026-06-25 |
| E | Speed fixed at 0.85 m/s; ramp disabled | ✅ Done 2026-06-25 |
| F | Single-drone max-speed experiment matrix | ⏭️ Next |
| G | Two-drone shared map + dynamic inter-agent avoidance | ⏭️ Next |
| H | Two-drone speed sweep | ⏭️ After G |
| B | Height layer scheduler + per-layer coverage fraction | ❌ Open |
| — | Mission dashboard (3D scatter, PyQt5, free voxel layer) | ✅ Done 2026-06-17/20 |

---

## 8. Open Gaps (not blocking simulation)

| Gap | Required for | Status |
|-----|-------------|--------|
| Optical flow → EKF3 localization | Real hardware only | Out of scope for SITL |
| Camera/VIO or RTAB-Map 6-DOF pose | Removing temporary LiDAR SLAM | Future hardware/SITL branch |
| Multi-drone inter-agent repulsion | Two-drone experiments | Next phase |
| Shared frontier ownership | Two-drone experiments | Next phase |
| Motion-blur detection quality model | Performance analysis | Thesis section 5 |
