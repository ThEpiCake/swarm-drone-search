# Current Stack Status

Last updated: 2026-07-03

Single source of truth for the active single- and dual-drone search simulation and the
final report validation state.
Priority over README.md, STRATEGY_3D_SCAN.md, and all session logs.

---

## Project Delivery State

Deadline: **2026-07-03**.

The controlled experiment series has been executed and the final report draft builds.
The remaining work is review and presentation preparation, not new runtime
development.

Completed:
- 1-drone and 2-drone controlled runs at `v_max = 0.70, 1.30, 1.80, 2.00 m/s`.
- Final result plots generated into `report/drone_report_latex/images/results/`.
- Results, discussion, conclusions, limitations, and simulation wording added to the report.
- `report/drone_report_latex/main.pdf` builds cleanly.

Remaining before submission:
- Proofread `report/drone_report_latex/main.pdf` end-to-end.
- Verify figure/table references and numeric claims against experiment summaries.
- Keep the report wording clear that the validated deliverable is SITL simulation, not physical hardware.
- Prepare the academic poster from the report figures and conclusions.

## Airframe & Localization

**Airframe:** 4022 (`x500_swarm`) — **GNSS-denied. No GPS. GPS must never be added.**

EKF2 parameters (set in PX4 airframe RC):

| Parameter | Value | Why |
|-----------|-------|-----|
| `EKF2_GPS_CTRL` | 0 | GPS fusion disabled |
| `SYS_HAS_GPS` | 0 | No GPS hardware declared |
| `COM_ARM_WO_GPS` | 1 | Arm without GPS |
| `EKF2_EV_CTRL` | 15 | Fuse EV: XY + Z + vel + yaw (bits 0–3) |
| `EKF2_HGT_REF` | 3 | Vision as primary height reference |
| `EKF2_MAG_TYPE` | 5 | Magnetometer disabled (VIO supplies yaw) |

Height input: `range/down` ToF → `slam_to_px4.py` → `d_ned = -range.range` (direct,
avoids Z feedback loop from round-tripping through the depth camera).

---

## Active Runtime Stack

Single launch command:
```bash
cd ros2_ws && source install/setup.bash
ros2 launch swarm_bringup d1_single_agent_search.launch.py
```

Active nodes:
- `gz sim` — Gazebo Harmonic, world: `single_agent_search_room_easy`
- `px4_sitl` — airframe 4022, GNSS-denied
- `MicroXRCEAgent` — UDP port 8888
- `slam_toolbox` — active 2-D LiDAR SLAM pose source for the current SITL stopgap
- `slam_to_px4.py` — publishes `VehicleVisualOdometry` + rangefinder Z to PX4 (includes SLAM freeze/jump/yaw Gatekeeper)
- `slam_pose_fusion.py` — publishes guarded `slam/odom_ned` for mapper/controller pose alignment
- `voxel_mapper` — depth-camera 3D log-odds map, frontier goals, free voxel cloud; LiDAR voxel integration is disabled in the single-agent launch
- `search_mission_controller` — FSM (PRIMING→TAKEOFF→SCAN→RETURN→LANDING) + holonomic VFH nav
- `target_detector` — HSV red-can detection → `/mission/result`
- `mission_dashboard` (optional, Terminal 2) — PyQt5 3D interactive map

**Architecture note:** the current SITL path still uses 2-D LiDAR SLAM as a temporary
pose source. The thesis/FUEL direction remains camera/VIO or RTAB-Map 6-DOF pose
feeding EKF2/3 in GNSS-denied mode, not GPS and not simulator truth.

### Dual-Drone Frame Rule (validated 2026-06-28)

In `d2_dual_agent_search.launch.py`, the shared map frame is the Gazebo/PX4 world
frame. PX4 fallback poses from `VehicleOdometry` are already in that frame, so
`map_merger` and `mission_dashboard` must keep D1 offsets at `0.0`.
`slam_pose_fusion.py` applies each drone's spawn offset only when a real SLAM TF
is used, because `slam_toolbox` starts each drone map at its own local origin.

Diagnostic: at dual takeoff D0 should appear near `E=-0.75 m` and D1 near
`E=+0.75 m`. If D1 appears near `E=+2.25 m`, the `+1.5 m` inter-drone offset has
been applied twice.

### Dual-Drone Shared Exploration Map (updated 2026-06-29)

Dual-drone exploration now follows the "one puzzle, two observers" model:

- Each drone still owns a local log-odds voxel map from its own RGB-D depth stream.
  That local map is what it publishes as `/px4_i/voxel_map` and
  `/px4_i/free_voxel_map`.
- `map_merger.py` publishes both `/merged_voxel_map` and
  `/merged_free_voxel_map` in the shared `map_ned` frame.
- In D2, each `voxel_mapper` subscribes to those merged clouds as a planning
  overlay. The overlay is applied to the current altitude slice before frontier
  selection, obstacle masking, A*, and layer coverage are computed.
- The overlay is not written back into the local log-odds map. This avoids a
  feedback loop where merged data gets repeatedly re-merged as if it were new
  sensor evidence.
- Local safety remains local: VFH, nearest-obstacle slowdown, emergency hover, and
  peer avoidance still run from each drone's own live sensors/pose.

Because `slam_pose_fusion` normalizes both drones into the shared Gazebo/PX4 world
frame, `map_merger` offsets stay `N=0/E=0`. Do not add the 1.5 m spawn separation
again in the merger or dashboard.

### Dual-Drone Coverage Rule (updated 2026-06-29)

For two-drone experiments, coverage is a team/global metric, not a separate final
metric per drone.

- `map_merger.py` publishes `/shared_map_update_summary` with one global
  `coverage_fraction` for the combined team map.
- Shared layer coverage is derived from the union of the drones' `free_voxel_map`
  cells only. Occupied shelves, walls, and cubes are not counted as "covered"
  floor-area cells, so wall/obstacle mass does not inflate layer completion.
- Shared layer denominators use the best current reachable-layer estimate
  (`max(layer_total_cells)` across drones at that altitude), not a sum, because both
  drones are describing the same physical layer. Shared observed cells use the union
  of free XY cells from both drones, so one drone can raise the team coverage of a
  layer assigned to the other drone.
- Both controllers subscribe to `/shared_map_update_summary`; in D2, layer transitions
  use only the shared/team layer coverage for the current altitude. There is no
  fallback to individual drone coverage for climb decisions.
- The dashboard shows the same `Coverage: team X%` in both drone status panels.
  Layer coverage remains split as `local ... | team ...` so contribution and
  cooperation can be diagnosed during a run.
- Layer climb logs now print `shared_lc` as the decision value and
  `local_debug_lc` only as a diagnostic. The current experiment thresholds are
  `layer_complete_frac = 0.83` for layer transitions and
  `scan_complete_frac = 0.75` global/team coverage for mission completion.

---

## Navigation Architecture

### A* Route Contract + VFH (updated 2026-07-02)

`voxel_mapper` is the only global route selector. At 2 Hz it:

- builds the current 2-D slice and inflated obstacle mask from the 3-D voxel map,
- scores frontier viewpoints by information gain, path length, yaw effort, local
  awareness, and Voronoi/clearance distance,
- rejects viewpoints already inside the active goal radius and tiny zero-gain
  frontiers, so the controller is not fed a route whose endpoint is the current pose,
- keeps each selected viewpoint in known-free, volume-clear space,
- runs A* through obstacle-safe cells inside the physical world mask; unknown route
  cells are allowed only when they are not inside the inflated obstacle mask, so VFH
  can continue exploration through thin unmapped bands instead of deadlocking,
- ranks frontier clusters connected to the current reachable component before
  disconnected clusters, then applies information-gain/distance scoring within
  that reachable set,
- publishes `frontier_path` only when that A* path is valid, and publishes an empty
  `frontier_path` when no route exists so the dashboard does not display stale
  mapper plans.

`search_mission_controller` does not choose direct entropy/frontier fallback targets.
In SCAN it latches one `frontier_path` at a time and follows its waypoints in order;
new mapper candidates are ignored while the active route is still being executed,
except when local safety reports a truly blocked or stalled route. Merely being
near a wall or pillar is handled locally by VFH plus a nearest-obstacle APF
repulsion term; it is not a reason to accept a new mapper path. A waypoint that
was just blocked is rejected for a short cooldown, so the mapper cannot
immediately feed the same unsafe first step back to the controller.
If no active route exists it holds position and waits for the mapper. If route
progress stalls or VFH reports all sectors blocked, the controller clears the
route and waits for a new map-planned path instead of flying direct. In addition,
`route_conflict_replan_enabled=true` adds a fast break condition: when a newly
observed obstacle lies on the active route and VFH strongly disagrees with the
route bearing for `route_conflict_replan_s=0.8`, the active route is cleared and
the blocked waypoint is rejected for a short cooldown. This prevents the drone
from continuing into a path that was valid when planned but became unsafe after
new mapping observations.

Mode 3 follows the same latched-route rule. `voxel_mapper` publishes `return_path`
from current pose to home `(0,0)` via the same obstacle-safe A*. RETURN follows that
path and lands only when the home radius is reached. If no safe return route exists,
RETURN holds for the mapper.

### Target Handling

Confirmed target detection no longer ends exploration or starts a blind orbit. The
controller enters `TARGET_INSPECT`:

- records/publishes the target position,
- holds XY at the current safe scan altitude,
- yaws toward the target for `target_hold_s = 10.0` s,
- suppresses repeated detections within `target_revisit_radius_m = 1.25` m,
- returns to `SCAN` and continues the active exploration workflow.

This fixes the shelf/corner failure mode where the old ORBIT behavior could command a
circle through racks or walls after finding a can.

**Voronoi/clearance scoring (Lecture 4 retraction principle):**
- Frontier viewpoint candidates scored with `frontier_clearance_weight=10.0` ×
  obstacle distance (capped at 10 cells = 2.0 m) — rewards corridor centres.
- Gradient ascent on `compute_obstacle_dist_map()` (BFS distance transform from
  the inflated planning obstacle mask and world boundaries) produces a
  Voronoi-optimal candidate per frontier cluster.
- Frontier candidates must also satisfy `frontier_min_obstacle_clearance_m=0.75`
  and `frontier_world_boundary_clearance_m=1.20`, so a goal cannot sit on the
  shelf/wall edge just because it has high information gain.
- A* clearance cost now uses the same inflated obstacle mask as route passability,
  so target scoring, path planning, shortcutting, and B-spline smoothing agree on
  what "too close to a wall" means. Before B-spline smoothing, intermediate path
  points are pulled a few cells toward the local clearance ridge if the adjusted
  segments remain passable, which rounds paths away from walls instead of relying
  on local safety to rescue a wall-hugging route.
- The mapper world mask stays on the physical room bounds (`N≈±8.5 m`) so wall
  voxels remain visible. Goal selection is kept away from walls by
  `frontier_world_boundary_clearance_m` and obstacle-distance clearance, not by
  cropping the map itself.
- This implements the lecture roadmap/retraction idea in the grid domain: plan in
  Qfree, bias toward the skeleton/clearance ridge, then use VFH only as local safety.

**Arrival look at frontier goals (updated 2026-07-02):** when the controller reaches
the final waypoint of a frontier route, it holds XY for `frontier_arrival_look_s=6.0`
s and performs one full 360-degree yaw sweep. Intermediate route waypoints are not
paused. The intent is to let the RGB-D camera inspect nearby unknown space after
reaching a useful viewpoint, without reintroducing route thrashing.

### VFH (Vector Field Histogram)
- 72 sectors × 5° = 360° polar histogram, 2D horizontal plane
- `block_range = min(lidar_rho0_m=2.0, max(rho0_=0.55, fwd_stop=0.60)) = 0.60 m`
- `clear_range = min(lidar_rho0_m=2.0, block_range + max(0.30, resume_margin=0.25)) = 0.90 m`
- Sector blocked when per-sector min LiDAR range < block_range (0.60 m)
- Blocking dilated ±1 sector (kDilate=1); stays blocked until clearance > clear_range (0.90 m)
- Valley search outward from goal sector; ≥3 consecutive free sectors required (kMinV=3)
- LiDAR floor/ceiling ray rejection active (attitude + altitude filter)

### Holonomic VFH Kinematic Model
- VFH outputs a direct XY velocity vector in NED; the quadrotor no longer has to yaw
  toward every transient histogram valley before moving.
- Yaw is perception-aware and rate-limited (`yaw_rate_limit_dps`): in production it
  tracks the active route/frontier direction slowly so the RGB-D camera keeps seeing
  the space being mapped, but it does not chase every VFH avoidance valley.
- If the camera yaw error is large, XY speed is scaled down until the camera catches
  up. When no route exists, the drone performs a slow in-place yaw scan.
- This matches the thesis first-order velocity model better than the previous
  yaw-first unicycle approximation and reduces scan-matching drift in corridors.
- A local APF-style nearest-obstacle repulsion term is added after VFH. It pushes
  the drone away from the nearest wall/pillar while preserving the currently
  latched route, preventing rapid replanning loops near obstacles.
- After the APF/centering terms, the nearest-obstacle approach guard runs again
  and removes any velocity component that would still move toward the closest
  obstacle. This prevents the additive local correction from slowly sliding the
  drone into an opposite wall or shelf.

### Speed Formula (decoupled from VFH blocking radius, 2026-06-25)
Speed uses `forward_range_` (actual forward LiDAR distance, not `lidar_rho0_m`):
```cpp
const float fwd = std::min(forward_range_, block_range + 1.0f);
const float spd_scale = std::clamp((fwd - block_range) / 1.0f, 0.0f, 1.0f);
```
Zero speed at `block_range` (0.60 m), full speed at `block_range + 1.0 m` (1.60 m).
`lidar_rho0_m = 2.0` is the VFH sector-initialisation sentinel (no obstacle); actual
blocking and speed-ramp thresholds are derived from `rho0_` and `fwd_stop_m` as above.

**Tracking-speed tuning (2026-06-28):** the effective XY speed is also capped by
`goal_distance * attraction_gain` while following the latched A*/smoothed path. To
make `v_max_mps` visible in the speed-sweep experiments, both single- and dual-drone
launches now use `attraction_gain = 1.10`, `waypoint_lookahead_radius_m = 1.80`, and
`max_accel_mps2 = 1.0`. Local VFH/narrow-passage/nearest-obstacle caps still reduce
speed near shelves, walls, pillars, and boxes.

### SLAM Gatekeeper
`slam_to_px4.py` tracks SLAM pose delta per frame. If delta < 0.1 mm for ≥5 consecutive
frames (algorithmic freeze on featureless wall), it stops publishing EV to PX4. PX4 falls
back to IMU — which correctly senses acceleration and allows the controller to brake,
instead of accumulating integral error and driving into the wall.

It also rejects SLAM pose/yaw updates on large XY jumps, yaw residual jumps, or SLAM yaw
rate spikes above the configured physical limit. This is GNSS-denied gating only; no GPS
or simulator truth is used.

---

## Current Launch Parameters

File: `ros2_ws/src/swarm_bringup/launch/d1_single_agent_search.launch.py`

| Parameter | Value | Notes |
|-----------|-------|-------|
| `v_max_mps` | launch/script variable | Current experiment sweep uses `0.70, 1.30, 1.80, 2.00 m/s` |
| `v_max_ramp_step_mps` | 0.0 | Speed ramp disabled by design for controlled experiments |
| `v_max_target_mps` | same as `v_max_mps` | Same as base speed while ramp is disabled |
| `attraction_gain` | 1.10 | Route waypoint attraction; keeps tracking speed high enough for speed-sweep comparison |
| `kd_vel_damp` | 0.20 | Velocity damping for holonomic XY commands |
| `safety_margin_m` | 0.30 | rho0 = 0.25 + 0.30 = 0.55 m (VFH block_range) |
| `forward_stop_m` | 0.70 | Braking-safe VFH block range term; exceeds rho0=0.55 |
| `emergency_stop_range_m` | 0.42 | Hard hover threshold |
| `lidar_repulsion_rho0_m` | 2.0 | VFH sector sentinel (not the block threshold; see block_range above) |
| `holonomic_vfh` | true | VFH commands N/E velocity directly instead of yaw-first unicycle motion |
| `yaw_rate_limit_dps` | 30.0 | Limits commanded yaw to protect SLAM |
| `camera_yaw_track_goal` | true | Slowly points RGB-D camera toward the active route/frontier while keeping holonomic XY control |
| `hold_scan_yaw_rate_dps` | 10.0 | Slow scan yaw while waiting for mapper/frontier route |
| `blocked_waypoint_reject_radius_m` | 0.95 | Rejects a newly planned route if its first waypoint is near a waypoint that was just blocked |
| `blocked_waypoint_reject_s` | 10.0 | Cooldown window for blocked-waypoint rejection |
| `integrate_lidar_into_map` | false | Depth camera owns voxel occupancy; LiDAR remains for VFH and temporary SLAM pose |
| `takeoff_lookaround_enabled` | true | Slow 360° camera scan after takeoff before SCAN/navigation starts |
| `takeoff_yaw_rate_dps` | 10.0 | Slow initial scan yaw rate to build local depth map without SLAM yaw shocks |
| `vfh_fov_half_deg` | 0.0 | Full 360-degree LiDAR VFH |
| `obstacle_inflate_cells` | 3 | Inflated planner obstacle mask for A*/frontier clearance |
| `obstacle_band_high_alt_threshold_m` | 3.5 | From 4 m+ use narrower vertical obstacle band and keep known Room A/B connector open |
| `obstacle_band_high_half_m` | 0.45 | High-layer vertical obstacle band half-width |
| `local_start_clearance_cells` | 5 | Keeps active pose connected if a self/near hit is inflated around the drone |
| `frontier_min_goal_distance_m` | 0.60 | Rejects too-near SCAN viewpoints while still allowing narrow-room progress |
| `frontier_max_clusters_scored` | 24 | Lets farther clusters survive the coarse pre-filter before detailed scoring |
| `frontier_candidate_clearance_m` | 0.60 | Viewpoint volume clearance guard |
| `frontier_min_obstacle_clearance_m` | 0.65 | Minimum obstacle-distance guard for candidate viewpoints |
| `frontier_world_boundary_clearance_m` | 1.00 | Keeps goals away from physical world boundaries |
| `frontier_cluster_weight` | 2.0 | |
| `frontier_info_gain_weight` | 5.2 | Pulls toward frontiers that expose unknown voxels |
| `frontier_distance_weight` | 0.28 | Path-length penalty through the A* grid; low enough that far information-rich regions can win |
| `frontier_progress_weight` | 2.0 | Bounded 0→5 m bonus for useful outward progress |
| `frontier_yaw_weight` | 0.4 | |
| `frontier_clearance_weight` | 10.0 | Voronoi clearance bonus; strongly prefers open corridor/room-center viewpoints |
| `frontier_awareness_weight` | 0.3 | Low known-space bias so Room B/unknown regions can still win |
| `layer_dwell_s` | 300.0 | Fallback dwell value; normal climb uses layer coverage |
| `layer_stagnation_s` | 120.0 | Progress monitor/fallback timing |
| `layer_complete_frac` | 0.83 | Current layer climb threshold |
| `layer_complete_stable_s` | 8.0 | Layer coverage must remain complete/stable before advancing |
| `layer_complete_max_reachable_frontier_cells` | 120 | Legacy telemetry/config value; it no longer gates layer transition |
| `goal_progress_timeout_s` | 8.0 | Clear route after this long without progress |
| `scan_complete_frac` | 0.75 | Current global/team mission-completion threshold |
| `target_hold_s` | 10.0 | Inspect confirmed target before resuming scan |
| `target_revisit_radius_m` | 1.25 | Suppress repeated reports of the same target |
| `target_confirm_radius_m` | 0.75 | Nearby detections must agree before inspection |
| `home_north_m`, `home_east_m` | 0.0, 0.0 | Return-path A* target |

---

## Voxel Mapper Key Facts

- **Free voxel cloud:** PointCloud2 with **3 floats per point** (x, y, z) — not 4
- **Free cloud throttle:** publishes at ~1 Hz via `free_cloud_tick_ % publish_rate_hz_`
  (prevents UI choke at high timer rates)
- **Layer coverage denominator:** uses reachable free + reachable unknown cells in the
  current slice; confirmed occupied walls/shelves/cubes are removed from the denominator
  so blocked space does not make a layer look incomplete.
- This is the operational definition of "actual layer area": the configured physical
  scan mask minus closed occupied obstacles/walls and minus regions no longer reachable
  by flood-fill from the drone through passable cells.
- **`estimate_information_gain`:** z-bounded to `±slice_half_thickness_m_` at candidate
  altitude — fixes keyhole effect (upward camera rays no longer inflate scores with
  ceiling voxels, allowing room-2 frontiers to outcompete nearby ceiling-heavy frontiers)
- **Frontier Z-penalty:** `dist_z * 15.0 * frontier_distance_weight` — discourages
  cross-layer jumping during same-layer sweep

---

## Mission Dashboard

File: `ros2_ws/src/swarm_gcs/swarm_gcs/mission_dashboard.py`

Layers (updated 2026-06-18/20):
- Red scatter: occupied voxels (`/px4_0/voxel_map`)
- Green scatter: free voxels (`/px4_0/free_voxel_map`, subsampled to ≤5000 pts, alpha 0.20)
- Blue line: drone path
- Cyan: drone position + heading arrow
- Colored ★: active committed waypoint from `/px4_0/committed_goal`
- White square: first waypoint in the controller-published `/px4_0/active_path`
- Yellow X: frontier endpoint from `/px4_0/frontier_goal`
- Orange ◆: entropy centroid

`_parse_cloud` reads `msg.point_step // 4` floats per point — handles both 3-float and
4-float PointCloud2 without crashing.

---

## Change Log

| Date | Change | File |
|------|--------|------|
| 2026-06-28 | High-speed runs stabilized with local narrow-passage speed cap, slightly earlier waypoint lookahead, and dashboard fallback from empty active path to mapper frontier path so D1 routes remain visible | search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py, mission_dashboard.py |
| 2026-06-28 | Dual-drone shared-map frame alignment fixed: PX4 fallback stays world-frame, real SLAM gets spawn offset only, and D1 map/dashboard offsets stay zero | d2_dual_agent_search.launch.py, slam_pose_fusion.py |
| 2026-06-17 | APF → VFH horizontal navigation | search_mission_controller.cpp |
| 2026-06-17 | SLAM Gatekeeper (EV freeze detection) | slam_to_px4.py |
| 2026-06-17 | Unicycle kinematic model (yaw-first thrust) | search_mission_controller.cpp |
| 2026-06-17 | Rangefinder Z: range/down → d_ned (no feedback loop) | slam_to_px4.py |
| 2026-06-17 | LiDAR floor/ceiling ray rejection | search_mission_controller.cpp |
| 2026-06-18 | Airframe 4022, GNSS-denied (GPS removed) | single_drone_sim.launch.py |
| 2026-06-18 | Gazebo world default: single_agent_search_room | single_drone_sim.launch.py |
| 2026-06-18 | Dashboard crash fix (_parse_cloud dynamic point step) | mission_dashboard.py |
| 2026-06-18 | Free voxel map layer in dashboard (green scatter) | mission_dashboard.py |
| 2026-06-18 | Free voxel cloud publisher in voxel_mapper | voxel_mapper.cpp |
| 2026-06-20 | Speed formula decoupled from lidar_rho0_m (uses forward_range_) | search_mission_controller.cpp |
| 2026-06-20 | lidar_repulsion_rho0_m: 1.5 → 0.35 (doorway traversal) | d1_single_agent_search.launch.py |
| 2026-06-20 | goal_progress_timeout_s: 8 → 25, layer_stagnation_s: 20 → 45 | d1_single_agent_search.launch.py |
| 2026-06-20 | frontier_distance_weight: 0.8 → 0.24 (cross-room bias) | d1_single_agent_search.launch.py |
| 2026-06-20 | Keyhole fix: estimate_information_gain z-bounded | voxel_mapper.cpp |
| 2026-06-20 | Free cloud throttle ~1 Hz | voxel_mapper.cpp |
| 2026-06-20 | emergency_stop_range_m raised 0.25 → 0.35 m; VFH parameter doc corrected | d1_single_agent_search.launch.py |
| 2026-06-21 | Voronoi clearance bonus: frontier candidates scored by obstacle-distance map | voxel_mapper.cpp, d1_single_agent_search.launch.py |
| 2026-06-21 | Voronoi gradient-ascent candidate: ascends to corridor centre per cluster | voxel_mapper.cpp |
| 2026-06-21 | BFS path lookahead: voxel_mapper publishes 1.5 m waypoint on BFS path (not direct viewpoint) | voxel_mapper.cpp |
| 2026-06-21 | BFS failure gate: skip publish if no BFS path to selected viewpoint (unsafe direct goal removed) | voxel_mapper.cpp |
| 2026-06-21 | Center fallback removed from pick_scan_goal(); hold position when no safe goal | search_mission_controller.cpp |
| 2026-06-23 | Controller route contract simplified: SCAN follows only `frontier_path`; entropy/direct fallbacks removed from controller | search_mission_controller.cpp |
| 2026-06-23 | Active SCAN/RETURN routes are latched until reached, stalled, or VFH-blocked; mapper candidate churn no longer interrupts progress | search_mission_controller.cpp |
| 2026-06-23 | Mapper publishes `frontier_goal` only after strict A* path succeeds; coarse fallback candidate removed | voxel_mapper.cpp |
| 2026-06-23 | Mode 3 RETURN now follows mapper-published `return_path` via strict A* to home | voxel_mapper.cpp, search_mission_controller.cpp |
| 2026-06-23 | Default Room B world rebuilt as warehouse racks: west half N-S shelves, east half E-W shelves, can inside shelf | single_agent_search_room_easy.sdf, warehouse_shelf |
| 2026-06-23 | Warehouse shelf model now has one opaque full-height divider side, blocking line-of-sight through each rack | warehouse_shelf/model.sdf |
| 2026-06-23 | 90 s warehouse smoke run passed TAKEOFF→SCAN; latched SCAN route advanced through multiple waypoints while mapper candidate churn was ignored | d1_single_agent_search.launch.py |
| 2026-06-23 | Planner safety margins retuned after log showing `reachable_all=1`: VFH block range 0.55 m, A* inflation 1 cell, local start bubble 2 cells, candidate clearance 0.40 m | d1_single_agent_search.launch.py, voxel_mapper.cpp |
| 2026-06-23 | Log retune after slow/low-exploration run: waypoint attraction 0.55, velocity damping 0.18, frontier weights biased back toward information gain, near/current zero-gain viewpoints rejected, SCAN drops already-reached waypoints before tracking | d1_single_agent_search.launch.py, voxel_mapper.cpp, search_mission_controller.cpp |
| 2026-06-23 | Room B warehouse spacing opened up: north/south wall shelf rows removed, west half reduced to two wider-spaced N-S rows, east half reduced to three E-W rows pushed near the east wall, red can moved with the south E-W shelf | single_agent_search_room_easy.sdf |
| 2026-06-23 | Room A stair corner opened after run stuck near `(3,-3)`: compacted and moved SE stairs, split south mezzanine slab to leave a stairwell opening, removed the south stair-side support columns | single_agent_search_room_easy.sdf |
| 2026-06-23 | Frontier target selection retuned for wider exploration: coarse pre-filter now uses `size / sqrt(distance)`, more clusters are scored, far-progress gets a bounded bonus, and dashboard separates active waypoint ★ from frontier endpoint X | voxel_mapper.cpp, d1_single_agent_search.launch.py, mission_dashboard.py |
| 2026-06-23 | Room A SE stairs moved fully into the corner, and two west-wall-connected ground dividers were added to make the ground-floor partitions behave like walls rather than isolated obstacles | single_agent_search_room_easy.sdf |
| 2026-06-23 | Active route contract fixed: controller now publishes `/active_path`, dashboard draws that route before mapper candidates, new `frontier_path` updates can refresh the active queue when endpoint is stable, route is stalled, or the new front is better | search_mission_controller.cpp, mission_dashboard.py |
| 2026-06-23 | Live run showed stale `frontier_path` can remain latched while controller has no active route; dashboard now keeps `/active_path` authoritative even when it is empty | mission_dashboard.py |
| 2026-06-23 | Live mapping audit: voxel/depth mapping continued, but `slam_toolbox` stayed `unconfigured` so `/map` and `/slam/odom_ned` never published; launch now configures and activates `slam_toolbox` automatically | d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-23 | Live route-gap fix: frontier and return A* now route through obstacle-safe world-mask cells while keeping endpoints known-free; empty `frontier_path` clears stale dashboard goals when the mapper has no continuation | voxel_mapper.cpp, mission_dashboard.py |
| 2026-06-23 | Second live route-gap fix: frontier cluster pruning now prefers clusters connected to the current reachable component before scoring global information gain; SLAM lifecycle launch uses retry-loop autostart instead of one-shot timers | voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-24 | SCAN/RETURN VFH changed from yaw-first unicycle motion to holonomic XY velocity with yaw-rate limiting to reduce SLAM drift in corridors | search_mission_controller.cpp, d1_single_agent_search.launch.py |
| 2026-06-24 | SLAM gatekeeper expanded with XY jump, yaw residual/jump, and SLAM yaw-rate rejection; remains fully GNSS-denied | slam_to_px4.py, slam_pose_fusion.py |
| 2026-06-24 | Single-agent voxel map made depth-only by default; LiDAR remains active for VFH safety and temporary LiDAR SLAM pose source | voxel_mapper.cpp, d1_single_agent_search.launch.py |
| 2026-06-24 | Holonomic VFH made camera-aware: yaw tracks route/frontier slowly, speed scales down on large camera yaw error, and hold state performs a slow scan | search_mission_controller.cpp, d1_single_agent_search.launch.py |
| 2026-06-24 | Initial takeoff lookaround restored: slow 360° depth-camera scan before SCAN/path following starts | d1_single_agent_search.launch.py |
| 2026-06-25 | Single-drone autonomous scan/search validated: drone navigates, maps, and detects Coca-Cola cans in the warehouse world | runtime validation |
| 2026-06-25 | Fixed speed raised to 0.85 m/s; speed ramp remains disabled to keep speed experiments controlled | d1_single_agent_search.launch.py |
| 2026-06-25 | Target handling changed from blind ORBIT to `TARGET_INSPECT`: hold/look 10 s, suppress same-target revisits, resume SCAN | search_mission_controller.cpp, d1_single_agent_search.launch.py |
| 2026-06-25 | Added structured experiment run-folder convention; the active convention is now documented in this file under Experiment & Reporting Plan | experiment scripts/docs |
| 2026-06-26 | First 0.70 m/s speed-sweep run archived with dashboard screenshot; dashboard now handles Ctrl+C/SIGTERM cleanly | experiments/experiment_2026-06-26_Ndrones_1_vmax_0.70mps_speed_sweep, mission_dashboard.py |
| 2026-06-28 | Dual-map frame fix documented: with `slam_pose_fusion` active, both voxel maps are already in the shared Gazebo/world frame, so `map_merger` offsets stay at N=0/E=0 | d2_dual_agent_search.launch.py, map_merger.py |
| 2026-06-28 | High-speed local safety improved: VFH now caps speed from both selected passage clearance and nearest 360-degree obstacle, and removes only the velocity component that drives into a nearby obstacle | search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-28 | A* route shortcutting made adaptive: long segments remain in open space, while narrow/near-obstacle regions keep shorter route segments for safer tracking | voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-28 | Shared team coverage now derives layer coverage from the union of free voxels from both drones, so one drone can contribute observations to another drone's layer coverage without counting occupied shelves/walls as covered floor area | map_merger.py, d2_dual_agent_search.launch.py |
| 2026-06-28 | Wall-hugging mitigation added: near obstacles the controller disables early lookahead, adds a small centering velocity away from the closest obstacle, and A* clearance cost is stronger without increasing obstacle inflation | search_mission_controller.cpp, voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-28 | Route invalidation tightened after failed high-speed dual run: near-obstacle lookahead is disabled, blocked first waypoints are rejected for 10 s, safety-stressed replans can replace the active route, and A* shortcuts are shorter. At this point layer advancement still required stable high coverage plus low reachable frontier count; later changed to the 90% dynamic coverage gate below | search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-28 | Safe path smoothing added: voxel_mapper keeps A* waypoints as the safety corridor, applies a bounded quadratic B-spline/Chaikin smoothing pass, and publishes the smoothed route only if every sampled point and segment remains passable with clearance; otherwise it falls back to the original A* shortcut path | voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-28 | Layer transition normalized for experiments: every climb path now requires at least 90% dynamic reachable-layer coverage; frontier/no-frontier/stagnation data remains exploration telemetry and target-selection input rather than a second independent climb gate | search_mission_controller.cpp, voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-29 | Layer transition simplified further: coverage is now the only reason to climb. No-frontier, stagnation, and layer timeout are diagnostic signals only and cannot advance altitude layers | search_mission_controller.cpp |
| 2026-06-29 | Dual-drone planning now uses the merged occupied/free voxel clouds as a shared exploration overlay inside each voxel mapper, so both drones choose frontiers from the same team map while keeping local log-odds sensing and local VFH safety | voxel_mapper.cpp, map_merger.py, d2_dual_agent_search.launch.py |
| 2026-06-29 | D2 layer completion now uses only shared layer coverage from `/shared_map_update_summary`; individual drone layer coverage is diagnostic only and cannot trigger or block a climb | search_mission_controller.cpp |
| 2026-06-30 | High-speed D2 failure audit: D1 accepted a frontier route near the Room B north boundary, reached emergency hover at `fwd=0.26 m`, and PX4 later reported attitude failure. Planning clearances were tightened so frontier viewpoints, A* shortcutting, and path smoothing reject wall/shelf-adjacent routes earlier. The filtered LiDAR scan now publishes an explicit SLAM frame (`lidar_2d_link` / `d1_lidar_2d_link`) because the run showed `slam_pose_fusion` falling back to PX4 pose for the whole flight (`map`/`d1_map` TF unavailable), which can explain small merged-map shifts | lidar_tilt_filter.py, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-30 | Follow-up D2 run showed no emergency hovers, but D0 was slowed/stalled in a passable gap: `active_goal_blocked_range_m=1.05` cleared a waypoint at `front=1.04 m`, and nearest-obstacle slowdown capped VFH to `0.20 m/s` for long periods at `nearest≈0.8-1.1 m`. Local safety was relaxed slightly (`active_goal_blocked_range_m=0.90`, reject radius `0.95`, nearest slowdown `1.05→1.65 m`, min speed scale `0.14`) while keeping the stricter planner/frontier clearances | d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-30 | Shared layer coverage overestimate fixed: the team free-cloud union can no longer be capped into a local denominator and report false 100% while visible frontiers remain. If merged observed cells exceed the best local layer total, the shared denominator expands by the remaining local unobserved margin. Arrival-look also now triggers on the final waypoint before requesting a new frontier, disables final-waypoint lookahead, and sweeps ±90° for 6 s so the RGB-D camera exposes side/frontier regions from the reached viewpoint | map_merger.py, search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-30 | Room B N-S warehouse shelves moved 0.5 m west (`E=16.8/21.4` → `E=16.3/20.9`) so the wall-side micro-aisle is less attractive to the planner and should not be treated as a narrow valid passage during high-speed runs | single_agent_search_room_easy.sdf |
| 2026-06-30 | Layer coverage definition corrected after D2 runs showed false local/shared 95-100% jumps: coverage no longer depends on the currently reachable component around each drone. It is now computed over the physical layer mask: free cells count as covered, unknown cells remain missing, and confirmed occupied cells are removed from the denominator. `map_merger` also stopped recomputing shared layer coverage from raw free-cloud points, because free clouds have no unknown-cell denominator and can falsely complete a layer | voxel_mapper.cpp, map_merger.py, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-30 | Planner wall-hugging fix: `compute_obstacle_dist_map()` now measures clearance from the same inflated planning obstacle mask used by passability, plus world boundaries. Frontier scoring, A*, shortcutting, and B-spline validation therefore share one clearance model. Intermediate path points are pulled toward the local clearance ridge before smoothing, so routes round away from shelves/walls without increasing VFH/local emergency safety | voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-30 | Frontier deadlock recovery added after the first 0.50 m/s sweep attempt timed out at ~82.8% layer coverage: if all normal frontier viewpoints are rejected while reachable free space still exists, the mapper now selects a recovery viewpoint from reachable, obstacle-clear, path-connected cells. The recovery score prefers high clearance, useful camera information, and outward progress, so the controller gets a least-risk route instead of holding indefinitely with `no planned route available` | voxel_mapper.cpp |
| 2026-06-30 | Layer/deadlock audit follow-up: layer coverage is again based on the reachable physical component, so closed-off unknown pockets behind mapped walls/shelves do not keep the denominator artificially low. The normal climb condition remains stable 90% layer coverage, but documented fallback reasons (`stagnation`, `timeout`, `no_frontier`) are restored at a lower fallback threshold to prevent infinite 82% deadlocks. Shared occupied voxels now feed the inflated obstacle mask, and route invalidation drops only the active waypoint instead of clearing the whole path. Duplicate local slowdowns were reduced so VFH remains the primary obstacle-avoidance layer | voxel_mapper.cpp, search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-06-30 | SLAM namespace/load fix: `slam_toolbox` parameters now use a wildcard YAML key (`/**`) so namespaced nodes (`/px4_0/slam_toolbox`, `/px4_1/slam_toolbox`) receive `scan_topic=lidar/scan_filtered` and the correct base/map/odom frames. Once the params actually loaded, Jazzy rejected `correlation_search_space_smear_deviation=0.15`; it is now set to `0.08` in both SLAM configs so the node should stay alive and publish `map->odom` / `d1_map->d1_odom` | slam_toolbox_params.yaml, slam_toolbox_params_d1.yaml |
| 2026-07-01 | Long D2 run audit: SLAM/mapping now works and both drones reach Room B, but D0 advanced from layer 2 at 86.2% while 1447 reachable frontier cells remained, and D1 repeatedly tried to climb under low headroom (`range_up=0.74 m`, required `2.81 m`). Layer completion now requires the reachable frontier-cell count to be below `layer_complete_max_reachable_frontier_cells` for coverage/stagnation/timeout completion, and a blocked layer climb starts a ceiling-relocation scan/escape instead of immediately retrying in place | search_mission_controller.cpp |
| 2026-07-01 | Reachability-collapse guard added after the 2.50 m/s single-drone run showed `reachable_path` collapsing from thousands of cells to ~81 cells and layer coverage jumping falsely to 100%. `voxel_mapper` now keeps the last good denominator per scan layer and rejects sudden denominator drops (`coverage_denominator_drop_ratio=0.45`, min 500 cells), so coverage returns to the last credible value instead of treating a tiny disconnected island as the whole layer. Controller no-frontier completion was also shortened when `lc>=90%`, so a genuinely complete layer with no route can advance without waiting the full no-frontier grace period | voxel_mapper.cpp, search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-07-01 | Mezzanine-layer route-mask fix after the second 2.50 m/s retry stuck at the 4 m layer: the coverage guard correctly rejected fake 100%, but the planner's vertical obstacle band (`±0.80 m`) could project ceiling/floor voxels from the Room A second floor into the 2D passable map and collapse reachability. `obstacle_band_half_m` is now `0.45 m`, so route planning blocks obstacles near the flight body while ceiling/headroom remains handled by `range_up` logic | d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-07-01 | Upper-layer Room A omission fixed after the longer 2.50 m/s run: layer coverage no longer uses only the component reachable from the drone's current XY cell, because that made Room B at 8-9 m look like the entire layer when Room A was disconnected by mezzanine/corridor geometry. `coverage_use_reachable_component=false` keeps the physical room/corridor mask in the denominator. Frontier viewpoint selection also no longer hard-rejects candidates solely because a same-layer 2D BFS marks them unreachable; A*/3D A* now gets a chance to prove a route. The Room B eastward bootstrap bonus is disabled above 5 m so upper layers can return to Room A instead of staying in the warehouse | voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-07-02 | Frontier-arrival camera behavior changed from a left/right look to one full 360-degree yaw sweep over `frontier_arrival_look_s=6.0`. This improved local exposure and helped the coverage metric rise after reaching a viewpoint without adding extra route waypoints | search_mission_controller.cpp |
| 2026-07-02 | Active-route obstacle break added: if VFH detects that a newly observed obstacle blocks the active route for `0.8 s` and the safe VFH direction differs from the route bearing by more than `55 deg`, the controller clears the route, remembers the waypoint as blocked, holds, and waits for a new mapper route. This handles the case where a path was valid at planning time but became invalid as the RGB-D map improved during flight | search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-07-02 | Experiment completion threshold changed to `scan_complete_frac=0.75` global/team coverage after the arrival 360-degree sweep improved useful coverage. Layer transition remains `layer_complete_frac=0.83` for the current experiment configuration | d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-07-02 | 4 m+ room-transition fix: the high-layer obstacle band/known connector logic now starts at `obstacle_band_high_alt_threshold_m=3.5` instead of only upper layers. This keeps the Room A -> corridor -> Room B planning spine available at the 4 m layer, where mezzanine/ceiling/shelf voxels could otherwise disconnect the passable graph even though the physical passage is open | voxel_mapper.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-07-02 | Wall-approach guard decoupled from the nearest-obstacle speed cap. Disabling `nearest_obstacle_slowdown_enabled` (done to stop gap stalls) had silently disabled `apply_nearest_obstacle_component_guard` too, because both shared one flag — two consecutive `v=1.30` runs then crashed into the Room B north wall at `y≈28` (`fwd` 1.28→0.41 m in ~2 s, SLAM jumps, attitude failure roll). New `nearest_obstacle_guard_enabled` param (default true) gates the guard independently; launch files keep the speed cap off and the guard on. Re-enabling the cap alone was tried first and reproduced the 2026-06-30 stall (drone never left Room A) | search_mission_controller.cpp, d1_single_agent_search.launch.py, d2_dual_agent_search.launch.py |
| 2026-07-02 | New metrics `time_to_first_detection` / `time_to_first_confirmed_target` (seconds from mission start, same SCAN-anchor time base as `last_logged_scan_time`) added to the finalize script; new `scripts/plot_experiment_series.py` renders the speed-sweep plots and prints LaTeX table rows; series default speeds aligned to `0.70 1.30 1.80 2.00` | finalize_search_experiment.sh, plot_experiment_series.py, run_search_experiment_series.sh |

---

## Experiment & Reporting Plan

All future runs should be stored under `experiments/` instead of a loose `results/`
folder. Folder name format:

```text
experiment_YYYY-MM-DD_Ndrones_<N>_vmax_<V>mps_<short_label>/
```

Examples:

```text
experiments/experiment_2026-06-25_Ndrones_1_vmax_0.85mps_baseline/
experiments/experiment_2026-06-26_Ndrones_1_vmax_1.00mps_speed_sweep/
experiments/experiment_2026-06-28_Ndrones_2_vmax_0.85mps_shared_map/
```

Required artifacts per experiment:

- `README.md` — hypothesis, world, launch file, speed, drone count, code commit/state.
- `logs/` — ROS logs, Gazebo logs, controller output.
- `bags/` — selected ROS bags if recorded.
- `plots/` — route, coverage, speed, target detections, collision/stall markers.
- `data/` — CSV output from `drone_logger.py` or follow-up analysis scripts.
- `summary.md` — result, failure modes, recommended next change.

Planned sequence:

1. One-drone fixed-speed sweep: `v_max = 0.70, 1.30, 1.80, 2.00 m/s`.
2. Two-drone fixed-speed sweep: same speed matrix with shared map/coverage enabled.
3. Keep all parameters stable except `v_max_mps` and `drone_count` during the sweep.
4. Final LaTeX report: import experiment summaries, plots, and conclusions; send to supervisor, revise, and submit.

---

## Open Gaps

| Gap | Status |
|-----|--------|
| Single-drone full stack in warehouse world | ✅ Working: scan, navigation, map, target detection |
| Validate RETURN `return_path` from far Room B back to home | ⚠️ Uses obstacle-safe A* now; forced/full RETURN run still pending |
| Verify Voronoi gradient-ascent candidates reach corridor/rack aisle centres in practice | ⚠️ Pending RViz/flight observation |
| RViz2 config | ✅ Exists at `ros2_ws/src/swarm_bringup/config/scan_rviz.rviz`; still secondary to dashboard |
| Per-layer coverage fraction publisher | ✅ Working in dashboard and mission summaries |
| Structured experiment analysis scripts/plots | ❌ Open |
| Two-drone shared-map validation | ✅ Running: shared occupied/free overlay and team coverage are active; keep monitoring map alignment |
| High-layer Room A/B traversal | ⚠️ Improved: 4 m+ connector added; validate in the next run |
| Two-drone dynamic collision avoidance | ⚠️ Basic peer separation exists; high-speed wall/shelf stability still under validation |
| Hardware localization (VIO/SLAM → EKF3) | ❌ Out of scope for SITL |
