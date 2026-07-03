// Single-drone VFH search mission controller (thesis §4.2).
//
// Mode 1 — TAKEOFF: climb to scan_alt_start (optional 360° lookaround), brief settle.
// Mode 2 — SCAN:    Follow voxel_mapper A* route; VFH is the local safety layer.
//   LiDAR 360° → 72 sectors (5° each) → blocked if obstacle within rho0_.
//   Best open sector toward current route waypoint → velocity setpoint.
//   No local minima: VFH explicitly selects free space (APF cannot guarantee this).
//   Z: P-controller (calc_vz) holds each scan layer altitude independently.
//   All setpoints via pure VELOCITY mode — never MIXED (position+velocity).
//       MIXED with position.xy=NaN causes PX4 to treat velocity as feedforward only.
// Mode 3 — RETURN: Follow voxel_mapper A* return_path to home, then LAND.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <swarm_msgs/msg/map_update_summary.hpp>
#include <swarm_msgs/msg/target_found.hpp>

using namespace std::chrono_literals;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleAttitude;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::Range;
using std_msgs::msg::Empty;
using std_msgs::msg::String;
using swarm_msgs::msg::MapUpdateSummary;
using swarm_msgs::msg::TargetFound;

enum class State {
    PRIMING, REQUESTING, TAKEOFF, SCAN, TARGET_INSPECT, ORBIT, RETURN, LANDING, DONE, TEST_FORWARD
};
enum class TakeoffPhase { CLIMB, LOOKAROUND, SETTLE };
// POSITION: OCM.position=true  — PX4 position controller, requires valid position.xyz setpoint.
// VELOCITY: OCM.velocity=true  — PX4 velocity controller, requires valid velocity.xyz setpoint.
//           Uses publish_velocity_sp(vn, ve, vz, yaw).  All 3 velocity components are
//           explicit; vz drives altitude via the calc_vz() P-controller.
//           This avoids the MIXED (position=true+velocity=true with position.xy=NaN) pitfall
//           where PX4 treats velocity as feedforward only and barely moves horizontally.
enum class Mode  { POSITION, VELOCITY };

static constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
static constexpr float k2Pi = 2.0f * static_cast<float>(M_PI);

static float wrap_pi(float a)
{
    while (a >  static_cast<float>(M_PI)) a -= k2Pi;
    while (a < -static_cast<float>(M_PI)) a += k2Pi;
    return a;
}

static float angle_error_abs(float a, float b)
{
    return std::fabs(wrap_pi(a - b));
}

static float percentile_sample(std::vector<float> &values, float percentile)
{
    if (values.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    percentile = std::clamp(percentile, 0.0f, 100.0f);
    const size_t idx = static_cast<size_t>(
        std::round((percentile / 100.0f) * static_cast<float>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + idx, values.end());
    return values[idx];
}

static float yaw_from_xyzw(float x, float y, float z, float w)
{
    const float siny_cosp = 2.0f * (w * z + x * y);
    const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

struct Vec3f {
    float x;
    float y;
    float z;
};

static Vec3f quat_rotate_wxyz(float w, float x, float y, float z, const Vec3f &v)
{
    const Vec3f qv{x, y, z};
    const Vec3f t{
        2.0f * (qv.y * v.z - qv.z * v.y),
        2.0f * (qv.z * v.x - qv.x * v.z),
        2.0f * (qv.x * v.y - qv.y * v.x)
    };
    return {
        v.x + w * t.x + (qv.y * t.z - qv.z * t.y),
        v.y + w * t.y + (qv.z * t.x - qv.x * t.z),
        v.z + w * t.z + (qv.x * t.y - qv.y * t.x)
    };
}

class SearchMissionController : public rclcpp::Node
{
public:
    SearchMissionController() : Node("search_mission_controller")
    {
        // ── Parameters ────────────────────────────────────────────────────────
        target_alt_   = declare_parameter<float>("target_altitude_m",   8.5f);
        v_max_        = declare_parameter<float>("v_max_mps",           0.3f);
        max_accel_    = declare_parameter<float>("max_accel_mps2",      0.5f);
        v_max_ramp_step_ = declare_parameter<float>("v_max_ramp_step_mps", 0.0f);  // 0 = no ramp
        v_max_target_ = declare_parameter<float>("v_max_target_mps",   2.0f);

        xi_           = declare_parameter<float>("attraction_gain",     0.55f);
        xi_z_         = declare_parameter<float>("attraction_gain_z",   1.5f);  // same as kZKp_
        eta_          = declare_parameter<float>("repulsion_gain",      3.0f);
        drone_radius_m_ = declare_parameter<float>("drone_radius_m",   0.25f);
        { float safe  = declare_parameter<float>("safety_margin_m",    0.30f);
          rho0_        = drone_radius_m_ + safe;
          rho0_base_   = rho0_; }
        voxel_alt_band_m_ = declare_parameter<float>("voxel_alt_band_m", 1.0f);

        north_min_    = declare_parameter<float>("north_min_m",        -5.0f);
        north_max_    = declare_parameter<float>("north_max_m",         5.0f);
        east_min_     = declare_parameter<float>("east_min_m",         -5.0f);
        east_max_     = declare_parameter<float>("east_max_m",          5.0f);

        scan_done_frac_  = declare_parameter<float>("scan_complete_frac",  0.90f);
        goal_radius_     = declare_parameter<float>("goal_reach_radius_m", 0.5f);
        arm_timeout_s_   = declare_parameter<float>("arm_timeout_s",      180.0f);
        altitude_tolerance_m_ = declare_parameter<float>("altitude_tolerance_m", 0.3f);
        search_timeout_s_= declare_parameter<float>("search_timeout_s",   720.0f);
        hold_target_s_   = declare_parameter<float>("target_hold_s",        5.0f);
        target_revisit_radius_m_ =
            declare_parameter<float>("target_revisit_radius_m", 1.25f);
        target_confirm_radius_m_ =
            declare_parameter<float>("target_confirm_radius_m", 0.75f);
        climb_rate_      = declare_parameter<float>("climb_rate_mps",       0.15f);
        return_alt_      = declare_parameter<float>("return_altitude_m",    1.5f);
        home_north_m_    = declare_parameter<float>("home_north_m",         0.0f);
        home_east_m_     = declare_parameter<float>("home_east_m",          0.0f);
        ceiling_clearance_m_ = declare_parameter<float>("ceiling_clearance_m", 0.80f);
        ceiling_headroom_tolerance_m_ =
            declare_parameter<float>("ceiling_headroom_tolerance_m", 0.15f);
        ceiling_escape_timeout_s_ =
            declare_parameter<float>("ceiling_escape_timeout_s", 45.0f);
        scan_enabled_    = declare_parameter<bool> ("scan_enabled",         true);
        floor_clearance_m_ = declare_parameter<float>("floor_clearance_m",  0.45f);
        floor_recovery_climb_m_ =
            declare_parameter<float>("floor_recovery_climb_m", 0.40f);
        frontier_goal_yaw_capture_radius_m_ =
            declare_parameter<float>("frontier_goal_yaw_capture_radius_m", 1.2f);
        frontier_arrival_look_enabled_ =
            declare_parameter<bool>("frontier_arrival_look_enabled", true);
        frontier_arrival_look_s_ =
            declare_parameter<float>("frontier_arrival_look_s", 3.0f);
        {
            const float deg =
                declare_parameter<float>("frontier_arrival_look_half_angle_deg", 35.0f);
            frontier_arrival_look_half_angle_rad_ =
                deg * static_cast<float>(M_PI) / 180.0f;
        }
        transit_timeout_s_ =
            declare_parameter<float>("transit_timeout_s", 20.0f);
        goal_progress_timeout_s_ =
            declare_parameter<float>("goal_progress_timeout_s", 3.5f);
        goal_progress_epsilon_m_ =
            declare_parameter<float>("goal_progress_epsilon_m", 0.25f);
        active_goal_blocked_range_m_ =
            declare_parameter<float>("active_goal_blocked_range_m", 1.00f);
        route_blocked_range_m_ =
            declare_parameter<float>("route_blocked_range_m", 0.90f);
        {
            const float deg = declare_parameter<float>("route_blocked_half_angle_deg", 18.0f);
            route_blocked_half_angle_rad_ =
                deg * static_cast<float>(M_PI) / 180.0f;
        }
        route_conflict_replan_enabled_ =
            declare_parameter<bool>("route_conflict_replan_enabled", true);
        route_conflict_range_m_ =
            declare_parameter<float>("route_conflict_range_m", 1.25f);
        {
            const float deg = declare_parameter<float>("route_conflict_angle_deg", 55.0f);
            route_conflict_angle_rad_ =
                deg * static_cast<float>(M_PI) / 180.0f;
        }
        route_conflict_replan_s_ =
            declare_parameter<float>("route_conflict_replan_s", 0.8f);
        emergency_stop_range_m_ =
            declare_parameter<float>("emergency_stop_range_m", 0.30f);
        emergency_stop_base_ = emergency_stop_range_m_;
        kd_vel_damp_ = declare_parameter<float>("kd_vel_damp", 0.18f);

        scan_alt_start_    = declare_parameter<float>("scan_alt_start_m",   1.5f);
        scan_layer_step_   = declare_parameter<float>("scan_layer_step_m",  1.0f);
        scan_follow_path_altitude_ =
            declare_parameter<bool>("scan_follow_path_altitude", false);
        layer_settle_s_    = declare_parameter<float>("layer_settle_s",     5.0f);
        layer_dwell_s_     = declare_parameter<float>("layer_dwell_s",     45.0f);
        layer_stagnation_s_= declare_parameter<float>("layer_stagnation_s",30.0f);
        layer_complete_frac_ =
            declare_parameter<float>("layer_complete_frac", 0.80f);
        layer_stagnation_min_frac_ =
            declare_parameter<float>("layer_stagnation_min_frac", 0.88f);
        layer_timeout_min_frac_ =
            declare_parameter<float>("layer_timeout_min_frac", 0.90f);
        layer_complete_stable_s_ =
            declare_parameter<float>("layer_complete_stable_s", 8.0f);
        {
            const int max_cells = static_cast<int>(
                declare_parameter<int>("layer_complete_max_reachable_frontier_cells", 120));
            layer_complete_max_reachable_frontier_cells_ =
                static_cast<uint32_t>(std::max(0, max_cells));
        }
        min_layer_dwell_s_ =
            declare_parameter<float>("min_layer_dwell_s", 60.0f);
        no_frontier_layer_grace_s_ =
            declare_parameter<float>("no_frontier_layer_grace_s", 45.0f);
        layer_index_start_ =
            static_cast<int>(declare_parameter<int>("layer_index_start", 0));
        layer_index_stride_ =
            std::max(1, static_cast<int>(declare_parameter<int>("layer_index_stride", 1)));
        waypoint_lookahead_radius_m_ =
            declare_parameter<float>("waypoint_lookahead_radius_m", 0.85f);
        waypoint_lookahead_cos_ =
            declare_parameter<float>("waypoint_lookahead_cos", 0.25f);
        waypoint_lookahead_near_obstacle_m_ =
            declare_parameter<float>("waypoint_lookahead_near_obstacle_m", 0.90f);
        waypoint_lookahead_near_radius_m_ =
            declare_parameter<float>("waypoint_lookahead_near_radius_m", 0.55f);
        waypoint_lookahead_near_cos_ =
            declare_parameter<float>("waypoint_lookahead_near_cos", 0.80f);
        blocked_waypoint_reject_radius_m_ =
            declare_parameter<float>("blocked_waypoint_reject_radius_m", 0.80f);
        blocked_waypoint_reject_s_ =
            declare_parameter<float>("blocked_waypoint_reject_s", 10.0f);
        local_apf_repulsion_enabled_ =
            declare_parameter<bool>("local_apf_repulsion_enabled", true);
        local_apf_repulsion_range_m_ =
            declare_parameter<float>("local_apf_repulsion_range_m", 1.10f);
        local_apf_repulsion_gain_ =
            declare_parameter<float>("local_apf_repulsion_gain", 0.22f);
        local_apf_repulsion_max_mps_ =
            declare_parameter<float>("local_apf_repulsion_max_mps", 0.65f);

        // Forward obstacle — emergency stop only (VFH handles normal avoidance)
        fwd_stop_m_      = declare_parameter<float>("forward_stop_m",      1.0f);
        fwd_stop_m_base_ = fwd_stop_m_;
        fwd_resume_margin_m_ = declare_parameter<float>("forward_resume_margin_m", 0.20f);
        fwd_decel_m_     = declare_parameter<float>("forward_decel_m",     2.5f);
        // VFH: each LiDAR ray contributes to a 360° sector occupancy histogram.
        // lidar_rho0: max range considered by the histogram.
        // eta is kept only for launch-file compatibility with old APF configs.
        lidar_rho0_m_ = declare_parameter<float>("lidar_repulsion_rho0_m", 2.5f);
        lidar_eta_    = declare_parameter<float>("lidar_repulsion_eta",     2.0f);
        { float deg = declare_parameter<float>("lidar_forward_angle_deg", 0.0f);
          lidar_forward_angle_rad_ = deg * static_cast<float>(M_PI) / 180.0f; }
        { float deg = declare_parameter<float>("lidar_forward_half_angle_deg", 60.0f);
          deg = std::clamp(deg, 5.0f, 120.0f);
          lidar_forward_half_angle_rad_ = deg * static_cast<float>(M_PI) / 180.0f; }
        align_threshold_   = declare_parameter<float>("align_threshold_rad",  0.52f); // 30°
        yaw_slew_alpha_    = declare_parameter<float>("yaw_slew_alpha",       0.20f);
        { float dps = declare_parameter<float>("yaw_rate_limit_dps", 35.0f);
          yaw_rate_limit_rad_s_ = dps * static_cast<float>(M_PI) / 180.0f; }
        holonomic_vfh_ = declare_parameter<bool>("holonomic_vfh", true);
        disable_vfh_ = declare_parameter<bool>("disable_vfh", false);
        camera_yaw_track_goal_ = declare_parameter<bool>("camera_yaw_track_goal", true);
        camera_yaw_slowdown_rad_ =
            declare_parameter<float>("camera_yaw_slowdown_rad", 0.90f);
        camera_yaw_stop_rad_ =
            declare_parameter<float>("camera_yaw_stop_rad", 1.80f);
        camera_yaw_min_speed_scale_ =
            declare_parameter<float>("camera_yaw_min_speed_scale", 0.25f);
        narrow_passage_slowdown_enabled_ =
            declare_parameter<bool>("narrow_passage_slowdown_enabled", true);
        narrow_passage_clearance_m_ =
            declare_parameter<float>("narrow_passage_clearance_m", 0.85f);
        narrow_passage_resume_clearance_m_ =
            declare_parameter<float>("narrow_passage_resume_clearance_m", 1.35f);
        narrow_passage_min_speed_scale_ =
            declare_parameter<float>("narrow_passage_min_speed_scale", 0.25f);
        nearest_obstacle_slowdown_enabled_ =
            declare_parameter<bool>("nearest_obstacle_slowdown_enabled", true);
        nearest_obstacle_slowdown_m_ =
            declare_parameter<float>("nearest_obstacle_slowdown_m", 0.90f);
        nearest_obstacle_resume_m_ =
            declare_parameter<float>("nearest_obstacle_resume_m", 1.50f);
        nearest_obstacle_min_speed_scale_ =
            declare_parameter<float>("nearest_obstacle_min_speed_scale", 0.12f);
        nearest_obstacle_approach_guard_m_ =
            declare_parameter<float>("nearest_obstacle_approach_guard_m", 0.70f);
        // Safety guard is independent of the speed-cap flag: the cap can stall
        // the drone in passable gaps, but the guard must keep stripping any
        // velocity component toward the nearest wall even when the cap is off.
        nearest_obstacle_guard_enabled_ =
            declare_parameter<bool>("nearest_obstacle_guard_enabled", true);
        nearest_obstacle_centering_enabled_ =
            declare_parameter<bool>("nearest_obstacle_centering_enabled", true);
        nearest_obstacle_centering_m_ =
            declare_parameter<float>("nearest_obstacle_centering_m", 0.95f);
        nearest_obstacle_centering_max_mps_ =
            declare_parameter<float>("nearest_obstacle_centering_max_mps", 0.35f);
        { float dps = declare_parameter<float>("hold_scan_yaw_rate_dps", 10.0f);
          hold_scan_yaw_rate_rad_s_ = dps * static_cast<float>(M_PI) / 180.0f; }
        {
            // VFH camera-FOV mode: sectors outside ±vfh_fov_half_deg of the drone's
            // heading are treated as free. 0 = full 360° (LiDAR mode).
            // 60° matches a typical depth camera horizontal half-FOV.
            const float fov_deg = declare_parameter<float>("vfh_fov_half_deg", 0.0f);
            // kSRad = 5° per sector → sectors = deg / 5
            vfh_fov_half_sectors_ = (fov_deg > 0.0f)
                ? static_cast<int>(std::round(fov_deg / 5.0f))
                : 0;
        }

        // Mode 1 (fast takeoff to scan altitude)
        takeoff_lookaround_enabled_ =
            declare_parameter<bool>("takeoff_lookaround_enabled", false);
        { float dps = declare_parameter<float>("takeoff_yaw_rate_dps", 30.0f);
          takeoff_yaw_rate_ = dps * static_cast<float>(M_PI) / 180.0f; }
        takeoff_map_settle_s_ = declare_parameter<float>("takeoff_map_settle_s", 0.5f);
        test_forward_mode_   = declare_parameter<bool> ("test_forward_mode",    false);
        test_skip_rotation_  = declare_parameter<bool> ("test_skip_rotation",   false);
        test_forward_dist_m_ = declare_parameter<float>("test_forward_dist_m",  2.0f);
        test_wall_stop_m_    = declare_parameter<float>("test_wall_stop_m",     1.0f);
        test_hold_s_         = declare_parameter<float>("test_hold_s",          2.0f);
        test_goal_radius_m_  = declare_parameter<float>("test_goal_radius_m",   0.20f);
        test_alt_tol_m_      = declare_parameter<float>("test_alt_tolerance_m", 0.15f);
        test_ignore_depth_until_phase3_ =
            declare_parameter<bool>("test_ignore_depth_until_phase3", false);
        test_phase3_enabled_ =
            declare_parameter<bool>("test_phase3_enabled", true);
        test_lidar_nearest_stop_ =
            declare_parameter<bool>("test_lidar_nearest_stop", true);
        depth_roi_width_frac_ = std::clamp(
            static_cast<float>(declare_parameter<double>("depth_roi_width_frac", 0.60)),
            0.05f, 1.0f);
        depth_percentile_ = std::clamp(
            static_cast<float>(declare_parameter<double>("depth_percentile", 0.0)),
            0.0f, 100.0f);
        { float deg = declare_parameter<float>("depth_min_world_elev_deg", -15.0f);
          depth_min_world_elev_rad_ = deg * static_cast<float>(M_PI) / 180.0f; }
        { float deg = declare_parameter<float>("depth_max_world_elev_deg", 40.0f);
          depth_max_world_elev_rad_ = deg * static_cast<float>(M_PI) / 180.0f; }
        depth_forward_fusion_enabled_ =
            declare_parameter<bool>("depth_forward_fusion_enabled", true);
        depth_forward_timeout_s_ =
            declare_parameter<float>("depth_forward_timeout_s", 0.5f);
        depth_forward_min_samples_ = static_cast<uint32_t>(std::max<int64_t>(0,
            declare_parameter<int>("depth_forward_min_samples", 6)));
        // Low-ledge recovery: keep below-horizon depth returns that are clearly
        // closer than the floor would be (real low steps the legs clip), instead
        // of blanket-rejecting everything under depth_min_world_elev as "floor".
        depth_low_obstacle_enabled_ =
            declare_parameter<bool>("depth_low_obstacle_enabled", true);
        depth_low_obstacle_max_m_ =
            declare_parameter<float>("depth_low_obstacle_max_m", 1.5f);
        depth_floor_reject_frac_ = std::clamp(
            static_cast<float>(declare_parameter<float>("depth_floor_reject_frac", 0.80f)),
            0.1f, 0.99f);
        use_nav_pose_ = declare_parameter<bool>("use_nav_pose", true);
        nav_pose_timeout_s_ = declare_parameter<float>("nav_pose_timeout_s", 1.0f);
        nav_pose_frame_id_ = declare_parameter<std::string>("nav_pose_frame_id", "map_ned");
        px4_goal_frame_id_ = declare_parameter<std::string>("px4_goal_frame_id", "odom");
        peer_avoidance_enabled_ =
            declare_parameter<bool>("peer_avoidance_enabled", false);
        peer_local_position_topic_ =
            declare_parameter<std::string>("peer_local_position_topic", "");
        own_spawn_north_m_ =
            declare_parameter<float>("own_spawn_north_m", 0.0f);
        own_spawn_east_m_ =
            declare_parameter<float>("own_spawn_east_m", 0.0f);
        peer_spawn_north_m_ =
            declare_parameter<float>("peer_spawn_north_m", 0.0f);
        peer_spawn_east_m_ =
            declare_parameter<float>("peer_spawn_east_m", 0.0f);
        peer_min_separation_m_ =
            declare_parameter<float>("peer_min_separation_m", 1.50f);
        peer_avoidance_gain_ =
            declare_parameter<float>("peer_avoidance_gain", 0.60f);
        peer_pose_timeout_s_ =
            declare_parameter<float>("peer_pose_timeout_s", 1.0f);
        peer_avoidance_z_tolerance_m_ =
            declare_parameter<float>("peer_avoidance_z_tolerance_m", 1.5f);
        shared_summary_topic_ =
            declare_parameter<std::string>("shared_summary_topic", "");

        // ── Publishers / subscribers ──────────────────────────────────────────
        const rclcpp::QoS qos_be  = rclcpp::QoS(1).best_effort();
        const rclcpp::QoS qos_rel = rclcpp::QoS(10);

        ocm_pub_    = create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", qos_rel);
        sp_pub_     = create_publisher<TrajectorySetpoint> ("fmu/in/trajectory_setpoint",   qos_rel);
        cmd_pub_    = create_publisher<VehicleCommand>     ("fmu/in/vehicle_command",        qos_rel);
        result_pub_ = create_publisher<String>("mission/result", rclcpp::QoS(10).reliable().transient_local());
        state_pub_  = create_publisher<String>("mission/state", rclcpp::QoS(10).reliable().transient_local());
        vfh_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "vfh_vectors", rclcpp::QoS(1).best_effort());
        raw_vfh_yaw_pub_ = create_publisher<std_msgs::msg::Float32>(
            "telemetry/raw_vfh_yaw", rclcpp::QoS(1).best_effort());
        cmd_yaw_pub_ = create_publisher<std_msgs::msg::Float32>(
            "telemetry/cmd_yaw", rclcpp::QoS(1).best_effort());
        committed_goal_pub_ = create_publisher<PointStamped>(
            "committed_goal", rclcpp::QoS(1).reliable().transient_local());
        active_path_pub_ = create_publisher<Path>(
            "active_path", rclcpp::QoS(1).reliable().transient_local());
        status_sub_ = create_subscription<VehicleStatus>(
            "fmu/out/vehicle_status_v4", qos_be,
            [this](VehicleStatus::SharedPtr m) { status_ = *m; });

        pos_sub_ = create_subscription<VehicleLocalPosition>(
            "fmu/out/vehicle_local_position_v1", qos_be,
            [this](VehicleLocalPosition::SharedPtr m) { pos_ = *m; });

        if (peer_avoidance_enabled_ && !peer_local_position_topic_.empty()) {
            peer_pos_sub_ = create_subscription<VehicleLocalPosition>(
                peer_local_position_topic_, qos_be,
                [this](VehicleLocalPosition::SharedPtr m) {
                    peer_pos_ = *m;
                    have_peer_pos_ = true;
                    last_peer_pos_ns_ = now().nanoseconds();
                });
        }

        nav_pose_sub_ = create_subscription<Odometry>(
            "slam/odom_ned", qos_be,
            [this](Odometry::SharedPtr m) {
                nav_odom_ = *m;
                have_nav_odom_ = true;
                last_nav_odom_ns_ = now().nanoseconds();
            });

        dashboard_cmd_sub_ = create_subscription<String>(
            "dashboard/cmd", qos_rel,
            [this](String::SharedPtr m) {
                if (m->data == "LAND") {
                    RCLCPP_WARN(get_logger(), "Dashboard command: LAND");
                    transition(State::LANDING);
                } else if (m->data == "RETURN") {
                    RCLCPP_WARN(get_logger(), "Dashboard command: RETURN HOME");
                    transition(State::RETURN);
                } else if (m->data == "DISARM") {
                    RCLCPP_ERROR(get_logger(), "Dashboard command: DISARM");
                    send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                    transition(State::DONE);
                }
            });

        // VehicleAttitude: extract drone pitch for depth camera floor-rejection filter.
        // PX4 quaternion convention: q=[w,x,y,z], FRD body → NED world.
        // pitch = asin(2*(w*y - z*x));  positive = nose UP (aerospace convention).
        attitude_sub_ = create_subscription<VehicleAttitude>(
            "fmu/out/vehicle_attitude", qos_be,
            [this](VehicleAttitude::SharedPtr m) {
                const float w = m->q[0], x = m->q[1], y = m->q[2], z = m->q[3];
                att_q_w_ = w;
                att_q_x_ = x;
                att_q_y_ = y;
                att_q_z_ = z;
                drone_pitch_rad_ = std::asin(std::clamp(2.0f*(w*y - z*x), -1.0f, 1.0f));
            });

        // 2-D horizontal LiDAR — 360° scan, 1°/ray, 20 Hz.
        // Sensor frame is expected to have angle=0 at drone forward. The active
        // forward window is configurable so scan-frame mistakes show up in logs
        // instead of becoming silent wall collisions.
        // Full scan is stored for VFH. Sector minimums are kept for safety/logging.
        lidar_sub_ = create_subscription<LaserScan>(
            "lidar/scan", qos_be,
            [this](LaserScan::SharedPtr m) {
                const size_t n = m->ranges.size();
                if (n == 0) return;
                constexpr float kSideLo   = 45.0f * static_cast<float>(M_PI) / 180.0f;
                constexpr float kSideHi   = 135.0f * static_cast<float>(M_PI) / 180.0f;
                const float max_range = std::isfinite(m->range_max) ? m->range_max : 10.0f;
                float fwd_min = max_range, right_min = max_range, left_min = max_range;
                float nearest_min = max_range;
                float nearest_angle = 0.0f;
                for (size_t i = 0; i < n; ++i) {
                    const float r = m->ranges[i];
                    if (!std::isfinite(r) || r < m->range_min || r > m->range_max) continue;
                    const float a = m->angle_min + static_cast<float>(i) * m->angle_increment;
                    if (lidar_ray_hits_floor_or_ceiling(a, r)) continue;
                    if (r < nearest_min) {
                        nearest_min = r;
                        nearest_angle = a;
                    }
                    if (angle_error_abs(a, lidar_forward_angle_rad_) <= lidar_forward_half_angle_rad_)
                        fwd_min = std::min(fwd_min, r);
                    if (a >=  kSideLo && a <=  kSideHi)           right_min = std::min(right_min, r);
                    if (a <= -kSideLo && a >= -kSideHi)           left_min  = std::min(left_min,  r);
                }
                lidar_fwd_range_   = fwd_min;
                lidar_right_range_ = right_min;
                lidar_left_range_  = left_min;
                lidar_nearest_range_ = nearest_min;
                lidar_nearest_angle_rad_ = nearest_angle;
                have_lidar_        = true;
                // Store full scan + yaw for VFH.
                lidar_scan_        = *m;
                // Use PX4's continuous local heading for obstacle-sector world alignment.
                // SLAM/map yaw can jump during scan-matching corrections.
                lidar_scan_yaw_    = current_heading();
                have_lidar_scan_   = true;
            });


        // Depth image forward range — attitude-compensated obstacle detection.
        // For each pixel row, the world-frame elevation angle is:
        //   el_world = el_cam - kCamPitchDownRad_ + drone_pitch_rad_
        // Pixels with el_world < kMinWorldElevRad_ (-15°) are floor returns caused
        // by drone pitching forward during flight and are rejected.
        // This prevents false "wall detected" readings when the drone accelerates or climbs.
        depth_sub_ = create_subscription<Image>(
            "camera/depth/image_raw", qos_be,
            [this](Image::SharedPtr m) {
                if (m->encoding != "32FC1") return;
                const auto* data = reinterpret_cast<const float*>(m->data.data());
                const int W = static_cast<int>(m->width);
                const int H = static_cast<int>(m->height);
                // Use a narrow central ROI and a low percentile rather than the
                // absolute minimum so thin side obstacles and single-pixel spikes
                // do not masquerade as a wall directly ahead.
                const int u_margin = static_cast<int>(
                    0.5f * static_cast<float>(W) * (1.0f - depth_roi_width_frac_));
                const int u0 = std::clamp(u_margin, 0, std::max(0, W - 1));
                const int u1 = std::clamp(W - u_margin, u0 + 1, W);
                const float half_fov = kCameraFovVRad_ / 2.0f;
                const float H_f = static_cast<float>(H);
                float raw_min = std::numeric_limits<float>::infinity();
                std::vector<float> samples;
                samples.reserve(static_cast<size_t>((H / 2) * std::max(1, (u1 - u0) / 2)));
                for (int v = 0; v < H; v += 2) {
                    // Elevation in camera frame: top row = +half_fov, bottom = -half_fov
                    const float el_cam   = half_fov * (1.0f - 2.0f * v / H_f);
                    // Elevation in world frame (positive = above horizon)
                    const float el_world = el_cam - kCamPitchDownRad_ + drone_pitch_rad_;
                    if (el_world > depth_max_world_elev_rad_) continue;  // ceiling rows

                    // Rows below depth_min_world_elev are normally dropped to avoid
                    // mapping the floor as a wall. But a CLOSE return there is a real
                    // low ledge/step whose top sits under the lidar plane — exactly
                    // what the landing gear clips. Keep only pixels clearly nearer
                    // than where the floor would intersect this ray; the floor itself
                    // (d ≈ floor_dist) is still rejected.
                    bool  below_gate = el_world < depth_min_world_elev_rad_;
                    float floor_dist = std::numeric_limits<float>::infinity();
                    if (below_gate) {
                        if (!depth_low_obstacle_enabled_ || !pos_.z_valid) continue;
                        const float sin_down = std::sin(-el_world);
                        if (sin_down < 0.05f) continue;
                        floor_dist = std::max(0.0f, -pos_.z) / sin_down;
                    }
                    for (int u = u0; u < u1; u += 2) {
                        const float d = data[v * W + u];
                        if (!std::isfinite(d) || d <= 0.15f) continue;
                        if (below_gate &&
                            (d > depth_low_obstacle_max_m_ ||
                             d > floor_dist * depth_floor_reject_frac_)) {
                            continue;  // too far to trust, or consistent with the floor
                        }
                        raw_min = std::min(raw_min, d);
                        samples.push_back(d);
                    }
                }
                forward_range_samples_ = static_cast<uint32_t>(samples.size());
                forward_range_raw_min_ = std::isfinite(raw_min) ? raw_min : 10.0f;
                const float depth_fwd = std::isfinite(raw_min)
                    ? percentile_sample(samples, depth_percentile_)
                    : 10.0f;
                depth_fwd_range_   = depth_fwd;
                depth_fwd_samples_ = static_cast<uint32_t>(samples.size());
                last_depth_fwd_ns_ = now().nanoseconds();
                // Pre-lidar fallback: before the first lidar scan arrives, the depth
                // value is the only forward estimate. Once lidar is present, loop()
                // fuses the two (min) so this assignment is immediately overwritten.
                forward_range_ = depth_fwd;
                have_fwd_range_ = true;
            });

        ceiling_sub_ = create_subscription<Range>(
            "range/up", qos_be,
            [this](Range::SharedPtr m) {
                if (!std::isfinite(m->range) || m->range <= 0.0f) return;
                ceiling_range_m_ = m->range;
                have_ceiling_range_ = true;
            });
        floor_sub_ = create_subscription<Range>(
            "range/down", qos_be,
            [this](Range::SharedPtr m) {
                if (!std::isfinite(m->range) || m->range <= 0.0f) return;
                floor_range_m_ = m->range;
                have_floor_range_ = true;
            });

        // Entropy centroid — primary attraction target (published by voxel_mapper).
        // Reads all three NED components: x=North, y=East, z=Down (negative = above ground).
        // Z-component drives the vertical APF force toward high-entropy altitudes.
        entropy_sub_ = create_subscription<PointStamped>(
            "entropy_centroid", qos_rel,
            [this](PointStamped::SharedPtr m) {
                entropy_north_ = static_cast<float>(m->point.x);
                entropy_east_  = static_cast<float>(m->point.y);
                entropy_down_  = static_cast<float>(m->point.z);  // NED z (negative = up)
                have_entropy_  = true;
                entropy_frame_id_ = m->header.frame_id;
                last_entropy_ns_ = now().nanoseconds();
            });
        frontier_path_sub_ = create_subscription<Path>(
            "frontier_path", rclcpp::QoS(10).transient_local(),
            [this](Path::SharedPtr msg) {
                if (msg->poses.empty()) return;
                const std::string frame_id = msg->header.frame_id;

                const bool had_front = !waypoint_queue_.empty();
                const float old_front_n = had_front ? waypoint_queue_.front().north : 0.0f;
                const float old_front_e = had_front ? waypoint_queue_.front().east  : 0.0f;
                const std::string old_frame = waypoint_frame_id_;

                std::deque<WayPt> new_queue;
                for (size_t i = 0; i < msg->poses.size(); ++i) {
                    const auto &p = msg->poses[i].pose;
                    const bool is_last = (i + 1 == msg->poses.size());
                    float wp_yaw = 0.0f;
                    bool  wp_has_yaw = false;
                    if (is_last) {
                        wp_yaw = yaw_from_xyzw(
                            static_cast<float>(p.orientation.x),
                            static_cast<float>(p.orientation.y),
                            static_cast<float>(p.orientation.z),
                            static_cast<float>(p.orientation.w));
                        // identity quaternion (w=1) means no yaw encoded
                        wp_has_yaw = std::fabs(static_cast<float>(p.orientation.w) - 1.0f) > 0.01f;
                    }
                    new_queue.push_back({
                        static_cast<float>(p.position.x),
                        static_cast<float>(p.position.y),
                        wp_yaw,
                        static_cast<float>(p.position.z),  // NED z from 3D A*
                        wp_has_yaw
                    });
                }
                // Skip leading waypoints that are already "arrived" (< arrival_radius).
                // The A* first cell is the drone's own grid cell (~0.14 m away).
                auto waypoint_dist = [&](const WayPt &wp, const std::string &wp_frame) {
                    const Goal2D wp_goal = project_goal_to_control_frame({
                        wp.north, wp.east, wp.yaw, wp.has_yaw,
                        "waypoint", wp_frame,
                    });
                    return std::hypot(wp_goal.north - control_north(),
                                      wp_goal.east  - control_east());
                };
                while (new_queue.size() > 1) {
                    const auto &front = new_queue.front();
                    const float d = waypoint_dist(front, frame_id);
                    if (d >= goal_radius_) break;
                    new_queue.pop_front();
                }

                if (new_queue.empty()) {
                    waypoint_queue_.clear();
                    publish_active_scan_path();
                    committed_goal_valid_   = false;
                goal_best_dist_         = std::numeric_limits<float>::infinity();
                goal_no_progress_ticks_ = 0;
                vfh_all_blocked_ticks_  = 0;
                route_conflict_ticks_   = 0;
                return;
                }

                if (state_ == State::SCAN && frontier_arrival_look_ticks_left_ > 0) {
                    // The 360° arrival sweep is still exposing new voxels; accepting
                    // a route now would pick the next goal from the pre-sweep map.
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "frontier_path deferred: arrival look in progress — "
                        "selecting the next goal after the sweep");
                    return;
                }

                if (state_ == State::SCAN &&
                    recent_blocked_scan_waypoint(new_queue.front(), frame_id)) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "frontier_path rejected: front=(%.2f,%.2f) is near a recently "
                        "blocked waypoint — waiting for a different route",
                        new_queue.front().north, new_queue.front().east);
                    return;
                }

                const auto &last_pose = msg->poses.back().pose;
                const float new_end_n = static_cast<float>(last_pose.position.x);
                const float new_end_e = static_cast<float>(last_pose.position.y);
                const float endpoint_shift =
                    std::hypot(new_end_n - last_path_end_n_,
                               new_end_e - last_path_end_e_);

                if (state_ == State::SCAN && !waypoint_queue_.empty()) {
                    const float active_front_dist =
                        waypoint_dist(waypoint_queue_.front(), waypoint_frame_id_);
                    const bool active_front_reached =
                        active_front_dist <= goal_radius_;
                    const float candidate_front_shift =
                        std::hypot(new_queue.front().north - old_front_n,
                                   new_queue.front().east  - old_front_e);
                    const bool safety_refresh =
                        route_safety_stressed() &&
                        (frame_id != old_frame ||
                         candidate_front_shift > goal_radius_ * 0.5f ||
                         endpoint_shift > goal_radius_);
                    if (!active_front_reached && !safety_refresh) {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                            "frontier_path locked: active=%zu front=%.2fm > %.2fm; "
                            "ignoring replanned route until current waypoint is reached",
                            waypoint_queue_.size(), active_front_dist, goal_radius_);
                        return;
                    }

                    if (safety_refresh && !active_front_reached) {
                        RCLCPP_WARN(get_logger(),
                            "frontier_path refresh accepted under safety stress: "
                            "active=%zu front=%.2fm candidate_shift=%.2fm endpoint_shift=%.2fm",
                            waypoint_queue_.size(), active_front_dist,
                            candidate_front_shift, endpoint_shift);
                    } else {
                        RCLCPP_INFO(get_logger(),
                            "frontier_path refresh accepted after waypoint reached: "
                            "active=%zu front=%.2fm endpoint_shift=%.2fm",
                            waypoint_queue_.size(), active_front_dist, endpoint_shift);
                    }
                }

                waypoint_queue_ = std::move(new_queue);
                waypoint_frame_id_ = frame_id;
                last_path_end_n_ = new_end_n;
                last_path_end_e_ = new_end_e;
                publish_active_scan_path();

                const auto &new_front = waypoint_queue_.front();
                const bool front_changed =
                    !had_front ||
                    frame_id != old_frame ||
                    std::hypot(new_front.north - old_front_n,
                               new_front.east  - old_front_e) > goal_radius_ * 0.5f;
                if (front_changed && state_ == State::SCAN) {
                    committed_goal_valid_   = false;
                    goal_best_dist_         = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_  = 0;
                    route_conflict_ticks_   = 0;
                }

                RCLCPP_INFO(get_logger(),
                    "frontier_path: %zu waypoints, front=(%.2f,%.2f) endpoint=(%.2f,%.2f) shift=%.2fm",
                    waypoint_queue_.size(), new_front.north, new_front.east,
                    new_end_n, new_end_e, endpoint_shift);
            });
        return_path_sub_ = create_subscription<Path>(
            "return_path", rclcpp::QoS(10).transient_local(),
            [this](Path::SharedPtr msg) {
                if (msg->poses.empty()) return;
                if (state_ == State::RETURN && !return_waypoint_queue_.empty()) {
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                        "return_path candidate ignored while active route has %zu waypoints",
                        return_waypoint_queue_.size());
                    return;
                }
                const std::string frame_id = msg->header.frame_id;

                const bool had_front = !return_waypoint_queue_.empty();
                const float old_front_n = had_front ? return_waypoint_queue_.front().north : 0.0f;
                const float old_front_e = had_front ? return_waypoint_queue_.front().east  : 0.0f;
                const std::string old_frame = return_waypoint_frame_id_;

                std::deque<WayPt> new_queue;
                for (size_t i = 0; i < msg->poses.size(); ++i) {
                    const auto &p = msg->poses[i].pose;
                    new_queue.push_back({
                        static_cast<float>(p.position.x),
                        static_cast<float>(p.position.y),
                        0.0f,
                        static_cast<float>(p.position.z),
                        false
                    });
                }

                while (new_queue.size() > 1) {
                    const auto &front = new_queue.front();
                    const float d = std::hypot(front.north - control_north(),
                                               front.east  - control_east());
                    if (d >= goal_radius_) break;
                    new_queue.pop_front();
                }

                if (new_queue.empty()) {
                    return_waypoint_queue_.clear();
                    if (state_ == State::RETURN) {
                        committed_goal_valid_   = false;
                        goal_best_dist_         = std::numeric_limits<float>::infinity();
                        goal_no_progress_ticks_ = 0;
                        vfh_all_blocked_ticks_  = 0;
                    }
                    return;
                }

                return_waypoint_queue_ = std::move(new_queue);
                return_waypoint_frame_id_ = frame_id;

                const auto &new_front = return_waypoint_queue_.front();
                const bool front_changed =
                    !had_front ||
                    frame_id != old_frame ||
                    std::hypot(new_front.north - old_front_n,
                               new_front.east  - old_front_e) > goal_radius_ * 0.5f;
                if (front_changed && state_ == State::RETURN) {
                    committed_goal_valid_   = false;
                    goal_best_dist_         = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_  = 0;
                }

                RCLCPP_INFO(get_logger(),
                    "return_path: %zu waypoints, front=(%.2f,%.2f)",
                    return_waypoint_queue_.size(), new_front.north, new_front.east);
            });

        summary_sub_ = create_subscription<MapUpdateSummary>(
            "map_update_summary", qos_rel,
            [this](MapUpdateSummary::SharedPtr m) {
                coverage_fraction_       = m->coverage_fraction;
                entropy_mean_            = m->entropy_mean;
                layer_coverage_fraction_ = m->layer_coverage_fraction;
                summary_layer_altitude_m_ = m->layer_altitude_m;
                summary_layer_total_cells_ = m->layer_total_cells;
                frontier_clusters_ = m->frontier_clusters;
                reachable_frontier_clusters_ = m->reachable_frontier_clusters;
                reachable_frontier_cells_ = m->reachable_frontier_cells;
                frontier_route_available_ = m->frontier_route_available;
                have_layer_summary_ = true;
            });

        if (!shared_summary_topic_.empty()) {
            shared_summary_sub_ = create_subscription<MapUpdateSummary>(
                shared_summary_topic_, qos_rel,
                [this](MapUpdateSummary::SharedPtr m) {
                    shared_coverage_fraction_ = m->coverage_fraction;
                    have_shared_summary_ = true;

                    if (m->layer_total_cells > 0) {
                        const float max_alt_delta =
                            std::max(0.75f, scan_layer_step_ * 0.75f);
                        if (std::fabs(m->layer_altitude_m - scan_alt_) <= max_alt_delta) {
                            shared_layer_coverage_fraction_ = m->layer_coverage_fraction;
                            shared_summary_layer_altitude_m_ = m->layer_altitude_m;
                            shared_summary_layer_total_cells_ = m->layer_total_cells;
                            have_shared_layer_summary_ = true;
                        }
                    }
                });
        }

        target_sub_ = create_subscription<TargetFound>(
            "target_found", qos_rel,
            [this](TargetFound::SharedPtr m) {
                if (state_ != State::SCAN) {
                    return;
                }

                const float n = static_cast<float>(m->position_world.x);
                const float e = static_cast<float>(m->position_world.y);
                if (merge_inspected_target(n, e)) {
                    return;
                }

                if (!target_candidate_valid_ ||
                    std::hypot(n - target_candidate_north_,
                               e - target_candidate_east_) > target_confirm_radius_m_) {
                    target_candidate_valid_ = true;
                    target_candidate_north_ = n;
                    target_candidate_east_  = e;
                    target_confirm_count_   = 1;
                } else {
                    target_candidate_north_ = 0.7f * target_candidate_north_ + 0.3f * n;
                    target_candidate_east_  = 0.7f * target_candidate_east_  + 0.3f * e;
                    ++target_confirm_count_;
                }

                target_north_ = target_candidate_north_;
                target_east_  = target_candidate_east_;
                // Require 2 nearby detections before acting. This rejects single-frame
                // false positives and prevents detections from different objects mixing.
                if (target_confirm_count_ >= 2) target_found_ = true;
            });

        timer_ = create_wall_timer(50ms, [this]() { loop(); });

        RCLCPP_INFO(get_logger(),
            "VFH controller ready — box N[%.1f,%.1f] E[%.1f,%.1f] "
            "home=(%.1f,%.1f) v_max=%.2f xi=%.2f rho0=%.2f fwd_stop=%.1fm lidar_rho0=%.1fm",
            north_min_, north_max_, east_min_, east_max_,
            home_north_m_, home_east_m_,
            v_max_, xi_, rho0_, fwd_stop_m_, lidar_rho0_m_);
        publish_state();
    }

private:
    // ── Loop constants ────────────────────────────────────────────────────────
    static constexpr float kCoverageStagnationThreshold = 0.01f;
    static constexpr float kLoopPeriod_s                = 0.05f;

    // ── Altitude P-controller (calc_vz) ─────────────────────────────────────
    // calc_vz() is kept for transit/settle phases and for scan-layer altitude hold.
    static constexpr float kZKp_  = 1.5f;   // proportional gain (m/s per m error)
    static constexpr float kZMax_ = 0.40f;  // max vertical speed for transit (m/s)

    // ── Depth camera attitude compensation ──────────────────────────────────
    // World-frame elevation = el_cam - camera_pitch_down + drone_pitch.
    // Pixels whose world-frame elevation < kMinWorldElevRad_ are floor returns
    // and are rejected from forward obstacle detection.
    static constexpr float kCameraFovVRad_     = 55.0f * static_cast<float>(M_PI) / 180.0f;
    static constexpr float kCamPitchDownRad_   = 15.0f * static_cast<float>(M_PI) / 180.0f;
    // ── PX4 state ─────────────────────────────────────────────────────────────
    VehicleStatus        status_{};
    VehicleLocalPosition pos_{};
    Odometry             nav_odom_{};
    float drone_pitch_rad_ = 0.0f;  // from VehicleAttitude — used for depth camera floor filter
    float att_q_w_ = 1.0f;
    float att_q_x_ = 0.0f;
    float att_q_y_ = 0.0f;
    float att_q_z_ = 0.0f;

    // ── Parameters ────────────────────────────────────────────────────────────
    float target_alt_{}, v_max_{}, max_accel_{};
    float v_max_ramp_step_{}, v_max_target_{};
    float xi_{}, xi_z_{}, eta_{}, rho0_{};
    float rho0_base_{};          // rho0 at initial v_max; scales up with speed
    float drone_radius_m_{};
    float voxel_alt_band_m_{};
    float north_min_{}, north_max_{}, east_min_{}, east_max_{};
    float scan_done_frac_{}, goal_radius_{};
    float arm_timeout_s_{}, search_timeout_s_{}, hold_target_s_{};
    float target_revisit_radius_m_{};
    float target_confirm_radius_m_{};
    float altitude_tolerance_m_{};
    float climb_rate_{}, return_alt_{};
    float home_north_m_{}, home_east_m_{};
    float ceiling_clearance_m_{};
    float ceiling_headroom_tolerance_m_{};
    float floor_clearance_m_{}, floor_recovery_climb_m_{};
    float frontier_goal_yaw_capture_radius_m_{};
    bool  frontier_arrival_look_enabled_ = true;
    float frontier_arrival_look_s_ = 3.0f;
    float frontier_arrival_look_half_angle_rad_ = 35.0f * static_cast<float>(M_PI) / 180.0f;
    bool  scan_enabled_{};

    // Scan layers
    float    scan_alt_start_{};
    float    scan_layer_step_{};
    bool     scan_follow_path_altitude_ = false;
    float    layer_settle_s_{};
    float    layer_dwell_s_{};
    float    layer_stagnation_s_{};
    float    layer_complete_frac_{};
    float    layer_stagnation_min_frac_{};
    float    layer_timeout_min_frac_{};
    float    layer_complete_stable_s_ = 8.0f;
    uint32_t layer_complete_max_reachable_frontier_cells_ = 120;
    float    min_layer_dwell_s_{};
    float    no_frontier_layer_grace_s_{};
    int      layer_index_start_ = 0;
    int      layer_index_stride_ = 1;
    float    waypoint_lookahead_radius_m_ = 0.85f;
    float    waypoint_lookahead_cos_ = 0.25f;
    float    waypoint_lookahead_near_obstacle_m_ = 0.90f;
    float    waypoint_lookahead_near_radius_m_ = 0.55f;
    float    waypoint_lookahead_near_cos_ = 0.80f;
    float    blocked_waypoint_reject_radius_m_ = 0.80f;
    float    blocked_waypoint_reject_s_ = 10.0f;

    // Forward obstacle avoidance
    float    fwd_stop_m_{};
    float    fwd_stop_m_base_{};     // fwd_stop_m at initial v_max; scales up with speed
    float    fwd_resume_margin_m_{};
    float    fwd_decel_m_{};         // distance at which forward deceleration begins
    float    align_threshold_{};
    float    yaw_slew_alpha_{};
    float    yaw_rate_limit_rad_s_{};
    bool     holonomic_vfh_ = true;
    bool     disable_vfh_ = false;
    bool     camera_yaw_track_goal_ = true;
    float    camera_yaw_slowdown_rad_ = 0.90f;
    float    camera_yaw_stop_rad_ = 1.80f;
    float    camera_yaw_min_speed_scale_ = 0.25f;
    float    hold_scan_yaw_rate_rad_s_ = 0.0f;
    float    depth_roi_width_frac_ = 0.60f;
    float    depth_percentile_ = 0.0f;
    float    depth_min_world_elev_rad_ = -15.0f * static_cast<float>(M_PI) / 180.0f;
    float    depth_max_world_elev_rad_ = 40.0f * static_cast<float>(M_PI) / 180.0f;
    bool     use_nav_pose_ = true;
    float    nav_pose_timeout_s_ = 1.0f;
    std::string nav_pose_frame_id_{"map_ned"};
    std::string px4_goal_frame_id_{"odom"};
    bool     peer_avoidance_enabled_ = false;
    std::string shared_summary_topic_{};
    std::string peer_local_position_topic_{};
    float    own_spawn_north_m_ = 0.0f;
    float    own_spawn_east_m_ = 0.0f;
    float    peer_spawn_north_m_ = 0.0f;
    float    peer_spawn_east_m_ = 0.0f;
    float    peer_min_separation_m_ = 1.50f;
    float    peer_avoidance_gain_ = 0.60f;
    float    peer_pose_timeout_s_ = 1.0f;
    float    peer_avoidance_z_tolerance_m_ = 1.5f;

    // Mode 1 (vertical takeoff → optional lookaround)
    bool     takeoff_lookaround_enabled_ = false;
    float    takeoff_yaw_rate_     = 0.0f;
    float    takeoff_map_settle_s_ = 4.0f;
    float    yaw_accumulated_      = 0.0f;
    float    yaw_prev_heading_     = 0.0f;
    TakeoffPhase takeoff_phase_    = TakeoffPhase::CLIMB;
    uint32_t takeoff_settle_ticks_ = 0;
    float    takeoff_z_cmd_        = 0.0f;

    // Ceiling guard — hover + 360° scan in place instead of returning home
    bool  ceiling_scan_active_    = false;
    float ceiling_scan_yaw_accum_ = 0.0f;
    float ceiling_scan_yaw_prev_  = 0.0f;
    uint32_t ceiling_scan_ticks_  = 0;
    // Total time the drone has wanted to climb but a ceiling blocked it. Accumulates
    // across ceiling-scan re-triggers; reset when headroom recovers. If it exceeds
    // ceiling_escape_timeout_s the drone genuinely cannot climb higher here (and the
    // open-sky seek failed), so it stops spinning forever and returns home.
    uint32_t ceiling_stuck_ticks_ = 0;
    float    ceiling_escape_timeout_s_ = 45.0f;
    // Deferred climb: the layer is done but a local ceiling (balcony/shelf) blocks the
    // vertical climb here. Instead of a blind horizontal escape (which slid the drone
    // along under the same ceiling), keep scanning this layer and retry the climb once
    // the drone is under open sky. best_headroom_* records the most-open spot seen so
    // the drone can steer there if it has no frontier route left to follow.
    bool  climb_deferred_        = false;
    float best_headroom_range_m_ = -1.0f;
    float best_headroom_n_       = 0.0f;
    float best_headroom_e_       = 0.0f;
    uint32_t frontier_arrival_look_ticks_left_ = 0;
    uint32_t frontier_arrival_look_total_ticks_ = 0;
    float    frontier_arrival_look_center_yaw_ = 0.0f;

    // ── Sensor / map data ──────────────────────────────────────────────────────
    // 2-D horizontal LiDAR (primary obstacle sensor)
    float lidar_fwd_range_   = 10.0f;
    float lidar_right_range_ = 10.0f;
    float lidar_left_range_  = 10.0f;
    float lidar_nearest_range_ = 10.0f;
    float lidar_nearest_angle_rad_ = 0.0f;
    bool  have_lidar_        = false;
    // Full LiDAR scan for VFH (all rays used for sector-based obstacle mapping)
    LaserScan lidar_scan_{};
    float     lidar_scan_yaw_  = 0.0f;
    bool      have_lidar_scan_ = false;
    float     lidar_rho0_m_    = 2.5f;
    float     lidar_eta_       = 2.0f;  // kept for parameter compat; VFH uses rho0 only
    float     lidar_forward_angle_rad_ = 0.0f;
    float     lidar_forward_half_angle_rad_ = static_cast<float>(M_PI) / 3.0f;
    bool      narrow_passage_slowdown_enabled_ = true;
    float     narrow_passage_clearance_m_ = 0.85f;
    float     narrow_passage_resume_clearance_m_ = 1.35f;
    float     narrow_passage_min_speed_scale_ = 0.25f;
    bool      nearest_obstacle_slowdown_enabled_ = true;
    float     nearest_obstacle_slowdown_m_ = 0.90f;
    float     nearest_obstacle_resume_m_ = 1.50f;
    float     nearest_obstacle_min_speed_scale_ = 0.12f;
    float     nearest_obstacle_approach_guard_m_ = 0.70f;
    bool      nearest_obstacle_guard_enabled_ = true;
    bool      nearest_obstacle_centering_enabled_ = true;
    float     nearest_obstacle_centering_m_ = 0.95f;
    float     nearest_obstacle_centering_max_mps_ = 0.35f;
    // VFH sector blocked state — persists between ticks for hysteresis
    mutable std::array<bool, 72> vfh_blocked_{};
    // Set true by compute_vfh when every sector is blocked; cleared when any sector is free.
    // Used by SCAN state to detect and recover from sustained stuck-in-corner situations.
    mutable bool  vfh_all_blocked_     = false;
    mutable float vfh_min_clearance_   = 2.0f;  // clearance of the VFH-chosen sector; gated by kCreepStopRange
    mutable float vfh_passage_clearance_ = 2.0f; // side/valley clearance around the chosen VFH direction
    mutable float vfh_speed_cap_mps_ = 0.0f;     // local cap applied in narrow passages
    mutable float vfh_nearest_speed_cap_mps_ = 0.0f;

    float coverage_fraction_ = 0.0f;
    float layer_coverage_fraction_ = 0.0f;
    float summary_layer_altitude_m_ = 0.0f;
    uint32_t summary_layer_total_cells_ = 0;
    float shared_coverage_fraction_ = 0.0f;
    float shared_layer_coverage_fraction_ = 0.0f;
    float shared_summary_layer_altitude_m_ = 0.0f;
    uint32_t shared_summary_layer_total_cells_ = 0;
    bool have_shared_summary_ = false;
    bool have_shared_layer_summary_ = false;
    uint32_t frontier_clusters_ = 0;
    uint32_t reachable_frontier_clusters_ = 0;
    uint32_t reachable_frontier_cells_ = 0;
    bool frontier_route_available_ = false;
    bool have_layer_summary_ = false;
    float entropy_mean_      = 0.0f;
    float entropy_north_     = 0.0f;
    float entropy_east_      = 0.0f;
    float entropy_down_      = 0.0f;  // NED z of entropy centroid (negative = above ground)
    bool  have_entropy_      = false;
    float forward_range_     = 10.0f;  // effective obstacle distance (lidar ∩ depth)
    float forward_range_raw_min_ = 10.0f;
    uint32_t forward_range_samples_ = 0;
    bool  have_fwd_range_    = false;
    // Down-pitched depth camera forward range, kept separately so the loop can
    // fuse it with the horizontal lidar (min of both). The 2-D lidar only sees a
    // thin plane at body height; the depth camera catches low ledges/steps ahead
    // that the lidar misses — the cause of "legs clip a step the lidar called clear".
    float depth_fwd_range_   = 10.0f;
    uint32_t depth_fwd_samples_ = 0;
    int64_t last_depth_fwd_ns_ = 0;
    bool  depth_forward_fusion_enabled_ = true;
    float depth_forward_timeout_s_ = 0.5f;
    uint32_t depth_forward_min_samples_ = 6;  // ignore 1–2 px flicker; need real structure
    bool  depth_low_obstacle_enabled_ = true;
    float depth_low_obstacle_max_m_ = 1.5f;   // only trust close below-horizon returns
    float depth_floor_reject_frac_ = 0.80f;   // d > floor_dist·frac → it's the floor
    float ceiling_range_m_   = std::numeric_limits<float>::infinity();
    bool  have_ceiling_range_ = false;
    float floor_range_m_     = std::numeric_limits<float>::infinity();
    bool  have_floor_range_  = false;
    VehicleLocalPosition peer_pos_{};
    bool have_peer_pos_ = false;
    int64_t last_peer_pos_ns_ = 0;
    bool     target_found_        = false;
    uint32_t target_confirm_count_ = 0;   // 2 nearby detections required before inspection
    float    target_north_        = 0.0f;
    float    target_east_         = 0.0f;
    bool     target_candidate_valid_ = false;
    float    target_candidate_north_ = 0.0f;
    float    target_candidate_east_  = 0.0f;
    struct InspectedTarget {
        float north;
        float east;
        uint32_t observations;
    };
    std::vector<InspectedTarget> inspected_targets_{};
    // Legacy ORBIT state. TargetFound now uses TARGET_INSPECT, not blind orbit.
    float    orbit_angle_rad_    = 0.0f;   // current orbital angle in NED (atan2(E,N) from target)
    float    orbit_progress_rad_ = 0.0f;   // accumulated angle traversed (done when >= 2π)
    static constexpr float kOrbitRadius_m_  = 3.0f;
    static constexpr float kOrbitSpeed_mps_ = 0.35f;
    int64_t  last_entropy_ns_       = 0;
    bool     have_nav_odom_ = false;
    int64_t  last_nav_odom_ns_ = 0;
    std::string entropy_frame_id_;

    // ── Waypoint path queues (replace direct frontier_goal subscription) ──────
    // voxel_mapper publishes A* paths. The controller latches one active route
    // at a time, follows its waypoints in order, and lets VFH handle local safety.
    struct WayPt {
        float north, east, yaw, alt_ned;  // alt_ned: NED z (<0 = above ground)
        bool  has_yaw;
    };
    std::deque<WayPt>  waypoint_queue_{};
    std::string        waypoint_frame_id_{"odom"};
    float              last_path_end_n_ = 0.0f;  // endpoint of last committed path
    float              last_path_end_e_ = 0.0f;
    bool               blocked_scan_waypoint_valid_ = false;
    float              blocked_scan_waypoint_n_ = 0.0f;
    float              blocked_scan_waypoint_e_ = 0.0f;
    std::string        blocked_scan_waypoint_frame_id_{};
    int64_t            blocked_scan_waypoint_ns_ = 0;
    std::deque<WayPt>  return_waypoint_queue_{};
    std::string        return_waypoint_frame_id_{"odom"};

    // ── SCAN state ────────────────────────────────────────────────────────────
    float    yaw_cmd_           = 0.0f;
    float    scan_alt_          = 0.0f;
    float    prev_coverage_     = 0.0f;
    int      scan_layer_index_   = 0;
    uint32_t layer_ticks_       = 0;
    uint32_t stagnation_ticks_  = 0;
    uint32_t no_frontier_ticks_ = 0;
    uint32_t layer_complete_stable_ticks_ = 0;
    uint32_t transit_ticks_        = 0;
    float    transit_timeout_s_    = 20.0f;
    float    committed_goal_n_     = 0.0f;
    float    committed_goal_e_     = 0.0f;
    float    committed_goal_yaw_   = 0.0f;
    bool     committed_goal_has_yaw_ = false;
    bool     committed_goal_valid_ = false;
    std::string committed_goal_frame_id_{};
    std::string committed_goal_source_{"center"};
    float    goal_best_dist_       = std::numeric_limits<float>::infinity();
    uint32_t goal_no_progress_ticks_ = 0;
    float    goal_progress_timeout_s_ = 8.0f;
    float    goal_progress_epsilon_m_ = 0.25f;
    float    active_goal_blocked_range_m_ = 1.00f;
    float    route_blocked_range_m_ = 0.90f;
    float    route_blocked_half_angle_rad_ = 18.0f * static_cast<float>(M_PI) / 180.0f;
    bool     route_conflict_replan_enabled_ = true;
    float    route_conflict_range_m_ = 1.25f;
    float    route_conflict_angle_rad_ = 55.0f * static_cast<float>(M_PI) / 180.0f;
    float    route_conflict_replan_s_ = 0.8f;
    uint32_t route_conflict_ticks_ = 0;
    bool     local_apf_repulsion_enabled_ = true;
    float    local_apf_repulsion_range_m_ = 1.10f;
    float    local_apf_repulsion_gain_ = 0.22f;
    float    local_apf_repulsion_max_mps_ = 0.65f;
    // Counts consecutive ticks where VFH finds no free sector (all blocked).
    // Forces immediate goal replan after kVfhStuckThreshTicks to break corner deadlocks.
    // FOV limit: sectors outside ±vfh_fov_half_sectors_ from the drone's current
    // heading are treated as free (range_max). 0 = full 360° (disabled).
    int      vfh_fov_half_sectors_ = 0;
    uint32_t vfh_all_blocked_ticks_ = 0;
    float    emergency_stop_range_m_ = 0.30f;
    float    emergency_stop_base_    = 0.30f;  // scales up with speed
    // Emergency hover: counts consecutive ticks the forward range is below threshold.
    // We hover (not DISARM) to prevent fatal free-fall. DISARM kills motors mid-flight.
    uint32_t emergency_hover_ticks_ = 0;
    // Velocity damping gain K_d (from Lecture 12 slide 18: τ = -∇U - K_d·q̇).
    // Applied as v_cmd = v_vfh - kd_vel_damp_ * v_actual to implement Lyapunov dissipation.
    float    kd_vel_damp_    = 0.18f;

    // ── Velocity state ────────────────────────────────────────────────────────
    float cmd_vn_         = 0.0f;
    float cmd_ve_         = 0.0f;
    float cmd_vz_         = 0.0f;  // NED z velocity from 3D APF (negative = up)
    float fwd_speed_cmd_  = 0.0f;  // slewed scalar forward speed for unicycle SCAN
    bool  in_drive_phase_ = false;  // hysteresis state: enter<12°, exit>18°

    // ── TEST_FORWARD mode ─────────────────────────────────────────────────────
    bool     test_forward_mode_   = false;
    bool     test_skip_rotation_  = false;
    float    test_forward_dist_m_ = 2.0f;
    float    test_wall_stop_m_    = 1.0f;  // stop Phase 3 when fwd < this
    float    test_hold_s_         = 2.0f;  // Phase 2 hold duration
    float    test_goal_radius_m_  = 0.20f;
    float    test_alt_tol_m_      = 0.15f;
    bool     test_ignore_depth_until_phase3_ = false;
    bool     test_phase3_enabled_ = true;
    bool     test_lidar_nearest_stop_ = true;
    int      test_phase_          = 0;
    float    test_fwd_yaw_        = 0.0f;  // heading locked at test start
    float    test_start_n_        = 0.0f;  // position at test start
    float    test_start_e_        = 0.0f;
    float    test_target_n_       = 0.0f;  // 2m waypoint NED North
    float    test_target_e_       = 0.0f;  // 2m waypoint NED East
    float    test_hold_n_         = 0.0f;  // position hold target after phase transitions
    float    test_hold_e_         = 0.0f;
    uint32_t test_hold_ticks_     = 0;     // settle timer for phase 2
    uint32_t test_phase1_ticks_   = 0;     // elapsed ticks for phase 1 logging

    // ── FSM ───────────────────────────────────────────────────────────────────
    State    state_       = State::PRIMING;
    uint32_t state_ticks_ = 0;
    uint32_t scan_ticks_  = 0;

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::Publisher<OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  sp_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;
    rclcpp::Publisher<String>::SharedPtr              result_pub_;
    rclcpp::Publisher<String>::SharedPtr              state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vfh_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_vfh_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cmd_yaw_pub_;
    rclcpp::Publisher<PointStamped>::SharedPtr            committed_goal_pub_;
    rclcpp::Publisher<Path>::SharedPtr                    active_path_pub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr peer_pos_sub_;
    rclcpp::Subscription<String>::SharedPtr               dashboard_cmd_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr             nav_pose_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr      attitude_sub_;
    rclcpp::Subscription<LaserScan>::SharedPtr            lidar_sub_;
    rclcpp::Subscription<Image>::SharedPtr                depth_sub_;
    rclcpp::Subscription<Range>::SharedPtr                ceiling_sub_;
    rclcpp::Subscription<Range>::SharedPtr                floor_sub_;
    rclcpp::Subscription<PointStamped>::SharedPtr         entropy_sub_;
    rclcpp::Subscription<Path>::SharedPtr                 frontier_path_sub_;
    rclcpp::Subscription<Path>::SharedPtr                 return_path_sub_;
    rclcpp::Subscription<MapUpdateSummary>::SharedPtr     summary_sub_;
    rclcpp::Subscription<MapUpdateSummary>::SharedPtr     shared_summary_sub_;
    rclcpp::Subscription<TargetFound>::SharedPtr          target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Helpers ───────────────────────────────────────────────────────────────
    uint64_t timestamp_us() const
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static const char* takeoff_phase_name(TakeoffPhase phase)
    {
        switch (phase) {
        case TakeoffPhase::CLIMB: return "TAKEOFF_CLIMB";
        case TakeoffPhase::LOOKAROUND: return "TAKEOFF_LOOKAROUND";
        case TakeoffPhase::SETTLE: return "TAKEOFF_SETTLE";
        }
        return "TAKEOFF";
    }

    const char* state_name() const
    {
        switch (state_) {
        case State::PRIMING: return "PRIMING";
        case State::REQUESTING: return "REQUESTING";
        case State::TAKEOFF: return takeoff_phase_name(takeoff_phase_);
        case State::SCAN: return "SCAN";
        case State::TARGET_INSPECT: return "TARGET_INSPECT";
        case State::ORBIT: return "ORBIT";
        case State::RETURN: return "RETURN";
        case State::LANDING: return "LANDING";
        case State::DONE: return "DONE";
        case State::TEST_FORWARD: return "TEST_FORWARD";
        }
        return "UNKNOWN";
    }

    bool target_near_inspected(float north, float east) const
    {
        for (const auto &target : inspected_targets_) {
            if (std::hypot(north - target.north, east - target.east) <=
                target_revisit_radius_m_) {
                return true;
            }
        }
        return false;
    }

    bool merge_inspected_target(float north, float east)
    {
        for (auto &target : inspected_targets_) {
            if (std::hypot(north - target.north, east - target.east) <=
                target_revisit_radius_m_) {
                const float old_weight =
                    static_cast<float>(std::min<uint32_t>(target.observations, 8));
                const float denom = old_weight + 1.0f;
                target.north = (target.north * old_weight + north) / denom;
                target.east  = (target.east  * old_weight + east)  / denom;
                ++target.observations;
                return true;
            }
        }
        return false;
    }

    void remember_inspected_target(float north, float east)
    {
        if (!merge_inspected_target(north, east)) {
            inspected_targets_.push_back({north, east, 1});
        }
    }

    String mission_done_message(const std::string &reason) const
    {
        String msg;
        if (inspected_targets_.empty()) {
            msg.data = "TARGET NOT FOUND (" + reason + ")";
        } else {
            msg.data = "SCAN COMPLETE (" +
                std::to_string(inspected_targets_.size()) +
                " target(s) inspected, " + reason + ")";
        }
        return msg;
    }

    void publish_state() const
    {
        if (!state_pub_) {
            return;
        }
        String msg;
        msg.data = state_name();
        state_pub_->publish(msg);
    }

    void transition(State next)
    {
        state_ticks_          = 0;
        emergency_hover_ticks_ = 0;   // don't carry emergency counter across states
        committed_goal_valid_ = false;
        goal_best_dist_       = std::numeric_limits<float>::infinity();
        goal_no_progress_ticks_ = 0;
        vfh_all_blocked_ticks_ = 0;
        if (next == State::RETURN) {
            waypoint_queue_.clear();
            publish_active_scan_path();
        } else if (next == State::SCAN) {
            return_waypoint_queue_.clear();
        }
        state_                = next;
        publish_state();
    }

    float first_scan_altitude() const
    {
        const int first_layer = std::max(0, layer_index_start_);
        return scan_alt_start_ + static_cast<float>(first_layer) * scan_layer_step_;
    }

    float effective_coverage() const
    {
        return std::clamp(
            have_shared_summary_ ? shared_coverage_fraction_ : coverage_fraction_,
            0.0f, 1.0f);
    }

    float layer_coverage() const
    {
        if (have_shared_layer_summary_ && shared_summary_layer_total_cells_ > 0) {
            const float max_alt_delta =
                std::max(0.75f, scan_layer_step_ * 0.75f);
            if (std::fabs(shared_summary_layer_altitude_m_ - scan_alt_) <= max_alt_delta) {
                return std::clamp(shared_layer_coverage_fraction_, 0.0f, 1.0f);
            }
        }
        if (have_layer_summary_ && summary_layer_total_cells_ > 0) {
            const float max_alt_delta =
                std::max(0.75f, scan_layer_step_ * 0.75f);
            if (std::fabs(summary_layer_altitude_m_ - scan_alt_) <= max_alt_delta) {
                return std::clamp(layer_coverage_fraction_, 0.0f, 1.0f);
            }
        }
        return effective_coverage();
    }

    bool shared_layer_coverage_valid() const
    {
        if (!have_shared_layer_summary_ || shared_summary_layer_total_cells_ == 0) {
            return false;
        }
        const float max_alt_delta =
            std::max(0.75f, scan_layer_step_ * 0.75f);
        return std::fabs(shared_summary_layer_altitude_m_ - scan_alt_) <= max_alt_delta;
    }

    bool local_layer_coverage_valid() const
    {
        if (!have_layer_summary_ || summary_layer_total_cells_ == 0) {
            return false;
        }
        const float max_alt_delta =
            std::max(0.75f, scan_layer_step_ * 0.75f);
        return std::fabs(summary_layer_altitude_m_ - scan_alt_) <= max_alt_delta;
    }

    float local_layer_coverage() const
    {
        if (local_layer_coverage_valid()) {
            return std::clamp(layer_coverage_fraction_, 0.0f, 1.0f);
        }
        return layer_coverage();
    }

    float transition_layer_coverage() const
    {
        // In swarm mode, layer completion is a property of the shared map, not
        // of an individual drone. If no shared layer summary for this altitude
        // has arrived yet, do not fall back to local coverage and climb early.
        if (!shared_summary_topic_.empty()) {
            if (shared_layer_coverage_valid()) {
                return std::clamp(shared_layer_coverage_fraction_, 0.0f, 1.0f);
            }
            return 0.0f;
        }
        return local_layer_coverage();
    }

    bool is_armed()    const { return status_.arming_state == VehicleStatus::ARMING_STATE_ARMED; }
    bool is_offboard() const { return status_.nav_state   == VehicleStatus::NAVIGATION_STATE_OFFBOARD; }

    bool nav_pose_fresh() const
    {
        if (!use_nav_pose_ || !have_nav_odom_ || last_nav_odom_ns_ <= 0 || nav_pose_timeout_s_ <= 0.0f) {
            return false;
        }
        const int64_t age_ns = now().nanoseconds() - last_nav_odom_ns_;
        return age_ns >= 0 &&
               age_ns <= static_cast<int64_t>(nav_pose_timeout_s_ * 1.0e9f);
    }

    float active_north() const
    {
        return nav_pose_fresh()
            ? static_cast<float>(nav_odom_.pose.pose.position.x)
            : pos_.x;
    }

    float active_east() const
    {
        return nav_pose_fresh()
            ? static_cast<float>(nav_odom_.pose.pose.position.y)
            : pos_.y;
    }

    float active_heading() const
    {
        if (nav_pose_fresh()) {
            const auto &q = nav_odom_.pose.pose.orientation;
            return yaw_from_xyzw(
                static_cast<float>(q.x),
                static_cast<float>(q.y),
                static_cast<float>(q.z),
                static_cast<float>(q.w));
        }
        return current_heading();
    }

    float active_vn() const
    {
        return nav_pose_fresh()
            ? static_cast<float>(nav_odom_.twist.twist.linear.x)
            : pos_.vx;
    }

    float active_ve() const
    {
        return nav_pose_fresh()
            ? static_cast<float>(nav_odom_.twist.twist.linear.y)
            : pos_.vy;
    }

    bool active_xy_valid() const
    {
        return nav_pose_fresh() || pos_.xy_valid;
    }

    float control_north() const { return pos_.x; }
    float control_east()  const { return pos_.y; }
    float control_heading() const { return current_heading(); }
    float control_vn() const { return pos_.vx; }
    float control_ve() const { return pos_.vy; }
    bool control_xy_valid() const { return pos_.xy_valid; }

    bool at_altitude(float z_ned) const
    {
        return pos_.z_valid && std::fabs(pos_.z - z_ned) < altitude_tolerance_m_;
    }

    float current_heading() const
    {
        return std::isfinite(pos_.heading) ? pos_.heading : 0.0f;
    }

    float slew_yaw_cmd_toward(float desired_yaw)
    {
        const float err = wrap_pi(desired_yaw - yaw_cmd_);
        float step = err * yaw_slew_alpha_;
        if (yaw_rate_limit_rad_s_ > 0.0f) {
            const float max_step = yaw_rate_limit_rad_s_ * kLoopPeriod_s;
            step = std::clamp(step, -max_step, max_step);
        }
        yaw_cmd_ = wrap_pi(yaw_cmd_ + step);
        return yaw_cmd_;
    }

    static void clamp_xy_norm(float &vn, float &ve, float max_norm)
    {
        const float norm = std::hypot(vn, ve);
        if (norm > max_norm && norm > 1e-3f) {
            const float scale = max_norm / norm;
            vn *= scale;
            ve *= scale;
        }
    }

    void slew_xy_velocity(float target_vn, float target_ve, float max_delta_v)
    {
        float dvn = target_vn - cmd_vn_;
        float dve = target_ve - cmd_ve_;
        const float delta = std::hypot(dvn, dve);
        if (delta > max_delta_v && delta > 1e-4f) {
            const float scale = max_delta_v / delta;
            dvn *= scale;
            dve *= scale;
        }
        cmd_vn_ += dvn;
        cmd_ve_ += dve;
    }

    bool peer_pose_fresh() const
    {
        if (!peer_avoidance_enabled_ || !have_peer_pos_ || last_peer_pos_ns_ <= 0) {
            return false;
        }
        const int64_t age_ns = now().nanoseconds() - last_peer_pos_ns_;
        return age_ns >= 0 &&
               age_ns <= static_cast<int64_t>(peer_pose_timeout_s_ * 1.0e9f);
    }

    void apply_peer_avoidance(float &target_vn, float &target_ve, const char *phase)
    {
        if (!peer_pose_fresh() || !pos_.xy_valid) {
            return;
        }
        if (pos_.z_valid && peer_pos_.z_valid &&
            std::fabs(pos_.z - peer_pos_.z) > peer_avoidance_z_tolerance_m_) {
            return;
        }

        const float own_n = control_north() + own_spawn_north_m_;
        const float own_e = control_east()  + own_spawn_east_m_;
        const float peer_n = peer_pos_.x + peer_spawn_north_m_;
        const float peer_e = peer_pos_.y + peer_spawn_east_m_;
        const float dn = own_n - peer_n;
        const float de = own_e - peer_e;
        const float dist = std::hypot(dn, de);
        if (dist <= 1e-3f || dist >= peer_min_separation_m_) {
            return;
        }

        const float away_n = dn / dist;
        const float away_e = de / dist;
        const float approach_v =
            target_vn * (-away_n) + target_ve * (-away_e);
        if (approach_v > 0.0f) {
            target_vn += approach_v * away_n;
            target_ve += approach_v * away_e;
        }

        const float repel =
            peer_avoidance_gain_ * v_max_ *
            std::clamp((peer_min_separation_m_ - dist) /
                       std::max(peer_min_separation_m_, 0.1f), 0.0f, 1.0f);
        target_vn += repel * away_n;
        target_ve += repel * away_e;

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s peer avoidance: peer_dist=%.2fm < %.2fm  add=(%.2f,%.2f)",
            phase, dist, peer_min_separation_m_, repel * away_n, repel * away_e);
    }

    float desired_camera_yaw_to_goal(float goal_n, float goal_e,
                                     float current_n, float current_e,
                                     float fallback_yaw) const
    {
        if (!camera_yaw_track_goal_) {
            return fallback_yaw;
        }
        const float dn = goal_n - current_n;
        const float de = goal_e - current_e;
        if (std::hypot(dn, de) < 0.20f) {
            return fallback_yaw;
        }
        return wrap_pi(std::atan2(de, dn));
    }

    float camera_yaw_speed_scale(float desired_yaw, float current_yaw) const
    {
        if (!camera_yaw_track_goal_) {
            return 1.0f;
        }
        const float slow = std::max(0.05f, camera_yaw_slowdown_rad_);
        const float stop = std::max(slow + 0.05f, camera_yaw_stop_rad_);
        const float err = angle_error_abs(current_yaw, desired_yaw);
        if (err <= slow) {
            return 1.0f;
        }
        if (err >= stop) {
            return std::clamp(camera_yaw_min_speed_scale_, 0.0f, 1.0f);
        }
        const float t = (err - slow) / (stop - slow);
        const float min_scale = std::clamp(camera_yaw_min_speed_scale_, 0.0f, 1.0f);
        return 1.0f - t * (1.0f - min_scale);
    }

    float narrow_passage_speed_cap(float passage_clearance_m, float block_range_m) const
    {
        if (!narrow_passage_slowdown_enabled_ || !std::isfinite(passage_clearance_m)) {
            return v_max_;
        }
        const float min_scale =
            std::clamp(narrow_passage_min_speed_scale_, 0.05f, 1.0f);
        const float slow_clearance =
            std::max(block_range_m + 0.05f, narrow_passage_clearance_m_);
        const float resume_clearance =
            std::max(slow_clearance + 0.05f, narrow_passage_resume_clearance_m_);

        float scale = 1.0f;
        if (passage_clearance_m <= slow_clearance) {
            scale = min_scale;
        } else if (passage_clearance_m < resume_clearance) {
            const float t = (passage_clearance_m - slow_clearance) /
                            (resume_clearance - slow_clearance);
            scale = min_scale + std::clamp(t, 0.0f, 1.0f) * (1.0f - min_scale);
        }
        return std::clamp(v_max_ * scale, 0.05f, v_max_);
    }

    float nearest_obstacle_speed_cap() const
    {
        if (!nearest_obstacle_slowdown_enabled_ || !have_lidar_ ||
            !std::isfinite(lidar_nearest_range_)) {
            return v_max_;
        }

        const float min_scale =
            std::clamp(nearest_obstacle_min_speed_scale_, 0.05f, 1.0f);
        const float slow_m = std::max(0.05f, nearest_obstacle_slowdown_m_);
        const float resume_m = std::max(slow_m + 0.05f, nearest_obstacle_resume_m_);

        float scale = 1.0f;
        if (lidar_nearest_range_ <= slow_m) {
            scale = min_scale;
        } else if (lidar_nearest_range_ < resume_m) {
            const float t = (lidar_nearest_range_ - slow_m) / (resume_m - slow_m);
            scale = min_scale + std::clamp(t, 0.0f, 1.0f) * (1.0f - min_scale);
        }
        return std::clamp(v_max_ * scale, 0.05f, v_max_);
    }

    void apply_nearest_obstacle_component_guard(float &target_vn, float &target_ve,
                                                const char *phase)
    {
        if (!nearest_obstacle_guard_enabled_ || !have_lidar_ ||
            !std::isfinite(lidar_nearest_range_)) {
            return;
        }
        if (std::hypot(target_vn, target_ve) < 1e-4f) {
            return;
        }

        const float obs_yaw = wrap_pi(lidar_scan_yaw_ + lidar_nearest_angle_rad_);
        const float obs_n = std::cos(obs_yaw);
        const float obs_e = std::sin(obs_yaw);

        // The guard range must cover the physical braking distance: commands are
        // ramped at max_accel, so stripping the approach component at a fixed
        // range leaves v²/2a of coasting. At v=1.3 m/s, a=1.0 m/s² that is
        // 0.85 m — a fixed 1.10 m guard let the props reach the wall (fwd=0.24 m
        // crash, 2026-07-02). Use the current ramped command as the closing-speed
        // proxy and extend the guard to radius + margin + braking distance.
        const float closing = std::max(0.0f, cmd_vn_ * obs_n + cmd_ve_ * obs_e);
        const float accel = std::max(0.2f, max_accel_);
        const float dyn_guard =
            drone_radius_m_ + 0.30f + (closing * closing) / (2.0f * accel);
        const float guard_m =
            std::max({0.05f, nearest_obstacle_approach_guard_m_, dyn_guard});
        if (lidar_nearest_range_ > guard_m) {
            return;
        }

        const float approach = target_vn * obs_n + target_ve * obs_e;
        if (approach <= 0.0f) {
            return;
        }

        target_vn -= approach * obs_n;
        target_ve -= approach * obs_e;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s nearest-obstacle guard: nearest=%.2fm < %.2fm  removed approach %.2fm/s",
            phase, lidar_nearest_range_, guard_m, approach);
    }

    void apply_nearest_obstacle_centering(float &target_vn, float &target_ve,
                                          const char *phase)
    {
        if (!nearest_obstacle_centering_enabled_ || !have_lidar_ ||
            !std::isfinite(lidar_nearest_range_)) {
            return;
        }

        const float center_m = std::max(0.05f, nearest_obstacle_centering_m_);
        if (lidar_nearest_range_ >= center_m) {
            return;
        }

        const float obs_yaw = wrap_pi(lidar_scan_yaw_ + lidar_nearest_angle_rad_);
        const float away_n = -std::cos(obs_yaw);
        const float away_e = -std::sin(obs_yaw);
        const float strength =
            std::clamp((center_m - lidar_nearest_range_) / center_m, 0.0f, 1.0f);
        const float add =
            std::clamp(nearest_obstacle_centering_max_mps_, 0.0f, v_max_) * strength;
        if (add <= 1e-4f) {
            return;
        }

        target_vn += add * away_n;
        target_ve += add * away_e;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s nearest-obstacle centering: nearest=%.2fm < %.2fm  add=(%.2f,%.2f)",
            phase, lidar_nearest_range_, center_m, add * away_n, add * away_e);
    }

    void apply_local_apf_repulsion(float &target_vn, float &target_ve,
                                   const char *phase)
    {
        if (!local_apf_repulsion_enabled_ || !have_lidar_ ||
            !std::isfinite(lidar_nearest_range_)) {
            return;
        }

        const float rho0 = std::max(0.10f, local_apf_repulsion_range_m_);
        const float rho = std::max(0.05f, lidar_nearest_range_);
        if (rho >= rho0) {
            return;
        }

        const float obs_yaw = wrap_pi(lidar_scan_yaw_ + lidar_nearest_angle_rad_);
        const float away_n = -std::cos(obs_yaw);
        const float away_e = -std::sin(obs_yaw);
        const float mag = std::clamp(
            local_apf_repulsion_gain_ * (1.0f / rho - 1.0f / rho0) / (rho * rho),
            0.0f,
            std::max(0.0f, std::min(local_apf_repulsion_max_mps_, v_max_)));
        if (mag <= 1e-4f) {
            return;
        }

        target_vn += mag * away_n;
        target_ve += mag * away_e;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "%s local APF repulsion: nearest=%.2fm < %.2fm  add=(%.2f,%.2f)",
            phase, lidar_nearest_range_, rho0, mag * away_n, mag * away_e);
    }

    void advance_hold_scan_yaw()
    {
        if (camera_yaw_track_goal_ && hold_scan_yaw_rate_rad_s_ > 0.0f) {
            float step = hold_scan_yaw_rate_rad_s_ * kLoopPeriod_s;
            if (yaw_rate_limit_rad_s_ > 0.0f) {
                step = std::min(step, yaw_rate_limit_rad_s_ * kLoopPeriod_s);
            }
            yaw_cmd_ = wrap_pi(yaw_cmd_ + step);
        }
    }

    void start_frontier_arrival_look(float center_yaw)
    {
        if (!frontier_arrival_look_enabled_ || frontier_arrival_look_s_ <= 0.0f) {
            return;
        }
        frontier_arrival_look_total_ticks_ = static_cast<uint32_t>(
            std::max(1.0f, frontier_arrival_look_s_ / kLoopPeriod_s));
        frontier_arrival_look_ticks_left_ = frontier_arrival_look_total_ticks_;
        frontier_arrival_look_center_yaw_ =
            std::isfinite(yaw_cmd_) ? yaw_cmd_ : wrap_pi(center_yaw);
        RCLCPP_INFO(get_logger(),
            "SCAN arrival look: %.1fs full spin from yaw %.0f°",
            frontier_arrival_look_s_,
            frontier_arrival_look_center_yaw_ * 180.0f / static_cast<float>(M_PI));
    }

    void hold_frontier_arrival_look()
    {
        if (frontier_arrival_look_ticks_left_ == 0 ||
            frontier_arrival_look_total_ticks_ == 0) {
            return;
        }
        const float elapsed = static_cast<float>(
            frontier_arrival_look_total_ticks_ - frontier_arrival_look_ticks_left_ + 1);
        const float denom = static_cast<float>(
            std::max<uint32_t>(1, frontier_arrival_look_total_ticks_));
        const float t = std::clamp(elapsed / denom, 0.0f, 1.0f);
        yaw_cmd_ = wrap_pi(
            frontier_arrival_look_center_yaw_ +
            2.0f * static_cast<float>(M_PI) * t);
        --frontier_arrival_look_ticks_left_;
        publish_ocm(Mode::VELOCITY);
        publish_velocity_sp(0.0f, 0.0f, calc_vz(scan_alt_), yaw_cmd_);
    }

    bool ceiling_too_close() const
    {
        // Use rangefinder for ceiling (upward-facing, not affected by pitch tilt).
        return have_ceiling_range_ && ceiling_range_m_ < ceiling_clearance_m_;
    }

    bool floor_too_close() const
    {
        // Prefer EKF2 altitude: immune to rangefinder tilt artifacts.
        // Rangefinder fallback for pre-EKF2-convergence edge cases.
        if (pos_.z_valid) return (-pos_.z) < floor_clearance_m_;
        return have_floor_range_ && floor_range_m_ < floor_clearance_m_;
    }

    float lidar_nearest_angle_deg() const
    {
        return lidar_nearest_angle_rad_ * 180.0f / static_cast<float>(M_PI);
    }

    // World-frame bearing of the most open lidar direction: the candidate
    // heading whose ±20° window has the largest minimum clearance. Escaping
    // opposite the single nearest return deadlocks in corners — the nearest
    // wall flips between the two corner walls every tick and the escape
    // vector oscillates in place. The widest-window direction points into
    // the room instead.
    float most_open_escape_dir(bool &ok) const
    {
        ok = false;
        if (!have_lidar_scan_ || lidar_scan_.ranges.empty()) {
            return 0.0f;
        }
        const auto &s = lidar_scan_;
        const size_t n = s.ranges.size();
        constexpr float kWin  = 20.0f * static_cast<float>(M_PI) / 180.0f;
        constexpr float kStep = 5.0f * static_cast<float>(M_PI) / 180.0f;
        float best_score = -1.0f;
        float best_angle = 0.0f;
        for (float a = -static_cast<float>(M_PI); a < static_cast<float>(M_PI);
             a += kStep) {
            float window_min = std::numeric_limits<float>::infinity();
            bool any = false;
            for (size_t i = 0; i < n; ++i) {
                const float ai =
                    s.angle_min + static_cast<float>(i) * s.angle_increment;
                if (angle_error_abs(ai, a) > kWin) continue;
                float r = s.ranges[i];
                if (!std::isfinite(r) || r > s.range_max) r = s.range_max;
                if (r < s.range_min) continue;
                window_min = std::min(window_min, r);
                any = true;
            }
            if (!any || !std::isfinite(window_min)) continue;
            if (window_min > best_score) {
                best_score = window_min;
                best_angle = a;
            }
        }
        if (best_score <= 0.0f) {
            return 0.0f;
        }
        ok = true;
        return wrap_pi(lidar_scan_yaw_ + best_angle);
    }

    bool lidar_ray_hits_floor_or_ceiling(float scan_angle_rad, float range_m) const
    {
        if (!std::isfinite(range_m) || range_m <= 0.0f) {
            return false;
        }

        const Vec3f ray_body{std::cos(scan_angle_rad), std::sin(scan_angle_rad), 0.0f};
        const Vec3f ray_world = quat_rotate_wxyz(
            att_q_w_, att_q_x_, att_q_y_, att_q_z_, ray_body);
        const float vertical = ray_world.z;
        const float match_tol = std::max(0.12f, 0.08f * range_m);

        if (pos_.z_valid && vertical > 0.05f) {
            const float agl = std::max(0.0f, -pos_.z);
            if (agl > 0.05f) {
                const float floor_hit = agl / vertical;
                if (std::fabs(range_m - floor_hit) <= match_tol) {
                    return true;
                }
            }
        }

        if (have_ceiling_range_ && std::isfinite(ceiling_range_m_) && vertical < -0.05f) {
            const float ceiling_hit = ceiling_range_m_ / (-vertical);
            if (std::fabs(range_m - ceiling_hit) <= match_tol) {
                return true;
            }
        }

        return false;
    }

    float test_obstacle_range() const
    {
        if (test_lidar_nearest_stop_ && have_lidar_) {
            return std::min(forward_range_, lidar_nearest_range_);
        }
        return forward_range_;
    }

    float lidar_range_toward_world(float world_bearing_rad, float half_angle_rad) const
    {
        if (!have_lidar_scan_ || lidar_scan_.ranges.empty()) {
            return std::numeric_limits<float>::infinity();
        }
        const float max_range = std::isfinite(lidar_scan_.range_max)
            ? lidar_scan_.range_max
            : lidar_rho0_m_;
        float min_range = max_range;
        for (size_t i = 0; i < lidar_scan_.ranges.size(); ++i) {
            const float r = lidar_scan_.ranges[i];
            if (!std::isfinite(r) ||
                r < lidar_scan_.range_min ||
                r > lidar_scan_.range_max) {
                continue;
            }
            const float scan_angle =
                lidar_scan_.angle_min + static_cast<float>(i) * lidar_scan_.angle_increment;
            if (lidar_ray_hits_floor_or_ceiling(scan_angle, r)) {
                continue;
            }
            const float ray_world = wrap_pi(lidar_scan_yaw_ + scan_angle);
            if (angle_error_abs(ray_world, world_bearing_rad) <= half_angle_rad) {
                min_range = std::min(min_range, r);
            }
        }
        return min_range;
    }

    struct Goal2D {
        float north;
        float east;
        float yaw;
        bool has_yaw;
        const char* source;
        std::string frame_id;
        bool valid = true;  // false → no safe goal; caller must hold position
    };

    Goal2D project_goal_to_control_frame(const Goal2D &goal) const
    {
        Goal2D projected = goal;
        if (!control_xy_valid()) {
            return projected;
        }
        if (goal.frame_id != nav_pose_frame_id_ || !nav_pose_fresh()) {
            return projected;
        }

        const float nav_n = static_cast<float>(nav_odom_.pose.pose.position.x);
        const float nav_e = static_cast<float>(nav_odom_.pose.pose.position.y);
        const auto &q = nav_odom_.pose.pose.orientation;
        const float nav_yaw = yaw_from_xyzw(
            static_cast<float>(q.x),
            static_cast<float>(q.y),
            static_cast<float>(q.z),
            static_cast<float>(q.w));

        const float rel_dist = std::hypot(goal.north - nav_n, goal.east - nav_e);
        const float rel_bearing_map = std::atan2(goal.east - nav_e, goal.north - nav_n);
        const float delta_yaw = wrap_pi(control_heading() - nav_yaw);
        const float rel_bearing_local = wrap_pi(rel_bearing_map + delta_yaw);

        projected.north = control_north() + rel_dist * std::cos(rel_bearing_local);
        projected.east  = control_east()  + rel_dist * std::sin(rel_bearing_local);
        if (projected.has_yaw) {
            projected.yaw = wrap_pi(projected.yaw + delta_yaw);
        }
        projected.frame_id = px4_goal_frame_id_;
        return projected;
    }

    // Returns the next navigation goal.
    // The controller intentionally has a single route source: the A*-planned
    // waypoint queue from voxel_mapper. Entropy remains telemetry/scoring input
    // for the mapper, never a direct controller fallback.
    Goal2D pick_scan_goal() const
    {
        if (!waypoint_queue_.empty()) {
            const auto &wp = waypoint_queue_.front();
            return {wp.north, wp.east, wp.yaw, wp.has_yaw, "waypoint", waypoint_frame_id_};
        }
        return {0.0f, 0.0f, 0.0f, false, "none", px4_goal_frame_id_, false};
    }

    bool route_safety_stressed() const
    {
        if (!have_lidar_) {
            return false;
        }
        const bool blocked_ahead =
            std::isfinite(forward_range_) &&
            forward_range_ < active_goal_blocked_range_m_;
        const bool no_progress =
            goal_no_progress_ticks_ >
            static_cast<uint32_t>(std::max(1.0f, 0.75f * goal_progress_timeout_s_ / kLoopPeriod_s));
        return blocked_ahead || vfh_all_blocked_ || no_progress;
    }

    void remember_blocked_scan_waypoint(const Goal2D &goal)
    {
        if (std::string(goal.source) != "waypoint") {
            return;
        }
        blocked_scan_waypoint_valid_ = true;
        blocked_scan_waypoint_n_ = goal.north;
        blocked_scan_waypoint_e_ = goal.east;
        blocked_scan_waypoint_frame_id_ = goal.frame_id;
        blocked_scan_waypoint_ns_ = now().nanoseconds();
    }

    bool recent_blocked_scan_waypoint(const WayPt &wp, const std::string &frame_id)
    {
        if (!blocked_scan_waypoint_valid_ || blocked_waypoint_reject_s_ <= 0.0f) {
            return false;
        }
        const int64_t age_ns = now().nanoseconds() - blocked_scan_waypoint_ns_;
        if (age_ns < 0 ||
            age_ns > static_cast<int64_t>(blocked_waypoint_reject_s_ * 1.0e9f)) {
            blocked_scan_waypoint_valid_ = false;
            return false;
        }
        if (frame_id != blocked_scan_waypoint_frame_id_) {
            return false;
        }
        const float reject_radius = std::max(0.05f, blocked_waypoint_reject_radius_m_);
        return std::hypot(wp.north - blocked_scan_waypoint_n_,
                          wp.east  - blocked_scan_waypoint_e_) < reject_radius;
    }

    bool should_lookahead_waypoint(const std::deque<WayPt> &queue,
                                   const std::string &frame_id,
                                   float current_n,
                                   float current_e) const
    {
        if (have_lidar_) {
            const bool blocked_ahead =
                std::isfinite(forward_range_) &&
                forward_range_ < active_goal_blocked_range_m_ + 0.10f;
            const bool close_to_any_obstacle =
                std::isfinite(lidar_nearest_range_) &&
                lidar_nearest_range_ < waypoint_lookahead_near_obstacle_m_;
            if (blocked_ahead || close_to_any_obstacle) {
                return false;
            }
        }

        float lookahead_radius = waypoint_lookahead_radius_m_;
        float lookahead_cos = waypoint_lookahead_cos_;

        if (queue.size() < 2 || lookahead_radius <= goal_radius_) {
            return false;
        }

        const auto &front = queue.front();
        const auto &next = queue[1];
        const Goal2D front_goal = project_goal_to_control_frame({
            front.north, front.east, front.yaw, front.has_yaw, "waypoint", frame_id,
        });
        const Goal2D next_goal = project_goal_to_control_frame({
            next.north, next.east, next.yaw, next.has_yaw, "waypoint", frame_id,
        });

        const float f_n = front_goal.north - current_n;
        const float f_e = front_goal.east  - current_e;
        const float n_n = next_goal.north  - front_goal.north;
        const float n_e = next_goal.east   - front_goal.east;
        const float front_dist = std::hypot(f_n, f_e);
        if (front_dist >= lookahead_radius) {
            return false;
        }

        const float next_seg_len = std::hypot(n_n, n_e);
        if (front_dist < 1e-3f || next_seg_len < 1e-3f) {
            return true;
        }

        const float dir_cos = (f_n * n_n + f_e * n_e) / (front_dist * next_seg_len);
        return dir_cos >= lookahead_cos;
    }

    void publish_active_scan_path()
    {
        Path msg;
        msg.header.stamp = now();
        msg.header.frame_id =
            waypoint_frame_id_.empty() ? px4_goal_frame_id_ : waypoint_frame_id_;
        for (const auto &wp : waypoint_queue_) {
            PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = wp.north;
            ps.pose.position.y = wp.east;
            ps.pose.position.z = (wp.alt_ned < -0.1f) ? wp.alt_ned
                                 : (pos_.z_valid ? pos_.z : -scan_alt_);
            if (wp.has_yaw) {
                ps.pose.orientation.z = std::sin(wp.yaw * 0.5f);
                ps.pose.orientation.w = std::cos(wp.yaw * 0.5f);
            } else {
                ps.pose.orientation.w = 1.0;
            }
            msg.poses.push_back(ps);
        }
        active_path_pub_->publish(msg);
    }

    Goal2D pick_return_goal() const
    {
        if (!return_waypoint_queue_.empty()) {
            const auto &wp = return_waypoint_queue_.front();
            return {wp.north, wp.east, wp.yaw, wp.has_yaw,
                    "return_waypoint", return_waypoint_frame_id_};
        }
        return {home_north_m_, home_east_m_, 0.0f, false, "none", px4_goal_frame_id_, false};
    }

    void publish_ocm(Mode mode)
    {
        OffboardControlMode msg{};
        msg.timestamp = timestamp_us();
        msg.position  = (mode == Mode::POSITION);
        msg.velocity  = (mode == Mode::VELOCITY);
        ocm_pub_->publish(msg);
    }

    void publish_position_sp(float north, float east, float z, float yaw)
    {
        TrajectorySetpoint msg{};
        msg.timestamp    = timestamp_us();
        msg.position     = {north, east, z};
        msg.velocity     = {kNaN, kNaN, kNaN};
        msg.acceleration = {kNaN, kNaN, kNaN};
        msg.yaw          = yaw;
        msg.yawspeed     = kNaN;
        sp_pub_->publish(msg);
    }

    // Pure velocity setpoint — all three components explicit (NED frame).
    // vz: negative = climbing, positive = descending (NED convention).
    // Use calc_vz(target_agl) for altitude hold / transit.
    void publish_vfh_markers(float desired_yaw, float cmd_yaw)
    {
        if (vfh_markers_pub_->get_subscription_count() == 0) {
            return;
        }
        using Marker = visualization_msgs::msg::Marker;
        using MarkerArray = visualization_msgs::msg::MarkerArray;

        const float nx = pos_.x;
        const float ey = pos_.y;
        const float zz = pos_.z;  // NED z (negative = above ground)

        auto make_arrow = [&](int id, float yaw, float length,
                              float r, float g, float b) {
            Marker m;
            m.header.stamp    = now();
            m.header.frame_id = "odom";
            m.ns   = "vfh";
            m.id   = id;
            m.type = Marker::ARROW;
            m.action = Marker::ADD;
            m.scale.x = 0.05;   // shaft diameter
            m.scale.y = 0.10;   // arrowhead diameter
            m.scale.z = 0.15;   // arrowhead length
            m.color.r = r;  m.color.g = g;  m.color.b = b;  m.color.a = 0.9f;
            m.lifetime = rclcpp::Duration(0, 300'000'000);  // 0.3 s auto-expire

            geometry_msgs::msg::Point p0, p1;
            p0.x = static_cast<double>(nx);
            p0.y = static_cast<double>(ey);
            p0.z = static_cast<double>(zz);
            p1.x = static_cast<double>(nx + length * std::cos(yaw));
            p1.y = static_cast<double>(ey + length * std::sin(yaw));
            p1.z = static_cast<double>(zz);
            m.points.push_back(p0);
            m.points.push_back(p1);
            return m;
        };

        MarkerArray ma;
        // Yellow: raw VFH heading (where the histogram wants to go)
        ma.markers.push_back(make_arrow(0, desired_yaw, 1.5f, 1.0f, 1.0f, 0.0f));
        // Cyan: smoothed command heading (what the drone actually tracks)
        ma.markers.push_back(make_arrow(1, cmd_yaw,     2.0f, 0.0f, 1.0f, 1.0f));
        vfh_markers_pub_->publish(ma);
    }

    void publish_velocity_sp(float vn, float ve, float vz, float yaw)
    {
        TrajectorySetpoint msg{};
        msg.timestamp    = timestamp_us();
        msg.position     = {kNaN, kNaN, kNaN};
        msg.velocity     = {vn,   ve,   vz};
        msg.acceleration = {kNaN, kNaN, kNaN};
        msg.yaw          = yaw;
        msg.yawspeed     = kNaN;
        sp_pub_->publish(msg);
    }

    // Altitude P-controller for use with publish_velocity_sp.
    // Returns NED vz (negative = up) to reach target_agl (metres above ground).
    float calc_vz(float target_agl) const
    {
        if (!pos_.z_valid) return 0.0f;
        const float agl = -pos_.z;                             // current AGL (m)
        const float err = target_agl - agl;                    // positive = need to climb
        const float vz  = -std::clamp(err * kZKp_, -kZMax_, kZMax_);
        // NED: negative vz = upward motion
        return vz;
    }

    void send_command(uint16_t cmd, float p1 = 0.0f, float p2 = 0.0f)
    {
        VehicleCommand msg{};
        msg.timestamp        = timestamp_us();
        msg.command          = cmd;
        msg.param1           = p1;
        msg.param2           = p2;
        msg.target_system    = 0;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        cmd_pub_->publish(msg);
    }

    void publish_floor_recovery(float desired_alt_m, float yaw)
    {
        const float current_alt_m = pos_.z_valid ? -pos_.z : desired_alt_m;
        const float recovery_alt_m =
            std::max(desired_alt_m, current_alt_m + floor_recovery_climb_m_);
        cmd_vn_ = cmd_ve_ = fwd_speed_cmd_ = 0.0f;
        publish_ocm(Mode::POSITION);
        publish_position_sp(pos_.x, pos_.y, -recovery_alt_m, yaw);
    }

    // Increase v_max_ by one ramp step (called after each successful layer scan).
    // Scales fwd_stop_m_, rho0_, and emergency_stop_range_m_ proportionally so
    // braking distances stay safe at higher speeds.
    void apply_speed_ramp()
    {
        if (v_max_ramp_step_ <= 0.0f) return;
        const float prev = v_max_;
        v_max_ = std::min(v_max_ + v_max_ramp_step_, v_max_target_);
        if (v_max_ <= prev) return;
        // fwd_stop = physics-based braking distance v²/(2a) + 0.4m margin.
        // At 0.5 m/s: 0.25m + 0.4 = 0.65 → clamped to base 0.8m
        // At 1.0 m/s: 1.0m  + 0.4 = 1.4m
        const float brake_dist = (v_max_ * v_max_) / (2.0f * max_accel_) + 0.4f;
        fwd_stop_m_             = std::max(fwd_stop_m_base_,      brake_dist);
        rho0_                   = std::max(rho0_base_,             v_max_ * 0.60f);
        emergency_stop_range_m_ = std::max(emergency_stop_base_,  v_max_ * 0.25f);
        RCLCPP_INFO(get_logger(),
            "SPEED RAMP: v_max %.2f→%.2f m/s  fwd_stop=%.2fm rho0=%.2fm estop=%.2fm",
            prev, v_max_, fwd_stop_m_, rho0_, emergency_stop_range_m_);
    }

    // ── VFH: Vector Field Histogram ──────────────────────────────────────────
    // Ref: Borenstein & Koren, IEEE T. Robotics & Automation 7(3) 1991.
    // Adapted for holonomic quadrotor with 2-D horizontal LiDAR (360°, 1°/ray).
    //
    // Algorithm:
    //   1. Per-sector minimum LiDAR range (72 sectors × 5° = 360°).
    //   2. Block sector + ±kDilate neighbours when min_range < block_range.
    //      block_range = max(rho0_, forward_stop_m_) so VFH respects braking
    //      distance, not just the drone footprint.
    //   3. Hysteresis: stay blocked until clearance > block_range + resume_margin.
    //   4. Search outward from goal bearing for first open valley (≥ kMinV
    //      contiguous free sectors).
    //   5. Steer toward valley center (or exact goal if goal sector is free).
    //   6. Speed = v_max × clamp((min_all − block_range) / safety_ramp).
    //
    // No local minima: the algorithm explicitly selects free space; APF cannot
    // guarantee this because attraction and repulsion forces can cancel.
    struct Force2D { float n, e; };
    Force2D compute_vfh(float goal_n, float goal_e)
    {
        vfh_speed_cap_mps_ = v_max_;
        vfh_passage_clearance_ = lidar_rho0_m_;

        const float current_n = control_north();
        const float current_e = control_east();

        // No LiDAR yet — slow attraction toward goal
        if (!have_lidar_scan_) {
            const float dn = goal_n - current_n;
            const float de = goal_e - current_e;
            const float dist = std::hypot(dn, de);
            if (dist < 0.01f) return {0.f, 0.f};
            return {dn / dist * v_max_ * 0.3f, de / dist * v_max_ * 0.3f};
        }

        static constexpr int   kK      = 72;                           // sectors
        static constexpr float kSRad   = k2Pi / static_cast<float>(kK); // 5° / sector
        static constexpr int   kDilate = 1;  // ±1 sector (±5°) — reduced for doorway traversal
        static constexpr int   kMinV   = 3;  // minimum valley width (≥15°)
        const float block_range = std::min(
            lidar_rho0_m_,
            std::max(rho0_, fwd_stop_m_));
        const float clear_range = std::min(
            lidar_rho0_m_,
            block_range + std::max(0.30f, fwd_resume_margin_m_));

        const float yaw    = lidar_scan_yaw_;
        const size_t nrays = lidar_scan_.ranges.size();

        // Step 1: per-sector minimum range
        std::array<float, kK> sect_min{};
        sect_min.fill(lidar_rho0_m_);
        for (size_t i = 0; i < nrays; ++i) {
            const float d = lidar_scan_.ranges[i];
            if (!std::isfinite(d) || d < 0.05f) continue;
            const float a  = lidar_scan_.angle_min +
                             static_cast<float>(i) * lidar_scan_.angle_increment;
            if (lidar_ray_hits_floor_or_ceiling(a, d)) continue;
            const float aw = wrap_pi(yaw + a);             // world-frame angle (NED)
            int s = static_cast<int>(
                (aw + static_cast<float>(M_PI)) / kSRad) % kK;
            if (s < 0) s += kK;
            sect_min[s] = std::min(sect_min[s], d);
        }

        // Step 1b: camera-FOV mask.
        // When vfh_fov_half_sectors_ > 0, sectors outside ±FOV from the drone's
        // current heading are treated as free (set to lidar_rho0_m_). This simulates
        // a forward-facing depth camera: the drone only reacts to what is directly
        // ahead, preventing all-blocked scenarios when laterally surrounded by pillars.
        if (vfh_fov_half_sectors_ > 0) {
            const float h = wrap_pi(yaw);  // drone heading in world frame (NED)
            int fwd_sec = static_cast<int>(
                (h + static_cast<float>(M_PI)) / kSRad) % kK;
            if (fwd_sec < 0) fwd_sec += kK;
            for (int s = 0; s < kK; ++s) {
                int d = std::abs(s - fwd_sec);
                d = std::min(d, kK - d);  // circular distance
                if (d > vfh_fov_half_sectors_)
                    sect_min[s] = lidar_rho0_m_;
            }
        }

        // Step 2: block sectors closer than the braking-safe range.
        // VFH+ robot-width masking: each blocked sector also masks neighbors within
        // arcsin(drone_radius / obstacle_range) so the chosen direction is geometrically
        // safe — the drone body cannot clip obstacles in adjacent sectors.
        // Static kDilate is the lower bound (never mask less than before).
        std::array<bool, kK> blocked{};
        blocked.fill(false);
        for (int s = 0; s < kK; ++s) {
            if (sect_min[s] < block_range) {
                const float r   = std::max(sect_min[s], 0.01f);
                const float delta = std::asin(std::min(1.0f, drone_radius_m_ / r));
                const int   dil   = std::max(kDilate,
                                        static_cast<int>(std::ceil(delta / kSRad)));
                for (int k = -dil; k <= dil; ++k)
                    blocked[(s + k + kK) % kK] = true;
            }
        }

        // Step 3: hysteresis — stay blocked until enough clearance returns.
        for (int s = 0; s < kK; ++s) {
            if (!blocked[s] && vfh_blocked_[s] && sect_min[s] < clear_range)
                blocked[s] = true;
        }
        vfh_blocked_ = blocked;

        // Step 4: goal bearing and sector
        const float dn        = goal_n - current_n;
        const float de        = goal_e - current_e;
        const float goal_dist = std::hypot(dn, de);
        const float goal_ang  = wrap_pi(std::atan2(de, dn));
        int goal_sec = static_cast<int>(
            (goal_ang + static_cast<float>(M_PI)) / kSRad) % kK;
        if (goal_sec < 0) goal_sec += kK;

        auto circ_dist = [&](int a, int b) -> int {
            int d = std::abs(a - b);
            return std::min(d, kK - d);
        };

        // Valley check: center ± kMinV/2 sectors must all be free
        auto valley_ok = [&](int c) -> bool {
            for (int k = -(kMinV / 2); k <= (kMinV / 2); ++k)
                if (blocked[(c + k + kK) % kK]) return false;
            return true;
        };

        // Step 5: search candidate sectors inside open valleys.
        // Each candidate already guarantees ±kMinV/2 free neighbours.
        // Cost prefers:
        //   1. alignment with the goal
        //   2. alignment with current heading (less oscillation)
        //   3. larger local clearance (less wall hugging)
        int best_sec = -1;
        float best_cost = std::numeric_limits<float>::infinity();
        float best_sec_clear = lidar_rho0_m_;  // clearance of chosen sector (for speed)
        float best_passage_clear = lidar_rho0_m_;
        const float heading_ang = wrap_pi(control_heading());
        int free_candidates = 0;
        for (int s = 0; s < kK; ++s) {
            if (!valley_ok(s)) continue;
            ++free_candidates;

            const float sec_ang = wrap_pi(
                (static_cast<float>(s) + 0.5f) * kSRad - static_cast<float>(M_PI));
            const float goal_cost = static_cast<float>(circ_dist(s, goal_sec));
            const float heading_cost = angle_error_abs(sec_ang, heading_ang) / kSRad;

            float local_clear = lidar_rho0_m_;
            for (int k = -(kMinV / 2); k <= (kMinV / 2); ++k) {
                local_clear = std::min(local_clear, sect_min[(s + k + kK) % kK]);
            }
            const float clear_bonus = std::clamp(local_clear / lidar_rho0_m_, 0.0f, 1.0f);

            // Narrow-passage centering: penalise lateral asymmetry (left vs right at ±90°).
            // Sectors hugging one wall score worse than centred sectors, pulling the
            // drone to the middle of corridors without adding new code paths.
            static constexpr int kQuarter = kK / 4;  // 18 sectors = 90°
            const float left_clear  = sect_min[(s + kQuarter) % kK];
            const float right_clear = sect_min[(s - kQuarter + kK) % kK];
            const float balance_penalty =
                std::fabs(left_clear - right_clear) / lidar_rho0_m_;
            const float passage_clear =
                std::min(local_clear, std::min(left_clear, right_clear));

            const float cost = goal_cost + 0.35f * heading_cost
                             - 2.5f * clear_bonus + 1.5f * balance_penalty;
            if (cost < best_cost) {
                best_cost = cost;
                best_sec = s;
                best_sec_clear = sect_min[s];  // clearance of this exact sector (not valley min)
                best_passage_clear = passage_clear;
                vfh_min_clearance_ = best_sec_clear;
            }
        }

        // All sectors blocked — escape slowly toward the sector with the most clearance.
        // Returning zero here causes infinite hovering in corners; instead we nudge the
        // drone toward the least-obstructed direction so subsequent ticks can open a path.
        if (best_sec < 0) {
            vfh_all_blocked_ = true;
            int esc_sec = 0;
            float max_clr = sect_min[0];
            for (int s = 1; s < kK; ++s) {
                if (sect_min[s] > max_clr) { max_clr = sect_min[s]; esc_sec = s; }
            }
            // Move toward most-open sector (even if still within block_range) at 20% speed.
            const float esc_ang = wrap_pi(
                (static_cast<float>(esc_sec) + 0.5f) * kSRad - static_cast<float>(M_PI));
            const float esc_spd = v_max_ * 0.20f;
            vfh_min_clearance_ = max_clr;  // escape clears kCreepStopRange gate even if small
            vfh_passage_clearance_ = max_clr;
            vfh_nearest_speed_cap_mps_ = esc_spd;
            vfh_speed_cap_mps_ = esc_spd;
            return {esc_spd * std::cos(esc_ang), esc_spd * std::sin(esc_ang)};
        }
        vfh_all_blocked_ = false;

        // Step 6: steering angle — sector center, or exact goal when the goal is
        // itself well inside a valid valley. This avoids hugging a valley edge.
        float steer = wrap_pi(
            (static_cast<float>(best_sec) + 0.5f) * kSRad - static_cast<float>(M_PI));
        if (free_candidates > 0 && valley_ok(goal_sec) && goal_dist < lidar_rho0_m_)
            steer = goal_ang;

        // Step 7: speed from clearance of the selected sector.
        // vfh_min_clearance_ (set above) is used in the SCAN loop kCreepStopRange gate
        // so creep-stop only fires when the chosen direction itself is obstructed.
        // Zero speed at block_range, full speed at block_range + 1 m.
        const float fwd = std::min(best_sec_clear, block_range + 1.0f);
        const float spd_scale = std::clamp((fwd - block_range) / 1.0f, 0.0f, 1.0f);
        vfh_passage_clearance_ = std::min(best_sec_clear, best_passage_clear);
        vfh_nearest_speed_cap_mps_ = nearest_obstacle_speed_cap();
        vfh_speed_cap_mps_ = std::min(
            narrow_passage_speed_cap(vfh_passage_clearance_, block_range),
            vfh_nearest_speed_cap_mps_);
        float speed = std::min(v_max_ * spd_scale, goal_dist * xi_);
        speed = std::min(speed, vfh_speed_cap_mps_);

        // Dead-zone escape: when obstacle is at exactly block_range, spd_scale=0 and
        // the returned vector magnitude is zero. The unicycle then keeps vfh_heading
        // at old yaw_cmd_ (line: target_speed>0.05?...:yaw_cmd_), so the drone never
        // yaws toward the open direction found by VFH — permanent freeze.
        // Return a tiny creep in the steer direction so the unicycle can yaw-align.
        // The creep is negligible (< 2 cm/s) and is gated off by the 57° yaw threshold
        // so the drone does not advance into the wall; it only rotates toward the valley.
        if (speed < v_max_ * 0.03f) {
            speed = v_max_ * 0.03f;
        }
        speed = std::min(speed, goal_dist * xi_);  // never creep past the goal
        speed = std::min(speed, vfh_speed_cap_mps_);

        return {speed * std::cos(steer), speed * std::sin(steer)};
    }

    // ── Main loop (20 Hz) ──────────────────────────────────────────────────────
    void loop()
    {
        ++state_ticks_;

        // Forward obstacle distance = min(horizontal lidar, down-pitched depth).
        // The 2-D lidar is attitude-independent and floor-filtered, but it only
        // samples a thin plane at body height — it cannot see a low ledge/step
        // ahead that the drone's legs would clip. The depth camera (pitched 15°
        // down, floor returns rejected by its elevation gate) covers exactly that
        // blind band, so fusing the minimum stops the drone for obstacles either
        // sensor sees. Falls back to whichever source is present/fresh.
        if (have_lidar_) {
            float fwd = lidar_fwd_range_;
            const bool depth_fresh =
                depth_forward_fusion_enabled_ &&
                last_depth_fwd_ns_ > 0 &&
                depth_fwd_samples_ >= depth_forward_min_samples_ &&
                (now().nanoseconds() - last_depth_fwd_ns_) <
                    static_cast<int64_t>(depth_forward_timeout_s_ * 1.0e9f);
            if (depth_fresh && std::isfinite(depth_fwd_range_)) {
                fwd = std::min(fwd, depth_fwd_range_);
            }
            forward_range_ = fwd;
            have_fwd_range_ = true;
        }

        const float yaw_now = current_heading();

        switch (state_) {

        // ── ARM + OFFBOARD negotiation ────────────────────────────────────────
        case State::PRIMING:
            publish_ocm(Mode::POSITION);
            publish_position_sp(0.0f, 0.0f, 0.0f, yaw_now);
            if (state_ticks_ >= static_cast<uint32_t>(1.0f / kLoopPeriod_s)) { // 1 second at 20 Hz
                send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
                send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 21196.0f);
                transition(State::REQUESTING);
            }
            break;

        case State::REQUESTING:
            publish_ocm(Mode::POSITION);
            publish_position_sp(pos_.x, pos_.y, pos_.z, yaw_now); // Publish current position to prevent drift during request
            if (state_ticks_ % 10 == 0) {
                send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
                send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 21196.0f);
            }
            if (is_offboard() && is_armed()) {
                RCLCPP_INFO(get_logger(), "Armed + OFFBOARD — starting TAKEOFF");
                transition(State::TAKEOFF);
            }
            if (state_ticks_ > static_cast<uint32_t>(arm_timeout_s_ / kLoopPeriod_s)) {
                RCLCPP_ERROR(get_logger(), "Arm/OFFBOARD timeout — aborting");
                transition(State::DONE);
            }
            break;

        // ── Mode 1: climb to scan_alt; if takeoff_lookaround_enabled → rotate 360° ──
        case State::TAKEOFF: {
            const bool takeoff_ceiling_hold = ceiling_too_close() && pos_.z_valid;
            const float nominal_takeoff_alt =
                test_forward_mode_ ? scan_alt_start_ : first_scan_altitude();
            const float takeoff_alt_cmd = takeoff_ceiling_hold ? -pos_.z : nominal_takeoff_alt;
            const float z_to = -takeoff_alt_cmd;

            if (state_ticks_ == 1) {
                takeoff_z_cmd_     = pos_.z_valid ? pos_.z : 0.0f;
                yaw_cmd_           = yaw_now;
                yaw_prev_heading_  = yaw_now;
                yaw_accumulated_   = 0.0f;
                takeoff_phase_     = TakeoffPhase::CLIMB;
                takeoff_settle_ticks_ = 0;
                RCLCPP_INFO(get_logger(),
                    "TAKEOFF: vertical climb to %.2f m, then %s "
                    "(settle=%.1f s)",
                    takeoff_alt_cmd,
                    takeoff_lookaround_enabled_ ? "360 lookaround" : "direct SCAN handoff",
                    takeoff_map_settle_s_);
                publish_state();
            }

            if (takeoff_z_cmd_ > z_to + 0.01f) {
                takeoff_z_cmd_ -= climb_rate_ * kLoopPeriod_s;
                if (takeoff_z_cmd_ < z_to) takeoff_z_cmd_ = z_to;
            } else if (takeoff_z_cmd_ < z_to - 0.01f) {
                takeoff_z_cmd_ += climb_rate_ * kLoopPeriod_s;
                if (takeoff_z_cmd_ > z_to) takeoff_z_cmd_ = z_to;
            }

            // Emergency hold during TAKEOFF: if any obstacle is dangerously close,
            // freeze the spin and hold position until the coast is clear.
            // This catches wall drift that POSITION mode alone cannot prevent when EKF slips.
            {
                const bool have_safety = have_lidar_ || have_fwd_range_;
                const float nearest    = have_lidar_ ? test_obstacle_range() : forward_range_;
                if (have_safety && nearest < emergency_stop_range_m_) {
                    ++emergency_hover_ticks_;
                    if (emergency_hover_ticks_ % 20 == 1)
                        RCLCPP_WARN(get_logger(),
                            "TAKEOFF obstacle guard: nearest=%.2f m < %.2f m — holding pos",
                            nearest, emergency_stop_range_m_);
                    publish_ocm(Mode::POSITION);
                    publish_position_sp(pos_.x, pos_.y, takeoff_z_cmd_, yaw_cmd_);
                    break;
                }
                emergency_hover_ticks_ = 0;
            }

            if (takeoff_phase_ == TakeoffPhase::CLIMB && at_altitude(z_to)) {
                takeoff_settle_ticks_ = 0;
                yaw_prev_heading_ = current_heading();
                yaw_accumulated_ = 0.0f;
                if (!takeoff_lookaround_enabled_ || test_skip_rotation_) {
                    takeoff_phase_ = TakeoffPhase::SETTLE;
                    RCLCPP_INFO(get_logger(),
                        "TAKEOFF: climb complete at %.2f m — entering brief settle for %.1f s",
                        -pos_.z, takeoff_map_settle_s_);
                } else {
                    takeoff_phase_ = TakeoffPhase::LOOKAROUND;
                    RCLCPP_INFO(get_logger(),
                        "TAKEOFF: climb complete at %.2f m — starting 360° lookaround",
                        -pos_.z);
                }
                publish_state();
            } else if (takeoff_phase_ == TakeoffPhase::LOOKAROUND) {
                const float yaw_h = current_heading();
                yaw_accumulated_ += wrap_pi(yaw_h - yaw_prev_heading_);
                yaw_prev_heading_ = yaw_h;
                yaw_cmd_ = wrap_pi(yaw_cmd_ + takeoff_yaw_rate_ * kLoopPeriod_s);
                if (std::fabs(yaw_accumulated_) >= k2Pi * 0.99f) {
                    takeoff_phase_ = TakeoffPhase::SETTLE;
                    takeoff_settle_ticks_ = 0;
                    RCLCPP_INFO(get_logger(),
                        "TAKEOFF: 360° lookaround done at %.2f m — settling %.0f s",
                        -pos_.z, takeoff_map_settle_s_);
                    publish_state();
                }
            } else if (takeoff_phase_ == TakeoffPhase::SETTLE && at_altitude(z_to)) {
                ++takeoff_settle_ticks_;
            }

            publish_ocm(Mode::POSITION);
            // Hold current estimated position — do NOT command 0,0 which chases
            // a phantom origin when EKF drifts and causes altitude loss.
            publish_position_sp(pos_.x, pos_.y, takeoff_z_cmd_, yaw_cmd_);

            if (state_ticks_ % static_cast<uint32_t>(1.0f / kLoopPeriod_s * 2) == 0) { // Log every 2 seconds
                const char* ph = takeoff_phase_ == TakeoffPhase::CLIMB      ? "climbing"
                               : takeoff_phase_ == TakeoffPhase::LOOKAROUND ? "lookaround"
                                                                             : "settling";
                RCLCPP_INFO(get_logger(),
                    "TAKEOFF [%s]: alt=%.2f/%.2f m  yaw_cmd=%.0f°  settle=%u/%u",
                    ph, -pos_.z, takeoff_alt_cmd,
                    yaw_cmd_ * 180.0f / static_cast<float>(M_PI),
                    takeoff_settle_ticks_,
                    static_cast<uint32_t>(takeoff_map_settle_s_ * 20.f));
                if (takeoff_ceiling_hold) {
                    RCLCPP_WARN(get_logger(),
                        "TAKEOFF ceiling guard active: holding at %.2f m (range_up=%.2f m)",
                        takeoff_alt_cmd, ceiling_range_m_);
                }
            }

            if (at_altitude(z_to) && takeoff_phase_ == TakeoffPhase::SETTLE &&
                takeoff_settle_ticks_ >= static_cast<uint32_t>(takeoff_map_settle_s_ * 20.0f))
            {
                if (test_forward_mode_) {
                    test_phase_        = 1;
                    test_phase1_ticks_ = 0;
                    test_fwd_yaw_      = current_heading();
                    test_start_n_      = pos_.x;
                    test_start_e_      = pos_.y;
                    test_target_n_ = pos_.x + test_forward_dist_m_ * std::cos(test_fwd_yaw_);
                    test_target_e_ = pos_.y + test_forward_dist_m_ * std::sin(test_fwd_yaw_);
                    test_hold_ticks_ = 0;
                    RCLCPP_INFO(get_logger(),
                        "TAKEOFF done → TEST_FORWARD  alt=%.2f m  yaw=%.0f°  "
                        "start=(%.2f,%.2f)  target=(%.2f,%.2f)  dist=%.1f m  "
                        "wall_stop=%.1f m  hold=%.0f s  phase3=%d  "
                        "lidar_fwd=%.2f nearest=%.2f@%.0f°",
                        -pos_.z,
                        test_fwd_yaw_ * 180.0f / static_cast<float>(M_PI),
                        test_start_n_, test_start_e_,
                        test_target_n_, test_target_e_, test_forward_dist_m_,
                        test_wall_stop_m_, test_hold_s_, test_phase3_enabled_ ? 1 : 0,
                        lidar_fwd_range_, lidar_nearest_range_, lidar_nearest_angle_deg());
                    transition(State::TEST_FORWARD);
                } else if (scan_enabled_) {
                    scan_ticks_           = 0;
                    scan_layer_index_     = std::max(0, layer_index_start_);
                    scan_alt_             = first_scan_altitude();
                    yaw_cmd_              = current_heading();
                    layer_ticks_          = 0;
                    stagnation_ticks_     = 0;
                    no_frontier_ticks_    = 0;
                    layer_complete_stable_ticks_ = 0;
                    prev_coverage_        = 0.0f;
                    in_drive_phase_       = false;
                    transit_ticks_        = 0;
                    committed_goal_valid_ = false;
                    goal_best_dist_       = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_ = 0;

                    target_found_          = false;   // discard any detections from the TAKEOFF 360° scan
                    target_confirm_count_  = 0;
                    RCLCPP_INFO(get_logger(),
                        "TAKEOFF done → SCAN  layer=%d alt=%.2f m  cov=%.1f%% lc=%.1f%%",
                        scan_layer_index_, scan_alt_,
                        effective_coverage() * 100.0f, layer_coverage() * 100.0f);
                    transition(State::SCAN);
                } else if (state_ticks_ % static_cast<uint32_t>(1.0f / kLoopPeriod_s * 2) == 0) { // Log every 2 seconds
                    RCLCPP_INFO(get_logger(),
                        "HOVER alt=%.2f m  (scan_enabled=false)", -pos_.z);
                }
            }
            break;
        }

        // ── Mode 2: VFH + frontier/entropy goal selection ───────────────────────
        // Navigation law:
        //   Goal:    frontier viewpoint (preferred) → entropy centroid → box centre
        //   Heading: yaw_cmd_ slews toward the VFH steering direction
        //   Speed:   acceleration-limited slew toward the VFH velocity output
        //   v_cmd:   (vn, ve, vz) in NED; vz from calc_vz(scan_alt_)
        case State::SCAN: {
            ++scan_ticks_;

            // Mission completion is driven by finishing the assigned scan layers
            // (handled in the layer-advance block below). The global coverage
            // fraction is NOT a sufficient return trigger: the shared summary's
            // denominator only counts layers that already have summaries, so
            // early in the mission 83% on layer 0 reads as ~75% "global" and a
            // cov-triggered return ends the mission with layers 2+ unscanned
            // (observed in preseries_check7: return at t≈230 s after layer 0-1).
            if (target_found_) {
                remember_inspected_target(target_north_, target_east_);
                RCLCPP_INFO(get_logger(),
                    "[TARGET FOUND] at (%.2f, %.2f) - inspecting for %.1f s, then resuming scan",
                    target_north_, target_east_, hold_target_s_);
                String result_msg;
                result_msg.data = "TARGET FOUND at (" +
                    std::to_string(target_north_) + ", " +
                    std::to_string(target_east_) + ") - inspecting";
                result_pub_->publish(result_msg);
                target_found_ = false;
                target_confirm_count_ = 0;
                target_candidate_valid_ = false;
                cmd_vn_ = 0.0f;
                cmd_ve_ = 0.0f;
                fwd_speed_cmd_ = 0.0f;
                transition(State::TARGET_INSPECT);
                break;
            }
            if (scan_ticks_ >= static_cast<uint32_t>(search_timeout_s_ / kLoopPeriod_s)) {
                RCLCPP_WARN(get_logger(),
                    "Search timeout safety fuse (%.0f s) — returning home.",
                    search_timeout_s_);
                result_pub_->publish(mission_done_message(
                    "safety_timeout " + std::to_string(static_cast<int>(search_timeout_s_)) + "s"));
                transition(State::RETURN);
                break;
            }

            if (floor_too_close()) {
                publish_floor_recovery(scan_alt_, yaw_now);
                if (state_ticks_ % 20 == 0) {
                    const float agl = pos_.z_valid ? -pos_.z : floor_range_m_;
                    RCLCPP_WARN(get_logger(),
                        "SCAN floor guard: agl=%.2f m < %.2f m (ekf2=%d range_dn=%.2f)",
                        agl, floor_clearance_m_, pos_.z_valid ? 1 : 0, floor_range_m_);
                }
                break;
            }

            const bool wants_climb = pos_.z_valid && (-scan_alt_ < pos_.z - altitude_tolerance_m_);

            const bool climb_blocked_by_ceiling = ceiling_too_close() && wants_climb;
            if (climb_blocked_by_ceiling) {
                climb_deferred_ = true;
                ceiling_scan_active_ = false;
                ceiling_stuck_ticks_ = 0;
                transit_ticks_ = 0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "SCAN climb blocked by overhead structure: range_up=%.2f m < %.2f m "
                    "while targeting %.2f m — holding altitude and navigating horizontally "
                    "toward open sky",
                    ceiling_range_m_, ceiling_clearance_m_, scan_alt_);
            } else {
                ceiling_stuck_ticks_ = 0;
            }

            // ── Emergency hover — forward obstacle inside crash threshold ──
            // Use forward_range_ only (lidar 120° frontal arc), NOT lidar_nearest_range_
            // (360°).  Lateral walls in narrow corridors must NOT trigger emergency hover —
            // VFH handles them at block_range (1.0 m).  Using nearest-360° here caused the
            // drone to freeze inside any corridor narrower than 2×emergency_stop_range_m_.
            // CRITICAL: Never DISARM in flight — that cuts motors and causes free-fall.
            {
                const bool have_safety = have_lidar_ || have_fwd_range_;
                const float nearest    = forward_range_;   // forward arc only
                if (have_safety && nearest < emergency_stop_range_m_) {
                    ++emergency_hover_ticks_;
                    const bool log_now = (emergency_hover_ticks_ % 20 == 1);
                    if (log_now) {
                        RCLCPP_ERROR(get_logger(),
                            "EMERGENCY HOVER [%u ticks]: fwd=%.2f m < %.2f m  "
                            "nearest_all=%.2f m  pos=(%.2f,%.2f,%.2f) — hovering, NOT disarming",
                            emergency_hover_ticks_,
                            nearest, emergency_stop_range_m_, lidar_nearest_range_,
                            pos_.x, pos_.y, pos_.z_valid ? -pos_.z : 0.0f);
                    }
                    cmd_vn_ = cmd_ve_ = fwd_speed_cmd_ = 0.0f;
                    publish_ocm(Mode::VELOCITY);
                    // After 1 s still stuck: escape away from nearest obstacle.
                    constexpr uint32_t kEscapeTicks =
                        static_cast<uint32_t>(1.0f / kLoopPeriod_s);
                    if (emergency_hover_ticks_ > kEscapeTicks) {
                        const float esc = v_max_ * 0.15f;
                        if (have_lidar_) {
                            // Prefer the most open lidar window (corner-safe);
                            // fall back to opposite-of-nearest.
                            bool open_ok = false;
                            float esc_dir = most_open_escape_dir(open_ok);
                            if (!open_ok) {
                                const float aw_near = wrap_pi(lidar_scan_yaw_ + lidar_nearest_angle_rad_);
                                esc_dir = wrap_pi(aw_near + static_cast<float>(M_PI));
                            }
                            publish_velocity_sp(esc * std::cos(esc_dir),
                                               esc * std::sin(esc_dir),
                                               climb_blocked_by_ceiling ? 0.0f : calc_vz(scan_alt_),
                                               current_heading());
                        } else {
                            const float h = current_heading();
                            publish_velocity_sp(-esc * std::cos(h), -esc * std::sin(h),
                                                climb_blocked_by_ceiling ? 0.0f : calc_vz(scan_alt_), h);
                        }
                    } else {
                        publish_velocity_sp(0.0f, 0.0f,
                                            climb_blocked_by_ceiling ? 0.0f : calc_vz(scan_alt_),
                                            current_heading());
                    }
                    break;
                }
            }
            emergency_hover_ticks_ = 0;

            // ── Altitude transit ───────────────────────────────────────────
            // Hold current XY position (POSITION mode) while PX4 climbs/descends
            // to scan_alt_.  The previous omnidirectional APF during transit was
            // the primary crash cause: the drone flew toward a goal at an altitude
            // where the voxel map had no data, ignoring walls entirely.
            if (!climb_blocked_by_ceiling && !at_altitude(-scan_alt_)) {
                ++transit_ticks_;
                const bool transit_timed_out =
                    transit_ticks_ >= static_cast<uint32_t>(transit_timeout_s_ / kLoopPeriod_s);

                if (!transit_timed_out) {
                    cmd_vn_ = cmd_ve_ = fwd_speed_cmd_ = 0.0f;
                    // POSITION mode holds XY while PX4 climbs to target altitude.
                    // VELOCITY(0,0,vz) can slowly drift sideways; POSITION cannot.
                    publish_ocm(Mode::POSITION);
                    publish_position_sp(pos_.x, pos_.y, -scan_alt_, yaw_cmd_);
                    if (state_ticks_ % 20 == 0)
                        RCLCPP_INFO(get_logger(),
                            "SCAN transit [%u/%.0fs]: alt=%.2f→%.2f m  "
                            "ceiling=%.2f m  pos_z_valid=%d",
                            transit_ticks_, transit_timeout_s_,
                            pos_.z_valid ? -pos_.z : 0.0f, scan_alt_,
                            ceiling_range_m_, pos_.z_valid ? 1 : 0);
                    break;
                }

                // Transit timed out — snap scan_alt_ to where we actually are and
                // fall through so this tick still enters the at-altitude path.
                const float current_agl = pos_.z_valid ? std::max(0.0f, -pos_.z) : scan_alt_;
                RCLCPP_WARN(get_logger(),
                    "SCAN transit timeout (%.0f s) — snapping alt %.2f→%.2f m  "
                    "ceiling=%.2f m  pos_z_valid=%d",
                    transit_timeout_s_, scan_alt_, current_agl,
                    ceiling_range_m_, pos_.z_valid ? 1 : 0);
                scan_alt_             = current_agl;
                transit_ticks_        = 0;
                layer_ticks_          = 0;
                stagnation_ticks_     = 0;
                no_frontier_ticks_    = 0;
                layer_complete_stable_ticks_ = 0;
                prev_coverage_        = 0.0f;
                committed_goal_valid_ = false;
                goal_best_dist_       = std::numeric_limits<float>::infinity();
                goal_no_progress_ticks_ = 0;
                fwd_speed_cmd_        = 0.0f;
                cmd_vn_ = cmd_ve_     = 0.0f;
                // Fall through to the at-altitude section.
            }

            // ── At target altitude ─────────────────────────────────────────
            transit_ticks_ = 0;   // clear on successful arrival
            ++layer_ticks_;

            // Settle: hold after altitude change for fresh depth/voxel map slice
            const uint32_t settle_ticks = static_cast<uint32_t>(layer_settle_s_ / kLoopPeriod_s);
            if (layer_ticks_ <= settle_ticks) {
                cmd_vn_ = cmd_ve_ = 0.0f;
                // POSITION mode holds XY precisely while the map slice settles.
                publish_ocm(Mode::POSITION);
                publish_position_sp(pos_.x, pos_.y, -scan_alt_, yaw_cmd_);
                if (layer_ticks_ == settle_ticks)
                    RCLCPP_INFO(get_logger(),
                        "SCAN settle done at %.2f m", scan_alt_);
                else if (layer_ticks_ % static_cast<uint32_t>(1.0f / kLoopPeriod_s) == 0) // Log every 1 second
                    RCLCPP_INFO(get_logger(),
                        "SCAN settling: %.0f/%.0f s at %.2f m",
                        layer_ticks_/20.f, layer_settle_s_, scan_alt_);
                break;
            }

            // ── Layer advance ──────────────────────────────────────────────
            const float local_lc = local_layer_coverage();
            const float lc = transition_layer_coverage();
            if (lc > prev_coverage_ + kCoverageStagnationThreshold) {
                prev_coverage_ = lc;
                stagnation_ticks_ = 0;
            } else {
                ++stagnation_ticks_;
            }

            const bool no_frontier_now =
                !frontier_route_available_ ||
                reachable_frontier_clusters_ == 0 ||
                reachable_frontier_cells_ == 0;
            if (no_frontier_now) {
                ++no_frontier_ticks_;
            } else {
                no_frontier_ticks_ = 0;
            }
            if (lc >= layer_complete_frac_) {
                ++layer_complete_stable_ticks_;
            } else {
                layer_complete_stable_ticks_ = 0;
            }

            const uint32_t min_dwell_thresh =
                static_cast<uint32_t>(std::max(1.0f, min_layer_dwell_s_ / kLoopPeriod_s));
            const uint32_t stagnation_thresh =
                static_cast<uint32_t>(std::max(1.0f, layer_stagnation_s_ / kLoopPeriod_s));
            const uint32_t complete_stable_thresh =
                static_cast<uint32_t>(std::max(1.0f, layer_complete_stable_s_ / kLoopPeriod_s));
            const uint32_t timeout_thresh =
                static_cast<uint32_t>(std::max(1.0f, layer_dwell_s_ / kLoopPeriod_s));
            const uint32_t no_frontier_thresh =
                static_cast<uint32_t>(std::max(1.0f, no_frontier_layer_grace_s_ / kLoopPeriod_s));
            const bool min_dwell_elapsed = layer_ticks_ >= min_dwell_thresh;
            const bool layer_done_by_coverage =
                min_dwell_elapsed &&
                (layer_complete_stable_ticks_ >= complete_stable_thresh);
            const bool layer_done_by_stagnation =
                min_dwell_elapsed &&
                stagnation_ticks_ >= stagnation_thresh;
            // Timeout is diagnostics only. Stagnation is a controlled fallback:
            // if layer coverage improves by <1% for layer_stagnation_s, the
            // remaining frontiers are treated as low-value or unreachable and
            // the drone advances to the next assigned layer. Persistent
            // no-frontier (0 reachable clusters/cells for the whole grace window
            // after min dwell) also advances: slow coverage creep from arrival
            // sweeps can keep resetting stagnation while the drone orbits a
            // layer it can no longer traverse.
            const bool layer_done_by_timeout =
                layer_ticks_ >= timeout_thresh;
            const bool layer_done_by_no_frontier =
                min_dwell_elapsed &&
                no_frontier_ticks_ >= no_frontier_thresh;
            const bool layer_done =
                layer_done_by_coverage || layer_done_by_stagnation ||
                layer_done_by_no_frontier;
            if (min_dwell_elapsed &&
                stagnation_ticks_ == stagnation_thresh &&
                lc < layer_complete_frac_) {
                RCLCPP_INFO(get_logger(),
                    "SCAN layer not complete: shared_lc=%.1f%% < %.1f%% "
                    "(local_debug_lc=%.1f%%) and "
                    "frontier=%u/%u cells=%u remain reachable",
                    lc * 100.0f, layer_complete_frac_ * 100.0f, local_lc * 100.0f,
                    reachable_frontier_clusters_, frontier_clusters_,
                        reachable_frontier_cells_);
            }
            if (!layer_done && layer_done_by_timeout) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                    "SCAN layer hold: ignoring timeout fallback until coverage reaches %.1f%% "
                    "or stagnates for %.0fs (lc=%.1f%%)",
                    layer_complete_frac_ * 100.0f, layer_stagnation_s_,
                    lc * 100.0f);
            }
            if (layer_done) {
                const int requested_next_layer =
                    scan_layer_index_ + layer_index_stride_;
                const float requested_next_alt =
                    scan_alt_start_ +
                    static_cast<float>(requested_next_layer) * scan_layer_step_;
                const char* layer_reason =
                    layer_done_by_coverage ? "coverage" :
                    layer_done_by_no_frontier ? "no_frontier" :
                    layer_done_by_stagnation ? "stagnation" :
                    "timeout";

                const float current_agl =
                    pos_.z_valid ? std::max(0.0f, -pos_.z) : scan_alt_;
                const float climb_delta =
                    std::max(0.0f, requested_next_alt - current_agl);
                const float required_headroom =
                    ceiling_clearance_m_ + std::max(scan_layer_step_, climb_delta);
                const bool enough_headroom =
                    !have_ceiling_range_ ||
                    ceiling_range_m_ + ceiling_headroom_tolerance_m_ >= required_headroom;
                const bool requested_layer_inside_box = requested_next_alt <= target_alt_ + 0.05f;
                if (requested_layer_inside_box) {
                    if (enough_headroom) {
                        if (layer_done) apply_speed_ramp();  // reward full coverage with more speed
                        scan_alt_             = requested_next_alt;
                        scan_layer_index_     = requested_next_layer;
                        layer_ticks_          = 0;
                        stagnation_ticks_     = 0;
                        no_frontier_ticks_    = 0;
                        layer_complete_stable_ticks_ = 0;
                        prev_coverage_        = 0.0f;
                        fwd_speed_cmd_        = 0.0f;
                        transit_ticks_        = 0;
                        climb_deferred_       = false;
                        // Keep best_headroom_* as a running max across layers (NOT reset):
                        // during the climb transit to this layer the ceiling seek needs a
                        // known open-sky XY, and the reset would blank it exactly then.
                        committed_goal_valid_ = false;
                        goal_best_dist_       = std::numeric_limits<float>::infinity();
                        goal_no_progress_ticks_ = 0;
                        vfh_all_blocked_ticks_ = 0;

                        RCLCPP_INFO(get_logger(),
                            "SCAN layer -> idx=%d alt=%.2f m  (%s shared_lc=%.1f%% local_debug_lc=%.1f%% "
                            "frontier=%u/%u cells=%u)",
                            scan_layer_index_, scan_alt_, layer_reason,
                            lc * 100.f, local_lc * 100.f,
                            reachable_frontier_clusters_, frontier_clusters_,
                            reachable_frontier_cells_);
                        break;
                    }

                    // Local ceiling (balcony/shelf) blocks the climb HERE. A blind
                    // horizontal escape slid the drone along under the same ceiling.
                    // Defer instead: keep the layer "done", do NOT spin or reset the
                    // counters, and fall through to normal navigation so exploration
                    // relocates the drone. The enough_headroom branch above advances
                    // the layer automatically once the drone reaches open sky.
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "SCAN layer %d done but climb to %.2f m blocked: range_up=%.2f m < "
                        "required %.2f m — deferring climb, scanning for open sky",
                        scan_layer_index_, requested_next_alt,
                        ceiling_range_m_, required_headroom);
                    climb_deferred_ = true;
                    waypoint_queue_.clear();
                    publish_active_scan_path();
                    committed_goal_valid_ = false;
                    goal_best_dist_ = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    // (no break — continue to navigation below)
                } else {
                    const float global_cov = effective_coverage();
                    result_pub_->publish(mission_done_message(
                        "all scan layers complete, cov=" +
                        std::to_string(static_cast<int>(global_cov * 100.0f)) +
                        "%, layer_cov=" +
                        std::to_string(static_cast<int>(lc * 100.0f)) + "%"));
                    RCLCPP_INFO(get_logger(),
                        "SCAN complete: top layer done (idx=%d alt=%.2f m, %s lc=%.1f%%, "
                        "global cov=%.1f%%) — returning home",
                        scan_layer_index_, scan_alt_, layer_reason, lc * 100.f,
                        global_cov * 100.0f);
                    transition(State::RETURN);
                    break;
                }
            }

            // ── Route following: mapper owns the path; controller follows its front waypoint ──
            auto hold_scan_position = [&]() {
                cmd_vn_ = 0.0f;
                cmd_ve_ = 0.0f;
                fwd_speed_cmd_ = 0.0f;
                advance_hold_scan_yaw();
                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(0.0f, 0.0f, calc_vz(scan_alt_), yaw_cmd_);
            };

            if (frontier_arrival_look_ticks_left_ > 0) {
                cmd_vn_ = 0.0f;
                cmd_ve_ = 0.0f;
                fwd_speed_cmd_ = 0.0f;
                hold_frontier_arrival_look();
                break;
            }

            const float current_n = control_north();
            const float current_e = control_east();

            // Remember the most open-sky spot seen on this layer, so a deferred climb
            // can steer back to it if the drone runs out of frontiers to follow.
            if (have_ceiling_range_ && ceiling_range_m_ > best_headroom_range_m_) {
                best_headroom_range_m_ = ceiling_range_m_;
                best_headroom_n_ = current_n;
                best_headroom_e_ = current_e;
            }

            auto seek_open_sky_for_deferred_climb = [&]() -> bool {
                if (!climb_deferred_ || !have_ceiling_range_) {
                    return false;
                }
                if (best_headroom_range_m_ <= ceiling_range_m_ + 0.30f) {
                    return false;
                }
                const float dseek = std::hypot(best_headroom_n_ - current_n,
                                               best_headroom_e_ - current_e);
                if (dseek <= goal_radius_) {
                    return false;
                }

                Force2D sk = disable_vfh_
                    ? Force2D{(best_headroom_n_ - current_n) / dseek *
                                  std::min(v_max_, dseek),
                              (best_headroom_e_ - current_e) / dseek *
                                  std::min(v_max_, dseek)}
                    : compute_vfh(best_headroom_n_, best_headroom_e_);
                float svn = sk.n;
                float sve = sk.e;
                apply_local_apf_repulsion(svn, sve, "OPENSKY");
                clamp_xy_norm(svn, sve, std::min(v_max_, vfh_speed_cap_mps_));
                slew_xy_velocity(svn, sve, max_accel_ * kLoopPeriod_s);
                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(cmd_vn_, cmd_ve_,
                                    climb_blocked_by_ceiling ? 0.0f : calc_vz(scan_alt_),
                                    yaw_cmd_);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "SCAN deferred-climb override: seeking open sky at (%.1f,%.1f) "
                    "range_up=%.2f best=%.2f dist=%.1fm",
                    best_headroom_n_, best_headroom_e_,
                    ceiling_range_m_, best_headroom_range_m_, dseek);
                return true;
            };

            auto drop_reached_scan_waypoints = [&]() -> size_t {
                size_t dropped = 0;
                while (!waypoint_queue_.empty()) {
                    const auto &wp = waypoint_queue_.front();
                    const bool final_waypoint = waypoint_queue_.size() == 1;
                    const Goal2D wp_goal = project_goal_to_control_frame({
                        wp.north, wp.east, wp.yaw, wp.has_yaw,
                        "waypoint", waypoint_frame_id_,
                    });
                    const float wp_dist = std::hypot(wp_goal.north - current_n,
                                                     wp_goal.east  - current_e);
                    const bool reached = wp_dist < goal_radius_;
                    const bool smooth_lookahead =
                        !final_waypoint &&
                        !reached &&
                        should_lookahead_waypoint(
                            waypoint_queue_, waypoint_frame_id_, current_n, current_e);
                    if (!reached && !smooth_lookahead) {
                        break;
                    }
                    RCLCPP_INFO(get_logger(),
                        "SCAN waypoint %s (%.2f,%.2f) dist=%.2fm — %zu remaining",
                        reached ? "reached" : "lookahead",
                        wp.north, wp.east, wp_dist, waypoint_queue_.size() - 1);
                    if (reached && final_waypoint) {
                        const float look_yaw = wp_goal.has_yaw ? wp_goal.yaw : current_heading();
                        start_frontier_arrival_look(look_yaw);
                    }
                    waypoint_queue_.pop_front();
                    ++dropped;
                }
                if (dropped > 0) {
                    publish_active_scan_path();
                    committed_goal_valid_   = false;
                    goal_best_dist_         = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_  = 0;
                }
                return dropped;
            };
            drop_reached_scan_waypoints();
            if (frontier_arrival_look_ticks_left_ > 0) {
                cmd_vn_ = 0.0f;
                cmd_ve_ = 0.0f;
                fwd_speed_cmd_ = 0.0f;
                hold_frontier_arrival_look();
                break;
            }

            // Early endpoint: when on the last waypoint and within 1 m, clear the
            // queue so voxel_mapper plans the next goal before the drone stops.
            if (waypoint_queue_.size() == 1) {
                const auto &last_wp = waypoint_queue_.front();
                const Goal2D last_wp_goal = project_goal_to_control_frame({
                    last_wp.north, last_wp.east, last_wp.yaw, last_wp.has_yaw,
                    "waypoint", waypoint_frame_id_,
                });
                const float last_dist = std::hypot(last_wp_goal.north - current_n,
                                                   last_wp_goal.east  - current_e);
                (void)last_dist;
            }

            if (seek_open_sky_for_deferred_climb()) {
                break;
            }

            const auto raw_goal = pick_scan_goal();
            if (!raw_goal.valid) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "SCAN no planned route available — holding for mapper");
                committed_goal_valid_   = false;
                goal_best_dist_         = std::numeric_limits<float>::infinity();
                goal_no_progress_ticks_ = 0;
                vfh_all_blocked_ticks_  = 0;
                hold_scan_position();
                break;
            }

            const Goal2D goal = project_goal_to_control_frame({
                raw_goal.north, raw_goal.east, raw_goal.yaw,
                raw_goal.has_yaw, raw_goal.source, raw_goal.frame_id,
            });
            const float goal_n = goal.north;
            const float goal_e = goal.east;
            const float goal_dist = std::hypot(goal_n - current_n, goal_e - current_e);

            if (goal_dist < goal_radius_) {
                if (waypoint_queue_.size() <= 1 && goal.has_yaw) {
                    start_frontier_arrival_look(goal.yaw);
                }
                drop_reached_scan_waypoints();
                hold_scan_position();
                break;
            }

            const float goal_bearing = wrap_pi(std::atan2(goal_e - current_e,
                                                          goal_n - current_n));
            const float nose_to_goal = angle_error_abs(current_heading(), goal_bearing);
            const float route_range = have_lidar_scan_
                ? lidar_range_toward_world(goal_bearing, route_blocked_half_angle_rad_)
                : std::numeric_limits<float>::infinity();
            const float route_block_range =
                std::max(active_goal_blocked_range_m_, route_blocked_range_m_);
            const bool route_blocked =
                !disable_vfh_ &&
                have_lidar_scan_ &&
                std::isfinite(route_range) &&
                goal_dist > goal_radius_ + 0.20f &&
                route_range < std::min(route_block_range, goal_dist - 0.15f);
            const bool nose_blocked =
                !disable_vfh_ &&
                have_lidar_ &&
                forward_range_ < active_goal_blocked_range_m_ &&
                nose_to_goal < 0.75f;
            if (nose_blocked || route_blocked) {
                RCLCPP_WARN(get_logger(),
                    "SCAN active waypoint blocked: front=%.2fm route=%.2fm "
                    "(limits front=%.2fm route=%.2fm, bearing_err=%.0f°) toward [%s] "
                    "(%.2f,%.2f), dist=%.2fm — dropping active waypoint",
                    forward_range_, route_range,
                    active_goal_blocked_range_m_, route_block_range,
                    nose_to_goal * 180.0f / static_cast<float>(M_PI),
                    raw_goal.source, raw_goal.north, raw_goal.east, goal_dist);
                remember_blocked_scan_waypoint(raw_goal);
                if (!waypoint_queue_.empty()) {
                    waypoint_queue_.pop_front();
                } else {
                    waypoint_queue_.clear();
                }
                publish_active_scan_path();
                committed_goal_valid_   = false;
                goal_best_dist_         = std::numeric_limits<float>::infinity();
                goal_no_progress_ticks_ = 0;
                vfh_all_blocked_ticks_  = 0;
                hold_scan_position();
                break;
            }

            const bool active_goal_changed =
                !committed_goal_valid_ ||
                committed_goal_frame_id_ != raw_goal.frame_id ||
                committed_goal_source_ != raw_goal.source ||
                std::hypot(committed_goal_n_ - raw_goal.north,
                           committed_goal_e_ - raw_goal.east) > goal_radius_ * 0.5f;
            if (active_goal_changed) {
                committed_goal_n_        = raw_goal.north;
                committed_goal_e_        = raw_goal.east;
                committed_goal_yaw_      = raw_goal.yaw;
                committed_goal_has_yaw_  = raw_goal.has_yaw;
                committed_goal_frame_id_ = raw_goal.frame_id;
                committed_goal_source_   = raw_goal.source;
                committed_goal_valid_    = true;
                goal_best_dist_          = goal_dist;
                goal_no_progress_ticks_  = 0;
                vfh_all_blocked_ticks_   = 0;
                RCLCPP_INFO(get_logger(),
                    "SCAN tracking [%s/%s] raw=(%.2f,%.2f) ctl=(%.2f,%.2f)",
                    raw_goal.source, raw_goal.frame_id.c_str(),
                    raw_goal.north, raw_goal.east, goal_n, goal_e);
            } else if (goal_dist + goal_progress_epsilon_m_ < goal_best_dist_) {
                goal_best_dist_ = goal_dist;
                goal_no_progress_ticks_ = 0;
            } else {
                ++goal_no_progress_ticks_;
            }

            const uint32_t goal_progress_timeout_ticks =
                static_cast<uint32_t>(goal_progress_timeout_s_ / kLoopPeriod_s);
            if (goal_no_progress_ticks_ >= goal_progress_timeout_ticks) {
                RCLCPP_WARN(get_logger(),
                    "SCAN route made no progress for %.0f s toward (%.2f,%.2f) "
                    "— dropping active waypoint",
                    goal_progress_timeout_s_, raw_goal.north, raw_goal.east);
                remember_blocked_scan_waypoint(raw_goal);
                if (!waypoint_queue_.empty()) {
                    waypoint_queue_.pop_front();
                } else {
                    waypoint_queue_.clear();
                }
                publish_active_scan_path();
                committed_goal_valid_   = false;
                goal_best_dist_         = std::numeric_limits<float>::infinity();
                goal_no_progress_ticks_ = 0;
                vfh_all_blocked_ticks_  = 0;
                hold_scan_position();
                break;
            }

            // ── VFH/direct: find horizontal command toward goal ───────────────
            Force2D scan_cmd{};
            if (disable_vfh_) {
                const float dn = goal_n - current_n;
                const float de = goal_e - current_e;
                const float d = std::hypot(dn, de);
                if (d > 1e-3f) {
                    const float speed = std::min(v_max_, d);
                    scan_cmd = {speed * dn / d, speed * de / d};
                }
                vfh_all_blocked_ = false;
                vfh_min_clearance_ = lidar_rho0_m_;
                vfh_passage_clearance_ = lidar_rho0_m_;
                vfh_speed_cap_mps_ = v_max_;
            } else {
                scan_cmd = compute_vfh(goal_n, goal_e);
            }
            const auto [vn_vfh, ve_vfh] = scan_cmd;

            // Corner deadlock recovery: when VFH cannot find ANY free sector it sets
            // vfh_all_blocked_ and returns a slow escape velocity instead of zero.
            // Give the local escape time to work before declaring the route impossible.
            static constexpr uint32_t kVfhStuckThreshTicks =
                static_cast<uint32_t>(8.0f / kLoopPeriod_s);
            if (vfh_all_blocked_) {
                if (++vfh_all_blocked_ticks_ >= kVfhStuckThreshTicks) {
                    RCLCPP_WARN(get_logger(),
                        "SCAN stuck: all VFH sectors blocked for 8 s "
                        "— dropping active waypoint (was [%s] %.2f,%.2f)",
                        committed_goal_source_.c_str(), committed_goal_n_, committed_goal_e_);
                    remember_blocked_scan_waypoint(raw_goal);
                    if (!waypoint_queue_.empty()) {
                        waypoint_queue_.pop_front();
                    } else {
                        waypoint_queue_.clear();
                    }
                    publish_active_scan_path();
                    committed_goal_valid_ = false;
                    goal_best_dist_ = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_ = 0;
                    hold_scan_position();
                    break;
                }
            } else {
                vfh_all_blocked_ticks_ = 0;
            }

            // Z: hold the active scan layer. 3D A* may route through nearby
            // vertical cells, but using those waypoint altitudes in SCAN fights
            // the layer controller and creates visible up/down oscillation.
            float z_target_agl = scan_alt_;
            if (scan_follow_path_altitude_ && !waypoint_queue_.empty()) {
                const float wp_ned = waypoint_queue_.front().alt_ned;
                if (wp_ned < -0.1f) {
                    z_target_agl = -wp_ned;
                }
            }
            cmd_vz_ = climb_blocked_by_ceiling
                ? 0.0f
                : calc_vz(std::max(z_target_agl, floor_clearance_m_ + 0.05f));

            // ── VFH command shaping ──────────────────────────────────────────
            // A quadrotor is holonomic in XY: it can move toward the VFH valley
            // without yawing the nose to every histogram update.  Keeping yaw
            // stable prevents scan-matching drift in long/symmetric corridors.
            const float target_speed = std::hypot(vn_vfh, ve_vfh);
            const float vfh_heading  = target_speed > 0.001f
                ? wrap_pi(std::atan2(ve_vfh, vn_vfh))
                : yaw_cmd_;
            const float route_conflict_angle =
                angle_error_abs(vfh_heading, goal_bearing);
            const float route_conflict_limit =
                std::max(route_blocked_range_m_, route_conflict_range_m_);
            const bool route_conflict =
                route_conflict_replan_enabled_ &&
                !disable_vfh_ &&
                !vfh_all_blocked_ &&
                have_lidar_scan_ &&
                std::isfinite(route_range) &&
                goal_dist > goal_radius_ + 0.50f &&
                route_range < std::min(route_conflict_limit, goal_dist - 0.15f) &&
                route_conflict_angle > route_conflict_angle_rad_;
            const uint32_t route_conflict_thresh_ticks =
                static_cast<uint32_t>(std::max(1.0f, route_conflict_replan_s_ / kLoopPeriod_s));
            if (route_conflict) {
                if (++route_conflict_ticks_ >= route_conflict_thresh_ticks) {
                    RCLCPP_WARN(get_logger(),
                        "SCAN route conflict: obstacle on active route for %.1fs "
                        "(route=%.2fm < %.2fm, vfh_err=%.0f° > %.0f°) toward [%s] "
                        "(%.2f,%.2f), dist=%.2fm — clearing route for replanning",
                        route_conflict_replan_s_, route_range, route_conflict_limit,
                        route_conflict_angle * 180.0f / static_cast<float>(M_PI),
                        route_conflict_angle_rad_ * 180.0f / static_cast<float>(M_PI),
                        raw_goal.source, raw_goal.north, raw_goal.east, goal_dist);
                    remember_blocked_scan_waypoint(raw_goal);
                    waypoint_queue_.clear();
                    publish_active_scan_path();
                    committed_goal_valid_   = false;
                    goal_best_dist_         = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_  = 0;
                    route_conflict_ticks_   = 0;
                    hold_scan_position();
                    break;
                }
            } else {
                route_conflict_ticks_ = 0;
            }
            float desired_yaw = holonomic_vfh_
                ? desired_camera_yaw_to_goal(goal_n, goal_e, current_n, current_e, yaw_cmd_)
                : vfh_heading;
            if (goal.has_yaw) {
                const float cap_r = std::max(goal_radius_ * 2.0f,
                                             frontier_goal_yaw_capture_radius_m_);
                if (goal_dist < cap_r) {
                    const float blend = 1.0f - std::clamp(goal_dist / cap_r, 0.0f, 1.0f);
                    desired_yaw = wrap_pi(desired_yaw + wrap_pi(goal.yaw - desired_yaw) * blend);
                }
            }
            slew_yaw_cmd_toward(desired_yaw);

            const float current_yaw = current_heading();
            const float max_dv      = max_accel_ * kLoopPeriod_s;

            // Gate on the chosen VFH sector's clearance, not the nose direction.
            // When VFH picks sideways/backward, forward_range_ may be small while the
            // chosen direction is wide open — gating on forward_range_ would freeze the drone.
            // vfh_all_blocked_ bypasses the gate so the escape creep always goes through.
            //
            // INVARIANT: kCreepStopRange < block_range.
            // With kCreepStopRange < block_range, valid sectors (clearance >= block_range)
            // always pass the gate — dead code as intended. If kCreepStopRange were ever
            // hardcoded above block_range, sectors just outside block_range would have speed
            // zeroed despite being "open", creating a deadlock (observed with 0.60 > 0.55).
            const float vfh_block_range   = std::min(lidar_rho0_m_, std::max(rho0_, fwd_stop_m_));
            const float kCreepStopRange_m = vfh_block_range - 0.05f;  // derived, never above block_range

            if (holonomic_vfh_) {
                float target_vn = vn_vfh;
                float target_ve = ve_vfh;
                if (!vfh_all_blocked_ && vfh_min_clearance_ < kCreepStopRange_m) {
                    target_vn = 0.0f;
                    target_ve = 0.0f;
                }
                const float look_speed_scale = camera_yaw_speed_scale(desired_yaw, current_yaw);
                target_vn *= look_speed_scale;
                target_ve *= look_speed_scale;
                apply_peer_avoidance(target_vn, target_ve, "SCAN");
                apply_nearest_obstacle_centering(target_vn, target_ve, "SCAN");
                apply_local_apf_repulsion(target_vn, target_ve, "SCAN");
                apply_nearest_obstacle_component_guard(target_vn, target_ve, "SCAN");

                // If the nose is close to a wall, remove only the velocity component
                // that would keep driving into it.  Lateral/backward escape remains allowed.
                if (std::hypot(target_vn, target_ve) > 0.0f &&
                    forward_range_ < vfh_block_range &&
                    angle_error_abs(current_yaw, vfh_heading) > 0.30f) {
                    const float nose_v =
                        target_vn * std::cos(current_yaw) +
                        target_ve * std::sin(current_yaw);
                    if (nose_v > 0.0f) {
                        target_vn -= nose_v * std::cos(current_yaw);
                        target_ve -= nose_v * std::sin(current_yaw);
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "VFH nose-component stop: nose=%.0f° vfh=%.0f° "
                            "fwd=%.2fm < block=%.2fm",
                            current_yaw * 180.f / static_cast<float>(M_PI),
                            vfh_heading * 180.f / static_cast<float>(M_PI),
                            forward_range_, vfh_block_range);
                    }
                }

                target_vn -= kd_vel_damp_ * control_vn();
                target_ve -= kd_vel_damp_ * control_ve();
                const float local_speed_cap =
                    disable_vfh_ ? v_max_ : std::min(v_max_, vfh_speed_cap_mps_);
                clamp_xy_norm(target_vn, target_ve, local_speed_cap);
                slew_xy_velocity(target_vn, target_ve, max_dv);
                fwd_speed_cmd_ = std::hypot(cmd_vn_, cmd_ve_);

                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(cmd_vn_, cmd_ve_, cmd_vz_, yaw_cmd_);
                publish_vfh_markers(vfh_heading, yaw_cmd_);
                {
                    std_msgs::msg::Float32 m;
                    m.data = vfh_heading;
                    raw_vfh_yaw_pub_->publish(m);
                    m.data = yaw_cmd_;
                    cmd_yaw_pub_->publish(m);
                }

                if (scan_ticks_ % 10 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "NAV HOL: VFH_Yaw=%.0f° Cmd_Yaw=%.0f° Actual_Yaw=%.0f° "
                        "VFH_Spd=%.2f Cmd_Spd=%.2f Cap=%.2f NearCap=%.2f Passage=%.2f Nearest=%.2f Fwd=%.2f",
                        vfh_heading * 180.0f / static_cast<float>(M_PI),
                        yaw_cmd_     * 180.0f / static_cast<float>(M_PI),
                        current_yaw  * 180.0f / static_cast<float>(M_PI),
                        target_speed, fwd_speed_cmd_,
                        vfh_speed_cap_mps_, vfh_nearest_speed_cap_mps_,
                        vfh_passage_clearance_,
                        lidar_nearest_range_, forward_range_);

                    if (committed_goal_valid_) {
                        PointStamped cg;
                        cg.header.stamp    = now();
                        cg.header.frame_id = committed_goal_frame_id_.empty()
                            ? px4_goal_frame_id_
                            : committed_goal_frame_id_;
                        cg.point.x = committed_goal_n_;
                        cg.point.y = committed_goal_e_;
                        cg.point.z = pos_.z_valid ? pos_.z : -scan_alt_;
                        committed_goal_pub_->publish(cg);
                    }
                }

                if (state_ticks_ % 20 == 0) {
                    RCLCPP_INFO(get_logger(),
                        "SCAN t=%us alt=%.1f/%.1fm layer=%d pos=(%.2f,%.2f,%.2f) "
                        "cov=%.1f%% lc=%.1f%% frontier=%u/%u fwd=%.2fm "
                        "v=(%.2f,%.2f,%.2f) goal=[%s](%.1f,%.1f) dist=%.1fm "
                        "yaw=%.0f° cmd=%.0f° vfh=%.0f°",
                        scan_ticks_ / 20, scan_alt_, target_alt_,
                        scan_layer_index_,
                        current_n, current_e, pos_.z_valid ? -pos_.z : 0.0f,
                        effective_coverage() * 100.0f,
                        layer_coverage() * 100.0f,
                        reachable_frontier_clusters_, frontier_clusters_,
                        forward_range_,
                        cmd_vn_, cmd_ve_, cmd_vz_,
                        goal.source, goal_n, goal_e, goal_dist,
                        control_heading() * 180.0f / static_cast<float>(M_PI),
                        yaw_cmd_ * 180.0f / static_cast<float>(M_PI),
                        vfh_heading * 180.0f / static_cast<float>(M_PI));
                }
                break;
            }

            // ── Legacy unicycle kinematics ───────────────────────────────────
            // Align nose to VFH heading first, then apply forward thrust.
            // Kept behind holonomic_vfh=false for experiments and regression tests.
            float effective_speed = (!vfh_all_blocked_ && vfh_min_clearance_ < kCreepStopRange_m)
                ? 0.0f : target_speed;
            // Turn-blocked safety: when VFH wants a heading >17° from current nose AND the current
            // nose direction has an obstacle within block_range, zero thrust.
            // Prevents the unicycle from carrying the drone into the blocked sector while yawing
            // toward the open direction.  Thrust resumes once the nose has rotated past 17°.
            if (effective_speed > 0.0f &&
                angle_error_abs(current_yaw, vfh_heading) > 0.30f &&
                forward_range_ < vfh_block_range) {
                effective_speed  = 0.0f;
                fwd_speed_cmd_   = 0.0f;  // hard stop: bypass max_accel ramp-down
                // Zeroing desired_fwd alone takes 10+ ticks (max_accel=0.5 m/s²) to drain
                // fwd_speed_cmd_ from cruise speed — drone travels 0.25+ m before stopping.
                // At 0.40 m forward clearance that is a crash.  Hard-zero kills thrust immediately.
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "VFH turn-stop: nose=%.0f° vfh=%.0f° diff=%.0f° fwd=%.2fm < block=%.2fm",
                    current_yaw * 180.f / static_cast<float>(M_PI),
                    vfh_heading * 180.f / static_cast<float>(M_PI),
                    angle_error_abs(current_yaw, vfh_heading) * 180.f / static_cast<float>(M_PI),
                    forward_range_, vfh_block_range);
            }

            const float yaw_err    = angle_error_abs(current_yaw, yaw_cmd_);
            float desired_fwd      = (yaw_err < align_threshold_)
                ? effective_speed * std::cos(yaw_err)
                : 0.0f;

            // Lyapunov damping: subtract K_d·‖v_actual‖ to brake momentum
            // before a turn. Clamp to [0, v_max_]: no reverse thrust.
            desired_fwd = std::clamp(
                desired_fwd - kd_vel_damp_ * std::hypot(control_vn(), control_ve()),
                0.0f, v_max_);

            fwd_speed_cmd_ += std::clamp(desired_fwd - fwd_speed_cmd_, -max_dv, max_dv);
            cmd_vn_ = fwd_speed_cmd_ * std::cos(current_yaw);
            cmd_ve_ = fwd_speed_cmd_ * std::sin(current_yaw);

            publish_ocm(Mode::VELOCITY);
            publish_velocity_sp(cmd_vn_, cmd_ve_, cmd_vz_, yaw_cmd_);
            publish_vfh_markers(desired_yaw, yaw_cmd_);
            {
                std_msgs::msg::Float32 m;
                m.data = desired_yaw;
                raw_vfh_yaw_pub_->publish(m);
                m.data = yaw_cmd_;
                cmd_yaw_pub_->publish(m);
            }

            if (scan_ticks_ % 10 == 0) {
                RCLCPP_INFO(get_logger(),
                    "NAV UNI: VFH_Yaw=%.0f° Cmd_Yaw=%.0f° Actual_Yaw=%.0f° "
                    "VFH_Spd=%.2f Eff_Spd=%.2f Fwd_Cmd=%.2f Nearest=%.2f Fwd=%.2f",
                    desired_yaw * 180.0f / static_cast<float>(M_PI),
                    yaw_cmd_    * 180.0f / static_cast<float>(M_PI),
                    current_yaw * 180.0f / static_cast<float>(M_PI),
                    target_speed, effective_speed, fwd_speed_cmd_,
                    lidar_nearest_range_, forward_range_);

                if (committed_goal_valid_) {
                    PointStamped cg;
                    cg.header.stamp    = now();
                    cg.header.frame_id = committed_goal_frame_id_.empty()
                        ? px4_goal_frame_id_
                        : committed_goal_frame_id_;
                    cg.point.x = committed_goal_n_;
                    cg.point.y = committed_goal_e_;
                    cg.point.z = pos_.z_valid ? pos_.z : -scan_alt_;
                    committed_goal_pub_->publish(cg);
                }
            }

            if (state_ticks_ % 20 == 0) {
                RCLCPP_INFO(get_logger(),
                    "SCAN t=%us alt=%.1f/%.1fm layer=%d pos=(%.2f,%.2f,%.2f) "
                    "cov=%.1f%% lc=%.1f%% frontier=%u/%u fwd=%.2fm "
                    "v=(%.2f,%.2f,%.2f) goal=[%s](%.1f,%.1f) dist=%.1fm "
                    "yaw=%.0f°→%.0f°",
                    scan_ticks_ / 20, scan_alt_, target_alt_,
                    scan_layer_index_,
                    current_n, current_e, pos_.z_valid ? -pos_.z : 0.0f,
                    effective_coverage() * 100.0f,
                    layer_coverage() * 100.0f,
                    reachable_frontier_clusters_, frontier_clusters_,
                    forward_range_,
                    cmd_vn_, cmd_ve_, cmd_vz_,
                    goal.source, goal_n, goal_e, goal_dist,
                    control_heading() * 180.0f / static_cast<float>(M_PI),
                    desired_yaw * 180.0f / static_cast<float>(M_PI));
            }
            break;
        }

        // -- Mode 2b: TARGET_INSPECT -- hold position, look at target, then keep scanning.
        case State::TARGET_INSPECT: {
            const float yaw_to_target = std::atan2(target_east_  - control_east(),
                                                   target_north_ - control_north());
            slew_yaw_cmd_toward(yaw_to_target);
            publish_ocm(Mode::VELOCITY);
            publish_velocity_sp(0.0f, 0.0f, calc_vz(scan_alt_), yaw_cmd_);

            if (state_ticks_ % 20 == 1) {
                RCLCPP_INFO(get_logger(),
                    "TARGET_INSPECT t=%.1fs/%.1fs pos=(%.2f,%.2f,%.2f) "
                    "target=(%.2f,%.2f) yaw=%.0f cmd=%.0f",
                    state_ticks_ * kLoopPeriod_s, hold_target_s_,
                    control_north(), control_east(), pos_.z_valid ? -pos_.z : scan_alt_,
                    target_north_, target_east_,
                    current_heading() * 180.0f / static_cast<float>(M_PI),
                    yaw_cmd_ * 180.0f / static_cast<float>(M_PI));
            }

            if (state_ticks_ >= static_cast<uint32_t>(hold_target_s_ / kLoopPeriod_s)) {
                RCLCPP_INFO(get_logger(),
                    "TARGET_INSPECT complete for (%.2f, %.2f) - resuming scan",
                    target_north_, target_east_);
                target_found_ = false;
                target_confirm_count_ = 0;
                target_candidate_valid_ = false;
                cmd_vn_ = 0.0f;
                cmd_ve_ = 0.0f;
                fwd_speed_cmd_ = 0.0f;
                transition(State::SCAN);
            }
            break;
        }

        // -- Mode 2c: ORBIT -- legacy blind orbit, no longer used by TargetFound.
        // Tangential velocity in NED: for CCW orbit at angle θ from target,
        //   position = target + r*(cos θ, sin θ)
        //   vn = -sin θ * speed,  ve = cos θ * speed
        // Yaw toward the target so the camera stays pointed at the object.
        case State::ORBIT: {
            const float d_angle = kOrbitSpeed_mps_ / kOrbitRadius_m_ * kLoopPeriod_s;
            orbit_angle_rad_    = wrap_pi(orbit_angle_rad_ + d_angle);
            orbit_progress_rad_ += d_angle;

            const float vn_orb = -std::sin(orbit_angle_rad_) * kOrbitSpeed_mps_;
            const float ve_orb =  std::cos(orbit_angle_rad_) * kOrbitSpeed_mps_;
            const float yaw_to_target = std::atan2(target_east_  - control_east(),
                                                    target_north_ - control_north());
            publish_ocm(Mode::VELOCITY);
            publish_velocity_sp(vn_orb, ve_orb, calc_vz(scan_alt_), yaw_to_target);

            if (orbit_progress_rad_ >= 2.0f * static_cast<float>(M_PI)) {
                RCLCPP_INFO(get_logger(),
                    "[ORBIT] 360° complete around (%.2f, %.2f) — returning home",
                    target_north_, target_east_);
                String result_msg;
                result_msg.data = "TARGET FOUND at (" +
                    std::to_string(target_north_) + ", " +
                    std::to_string(target_east_) + ")";
                result_pub_->publish(result_msg);
                transition(State::RETURN);
            }
            break;
        }

        // ── Mode 3: return home ───────────────────────────────────────────────
        case State::RETURN: {
            if (floor_too_close()) {
                publish_floor_recovery(return_alt_, yaw_now);
                if (state_ticks_ % 20 == 0) {
                    RCLCPP_WARN(get_logger(),
                        "RETURN floor guard active: range_down=%.2f m < %.2f m",
                        floor_range_m_, floor_clearance_m_);
                }
                break;
            }
            // Emergency hover during RETURN — same pattern as SCAN state.
            // NEVER disarm in flight; hover until clear, escape if stuck >10 s.
            {
                const bool have_safety = have_lidar_ || have_fwd_range_;
                const float nearest    = have_lidar_ ? test_obstacle_range() : forward_range_;
                if (have_safety && nearest < emergency_stop_range_m_) {
                    ++emergency_hover_ticks_;
                    const bool log_now = (emergency_hover_ticks_ % 20 == 1);
                    if (log_now) {
                        RCLCPP_ERROR(get_logger(),
                            "RETURN EMERGENCY HOVER [%u ticks]: nearest=%.2f m < %.2f m"
                            " — hovering, NOT disarming",
                            emergency_hover_ticks_, nearest, emergency_stop_range_m_);
                    }
                    constexpr uint32_t kEscapeTicks =
                        static_cast<uint32_t>(10.0f / kLoopPeriod_s);
                    if (emergency_hover_ticks_ > kEscapeTicks) {
                        const float esc = v_max_ * 0.15f;
                        if (have_lidar_) {
                            // Prefer the most open lidar window (corner-safe);
                            // fall back to opposite-of-nearest.
                            bool open_ok = false;
                            float esc_dir = most_open_escape_dir(open_ok);
                            if (!open_ok) {
                                const float aw_near = wrap_pi(lidar_scan_yaw_ + lidar_nearest_angle_rad_);
                                esc_dir = wrap_pi(aw_near + static_cast<float>(M_PI));
                            }
                            publish_velocity_sp(esc * std::cos(esc_dir),
                                               esc * std::sin(esc_dir), calc_vz(return_alt_), current_heading());
                        } else {
                            const float h = current_heading();
                            publish_velocity_sp(-esc * std::cos(h), -esc * std::sin(h), calc_vz(return_alt_), h);
                        }
                    } else {
                        publish_velocity_sp(0.0f, 0.0f, calc_vz(return_alt_), current_heading());
                    }
                    break;
                }
            }
            emergency_hover_ticks_ = 0;

            const float return_n = control_north();
            const float return_e = control_east();
            const float home_dist = std::hypot(return_n - home_north_m_,
                                               return_e - home_east_m_);
            if (control_xy_valid() && home_dist < goal_radius_) {
                RCLCPP_INFO(get_logger(),
                    "Home reached (%.2f m from %.2f,%.2f) — landing",
                    home_dist, home_north_m_, home_east_m_);
                transition(State::LANDING);
                break;
            }

            auto hold_return_position = [&]() {
                cmd_vn_ = 0.0f;
                cmd_ve_ = 0.0f;
                fwd_speed_cmd_ = 0.0f;
                advance_hold_scan_yaw();
                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(0.0f, 0.0f, calc_vz(return_alt_), yaw_cmd_);
            };

            auto drop_reached_return_waypoints = [&]() -> size_t {
                size_t dropped = 0;
                while (!return_waypoint_queue_.empty()) {
                    const auto &wp = return_waypoint_queue_.front();
                    const Goal2D wp_goal = project_goal_to_control_frame({
                        wp.north, wp.east, wp.yaw, wp.has_yaw,
                        "return_waypoint", return_waypoint_frame_id_,
                    });
                    const float wp_dist = std::hypot(wp_goal.north - return_n,
                                                     wp_goal.east  - return_e);
                    const bool reached = wp_dist < goal_radius_;
                    const bool smooth_lookahead =
                        !reached &&
                        should_lookahead_waypoint(
                            return_waypoint_queue_, return_waypoint_frame_id_,
                            return_n, return_e);
                    if (!reached && !smooth_lookahead) {
                        break;
                    }
                    RCLCPP_INFO(get_logger(),
                        "RETURN waypoint %s (%.2f,%.2f) dist=%.2fm — %zu remaining",
                        reached ? "reached" : "lookahead",
                        wp.north, wp.east, wp_dist, return_waypoint_queue_.size() - 1);
                    return_waypoint_queue_.pop_front();
                    ++dropped;
                }
                if (dropped > 0) {
                    committed_goal_valid_ = false;
                    goal_best_dist_ = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_ = 0;
                }
                return dropped;
            };
            drop_reached_return_waypoints();

            const auto raw_goal = pick_return_goal();
            if (!raw_goal.valid) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "RETURN no planned route available — holding for mapper");
                committed_goal_valid_ = false;
                goal_best_dist_ = std::numeric_limits<float>::infinity();
                goal_no_progress_ticks_ = 0;
                vfh_all_blocked_ticks_ = 0;
                hold_return_position();
                break;
            }

            const Goal2D goal = project_goal_to_control_frame({
                raw_goal.north, raw_goal.east, raw_goal.yaw,
                raw_goal.has_yaw, raw_goal.source, raw_goal.frame_id,
            });
            const float goal_n = goal.north;
            const float goal_e = goal.east;
            const float goal_dist = std::hypot(goal_n - return_n, goal_e - return_e);

            if (goal_dist < goal_radius_) {
                drop_reached_return_waypoints();
                hold_return_position();
                break;
            }

            const bool active_goal_changed =
                !committed_goal_valid_ ||
                committed_goal_frame_id_ != raw_goal.frame_id ||
                committed_goal_source_ != raw_goal.source ||
                std::hypot(committed_goal_n_ - raw_goal.north,
                           committed_goal_e_ - raw_goal.east) > goal_radius_ * 0.5f;
            if (active_goal_changed) {
                committed_goal_n_        = raw_goal.north;
                committed_goal_e_        = raw_goal.east;
                committed_goal_yaw_      = raw_goal.yaw;
                committed_goal_has_yaw_  = raw_goal.has_yaw;
                committed_goal_frame_id_ = raw_goal.frame_id;
                committed_goal_source_   = raw_goal.source;
                committed_goal_valid_    = true;
                goal_best_dist_          = goal_dist;
                goal_no_progress_ticks_  = 0;
                vfh_all_blocked_ticks_   = 0;
                RCLCPP_INFO(get_logger(),
                    "RETURN tracking [%s/%s] raw=(%.2f,%.2f) ctl=(%.2f,%.2f)",
                    raw_goal.source, raw_goal.frame_id.c_str(),
                    raw_goal.north, raw_goal.east, goal_n, goal_e);
            } else if (goal_dist + goal_progress_epsilon_m_ < goal_best_dist_) {
                goal_best_dist_ = goal_dist;
                goal_no_progress_ticks_ = 0;
            } else {
                ++goal_no_progress_ticks_;
            }

            const uint32_t goal_progress_timeout_ticks =
                static_cast<uint32_t>(goal_progress_timeout_s_ / kLoopPeriod_s);
            if (goal_no_progress_ticks_ >= goal_progress_timeout_ticks) {
                RCLCPP_WARN(get_logger(),
                    "RETURN route made no progress for %.0f s toward (%.2f,%.2f) "
                    "— dropping active return waypoint",
                    goal_progress_timeout_s_, raw_goal.north, raw_goal.east);
                if (!return_waypoint_queue_.empty()) {
                    return_waypoint_queue_.pop_front();
                } else {
                    return_waypoint_queue_.clear();
                }
                committed_goal_valid_ = false;
                goal_best_dist_ = std::numeric_limits<float>::infinity();
                goal_no_progress_ticks_ = 0;
                vfh_all_blocked_ticks_ = 0;
                hold_return_position();
                break;
            }

            Force2D ret_cmd{};
            if (disable_vfh_) {
                const float dn = goal_n - return_n;
                const float de = goal_e - return_e;
                const float d = std::hypot(dn, de);
                if (d > 1e-3f) {
                    const float speed = std::min(v_max_, d);
                    ret_cmd = {speed * dn / d, speed * de / d};
                }
                vfh_all_blocked_ = false;
                vfh_min_clearance_ = lidar_rho0_m_;
                vfh_passage_clearance_ = lidar_rho0_m_;
                vfh_speed_cap_mps_ = v_max_;
            } else {
                ret_cmd = compute_vfh(goal_n, goal_e);
            }
            const auto [vn_ret, ve_ret] = ret_cmd;
            static constexpr uint32_t kVfhStuckThreshTicks =
                static_cast<uint32_t>(8.0f / kLoopPeriod_s);
            if (vfh_all_blocked_) {
                if (++vfh_all_blocked_ticks_ >= kVfhStuckThreshTicks) {
                    RCLCPP_WARN(get_logger(),
                        "RETURN stuck: all VFH sectors blocked for 8 s — dropping active return waypoint");
                    if (!return_waypoint_queue_.empty()) {
                        return_waypoint_queue_.pop_front();
                    } else {
                        return_waypoint_queue_.clear();
                    }
                    committed_goal_valid_ = false;
                    goal_best_dist_ = std::numeric_limits<float>::infinity();
                    goal_no_progress_ticks_ = 0;
                    vfh_all_blocked_ticks_ = 0;
                    hold_return_position();
                    break;
                }
            } else {
                vfh_all_blocked_ticks_ = 0;
            }

            const float target_speed = std::hypot(vn_ret, ve_ret);
            const float vfh_heading = target_speed > 0.001f
                ? wrap_pi(std::atan2(ve_ret, vn_ret))
                : yaw_cmd_;
            const float desired_yaw = holonomic_vfh_
                ? desired_camera_yaw_to_goal(goal_n, goal_e, return_n, return_e, yaw_cmd_)
                : vfh_heading;
            slew_yaw_cmd_toward(desired_yaw);

            const float current_yaw = current_heading();
            const float max_dv = max_accel_ * kLoopPeriod_s;
            const float vfh_block_range = std::min(lidar_rho0_m_, std::max(rho0_, fwd_stop_m_));
            const float kCreepStopRange_m = vfh_block_range - 0.05f;

            if (holonomic_vfh_) {
                float target_vn = vn_ret;
                float target_ve = ve_ret;
                if (!vfh_all_blocked_ && vfh_min_clearance_ < kCreepStopRange_m) {
                    target_vn = 0.0f;
                    target_ve = 0.0f;
                }
                const float look_speed_scale = camera_yaw_speed_scale(desired_yaw, current_yaw);
                target_vn *= look_speed_scale;
                target_ve *= look_speed_scale;
                apply_peer_avoidance(target_vn, target_ve, "RETURN");
                apply_nearest_obstacle_centering(target_vn, target_ve, "RETURN");
                apply_local_apf_repulsion(target_vn, target_ve, "RETURN");
                apply_nearest_obstacle_component_guard(target_vn, target_ve, "RETURN");
                if (std::hypot(target_vn, target_ve) > 0.0f &&
                    forward_range_ < vfh_block_range &&
                    angle_error_abs(current_yaw, vfh_heading) > 0.30f) {
                    const float nose_v =
                        target_vn * std::cos(current_yaw) +
                        target_ve * std::sin(current_yaw);
                    if (nose_v > 0.0f) {
                        target_vn -= nose_v * std::cos(current_yaw);
                        target_ve -= nose_v * std::sin(current_yaw);
                    }
                }
                target_vn -= kd_vel_damp_ * control_vn();
                target_ve -= kd_vel_damp_ * control_ve();
                const float local_speed_cap =
                    disable_vfh_ ? v_max_ : std::min(v_max_, vfh_speed_cap_mps_);
                clamp_xy_norm(target_vn, target_ve, local_speed_cap);
                slew_xy_velocity(target_vn, target_ve, max_dv);
                fwd_speed_cmd_ = std::hypot(cmd_vn_, cmd_ve_);

                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(cmd_vn_, cmd_ve_, calc_vz(return_alt_), yaw_cmd_);

                if (state_ticks_ % static_cast<uint32_t>(1.0f / kLoopPeriod_s * 2) == 0) {
                    RCLCPP_INFO(get_logger(),
                        "RETURN HOL: pos=(%.2f,%.2f) home=%.2f m goal=(%.2f,%.2f) "
                        "d=%.2f m v=(%.2f,%.2f) cap=%.2f passage=%.2f yaw=%.0f° cmd=%.0f° vfh=%.0f°",
                        return_n, return_e, home_dist, goal_n, goal_e, goal_dist,
                        cmd_vn_, cmd_ve_, vfh_speed_cap_mps_, vfh_passage_clearance_,
                        current_yaw * 180.0f / static_cast<float>(M_PI),
                        yaw_cmd_ * 180.0f / static_cast<float>(M_PI),
                        vfh_heading * 180.0f / static_cast<float>(M_PI));
                }
                break;
            }

            const float yaw_err = angle_error_abs(current_yaw, yaw_cmd_);
            float desired_fwd = (yaw_err < align_threshold_)
                ? target_speed * std::cos(yaw_err)
                : 0.0f;
            desired_fwd = std::clamp(
                desired_fwd - kd_vel_damp_ * std::hypot(control_vn(), control_ve()),
                0.0f, v_max_);

            fwd_speed_cmd_ += std::clamp(desired_fwd - fwd_speed_cmd_, -max_dv, max_dv);
            cmd_vn_ = fwd_speed_cmd_ * std::cos(current_yaw);
            cmd_ve_ = fwd_speed_cmd_ * std::sin(current_yaw);
            publish_ocm(Mode::VELOCITY);
            publish_velocity_sp(cmd_vn_, cmd_ve_, calc_vz(return_alt_), yaw_cmd_);

            if (state_ticks_ % static_cast<uint32_t>(1.0f / kLoopPeriod_s * 2) == 0) {
                RCLCPP_INFO(get_logger(),
                    "RETURN: pos=(%.2f,%.2f) home=%.2f m goal=(%.2f,%.2f) d=%.2f m",
                    return_n, return_e, home_dist, goal_n, goal_e, goal_dist);
            }
            break;
        }

        // ── LAND + disarm ─────────────────────────────────────────────────────
        case State::LANDING:
            cmd_vn_ = cmd_ve_ = 0.0f;
            publish_ocm(Mode::POSITION);
            publish_position_sp(pos_.x, pos_.y, 0.0f, yaw_now); // Command current XY, 0 altitude
            if (state_ticks_ == 1)
                send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
            if (pos_.z_valid && pos_.z > -altitude_tolerance_m_) {
                send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                transition(State::DONE);
            }
            if (!is_armed()) { transition(State::DONE); break; } // Check if already disarmed
            if (state_ticks_ > static_cast<uint32_t>(20.0f / kLoopPeriod_s)) { // 20 second timeout
                RCLCPP_WARN(get_logger(), "Landing timeout — force disarm");
                send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                transition(State::DONE);
            }
            break;

        // ── TEST_FORWARD: straight-ahead movement test ─────────────────────────
        // Phase 1 / 2 / 4 use POSITION mode for measurable displacement and stable holds.
        // Phase 3 uses VELOCITY mode to test forward wall-stop behavior.
        // Altitude is controlled either by a position setpoint at -scan_alt_start_ or
        // by calc_vz(scan_alt_start_) in the velocity-driven phase.
        //
        // Phase 0: altitude settle — climb to scan_alt_start_ within 0.15 m before moving.
        // Phase 1: position-mode flight to a point test_forward_dist_m_ straight ahead.
        //          Stops early if a wall appears within test_wall_stop_m_.
        // Phase 2: position hold for test_hold_s_ at the reached point.
        // Phase 3: velocity-mode drive straight ahead with ramp-down near the wall.
        //          Stops when fwd < test_wall_stop_m_.
        // Phase 4: position hold forever — Ctrl-C to end.
        case State::TEST_FORWARD: {
            const float dn = pos_.x - test_start_n_;
            const float de = pos_.y - test_start_e_;
            const float along_track = dn * std::cos(test_fwd_yaw_) + de * std::sin(test_fwd_yaw_);
            const float cross_track = -dn * std::sin(test_fwd_yaw_) + de * std::cos(test_fwd_yaw_);
            const bool depth_active = !(test_ignore_depth_until_phase3_ && test_phase_ < 3);
            // test_ignore_depth_until_phase3 suppresses only the depth camera.
            // LiDAR is the primary safety sensor and must remain active in every phase.
            const bool have_safety_range = have_lidar_ || (depth_active && have_fwd_range_);
            const float safety_range = have_lidar_ ? test_obstacle_range() : forward_range_;

            // Emergency stop (always active)
            if (have_safety_range && safety_range < emergency_stop_range_m_) {
                RCLCPP_FATAL(get_logger(),
                    "TEST EMERGENCY STOP: range=%.2f m fwd=%.2f m nearest=%.2f@%.0f° "
                    "raw_depth=%.2f m n=%u — disarming  pos=(%.2f,%.2f)  drone_pitch=%.1f°",
                    safety_range, forward_range_, lidar_nearest_range_,
                    lidar_nearest_angle_deg(), forward_range_raw_min_, forward_range_samples_,
                    pos_.x, pos_.y,
                    drone_pitch_rad_ * 180.0f / static_cast<float>(M_PI));
                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(0.0f, 0.0f, 0.0f, test_fwd_yaw_);
                send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                transition(State::DONE);
                break;
            }

            if (test_phase_ == 1 && have_lidar_ && safety_range < test_wall_stop_m_) {
                RCLCPP_WARN(get_logger(),
                    "TEST Phase 1 LiDAR wall stop before 2m target: range=%.2f m fwd=%.2f m "
                    "nearest=%.2f@%.0f° raw_depth=%.2f m n=%u after %.1f s  "
                    "along=%.2f/%.2f m  cross=%.2f m  pos=(%.2f,%.2f)",
                    safety_range, forward_range_, lidar_nearest_range_,
                    lidar_nearest_angle_deg(), forward_range_raw_min_, forward_range_samples_,
                    test_phase1_ticks_ * kLoopPeriod_s,
                    along_track, test_forward_dist_m_, cross_track,
                    pos_.x, pos_.y);
                test_hold_n_ = pos_.x;
                test_hold_e_ = pos_.y;
                publish_ocm(Mode::POSITION);
                publish_position_sp(test_hold_n_, test_hold_e_, -scan_alt_start_, test_fwd_yaw_);
                test_phase_      = 2;
                test_hold_ticks_ = 0;
                break;
            }

            // Phase 0: altitude settle — must be at scan_alt_start_ ±0.15 m before flying.
            const float agl_now = pos_.z_valid ? -pos_.z : 0.0f;
            if (test_phase_ == 1 && std::fabs(agl_now - scan_alt_start_) > test_alt_tol_m_) {
                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(0.0f, 0.0f, calc_vz(scan_alt_start_), test_fwd_yaw_);
                if (state_ticks_ % 20 == 0)
                    RCLCPP_INFO(get_logger(),
                        "TEST alt settle: agl=%.2f/%.2f m  pitch=%.1f°",
                        agl_now, scan_alt_start_,
                        drone_pitch_rad_ * 180.0f / static_cast<float>(M_PI));
                break;
            }

            if (test_phase_ == 1) {
                ++test_phase1_ticks_;
                const float dist_to_target =
                    std::hypot(test_target_n_ - pos_.x, test_target_e_ - pos_.y);

                // Wall detected early
                if (have_safety_range && safety_range < test_wall_stop_m_) {
                    RCLCPP_WARN(get_logger(),
                        "TEST Phase 1 early wall: range=%.2f m fwd=%.2f m nearest=%.2f@%.0f° "
                        "raw_depth=%.2f m n=%u after %.1f s  "
                        "along=%.2f/%.2f m  cross=%.2f m  pos=(%.2f,%.2f)",
                        safety_range, forward_range_, lidar_nearest_range_,
                        lidar_nearest_angle_deg(), forward_range_raw_min_, forward_range_samples_,
                        test_phase1_ticks_ * kLoopPeriod_s,
                        along_track, test_forward_dist_m_, cross_track,
                        pos_.x, pos_.y);
                    test_hold_n_ = pos_.x;
                    test_hold_e_ = pos_.y;
                    publish_ocm(Mode::POSITION);
                    publish_position_sp(test_hold_n_, test_hold_e_, -scan_alt_start_, test_fwd_yaw_);
                    test_phase_      = 2;
                    test_hold_ticks_ = 0;
                    break;
                }

                if (dist_to_target <= test_goal_radius_m_) {
                    RCLCPP_INFO(get_logger(),
                        "TEST Phase 1 done: reached %.1f m target in %.1f s  "
                        "along=%.2f m  cross=%.2f m  fwd=%.2f m raw=%.2f m n=%u  pos=(%.2f,%.2f) "
                        "— holding %.0f s",
                        test_forward_dist_m_,
                        test_phase1_ticks_ * kLoopPeriod_s,
                        along_track, cross_track,
                        forward_range_, forward_range_raw_min_, forward_range_samples_,
                        pos_.x, pos_.y, test_hold_s_);
                    test_hold_n_ = pos_.x;
                    test_hold_e_ = pos_.y;
                    publish_ocm(Mode::POSITION);
                    publish_position_sp(test_hold_n_, test_hold_e_, -scan_alt_start_, test_fwd_yaw_);
                    test_phase_      = 2;
                    test_hold_ticks_ = 0;
                    break;
                }

                publish_ocm(Mode::POSITION);
                publish_position_sp(test_target_n_, test_target_e_, -scan_alt_start_, test_fwd_yaw_);

                if (state_ticks_ % 20 == 0)
                    RCLCPP_INFO(get_logger(),
                        "TEST Phase 1: t=%.1f s  dist_rem=%.2f m  along=%.2f/%.2f m  "
                        "cross=%.2f m  fwd=%.2f m raw=%.2f m n=%u  pos=(%.2f,%.2f)",
                        test_phase1_ticks_ * kLoopPeriod_s,
                        dist_to_target, along_track, test_forward_dist_m_,
                        cross_track, forward_range_, forward_range_raw_min_,
                        forward_range_samples_, pos_.x, pos_.y);
                break;
            }

            if (test_phase_ == 2) {
                ++test_hold_ticks_;
                publish_ocm(Mode::POSITION);
                publish_position_sp(test_hold_n_, test_hold_e_, -scan_alt_start_, test_fwd_yaw_);
                if (state_ticks_ % 20 == 0)
                    RCLCPP_INFO(get_logger(),
                        "TEST Phase 2 hold: %.1f/%.0f s  fwd=%.2f m raw=%.2f m n=%u  "
                        "along=%.2f m  cross=%.2f m  pos=(%.2f,%.2f)",
                        test_hold_ticks_ * kLoopPeriod_s, test_hold_s_,
                        forward_range_, forward_range_raw_min_, forward_range_samples_,
                        along_track, cross_track, pos_.x, pos_.y);
                if (test_hold_ticks_ >= static_cast<uint32_t>(test_hold_s_ / kLoopPeriod_s)) {
                    if (test_phase3_enabled_) {
                        RCLCPP_INFO(get_logger(),
                            "TEST Phase 2 done — driving forward until %.1f m from wall",
                            test_wall_stop_m_);
                        test_phase_ = 3;
                    } else {
                        RCLCPP_INFO(get_logger(),
                            "TEST Phase 2 done — hold-only test complete");
                        test_phase_ = 4;
                    }
                }
                break;
            }

            if (test_phase_ == 3) {
                const float stop_range = test_obstacle_range();
                const bool have_stop_range = have_fwd_range_ ||
                    (test_lidar_nearest_stop_ && have_lidar_);
                if (have_stop_range && stop_range < test_wall_stop_m_) {
                    RCLCPP_INFO(get_logger(),
                        "TEST Phase 3 done: wall at %.2f m  fwd=%.2f m  "
                        "nearest=%.2f@%.0f°  raw_depth=%.2f m n=%u  "
                        "along=%.2f m  cross=%.2f m  pos=(%.2f,%.2f)",
                        stop_range, forward_range_, lidar_nearest_range_,
                        lidar_nearest_angle_deg(), forward_range_raw_min_,
                        forward_range_samples_,
                        along_track, cross_track, pos_.x, pos_.y);
                    test_hold_n_ = pos_.x;
                    test_hold_e_ = pos_.y;
                    publish_ocm(Mode::POSITION);
                    publish_position_sp(test_hold_n_, test_hold_e_, -scan_alt_start_, test_fwd_yaw_);
                    test_phase_ = 4;
                    break;
                }
                const float ramp_zone = 0.5f;
                float speed = v_max_;
                if (have_stop_range && stop_range < test_wall_stop_m_ + ramp_zone) {
                    speed = std::max(0.05f, v_max_ *
                        (stop_range - test_wall_stop_m_) / ramp_zone);
                }
                const float vn = speed * std::cos(test_fwd_yaw_);
                const float ve = speed * std::sin(test_fwd_yaw_);
                publish_ocm(Mode::VELOCITY);
                publish_velocity_sp(vn, ve, calc_vz(scan_alt_start_), test_fwd_yaw_);
                if (state_ticks_ % 20 == 0)
                    RCLCPP_INFO(get_logger(),
                        "TEST Phase 3: stop_range=%.2f m  fwd=%.2f m  "
                        "nearest=%.2f@%.0f°  raw_depth=%.2f m n=%u  "
                        "(stop<%.1f m)  spd=%.2f  along=%.2f m  cross=%.2f m  pos=(%.2f,%.2f)",
                        stop_range, forward_range_, lidar_nearest_range_,
                        lidar_nearest_angle_deg(), forward_range_raw_min_,
                        forward_range_samples_, test_wall_stop_m_, speed,
                        along_track, cross_track, pos_.x, pos_.y);
                break;
            }

            // Phase 4: hold — test complete.
            publish_ocm(Mode::POSITION);
            publish_position_sp(test_hold_n_, test_hold_e_, -scan_alt_start_, test_fwd_yaw_);
            if (state_ticks_ % 40 == 0)
                RCLCPP_INFO(get_logger(),
                    "TEST done — holding  fwd=%.2f m raw=%.2f m n=%u  "
                    "along=%.2f m  cross=%.2f m  pos=(%.2f,%.2f)",
                    forward_range_, forward_range_raw_min_, forward_range_samples_,
                    along_track, cross_track, pos_.x, pos_.y);
            break;
        }

        case State::DONE:
            if (state_ticks_ == 1)
                RCLCPP_INFO(get_logger(), "Mission complete.");
            break;
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SearchMissionController>());
    rclcpp::shutdown();
    return 0;
}
