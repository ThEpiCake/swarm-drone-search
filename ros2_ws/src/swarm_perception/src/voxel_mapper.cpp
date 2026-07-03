#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <swarm_msgs/msg/map_update_summary.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;
using px4_msgs::msg::VehicleOdometry;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;
using swarm_msgs::msg::MapUpdateSummary;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Empty;

namespace {

struct Vec3 { double x, y, z; };

Vec3 operator+(const Vec3 &a, const Vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }

// Rotate v by quaternion q=(w,x,y,z): body frame -> world frame.
// Same expansion as swarm_target_detection's _quat_rotate — kept consistent
// so both nodes project camera rays through the same pitched RGB-D mount.
Vec3 quat_rotate(const std::array<float, 4> &q, const Vec3 &v)
{
    double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    double uvx = qy * v.z - qz * v.y;
    double uvy = qz * v.x - qx * v.z;
    double uvz = qx * v.y - qy * v.x;
    double uuvx = qy * uvz - qz * uvy;
    double uuvy = qz * uvx - qx * uvz;
    double uuvz = qx * uvy - qy * uvx;
    return {
        v.x + 2.0 * (qw * uvx + uuvx),
        v.y + 2.0 * (qw * uvy + uuvy),
        v.z + 2.0 * (qw * uvz + uuvz),
    };
}

Vec3 pitch_about_body_y(const Vec3 &v, double pitch_rad)
{
    const double c = std::cos(pitch_rad);
    const double s = std::sin(pitch_rad);
    return {
        c * v.x - s * v.z,
        v.y,
        s * v.x + c * v.z,
    };
}

Vec3 rotate_yaw_ned(const Vec3 &v, double yaw_rad)
{
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    return {
        c * v.x - s * v.y,
        s * v.x + c * v.y,
        v.z,
    };
}

double wrap_pi(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double yaw_from_quat_ned(const std::array<float, 4> &q)
{
    const double qw = q[0];
    const double qx = q[1];
    const double qy = q[2];
    const double qz = q[3];
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

inline double sigmoid(double log_odds) { return 1.0 / (1.0 + std::exp(-log_odds)); }

inline double binary_entropy(double p)
{
    constexpr double kEps = 1e-6;
    p = std::clamp(p, kEps, 1.0 - kEps);
    return -p * std::log(p) - (1.0 - p) * std::log(1.0 - p);
}

// Integer voxel index — keyed into a hash map so the grid only stores cells
// that have actually been observed (a fixed 3D array over the whole map
// bound would be wasteful for a sparse, slowly-explored environment).
struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey &o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey &k) const
    {
        size_t h = static_cast<size_t>(k.x) * 73856093u;
        h ^= static_cast<size_t>(k.y) * 19349663u;
        h ^= static_cast<size_t>(k.z) * 83492791u;
        return h;
    }
};

struct SliceGrid {
    std::vector<int8_t> data;
    int rows = 0;
    int cols = 0;
    double z_center = 0.0;
};

struct LayerCoverageStats {
    double fraction = 0.0;
    double altitude_m = 0.0;
    uint32_t observed_cells = 0;
    uint32_t total_cells = 0;
};

enum class VoxelState : uint8_t {
    Unknown,
    Free,
    Occupied,
};

}  // namespace

// ── Live 3D log-odds occupancy/entropy voxel mapper (M3) ─────────────────────
// Back-projects the depth stream into world-frame points (same camera-mount
// transform chain as swarm_target_detection's _localize), ray-casts each point
// from the camera origin to mark traversed cells free and the endpoint
// occupied, and publishes a running entropy/coverage summary plus an occupied-
// voxel point cloud for live RViz visualisation.
class VoxelMapper : public rclcpp::Node
{
public:
    VoxelMapper() : Node("voxel_mapper")
    {
        // ── Parameters ────────────────────────────────────────────────────────
        drone_id_        = declare_parameter<int>("drone_id", 0);
        voxel_size_      = declare_parameter<double>("voxel_size_m", 0.2);
        log_odds_hit_    = declare_parameter<double>("log_odds_hit", 0.85);
        log_odds_miss_   = declare_parameter<double>("log_odds_miss", -0.4);
        log_odds_min_    = declare_parameter<double>("log_odds_min", -2.0);
        log_odds_max_    = declare_parameter<double>("log_odds_max", 3.5);
        occupied_prob_   = declare_parameter<double>("occupied_probability", 0.7);
        observed_delta_  = declare_parameter<double>("observed_probability_delta", 0.1);
        occupied_retain_prob_ =
            declare_parameter<double>("occupied_retain_probability", 0.75);
        occupied_miss_scale_ =
            declare_parameter<double>("occupied_miss_scale", 0.05);
        pixel_stride_    = declare_parameter<int>("pixel_stride", 8);
        max_range_       = declare_parameter<double>("max_range_m", 8.0);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
        use_slam_pose_   = declare_parameter<bool>("use_slam_pose", true);
        require_slam_pose_ =
            declare_parameter<bool>("require_slam_pose", false);
        integrate_lidar_into_map_ =
            declare_parameter<bool>("integrate_lidar_into_map", true);
        // Snap the coverage altitude label to the discrete scan-layer grid (1 m).
        // Coverage is keyed by altitude both here (dynamic_layer_coverage_) and in
        // map_merger; keying by the raw altitude created a fresh low-coverage bin
        // for every transient altitude crossed while climbing between layers, which
        // permanently inflated the global-coverage denominator. Snapping folds those
        // transient altitudes onto the nearest real layer.
        coverage_layer_snap_m_ =
            declare_parameter<double>("coverage_layer_snap_m", 1.0);
        coverage_use_reachable_component_ =
            declare_parameter<bool>("coverage_use_reachable_component", false);
        coverage_reachability_guard_enabled_ =
            declare_parameter<bool>("coverage_reachability_guard_enabled", true);
        coverage_denominator_drop_ratio_ =
            declare_parameter<double>("coverage_denominator_drop_ratio", 0.45);
        coverage_min_guard_total_cells_ =
            static_cast<uint32_t>(std::max<int64_t>(0,
                declare_parameter<int>("coverage_min_guard_total_cells", 500)));
        shared_map_enabled_ =
            declare_parameter<bool>("shared_map_enabled", false);
        shared_occupied_topic_ =
            declare_parameter<std::string>("shared_occupied_topic", "/merged_voxel_map");
        shared_free_topic_ =
            declare_parameter<std::string>("shared_free_topic", "/merged_free_voxel_map");
        slam_pose_timeout_s_ =
            declare_parameter<double>("slam_pose_timeout_s", 1.0);
        slice_half_thickness_m_ = declare_parameter<double>("slice_half_thickness_m", 0.6);
        frontier_cluster_radius_m_ = declare_parameter<double>("frontier_cluster_radius_m", 1.0);
        frontier_min_goal_distance_m_ = declare_parameter<double>("frontier_min_goal_distance_m", 1.0);
        frontier_clearance_cells_ = declare_parameter<int>("frontier_clearance_cells", 3);
        path_clearance_cells_     = declare_parameter<int>("path_clearance_cells", 3);
        obstacle_inflate_cells_   = declare_parameter<int>("obstacle_inflate_cells", 1);
        astar_clearance_penalty_weight_ =
            declare_parameter<double>("astar_clearance_penalty_weight", 12.0);
        path_shortcut_max_segment_m_ =
            declare_parameter<double>("path_shortcut_max_segment_m", 2.00);
        path_shortcut_near_clearance_m_ =
            declare_parameter<double>("path_shortcut_near_clearance_m", 0.80);
        path_shortcut_near_segment_m_ =
            declare_parameter<double>("path_shortcut_near_segment_m", 2.00);
        path_shortcut_mid_clearance_m_ =
            declare_parameter<double>("path_shortcut_mid_clearance_m", 1.40);
        path_shortcut_mid_segment_m_ =
            declare_parameter<double>("path_shortcut_mid_segment_m", 4.00);
        path_shortcut_line_clearance_m_ =
            declare_parameter<double>("path_shortcut_line_clearance_m", 0.40);
        path_smoothing_enabled_ =
            declare_parameter<bool>("path_smoothing_enabled", true);
        path_smoothing_iterations_ =
            declare_parameter<int>("path_smoothing_iterations", 2);
        path_smoothing_spacing_m_ =
            declare_parameter<double>("path_smoothing_spacing_m", 0.35);
        path_smoothing_clearance_m_ =
            declare_parameter<double>("path_smoothing_clearance_m", 0.45);
        path_smoothing_max_points_ =
            declare_parameter<int>("path_smoothing_max_points", 120);
        local_start_clearance_cells_ =
            declare_parameter<int>("local_start_clearance_cells", 2);
        obstacle_band_half_m_     = declare_parameter<double>("obstacle_band_half_m", 2.0);
        obstacle_band_high_alt_threshold_m_ =
            declare_parameter<double>("obstacle_band_high_alt_threshold_m", 5.0);
        obstacle_band_high_half_m_ =
            declare_parameter<double>("obstacle_band_high_half_m", 0.45);
        frontier_view_standoff_min_m_ =
            declare_parameter<double>("frontier_view_standoff_min_m", 1.0);
        frontier_view_standoff_max_m_ =
            declare_parameter<double>("frontier_view_standoff_max_m", 2.2);
        frontier_view_standoff_samples_ =
            declare_parameter<int>("frontier_view_standoff_samples", 3);
        frontier_view_angle_samples_ =
            declare_parameter<int>("frontier_view_angle_samples", 8);
        frontier_max_clusters_scored_ =
            declare_parameter<int>("frontier_max_clusters_scored", 16);
        frontier_candidate_clearance_m_ =
            declare_parameter<double>("frontier_candidate_clearance_m", 0.40);
        frontier_min_obstacle_clearance_m_ =
            declare_parameter<double>("frontier_min_obstacle_clearance_m", 0.40);
        frontier_world_boundary_clearance_m_ =
            declare_parameter<double>("frontier_world_boundary_clearance_m", 0.0);
        frontier_candidate_vertical_half_m_ =
            declare_parameter<double>("frontier_candidate_vertical_half_m", 0.45);
        frontier_gain_u_samples_ =
            declare_parameter<int>("frontier_gain_u_samples", 7);
        frontier_gain_v_samples_ =
            declare_parameter<int>("frontier_gain_v_samples", 4);
        frontier_cluster_weight_ =
            declare_parameter<double>("frontier_cluster_weight", 2.0);
        frontier_info_gain_weight_ =
            declare_parameter<double>("frontier_info_gain_weight", 3.5);
        frontier_distance_weight_ =
            declare_parameter<double>("frontier_distance_weight", 0.8);
        frontier_progress_weight_ =
            declare_parameter<double>("frontier_progress_weight", 0.0);
        frontier_yaw_weight_ =
            declare_parameter<double>("frontier_yaw_weight", 0.4);
        frontier_clearance_weight_ =
            declare_parameter<double>("frontier_clearance_weight", 3.0);
        frontier_awareness_weight_ =
            declare_parameter<double>("frontier_awareness_weight", 4.0);
        peer_frontier_goal_topic_ =
            declare_parameter<std::string>("peer_frontier_goal_topic", "");
        peer_exclusion_radius_m_ =
            (float)declare_parameter<double>("peer_frontier_exclusion_radius_m", 4.0);
        peer_exclusion_penalty_ =
            (float)declare_parameter<double>("peer_frontier_penalty", 3.0);
        home_north_m_ = declare_parameter<double>("home_north_m", 0.0);
        home_east_m_  = declare_parameter<double>("home_east_m", 0.0);
        // Bounds are in NED world frame: x=North, y=East, z=Down.
        // NED z is NEGATIVE above ground: z=-8 is 8m AGL, z=0 is ground level.
        // Default: room ±25m N/E, ceiling 10m AGL (z=-10) to 0.5m below ground (z=+0.5).
        const auto bmin = declare_parameter<std::vector<double>>(
            "map_bounds_min_m", {-25.0, -25.0, -10.0});
        const auto bmax = declare_parameter<std::vector<double>>(
            "map_bounds_max_m", {25.0, 25.0, 0.5});
        bounds_min_ = {bmin[0], bmin[1], bmin[2]};
        bounds_max_ = {bmax[0], bmax[1], bmax[2]};

        // World area mask: 3 axis-aligned rectangles in NED (x=North, y=East).
        // Default bounds match single_agent_search_room.sdf (Room A + Corridor + Room B).
        // Set world_area_mask_enabled=true in the launch file to activate.
        world_area_mask_enabled_ = declare_parameter<bool>("world_area_mask_enabled", false);
        if (world_area_mask_enabled_) {
            auto load_rect = [&](const std::string &name,
                                  std::vector<double> def) {
                const auto v = declare_parameter<std::vector<double>>(name, def);
                if (v.size() == 4)
                    world_area_rects_.push_back({v[0], v[1], v[2], v[3]});
            };
            load_rect("world_zone_a",        {-8.5,  8.5, -8.5,  8.5});
            load_rect("world_zone_corridor", {-2.5,  2.5,  7.5, 14.5});
            load_rect("world_zone_b",        {-8.5,  8.5, 13.5, 30.5});
            RCLCPP_INFO(get_logger(),
                "World area mask enabled with %zu zones", world_area_rects_.size());
        }

        // Precompute world_volume_voxels_: XY cells inside allowed zones × Z span.
        // Used as the coverage fraction denominator so the reported fraction reflects
        // the physically observable space rather than the full bounding box extents.
        {
            const int rows = std::max(1, static_cast<int>(
                std::ceil((bounds_max_.x - bounds_min_.x) / voxel_size_)));
            const int cols = std::max(1, static_cast<int>(
                std::ceil((bounds_max_.y - bounds_min_.y) / voxel_size_)));
            const int z_span = std::max(1, static_cast<int>(
                std::ceil((bounds_max_.z - bounds_min_.z) / voxel_size_)));
            uint64_t xy_count = 0;
            for (int r = 0; r < rows; ++r) {
                const double north = bounds_min_.x + (r + 0.5) * voxel_size_;
                for (int c = 0; c < cols; ++c) {
                    const double east = bounds_min_.y + (c + 0.5) * voxel_size_;
                    if (in_allowed_area(north, east)) ++xy_count;
                }
            }
            world_volume_voxels_ = std::max(1.0, static_cast<double>(xy_count) * z_span);
            RCLCPP_INFO(get_logger(),
                "Coverage denominator: %lu XY × %d Z = %.0f voxels (vs %.0f full bbox)",
                xy_count, z_span, world_volume_voxels_,
                static_cast<double>(rows) * cols * z_span);
        }

        // Fixed OakD-Lite mount offset in the FRD body frame — see the
        // rgbd_joint pose in swarm_sim_bringup/models/x500_swarm/model.sdf and
        // swarm_target_detection's matching `camera_offset_frd_m` parameter.
        const auto offset = declare_parameter<std::vector<double>>(
            "camera_offset_frd_m", {0.12, 0.03, 0.06});
        cam_offset_ = {offset[0], offset[1], offset[2]};
        camera_pitch_rad_ = declare_parameter<double>("camera_pitch_rad", 0.2617993877991494);  // 15° (π/12)

        // ── QoS ───────────────────────────────────────────────────────────────
        auto qos_sub = rclcpp::QoS(1).best_effort();

        // ── Subscriptions — relative names so namespace=px4_<i> maps them onto
        // /px4_<i>/camera/... and /px4_<i>/fmu/out/..., matching the convention
        // established in swarm_control/offboard_control.cpp.
        depth_sub_ = create_subscription<Image>(
            "camera/depth/image_raw", qos_sub,
            [this](Image::SharedPtr msg) { on_depth(msg); });
        info_sub_ = create_subscription<CameraInfo>(
            "camera/depth/camera_info", qos_sub,
            [this](CameraInfo::SharedPtr msg) { info_ = *msg; have_info_ = true; });
        odom_sub_ = create_subscription<VehicleOdometry>(
            "fmu/out/vehicle_odometry", qos_sub,
            [this](VehicleOdometry::SharedPtr msg) { odom_ = *msg; have_odom_ = true; });
        slam_odom_sub_ = create_subscription<Odometry>(
            "slam/odom_ned", qos_sub,
            [this](Odometry::SharedPtr msg) {
                slam_odom_ = *msg;
                have_slam_odom_ = true;
                last_slam_odom_ns_ = now().nanoseconds();
            });
        // 2-D LiDAR can be used for voxel hits, but the thesis/FUEL direction is
        // a single camera/VIO-driven exploration map. Keep LiDAR available for
        // VFH safety/SLAM while allowing voxel mapping to stay depth-only.
        if (integrate_lidar_into_map_) {
            lidar_sub_ = create_subscription<LaserScan>(
                "lidar/scan", qos_sub,
                [this](LaserScan::SharedPtr msg) { on_lidar_scan(msg); });
        }
        if (shared_map_enabled_) {
            shared_occupied_sub_ = create_subscription<PointCloud2>(
                shared_occupied_topic_, qos_sub,
                [this](PointCloud2::SharedPtr msg) { on_shared_cloud(msg, true); });
            shared_free_sub_ = create_subscription<PointCloud2>(
                shared_free_topic_, qos_sub,
                [this](PointCloud2::SharedPtr msg) { on_shared_cloud(msg, false); });
        }

        // ── Publishers ────────────────────────────────────────────────────────
        // Reliable for the periodic summary (matches the "found"/announcement
        // convention); best-effort for the point cloud since it's a high-volume
        // visualisation stream that the next publish will supersede anyway.
        summary_pub_ = create_publisher<MapUpdateSummary>(
            "map_update_summary", rclcpp::QoS(10));
        cloud_pub_ = create_publisher<PointCloud2>(
            "voxel_map", rclcpp::QoS(1).best_effort());
        free_cloud_pub_ = create_publisher<PointCloud2>(
            "free_voxel_map", rclcpp::QoS(1).best_effort());
        entropy_centroid_pub_ = create_publisher<PointStamped>(
            "entropy_centroid", rclcpp::QoS(10));
        slice_map_pub_ = create_publisher<OccupancyGrid>(
            "voxel_slice_map", rclcpp::QoS(1).transient_local().reliable());
        frontier_goal_pub_ = create_publisher<PointStamped>(
            "frontier_goal", rclcpp::QoS(10));
        frontier_goal_pose_pub_ = create_publisher<PoseStamped>(
            "frontier_goal_pose", rclcpp::QoS(10));
        frontier_path_pub_ = create_publisher<Path>(
            "frontier_path", rclcpp::QoS(10).transient_local());
        return_path_pub_ = create_publisher<Path>(
            "return_path", rclcpp::QoS(10).transient_local());
        obstacle_mask_pub_ = create_publisher<OccupancyGrid>(
            "obstacle_mask", rclcpp::QoS(1).best_effort());
        path_pub_ = create_publisher<Path>(
            "drone_path", rclcpp::QoS(1).best_effort());
        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        publish_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            [this]() { publish_summary_and_cloud(); });

        if (!peer_frontier_goal_topic_.empty()) {
            peer_frontier_sub_ = create_subscription<PointStamped>(
                peer_frontier_goal_topic_, rclcpp::QoS(10),
                [this](const PointStamped::SharedPtr m) {
                    peer_frontier_goal_ = *m;
                    have_peer_frontier_ = true;
                });
        }

        RCLCPP_INFO(get_logger(),
            "VoxelMapper ready — voxel=%.2f m, lidar_map=%s, shared_map=%s, "
            "bounds=[%.1f, %.1f] x [%.1f, %.1f] x [%.1f, %.1f]",
            voxel_size_, integrate_lidar_into_map_ ? "on" : "off",
            shared_map_enabled_ ? "on" : "off",
            bounds_min_.x, bounds_max_.x, bounds_min_.y, bounds_max_.y,
            bounds_min_.z, bounds_max_.z);
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────
    int    drone_id_;
    double voxel_size_, log_odds_hit_, log_odds_miss_, log_odds_min_, log_odds_max_;
    double occupied_prob_, observed_delta_, occupied_retain_prob_;
    double occupied_miss_scale_, max_range_, publish_rate_hz_;
    double slam_pose_timeout_s_;
    double camera_pitch_rad_, slice_half_thickness_m_;
    double frontier_cluster_radius_m_, frontier_min_goal_distance_m_;
    double frontier_view_standoff_min_m_, frontier_view_standoff_max_m_;
    double frontier_candidate_clearance_m_, frontier_min_obstacle_clearance_m_;
    double frontier_world_boundary_clearance_m_;
    double frontier_candidate_vertical_half_m_;
    double frontier_cluster_weight_, frontier_info_gain_weight_;
    double frontier_distance_weight_, frontier_progress_weight_, frontier_yaw_weight_;
    double frontier_clearance_weight_;
    double frontier_awareness_weight_;
    double astar_clearance_penalty_weight_ = 12.0;
    double path_shortcut_max_segment_m_ = 2.00;
    double path_shortcut_near_clearance_m_ = 0.80;
    double path_shortcut_near_segment_m_ = 2.00;
    double path_shortcut_mid_clearance_m_ = 1.40;
    double path_shortcut_mid_segment_m_ = 4.00;
    double path_shortcut_line_clearance_m_ = 0.40;
    bool   path_smoothing_enabled_ = true;
    int    path_smoothing_iterations_ = 2;
    double path_smoothing_spacing_m_ = 0.35;
    double path_smoothing_clearance_m_ = 0.45;
    int    path_smoothing_max_points_ = 120;
    double home_north_m_ = 0.0, home_east_m_ = 0.0;
    bool   integrate_lidar_into_map_ = true;
    bool   shared_map_enabled_ = false;
    std::string shared_occupied_topic_{"/merged_voxel_map"};
    std::string shared_free_topic_{"/merged_free_voxel_map"};

    // Failure-penalty tracking: remember the last published goal and the drone
    // position at that time.  If the drone hasn't moved on the next planning tick
    // and would select the same cluster again, apply a score penalty to force it
    // elsewhere.
    double frontier_last_goal_n_    = std::numeric_limits<double>::quiet_NaN();
    double frontier_last_goal_e_    = std::numeric_limits<double>::quiet_NaN();
    double frontier_last_drone_n_   = std::numeric_limits<double>::quiet_NaN();
    double frontier_last_drone_e_   = std::numeric_limits<double>::quiet_NaN();
    // Persistence: score of the last published goal cluster.  A competing
    // frontier must exceed this by kGoalPersistenceBonus to win the selection.
    // Prevents flip-flopping between equally-scored frontiers (path thrashing).
    double frontier_last_best_score_ = -std::numeric_limits<double>::infinity();

    int    pixel_stride_, frontier_clearance_cells_, path_clearance_cells_;
    int    obstacle_inflate_cells_, local_start_clearance_cells_;
    double obstacle_band_half_m_;
    double obstacle_band_high_alt_threshold_m_;
    double obstacle_band_high_half_m_;
    int    frontier_view_standoff_samples_, frontier_view_angle_samples_;
    int    frontier_max_clusters_scored_;
    int    frontier_gain_u_samples_, frontier_gain_v_samples_;
    Vec3   bounds_min_{}, bounds_max_{}, cam_offset_{};
    bool   use_slam_pose_ = true;
    bool   require_slam_pose_ = false;


    // Allowed-area mask: limits ray-cast updates and frontier candidates to
    // the known physical space (two rooms + corridor). Prevents ghost free
    // voxels behind exterior walls from becoming navigation goals.
    struct AreaRect { double n_min, n_max, e_min, e_max; };
    std::vector<AreaRect> world_area_rects_;
    bool   world_area_mask_enabled_ = false;
    double world_volume_voxels_     = 1.0;   // diagnostic fixed-volume denominator
    std::unordered_map<int, LayerCoverageStats> dynamic_layer_coverage_;
    std::unordered_map<int, LayerCoverageStats> last_good_layer_coverage_;
    double coverage_layer_snap_m_ = 1.0;
    bool   coverage_use_reachable_component_ = false;
    bool   coverage_reachability_guard_enabled_ = true;
    double coverage_denominator_drop_ratio_ = 0.45;
    uint32_t coverage_min_guard_total_cells_ = 500;
    uint32_t frontier_clusters_count_ = 0;
    uint32_t reachable_frontier_clusters_count_ = 0;
    uint32_t reachable_frontier_cells_count_ = 0;
    bool     frontier_route_available_ = false;

    // ── Cached telemetry ──────────────────────────────────────────────────────
    CameraInfo      info_{};
    VehicleOdometry odom_{};
    Odometry        slam_odom_{};
    bool have_info_ = false;
    bool have_odom_ = false;
    bool have_slam_odom_ = false;
    int64_t last_slam_odom_ns_ = 0;

    // ── Map state ─────────────────────────────────────────────────────────────
    std::unordered_map<VoxelKey, float, VoxelKeyHash> voxels_;
    std::unordered_set<VoxelKey, VoxelKeyHash> shared_occupied_voxels_;
    std::unordered_set<VoxelKey, VoxelKeyHash> shared_free_voxels_;
    uint32_t voxels_touched_since_publish_ = 0;

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::Subscription<Image>::SharedPtr           depth_sub_;
    rclcpp::Subscription<CameraInfo>::SharedPtr      info_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr        slam_odom_sub_;
    rclcpp::Subscription<LaserScan>::SharedPtr       lidar_sub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr      shared_occupied_sub_;
    rclcpp::Subscription<PointCloud2>::SharedPtr      shared_free_sub_;
    rclcpp::Publisher<MapUpdateSummary>::SharedPtr   summary_pub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr        cloud_pub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr        free_cloud_pub_;
    uint32_t                                         free_cloud_tick_ = 0;
    rclcpp::Publisher<PointStamped>::SharedPtr       entropy_centroid_pub_;
    rclcpp::Publisher<OccupancyGrid>::SharedPtr      slice_map_pub_;
    rclcpp::Publisher<PointStamped>::SharedPtr       frontier_goal_pub_;
    // Peer frontier penalty — set by dual-drone launch; empty = disabled.
    std::string  peer_frontier_goal_topic_{};
    float        peer_exclusion_radius_m_ = 4.0f;
    float        peer_exclusion_penalty_  = 3.0f;
    PointStamped peer_frontier_goal_{};
    bool         have_peer_frontier_ = false;
    rclcpp::Subscription<PointStamped>::SharedPtr    peer_frontier_sub_;
    rclcpp::Publisher<PoseStamped>::SharedPtr        frontier_goal_pose_pub_;
    rclcpp::Publisher<Path>::SharedPtr               frontier_path_pub_;
    rclcpp::Publisher<Path>::SharedPtr               return_path_pub_;
    rclcpp::Publisher<OccupancyGrid>::SharedPtr      obstacle_mask_pub_;
    rclcpp::Publisher<Path>::SharedPtr               path_pub_;

    rclcpp::TimerBase::SharedPtr                     publish_timer_;
    Path                                             drone_path_msg_{};

    // ── Helpers ───────────────────────────────────────────────────────────────
    VoxelKey world_to_key(const Vec3 &p) const
    {
        return {
            static_cast<int32_t>(std::floor(p.x / voxel_size_)),
            static_cast<int32_t>(std::floor(p.y / voxel_size_)),
            static_cast<int32_t>(std::floor(p.z / voxel_size_)),
        };
    }

    Vec3 key_to_center(const VoxelKey &k) const
    {
        return {
            (k.x + 0.5) * voxel_size_,
            (k.y + 0.5) * voxel_size_,
            (k.z + 0.5) * voxel_size_,
        };
    }

    bool in_bounds(const Vec3 &p) const
    {
        return p.x >= bounds_min_.x && p.x <= bounds_max_.x
            && p.y >= bounds_min_.y && p.y <= bounds_max_.y
            && p.z >= bounds_min_.z && p.z <= bounds_max_.z;
    }

    // Returns false when the point is outside the allowed physical space (e.g.
    // behind an exterior wall). When the mask is disabled, always returns true.
    bool in_allowed_area(double x_north, double y_east) const
    {
        if (!world_area_mask_enabled_) return true;
        for (const auto &r : world_area_rects_) {
            if (x_north >= r.n_min && x_north <= r.n_max &&
                y_east  >= r.e_min && y_east  <= r.e_max) return true;
        }
        return false;
    }

    bool in_allowed_area_with_clearance(double x_north, double y_east,
                                        double clearance_m) const
    {
        if (!world_area_mask_enabled_) return true;
        const double c = std::max(0.0, clearance_m);
        for (const auto &r : world_area_rects_) {
            if (x_north >= r.n_min + c && x_north <= r.n_max - c &&
                y_east  >= r.e_min + c && y_east  <= r.e_max - c) return true;
        }
        return false;
    }

    bool is_high_layer_room_connector(double x_north, double y_east,
                                      double z_center) const
    {
        const double alt_m = -z_center;
        if (alt_m < obstacle_band_high_alt_threshold_m_) {
            return false;
        }
        // Known geometry: the Room A <-> corridor <-> Room B centreline is open.
        // At high layers, sparse/occluded voxel slices can inflate the doorway or
        // corridor walls enough to split the planning graph.  Keep a narrow
        // planning spine open; VFH still enforces live obstacle safety while flying.
        return x_north >= -0.80 && x_north <= 0.80 &&
               y_east  >=  6.80 && y_east  <= 23.00;
    }

    void force_high_layer_room_connector(std::vector<bool> &passable,
                                         const SliceGrid &grid,
                                         double z_center) const
    {
        if (-z_center < obstacle_band_high_alt_threshold_m_) {
            return;
        }
        for (int r = 0; r < grid.rows; ++r) {
            for (int c = 0; c < grid.cols; ++c) {
                double north = 0.0;
                double east = 0.0;
                slice_cell_to_world(r, c, north, east);
                if (is_high_layer_room_connector(north, east, z_center) &&
                    in_allowed_area(north, east)) {
                    passable[static_cast<size_t>(r * grid.cols + c)] = true;
                }
            }
        }
    }

    bool point_to_slice_cell(double north, double east, int &row, int &col) const
    {
        row = static_cast<int>(std::floor((north - bounds_min_.x) / voxel_size_));
        col = static_cast<int>(std::floor((east  - bounds_min_.y) / voxel_size_));
        const int rows = std::max(1, static_cast<int>(
            std::ceil((bounds_max_.x - bounds_min_.x) / voxel_size_)));
        const int cols = std::max(1, static_cast<int>(
            std::ceil((bounds_max_.y - bounds_min_.y) / voxel_size_)));
        return row >= 0 && row < rows && col >= 0 && col < cols;
    }

    int slice_index(const SliceGrid &grid, int row, int col) const
    {
        return row * grid.cols + col;
    }

    VoxelState voxel_state(const VoxelKey &key) const
    {
        const auto it = voxels_.find(key);
        if (it == voxels_.end()) {
            return VoxelState::Unknown;
        }
        const double p = sigmoid(it->second);
        if (p > occupied_prob_) {
            return VoxelState::Occupied;
        }
        if (std::abs(p - 0.5) > observed_delta_) {
            return VoxelState::Free;
        }
        return VoxelState::Unknown;
    }

    VoxelState voxel_state_at(const Vec3 &p) const
    {
        if (!in_bounds(p)) {
            return VoxelState::Occupied;
        }
        return voxel_state(world_to_key(p));
    }

    double current_yaw() const
    {
        return yaw_from_quat_ned(active_quat());
    }

    bool slam_pose_fresh() const
    {
        if (!use_slam_pose_ || !have_slam_odom_ || last_slam_odom_ns_ <= 0) {
            return false;
        }
        const int64_t age_ns = now().nanoseconds() - last_slam_odom_ns_;
        return age_ns >= 0 &&
               age_ns <= static_cast<int64_t>(slam_pose_timeout_s_ * 1.0e9);
    }

    bool have_active_pose() const
    {
        if (slam_pose_fresh()) return true;
        if (use_slam_pose_ && require_slam_pose_) return false;
        if (!have_odom_) return false;
        // PX4 publishes NaN when EKF hasn't converged yet; also reject stray
        // large values that can occur before the first valid EKF estimate
        return std::isfinite(odom_.position[0]) && std::abs(odom_.position[0]) < 500.0f
            && std::isfinite(odom_.position[1]) && std::abs(odom_.position[1]) < 500.0f
            && std::isfinite(odom_.position[2]) && std::abs(odom_.position[2]) < 200.0f;
    }

    Vec3 active_position() const
    {
        if (slam_pose_fresh()) {
            return {
                slam_odom_.pose.pose.position.x,
                slam_odom_.pose.pose.position.y,
                slam_odom_.pose.pose.position.z,
            };
        }
        if (use_slam_pose_ && require_slam_pose_) {
            return {0.0, 0.0, 0.0};
        }
        return {odom_.position[0], odom_.position[1], odom_.position[2]};
    }

    std::array<float, 4> active_quat() const
    {
        if (slam_pose_fresh()) {
            return {
                static_cast<float>(slam_odom_.pose.pose.orientation.w),
                static_cast<float>(slam_odom_.pose.pose.orientation.x),
                static_cast<float>(slam_odom_.pose.pose.orientation.y),
                static_cast<float>(slam_odom_.pose.pose.orientation.z),
            };
        }
        if (use_slam_pose_ && require_slam_pose_) {
            return {1.0f, 0.0f, 0.0f, 0.0f};
        }
        return {odom_.q[0], odom_.q[1], odom_.q[2], odom_.q[3]};
    }

    bool volume_clear(const Vec3 &center, double radial_clearance_m,
                      double vertical_half_m) const
    {
        if (!in_bounds(center)) {
            return false;
        }

        const int radial_cells =
            std::max(1, static_cast<int>(std::ceil(radial_clearance_m / voxel_size_)));
        const int vertical_cells =
            std::max(1, static_cast<int>(std::ceil(vertical_half_m / voxel_size_)));
        const VoxelKey ck = world_to_key(center);

        for (int dz = -vertical_cells; dz <= vertical_cells; ++dz) {
            for (int dx = -radial_cells; dx <= radial_cells; ++dx) {
                for (int dy = -radial_cells; dy <= radial_cells; ++dy) {
                    if (dx * dx + dy * dy > radial_cells * radial_cells) {
                        continue;
                    }

                    const VoxelKey key{ck.x + dx, ck.y + dy, ck.z + dz};
                    if (voxel_state(key) == VoxelState::Occupied) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    double sample_axis(int index, int samples, int extent) const
    {
        if (samples <= 1) {
            return 0.5 * static_cast<double>(extent - 1);
        }
        return static_cast<double>(index) * static_cast<double>(extent - 1) /
               static_cast<double>(samples - 1);
    }

    double estimate_information_gain(const Vec3 &candidate_pos, double candidate_yaw) const
    {
        if (!have_info_ || info_.width == 0 || info_.height == 0) {
            return 0.0;
        }

        const auto &K = info_.k;
        const double fx = K[0];
        const double fy = K[4];
        const double cx = K[2];
        const double cy = K[5];
        if (fx == 0.0 || fy == 0.0) {
            return 0.0;
        }

        const Vec3 cam_world = candidate_pos + rotate_yaw_ned(cam_offset_, candidate_yaw);
        const int u_samples = std::max(2, frontier_gain_u_samples_);
        const int v_samples = std::max(2, frontier_gain_v_samples_);
        const double step = voxel_size_ * 0.75;
        std::unordered_set<VoxelKey, VoxelKeyHash> visible_unknown;

        for (int vi = 0; vi < v_samples; ++vi) {
            const double v = sample_axis(vi, v_samples, static_cast<int>(info_.height));
            for (int ui = 0; ui < u_samples; ++ui) {
                const double u = sample_axis(ui, u_samples, static_cast<int>(info_.width));

                const double x_opt = (u - cx) / fx;
                const double y_opt = (v - cy) / fy;
                const Vec3 body_frd = pitch_about_body_y(
                    {1.0, x_opt, y_opt}, camera_pitch_rad_);
                Vec3 dir_world = rotate_yaw_ned(body_frd, candidate_yaw);
                const double dir_norm = std::sqrt(dir_world.x * dir_world.x +
                                                  dir_world.y * dir_world.y +
                                                  dir_world.z * dir_world.z);
                if (dir_norm < 1e-6) {
                    continue;
                }
                dir_world = {
                    dir_world.x / dir_norm,
                    dir_world.y / dir_norm,
                    dir_world.z / dir_norm,
                };

                // Only count unknown voxels within the current altitude slice.
                // This prevents ceiling / floor voxels from inflating the score
                // and causing premature layer climbing (keyhole effect).
                const double z_lo = candidate_pos.z - slice_half_thickness_m_;
                const double z_hi = candidate_pos.z + slice_half_thickness_m_;

                for (double s = step; s <= max_range_; s += step) {
                    const Vec3 p{
                        cam_world.x + dir_world.x * s,
                        cam_world.y + dir_world.y * s,
                        cam_world.z + dir_world.z * s,
                    };
                    if (!in_bounds(p)) {
                        break;
                    }

                    const VoxelKey key = world_to_key(p);
                    const VoxelState state = voxel_state(key);
                    if (state == VoxelState::Occupied) {
                        break;
                    }
                    if (state == VoxelState::Unknown && p.z >= z_lo && p.z <= z_hi) {
                        visible_unknown.insert(key);
                    }
                }
            }
        }

        return static_cast<double>(visible_unknown.size());
    }

    int8_t slice_cell(const SliceGrid &grid, int row, int col) const
    {
        if (row < 0 || row >= grid.rows || col < 0 || col >= grid.cols) {
            return 100;
        }
        return grid.data[slice_index(grid, row, col)];
    }

    void force_local_start_clearance(std::vector<bool> &passable,
                                     const SliceGrid &grid,
                                     int start_r,
                                     int start_c) const
    {
        // 1. Radius bubble around drone start position.
        const int radius = std::max(0, local_start_clearance_cells_);
        for (int dr = -radius; dr <= radius; ++dr) {
            for (int dc = -radius; dc <= radius; ++dc) {
                if (dr * dr + dc * dc > radius * radius) {
                    continue;
                }
                const int r = start_r + dr;
                const int c = start_c + dc;
                if (r < 0 || r >= grid.rows || c < 0 || c >= grid.cols) {
                    continue;
                }
                double north = 0.0;
                double east = 0.0;
                slice_cell_to_world(r, c, north, east);
                if (!in_allowed_area(north, east)) {
                    continue;
                }
                passable[static_cast<size_t>(r * grid.cols + c)] = true;
            }
        }

        // 2. Small bubble around home. Do not force a whole drone→home corridor
        // passable: that can make A* plan through real shelves/walls and leave
        // VFH to stop the drone at execution time.
        int hr = 0, hc = 0;
        if (!point_to_slice_cell(home_north_m_, home_east_m_, hr, hc)) {
            return;
        }
        // Small bubble around home.
        for (int dr = -2; dr <= 2; ++dr) {
            for (int dc = -2; dc <= 2; ++dc) {
                const int r = hr + dr;
                const int c = hc + dc;
                if (r < 0 || r >= grid.rows || c < 0 || c >= grid.cols) continue;
                double north = 0.0, east = 0.0;
                slice_cell_to_world(r, c, north, east);
                if (!in_allowed_area(north, east)) continue;
                passable[static_cast<size_t>(r * grid.cols + c)] = true;
            }
        }
    }

    // Build a passable map: cell (r,c) is passable iff it is NOT confirmed-occupied
    // (100), AND no cell within path_clearance_cells_ radius is confirmed-occupied.
    // Unknown (-1) centres inside the physical world mask are allowed for route
    // topology. VFH handles live clearance while executing, and the endpoint itself
    // is still required to be known-free before it can become a viewpoint. Requiring
    // every route cell to be confirmed-free creates disconnected islands at room
    // edges: the mapper sees a valid frontier, but a thin unknown band in the current
    // altitude slice makes BFS reject every continuation.
    // A small bubble around the drone is force-enabled. A transient occupied voxel
    // on/near the vehicle otherwise gets inflated by obstacle_inflate_cells_ and
    // blocks every neighbour while the drone is physically able to leave. VFH still
    // enforces real local obstacle safety during execution.
    std::vector<bool> compute_passable_map(const SliceGrid &grid,
                                            int start_r, int start_c) const
    {
        // Use build_obstacle_mask() instead of the narrow slice (±0.6 m).
        // The mask projects occupied voxels from ±obstacle_band_half_m_ so
        // walls mapped at any height are correctly blocked, and inflates by
        // obstacle_inflate_cells_ for planner-level clearance.
        // This fixes BFS routing through walls that are only mapped above/below the
        // drone's current altitude slice.
        const auto blocked = build_obstacle_mask(grid);
        const int  N       = grid.rows * grid.cols;
        std::vector<bool> passable(static_cast<size_t>(N), false);
        for (int r = 0; r < grid.rows; ++r) {
            for (int c = 0; c < grid.cols; ++c) {
                const int i = r * grid.cols + c;
                double north = 0.0;
                double east = 0.0;
                slice_cell_to_world(r, c, north, east);
                passable[static_cast<size_t>(i)] =
                    !blocked[static_cast<size_t>(i)] &&
                    in_allowed_area(north, east);
            }
        }
        force_local_start_clearance(passable, grid, start_r, start_c);
        force_high_layer_room_connector(passable, grid, grid.z_center);
        return passable;
    }

    // BFS flood-fill from (start_r, start_c) through passable cells.
    // Returns a bit-map: reachable[r*cols+c] == true means the cell has a
    // confirmed-free inflated path from the drone.  Computed once per planning
    // tick and reused for all viewpoint candidates.
    std::vector<bool> compute_reachable_from(const std::vector<bool> &passable,
                                              int rows, int cols,
                                              int start_r, int start_c) const
    {
        const int N = rows * cols;
        std::vector<bool> reachable(static_cast<size_t>(N), false);
        const size_t si = static_cast<size_t>(start_r * cols + start_c);
        if (!passable[si]) return reachable;

        std::queue<int> q;
        reachable[si] = true;
        q.push(static_cast<int>(si));

        static constexpr std::array<std::pair<int, int>, 4> kNeighbours4{{
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        }};

        while (!q.empty()) {
            const int cur = q.front(); q.pop();
            const int r   = cur / cols;
            const int c   = cur % cols;
            for (const auto &[dr, dc] : kNeighbours4) {
                const int nr = r + dr, nc = c + dc;
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                const int nidx = nr * cols + nc;
                if (reachable[static_cast<size_t>(nidx)]) continue;
                if (!passable[static_cast<size_t>(nidx)]) continue;
                reachable[static_cast<size_t>(nidx)] = true;
                q.push(nidx);
            }
        }
        return reachable;
    }


    // BFS distance transform: each cell gets the distance (in cells) to the
    // nearest blocked planning cell.  When an inflated obstacle mask is supplied,
    // use it as the source of truth so goal scoring, A*, shortcutting, and path
    // smoothing all agree on the same clearance model.  Cells outside the allowed
    // world area are also treated as obstacles, so the planner prefers room and
    // corridor centrelines instead of hugging scan-boundary walls.
    std::vector<int> compute_obstacle_dist_map(
        const SliceGrid &grid,
        const std::vector<bool> *blocked_mask = nullptr) const
    {
        const int N   = grid.rows * grid.cols;
        const int INF = 9999;
        std::vector<int> dist(static_cast<size_t>(N), INF);
        std::queue<int>  q;

        for (int r = 0; r < grid.rows; ++r) {
            for (int c = 0; c < grid.cols; ++c) {
                const int idx = r * grid.cols + c;
                bool blocked = slice_cell(grid, r, c) == 100;
                if (blocked_mask != nullptr &&
                    static_cast<size_t>(idx) < blocked_mask->size()) {
                    blocked = blocked || (*blocked_mask)[static_cast<size_t>(idx)];
                }
                if (world_area_mask_enabled_) {
                    double north = 0.0;
                    double east = 0.0;
                    slice_cell_to_world(r, c, north, east);
                    blocked = blocked || !in_allowed_area(north, east);
                }
                if (blocked) {
                    dist[static_cast<size_t>(idx)] = 0;
                    q.push(idx);
                }
            }
        }
        while (!q.empty()) {
            const int cur = q.front(); q.pop();
            const int r   = cur / grid.cols;
            const int c   = cur % grid.cols;
            const int d   = dist[static_cast<size_t>(cur)];
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    if (dr == 0 && dc == 0) continue;
                    const int nr = r + dr, nc = c + dc;
                    if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) continue;
                    const int nidx = nr * grid.cols + nc;
                    if (dist[static_cast<size_t>(nidx)] <= d + 1) continue;
                    dist[static_cast<size_t>(nidx)] = d + 1;
                    q.push(nidx);
                }
            }
        }
        return dist;
    }

    // BFS distance map from drone cell through the selected passable cells.
    // Returns distances in grid cells; 9999 = unreachable.
    // Used to score frontier candidates by path length, not Euclidean distance.
    std::vector<int> compute_path_dist_map(
        const std::vector<bool> &passable, int rows, int cols,
        int start_r, int start_c) const
    {
        const int N = rows * cols;
        std::vector<int> dist(static_cast<size_t>(N), 9999);
        std::queue<int> q;
        const int si = start_r * cols + start_c;
        if (si < 0 || si >= N) return dist;
        dist[static_cast<size_t>(si)] = 0;
        q.push(si);
        const int dr4[] = {-1, 1, 0, 0};
        const int dc4[] = { 0, 0,-1, 1};
        while (!q.empty()) {
            const int cur = q.front(); q.pop();
            const int r = cur / cols, c = cur % cols;
            const int d = dist[static_cast<size_t>(cur)];
            for (int k = 0; k < 4; ++k) {
                const int nr = r + dr4[k], nc = c + dc4[k];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                const int ni = nr * cols + nc;
                if (!passable[static_cast<size_t>(ni)]) continue;
                if (dist[static_cast<size_t>(ni)] <= d + 1) continue;
                dist[static_cast<size_t>(ni)] = d + 1;
                q.push(ni);
            }
        }
        return dist;
    }

    // Fraction of cells within kR cells (Euclidean) of (row,col) that are
    // confirmed (slice_cell != -1).  Returns 0..1.  Used to score viewpoints:
    // a viewpoint surrounded by known cells is safer than one at the edge of
    // the mapped area.  radius=3 cells = 0.6 m covers the drone's reach.
    double local_awareness(const SliceGrid &grid, int row, int col,
                           int radius = 3) const
    {
        int known = 0, total = 0;
        for (int dr = -radius; dr <= radius; ++dr) {
            for (int dc = -radius; dc <= radius; ++dc) {
                if (dr * dr + dc * dc > radius * radius) continue;
                const int nr = row + dr, nc = col + dc;
                if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) continue;
                ++total;
                if (slice_cell(grid, nr, nc) != -1) ++known;
            }
        }
        return total > 0 ? static_cast<double>(known) / total : 0.0;
    }

    // A* path from drone cell to goal cell through passable cells (4-connected).
    // Returns {row,col} vector from start to goal inclusive; empty if unreachable.
    // A* with Voronoi-retraction cost: edge cost = 1 + W/obs_dist so the path
    // prefers the skeleton (corridor centre / room equidistant ridge) over the
    // geometrically shorter but wall-hugging shortest path.
    // With W=12: obs_dist=2 → penalty 6.0 (wall-adjacent);
    // obs_dist=6+ → penalty ≤ 2.0
    // (open space).  Result: in a 1.5 m corridor the centre costs 1.5–2.0 per
    // step vs 3.0 for wall-adjacent cells — A* routes down the middle.
    std::vector<std::pair<int,int>> compute_astar_path(
        const std::vector<bool> &passable, int rows, int cols,
        int start_r, int start_c, int goal_r, int goal_c,
        const std::vector<int> &obs_dist_map) const
    {
        if (start_r == goal_r && start_c == goal_c)
            return {{start_r, start_c}};
        const int N  = rows * cols;
        const int si = start_r * cols + start_c;
        const int gi = goal_r  * cols + goal_c;
        if (si < 0 || si >= N || gi < 0 || gi >= N) return {};
        if (!passable[static_cast<size_t>(gi)]) return {};

        std::vector<float> g(static_cast<size_t>(N),
                              std::numeric_limits<float>::infinity());
        std::vector<int>   parent(static_cast<size_t>(N), -1);
        using Node = std::pair<float, int>;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

        g[static_cast<size_t>(si)] = 0.0f;
        open.push({std::hypot(float(start_r - goal_r), float(start_c - goal_c)), si});

        const int dr4[] = {-1, 1, 0, 0};
        const int dc4[] = { 0, 0,-1, 1};
        while (!open.empty()) {
            auto [f, cur] = open.top(); open.pop();
            if (cur == gi) break;
            const int r = cur / cols, c = cur % cols;
            const float g_cur = g[static_cast<size_t>(cur)];
            const float h_cur = std::hypot(float(r - goal_r), float(c - goal_c));
            if (f > g_cur + h_cur + 1e-3f) continue;  // stale
            for (int k = 0; k < 4; ++k) {
                const int nr = r + dr4[k], nc = c + dc4[k];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                const int ni = nr * cols + nc;
                if (!passable[static_cast<size_t>(ni)]) continue;
                // Voronoi clearance penalty: penalise cells close to obstacles so
                // paths hug the skeleton (equidistant from walls) rather than edges.
                const int obs_d = obs_dist_map[static_cast<size_t>(ni)];
                const float clearance_penalty =
                    static_cast<float>(astar_clearance_penalty_weight_) /
                    static_cast<float>(std::max(1, obs_d));
                const float ng = g_cur + 1.0f + clearance_penalty;
                if (ng >= g[static_cast<size_t>(ni)]) continue;
                g[static_cast<size_t>(ni)] = ng;
                parent[static_cast<size_t>(ni)] = cur;
                open.push({ng + std::hypot(float(nr - goal_r), float(nc - goal_c)), ni});
            }
        }
        if (parent[static_cast<size_t>(gi)] == -1) return {};

        std::vector<std::pair<int,int>> path;
        for (int cur = gi; cur != -1; cur = parent[static_cast<size_t>(cur)])
            path.push_back({cur / cols, cur % cols});
        std::reverse(path.begin(), path.end());
        return path;
    }

    // 3D A* from (start_r,start_c,start_zi) to (goal_r,goal_c,goal_zi).
    // passable_3d[zi][r*cols+c] = true means the cell is traversable at layer zi.
    // 18-connectivity (6 face + 12 edge). Z moves carry a mild extra cost so the
    // planner prefers horizontal routes but will climb/descend when blocked.
    std::vector<std::tuple<int,int,int>> compute_astar_3d_path(
        const std::vector<std::vector<bool>> &passable_3d,
        int rows, int cols,
        int start_r, int start_c, int start_zi,
        int goal_r,  int goal_c,  int goal_zi,
        const std::vector<int> &obs_dist_2d) const
    {
        const int n_layers = static_cast<int>(passable_3d.size());
        const int N3 = rows * cols * n_layers;

        if (start_r == goal_r && start_c == goal_c && start_zi == goal_zi)
            return {{start_r, start_c, start_zi}};

        const int si = start_zi * rows * cols + start_r * cols + start_c;
        const int gi = goal_zi  * rows * cols + goal_r  * cols + goal_c;
        if (si < 0 || si >= N3 || gi < 0 || gi >= N3) return {};
        if (!passable_3d[static_cast<size_t>(goal_zi)][
                static_cast<size_t>(goal_r * cols + goal_c)]) return {};

        std::vector<float> g(static_cast<size_t>(N3),
                              std::numeric_limits<float>::infinity());
        std::vector<int>   parent(static_cast<size_t>(N3), -1);
        using Node = std::pair<float, int>;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

        g[static_cast<size_t>(si)] = 0.0f;
        open.push({std::hypot(std::hypot(float(start_r - goal_r),
                                          float(start_c - goal_c)),
                               float(start_zi - goal_zi)), si});

        // 18-connectivity: 6 face-adjacent + 12 edge-adjacent (no corner diagonals)
        static constexpr int kN = 18;
        static constexpr int kDR[kN] = {-1,1,0,0,0,0, -1,-1,1,1,-1,-1,1,1, 0,0,0,0};
        static constexpr int kDC[kN] = { 0,0,-1,1,0,0, -1,1,-1,1, 0,0,0,0,-1,-1,1,1};
        static constexpr int kDZ[kN] = { 0,0,0,0,-1,1,  0,0,0,0,-1,1,-1,1,-1,1,-1,1};

        while (!open.empty()) {
            auto [f, cur] = open.top(); open.pop();
            if (cur == gi) break;

            const int zi = cur / (rows * cols);
            const int rc = cur % (rows * cols);
            const int r  = rc / cols;
            const int c  = rc % cols;
            const float g_cur = g[static_cast<size_t>(cur)];
            const float h_cur = std::hypot(
                std::hypot(float(r - goal_r), float(c - goal_c)),
                float(zi - goal_zi));
            if (f > g_cur + h_cur + 1e-3f) continue;  // stale entry

            for (int k = 0; k < kN; ++k) {
                const int nr = r + kDR[k], nc = c + kDC[k], nzi = zi + kDZ[k];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                if (nzi < 0 || nzi >= n_layers) continue;
                if (!passable_3d[static_cast<size_t>(nzi)][
                        static_cast<size_t>(nr * cols + nc)]) continue;

                const float xy   = std::sqrt(float(kDR[k]*kDR[k] + kDC[k]*kDC[k]));
                const float move = xy + (kDZ[k] != 0 ? 1.5f : 0.0f);
                const float clear_pen =
                    static_cast<float>(astar_clearance_penalty_weight_) /
                    float(std::max(1, obs_dist_2d[static_cast<size_t>(nr * cols + nc)]));
                const float ng = g_cur + move + clear_pen;

                const int ni = nzi * rows * cols + nr * cols + nc;
                if (ng >= g[static_cast<size_t>(ni)]) continue;
                g[static_cast<size_t>(ni)] = ng;
                parent[static_cast<size_t>(ni)] = cur;
                const float h = std::hypot(
                    std::hypot(float(nr - goal_r), float(nc - goal_c)),
                    float(nzi - goal_zi));
                open.push({ng + h, ni});
            }
        }

        if (parent[static_cast<size_t>(gi)] == -1) return {};

        std::vector<std::tuple<int,int,int>> path;
        for (int cur = gi; cur != -1; cur = parent[static_cast<size_t>(cur)]) {
            const int nzi = cur / (rows * cols);
            const int rc  = cur % (rows * cols);
            path.emplace_back(rc / cols, rc % cols, nzi);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    bool slice_clearance_ok(const SliceGrid &grid, int row, int col) const
    {
        for (int dr = -frontier_clearance_cells_; dr <= frontier_clearance_cells_; ++dr) {
            for (int dc = -frontier_clearance_cells_; dc <= frontier_clearance_cells_; ++dc) {
                if (dr * dr + dc * dc > frontier_clearance_cells_ * frontier_clearance_cells_) {
                    continue;
                }
                if (slice_cell(grid, row + dr, col + dc) == 100) {
                    return false;
                }
            }
        }
        return true;
    }

    // Stricter version for viewpoint placement: viewpoint must be in confirmed-free space.
    // Rejects both occupied (100) and unscanned (-1) within the clearance radius.
    bool slice_viewpoint_clearance_ok(const SliceGrid &grid, int row, int col) const
    {
        for (int dr = -frontier_clearance_cells_; dr <= frontier_clearance_cells_; ++dr) {
            for (int dc = -frontier_clearance_cells_; dc <= frontier_clearance_cells_; ++dc) {
                if (dr * dr + dc * dc > frontier_clearance_cells_ * frontier_clearance_cells_) {
                    continue;
                }
                const int8_t v = slice_cell(grid, row + dr, col + dc);
                if (v == 100) {   // reject only confirmed-occupied; unknown (-1) allowed
                    return false;  // obstacle_mask + volume_clear protect against unknown walls
                }
            }
        }
        return true;
    }

    bool slice_is_frontier(const SliceGrid &grid, int row, int col) const
    {
        if (slice_cell(grid, row, col) != 0 || !slice_clearance_ok(grid, row, col)) {
            return false;
        }

        bool has_unknown_neighbour = false;
        for (int dr = -1; dr <= 1; ++dr) {
            for (int dc = -1; dc <= 1; ++dc) {
                if (dr == 0 && dc == 0) {
                    continue;
                }
                const int nr = row + dr;
                const int nc = col + dc;
                if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) {
                    continue;
                }
                if (slice_cell(grid, nr, nc) == -1) {
                    has_unknown_neighbour = true;
                }
            }
        }
        return has_unknown_neighbour;
    }

    bool slice_line_clear(const SliceGrid &grid, int r0, int c0, int r1, int c1) const
    {
        int dr = std::abs(r1 - r0);
        int dc = std::abs(c1 - c0);
        int sr = (r0 < r1) ? 1 : -1;
        int sc = (c0 < c1) ? 1 : -1;
        int err = dr - dc;
        int r = r0;
        int c = c0;

        while (true) {
            if (slice_cell(grid, r, c) == 100) {
                return false;
            }
            if (r == r1 && c == c1) {
                break;
            }
            const int e2 = 2 * err;
            if (e2 > -dc) {
                err -= dc;
                r += sr;
            }
            if (e2 < dr) {
                err += dr;
                c += sc;
            }
        }
        return true;
    }

    size_t shortcut_cells_for_clearance(int row, int col,
                                        const std::vector<int> &obs_dist_map,
                                        int rows, int cols) const
    {
        double segment_m = std::max(0.20, path_shortcut_max_segment_m_);
        if (row >= 0 && row < rows && col >= 0 && col < cols &&
            static_cast<size_t>(row * cols + col) < obs_dist_map.size()) {
            const double clearance_m =
                static_cast<double>(obs_dist_map[static_cast<size_t>(row * cols + col)]) *
                voxel_size_;
            if (clearance_m <= path_shortcut_near_clearance_m_) {
                segment_m = std::min(segment_m, std::max(0.20, path_shortcut_near_segment_m_));
            } else if (clearance_m <= path_shortcut_mid_clearance_m_) {
                segment_m = std::min(segment_m, std::max(0.20, path_shortcut_mid_segment_m_));
            }
        }
        return std::max<size_t>(
            1, static_cast<size_t>(
                std::ceil(segment_m / std::max(0.05, voxel_size_))));
    }

    bool shortcut_cell_clear_enough(int row, int col,
                                    const std::vector<int> &obs_dist_map,
                                    int rows, int cols) const
    {
        return cell_clearance_at_least(
            row, col, obs_dist_map, rows, cols, path_shortcut_line_clearance_m_);
    }

    bool cell_clearance_at_least(int row, int col,
                                 const std::vector<int> &obs_dist_map,
                                 int rows, int cols,
                                 double clearance_m) const
    {
        if (clearance_m <= 0.0) {
            return true;
        }
        if (row < 0 || row >= rows || col < 0 || col >= cols) {
            return false;
        }
        const size_t idx = static_cast<size_t>(row * cols + col);
        if (idx >= obs_dist_map.size()) {
            return false;
        }
        const double cell_clearance_m = static_cast<double>(obs_dist_map[idx]) * voxel_size_;
        return cell_clearance_m + 1e-6 >= clearance_m;
    }

    void slice_cell_to_world(int row, int col, double &north, double &east) const
    {
        north = bounds_min_.x + (static_cast<double>(row) + 0.5) * voxel_size_;
        east  = bounds_min_.y + (static_cast<double>(col) + 0.5) * voxel_size_;
    }

    struct SmoothPathPoint {
        double north;
        double east;
        double z_ned;
    };

    static SmoothPathPoint lerp_path_point(const SmoothPathPoint &a,
                                           const SmoothPathPoint &b,
                                           double t)
    {
        return {
            a.north + (b.north - a.north) * t,
            a.east  + (b.east  - a.east)  * t,
            a.z_ned + (b.z_ned - a.z_ned) * t,
        };
    }

    bool smooth_path_point_safe(const SmoothPathPoint &p,
                                const SliceGrid &grid,
                                const std::vector<bool> &passable,
                                const std::vector<int> &obs_dist_map) const
    {
        int row = 0;
        int col = 0;
        if (!point_to_slice_cell(p.north, p.east, row, col)) {
            return false;
        }
        const size_t idx = static_cast<size_t>(row * grid.cols + col);
        if (idx >= passable.size() || !passable[idx]) {
            return false;
        }
        return cell_clearance_at_least(
            row, col, obs_dist_map, grid.rows, grid.cols, path_smoothing_clearance_m_);
    }

    bool smooth_path_segment_safe(const SmoothPathPoint &a,
                                  const SmoothPathPoint &b,
                                  const SliceGrid &grid,
                                  const std::vector<bool> &passable,
                                  const std::vector<int> &obs_dist_map) const
    {
        int r0 = 0, c0 = 0, r1 = 0, c1 = 0;
        if (!point_to_slice_cell(a.north, a.east, r0, c0) ||
            !point_to_slice_cell(b.north, b.east, r1, c1)) {
            return false;
        }

        int dr = std::abs(r1 - r0);
        int dc = std::abs(c1 - c0);
        int sr = r0 < r1 ? 1 : -1;
        int sc = c0 < c1 ? 1 : -1;
        int err = dr - dc;
        int r = r0;
        int c = c0;
        while (true) {
            if (r < 0 || r >= grid.rows || c < 0 || c >= grid.cols) {
                return false;
            }
            const size_t idx = static_cast<size_t>(r * grid.cols + c);
            if (idx >= passable.size() || !passable[idx]) {
                return false;
            }
            if (!cell_clearance_at_least(
                    r, c, obs_dist_map, grid.rows, grid.cols,
                    path_smoothing_clearance_m_)) {
                return false;
            }
            if (r == r1 && c == c1) {
                break;
            }
            const int e2 = 2 * err;
            if (e2 > -dc) {
                err -= dc;
                r += sr;
            }
            if (e2 < dr) {
                err += dr;
                c += sc;
            }
        }
        return true;
    }

    std::vector<SmoothPathPoint> pull_path_to_clearance_ridge(
        const std::vector<SmoothPathPoint> &points,
        const SliceGrid &grid,
        const std::vector<bool> &passable,
        const std::vector<int> &obs_dist_map) const
    {
        if (points.size() < 3) {
            return points;
        }

        std::vector<SmoothPathPoint> adjusted = points;
        constexpr int kIterations = 2;
        constexpr int kSearchRadiusCells = 2;
        for (int iter = 0; iter < kIterations; ++iter) {
            std::vector<SmoothPathPoint> next = adjusted;
            for (size_t i = 1; i + 1 < adjusted.size(); ++i) {
                int row = 0;
                int col = 0;
                if (!point_to_slice_cell(adjusted[i].north, adjusted[i].east, row, col)) {
                    continue;
                }
                const size_t idx = static_cast<size_t>(row * grid.cols + col);
                if (idx >= obs_dist_map.size()) {
                    continue;
                }

                SmoothPathPoint best = adjusted[i];
                int best_clearance = obs_dist_map[idx];
                double best_offset = 0.0;
                for (int dr = -kSearchRadiusCells; dr <= kSearchRadiusCells; ++dr) {
                    for (int dc = -kSearchRadiusCells; dc <= kSearchRadiusCells; ++dc) {
                        if (dr == 0 && dc == 0) {
                            continue;
                        }
                        if (dr * dr + dc * dc > kSearchRadiusCells * kSearchRadiusCells) {
                            continue;
                        }
                        const int nr = row + dr;
                        const int nc = col + dc;
                        if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) {
                            continue;
                        }
                        const size_t nidx = static_cast<size_t>(nr * grid.cols + nc);
                        if (nidx >= passable.size() || !passable[nidx] ||
                            nidx >= obs_dist_map.size()) {
                            continue;
                        }
                        const int clearance = obs_dist_map[nidx];
                        if (clearance < best_clearance) {
                            continue;
                        }

                        SmoothPathPoint candidate = adjusted[i];
                        slice_cell_to_world(nr, nc, candidate.north, candidate.east);
                        const double offset =
                            std::hypot(candidate.north - points[i].north,
                                       candidate.east  - points[i].east);
                        if (clearance == best_clearance && offset >= best_offset) {
                            continue;
                        }
                        if (!smooth_path_point_safe(candidate, grid, passable, obs_dist_map)) {
                            continue;
                        }
                        if (!smooth_path_segment_safe(
                                adjusted[i - 1], candidate, grid, passable, obs_dist_map) ||
                            !smooth_path_segment_safe(
                                candidate, adjusted[i + 1], grid, passable, obs_dist_map)) {
                            continue;
                        }

                        best = candidate;
                        best_clearance = clearance;
                        best_offset = offset;
                    }
                }
                next[i] = best;
            }
            adjusted = std::move(next);
        }
        return adjusted;
    }

    std::vector<SmoothPathPoint> resample_polyline(
        const std::vector<SmoothPathPoint> &points) const
    {
        if (points.size() < 2) {
            return points;
        }

        double total_len = 0.0;
        for (size_t i = 1; i < points.size(); ++i) {
            total_len += std::hypot(points[i].north - points[i - 1].north,
                                    points[i].east  - points[i - 1].east);
        }
        if (total_len < 1e-3) {
            return points;
        }

        const int max_points = std::max(2, path_smoothing_max_points_);
        const double spacing = std::max(
            std::max(0.10, path_smoothing_spacing_m_),
            total_len / static_cast<double>(max_points - 1));

        std::vector<SmoothPathPoint> out;
        out.reserve(static_cast<size_t>(std::min(
            max_points,
            std::max(2, static_cast<int>(std::ceil(total_len / spacing)) + 1))));
        out.push_back(points.front());

        double dist_since_sample = 0.0;
        for (size_t i = 1; i < points.size(); ++i) {
            SmoothPathPoint a = points[i - 1];
            const SmoothPathPoint b = points[i];
            double seg_len = std::hypot(b.north - a.north, b.east - a.east);
            while (seg_len + dist_since_sample >= spacing &&
                   static_cast<int>(out.size()) < max_points - 1) {
                const double remain = spacing - dist_since_sample;
                const double t = seg_len > 1e-6 ? remain / seg_len : 1.0;
                SmoothPathPoint sample = lerp_path_point(a, b, std::clamp(t, 0.0, 1.0));
                out.push_back(sample);
                a = sample;
                seg_len = std::hypot(b.north - a.north, b.east - a.east);
                dist_since_sample = 0.0;
            }
            dist_since_sample += seg_len;
        }

        if (std::hypot(out.back().north - points.back().north,
                       out.back().east  - points.back().east) > 0.05) {
            out.push_back(points.back());
        } else {
            out.back() = points.back();
        }
        return out;
    }

    std::vector<SmoothPathPoint> make_safe_bspline_path(
        const std::vector<SmoothPathPoint> &control,
        const SliceGrid &grid,
        const std::vector<bool> &passable,
        const std::vector<int> &obs_dist_map) const
    {
        if (!path_smoothing_enabled_ ||
            control.size() < 4 ||
            path_smoothing_iterations_ <= 0) {
            return control;
        }

        std::vector<SmoothPathPoint> curve =
            pull_path_to_clearance_ridge(control, grid, passable, obs_dist_map);
        const int iterations = std::clamp(path_smoothing_iterations_, 1, 3);
        for (int iter = 0; iter < iterations; ++iter) {
            std::vector<SmoothPathPoint> next;
            next.reserve(curve.size() * 2);
            next.push_back(curve.front());
            for (size_t i = 0; i + 1 < curve.size(); ++i) {
                next.push_back(lerp_path_point(curve[i], curve[i + 1], 0.25));
                next.push_back(lerp_path_point(curve[i], curve[i + 1], 0.75));
            }
            next.push_back(curve.back());
            curve = std::move(next);
        }

        curve = resample_polyline(curve);
        if (curve.size() < 2) {
            return control;
        }

        for (const auto &p : curve) {
            if (!smooth_path_point_safe(p, grid, passable, obs_dist_map)) {
                return control;
            }
        }
        for (size_t i = 1; i < curve.size(); ++i) {
            if (!smooth_path_segment_safe(
                    curve[i - 1], curve[i], grid, passable, obs_dist_map)) {
                return control;
            }
        }
        return curve;
    }

    void overlay_shared_voxels(SliceGrid &grid) const
    {
        if (!shared_map_enabled_) {
            return;
        }

        const double z_min = grid.z_center - slice_half_thickness_m_;
        const double z_max = grid.z_center + slice_half_thickness_m_;
        auto overlay_set = [&](const std::unordered_set<VoxelKey, VoxelKeyHash> &keys,
                               int8_t value) {
            for (const auto &key : keys) {
                const Vec3 c = key_to_center(key);
                if (c.z < z_min || c.z > z_max) {
                    continue;
                }

                int row = 0;
                int col = 0;
                if (!point_to_slice_cell(c.x, c.y, row, col)) {
                    continue;
                }

                const int idx = row * grid.cols + col;
                if (value == 100) {
                    grid.data[idx] = 100;
                } else if (grid.data[idx] != 100) {
                    grid.data[idx] = 0;
                }
            }
        };

        // Shared free cells make already-explored space visible to both drones.
        // Shared occupied cells are applied last so obstacles remain conservative.
        overlay_set(shared_free_voxels_, 0);
        overlay_set(shared_occupied_voxels_, 100);
    }

    void on_shared_cloud(const PointCloud2::SharedPtr &msg, bool occupied)
    {
        std::unordered_set<VoxelKey, VoxelKeyHash> next;
        const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
        if (n == 0 || msg->point_step < 12 || msg->data.empty()) {
            if (occupied) {
                shared_occupied_voxels_.clear();
            } else {
                shared_free_voxels_.clear();
            }
            return;
        }

        const size_t max_points = occupied ? 80000u : 120000u;
        const size_t step = n > max_points ? (n / max_points + 1u) : 1u;
        next.reserve(std::min(n, max_points));
        for (size_t i = 0; i < n; i += step) {
            const uint8_t *base = msg->data.data() + i * msg->point_step;
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::memcpy(&x, base + 0, sizeof(float));
            std::memcpy(&y, base + 4, sizeof(float));
            std::memcpy(&z, base + 8, sizeof(float));
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            const Vec3 p{static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)};
            if (!in_bounds(p) || !in_allowed_area(p.x, p.y)) {
                continue;
            }
            next.insert(world_to_key(p));
        }

        if (occupied) {
            shared_occupied_voxels_ = std::move(next);
        } else {
            shared_free_voxels_ = std::move(next);
        }
    }

    SliceGrid build_slice_grid() const
    {
        SliceGrid grid;
        grid.rows = std::max(1, static_cast<int>(
            std::ceil((bounds_max_.x - bounds_min_.x) / voxel_size_)));
        grid.cols = std::max(1, static_cast<int>(
            std::ceil((bounds_max_.y - bounds_min_.y) / voxel_size_)));
        grid.z_center = have_active_pose() ? active_position().z : 0.0;
        grid.data.assign(static_cast<size_t>(grid.rows * grid.cols), -1);

        const double z_min = grid.z_center - slice_half_thickness_m_;
        const double z_max = grid.z_center + slice_half_thickness_m_;
        for (const auto &[key, log_odds] : voxels_) {
            const Vec3 c = key_to_center(key);
            if (c.z < z_min || c.z > z_max) {
                continue;
            }

            int row = 0;
            int col = 0;
            if (!point_to_slice_cell(c.x, c.y, row, col)) {
                continue;
            }

            const int idx = row * grid.cols + col;
            const double p = sigmoid(log_odds);
            if (p > occupied_prob_) {
                grid.data[idx] = 100;
            } else if (std::abs(p - 0.5) > observed_delta_ && grid.data[idx] != 100) {
                grid.data[idx] = 0;
            }
        }

        overlay_shared_voxels(grid);

        return grid;
    }

    LayerCoverageStats compute_layer_coverage(const SliceGrid &grid)
    {
        LayerCoverageStats stats;
        // Label this coverage sample with the nearest discrete scan layer, not the
        // raw altitude. Both the global-coverage map below and map_merger key by
        // this value; using the raw altitude spawned a new low-coverage bin at
        // every transient height crossed while climbing, inflating the denominator.
        const double raw_alt = -grid.z_center;
        const double snap = std::max(0.1, coverage_layer_snap_m_);
        stats.altitude_m = std::round(raw_alt / snap) * snap;

        std::vector<bool> reachable;
        if (have_active_pose()) {
            const Vec3 pose = active_position();
            int drone_row = 0;
            int drone_col = 0;
            if (point_to_slice_cell(pose.x, pose.y, drone_row, drone_col)) {
                const auto passable = compute_passable_map(grid, drone_row, drone_col);
                reachable = compute_reachable_from(
                    passable, grid.rows, grid.cols, drone_row, drone_col);
            }
        }

        const bool use_reachable_denominator =
            coverage_use_reachable_component_ &&
            reachable.size() == static_cast<size_t>(grid.rows * grid.cols) &&
            std::any_of(reachable.begin(), reachable.end(), [](bool v) { return v; });

        // Coverage is evaluated over the physical layer mask, not just the
        // component currently reachable from the drone. A component-only
        // denominator made high layers in Room B look complete while Room A was
        // disconnected by the mezzanine/corridor geometry and still unscanned.
        // The old reachable-component behaviour remains available as a parameter
        // for debugging, but experiments should keep it disabled.
        for (int row = 0; row < grid.rows; ++row) {
            for (int col = 0; col < grid.cols; ++col) {
                const int idx = slice_index(grid, row, col);
                if (use_reachable_denominator &&
                    !reachable[static_cast<size_t>(idx)]) {
                    continue;
                }

                double north = 0.0;
                double east = 0.0;
                slice_cell_to_world(row, col, north, east);
                if (!in_allowed_area(north, east)) {
                    continue;
                }

                const int8_t cell = grid.data[static_cast<size_t>(idx)];
                if (cell == 100) {
                    continue;
                }
                ++stats.total_cells;
                if (cell == 0) {
                    ++stats.observed_cells;
                }
            }
        }

        if (stats.total_cells > 0) {
            stats.fraction = static_cast<double>(stats.observed_cells) /
                             static_cast<double>(stats.total_cells);
        }

        if (!coverage_reachability_guard_enabled_ || stats.total_cells == 0) {
            return stats;
        }

        const int layer_key = coverage_layer_key(stats.altitude_m);
        const auto prev_it = last_good_layer_coverage_.find(layer_key);
        if (prev_it != last_good_layer_coverage_.end()) {
            const auto &prev = prev_it->second;
            const bool previous_large_enough =
                prev.total_cells >= coverage_min_guard_total_cells_;
            const bool current_collapsed =
                stats.total_cells < static_cast<uint32_t>(
                    std::ceil(static_cast<double>(prev.total_cells) *
                              coverage_denominator_drop_ratio_));
            if (previous_large_enough && current_collapsed) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Layer coverage reachability guard: rejecting collapsed denominator "
                    "at %.1fm (%u/%u cells, %.1f%%) and keeping last good %u/%u cells "
                    "(%.1f%%). This prevents false 100%% coverage after local "
                    "reachability disconnect.",
                    stats.altitude_m, stats.observed_cells, stats.total_cells,
                    stats.fraction * 100.0,
                    prev.observed_cells, prev.total_cells, prev.fraction * 100.0);
                return prev;
            }
        }

        if (stats.total_cells >= coverage_min_guard_total_cells_) {
            auto &prev = last_good_layer_coverage_[layer_key];
            if (prev.total_cells == 0 || stats.total_cells >= prev.total_cells ||
                stats.total_cells >= static_cast<uint32_t>(
                    std::ceil(static_cast<double>(prev.total_cells) *
                              coverage_denominator_drop_ratio_))) {
                prev = stats;
            }
        }
        return stats;
    }

    int coverage_layer_key(double altitude_m) const
    {
        return static_cast<int>(std::llround(altitude_m / std::max(1e-3, voxel_size_)));
    }

    double update_dynamic_global_coverage(const LayerCoverageStats &layer_stats)
    {
        if (layer_stats.total_cells > 0) {
            const int key = coverage_layer_key(layer_stats.altitude_m);
            dynamic_layer_coverage_[key] = layer_stats;
        }

        uint64_t observed = 0;
        uint64_t total = 0;
        for (const auto &[_, stats] : dynamic_layer_coverage_) {
            observed += stats.observed_cells;
            total += stats.total_cells;
        }
        if (total == 0) {
            return 0.0;
        }
        return std::clamp(static_cast<double>(observed) /
                          static_cast<double>(total), 0.0, 1.0);
    }

    void publish_slice_map(const SliceGrid &grid)
    {
        OccupancyGrid msg{};
        msg.header.stamp = now();
        msg.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";
        msg.info.resolution = static_cast<float>(voxel_size_);
        msg.info.width = static_cast<uint32_t>(grid.cols);
        msg.info.height = static_cast<uint32_t>(grid.rows);
        msg.info.origin.position.x = bounds_min_.y;
        msg.info.origin.position.y = bounds_min_.x;
        msg.info.origin.position.z = grid.z_center - slice_half_thickness_m_;
        msg.info.origin.orientation.w = 1.0;
        msg.data = grid.data;
        slice_map_pub_->publish(msg);
    }

    double obstacle_band_half_for_z(double z_center) const
    {
        const double alt_m = -z_center;
        if (alt_m >= obstacle_band_high_alt_threshold_m_) {
            return std::min(obstacle_band_half_m_, obstacle_band_high_half_m_);
        }
        return obstacle_band_half_m_;
    }

    // Build an inflated 2D obstacle mask from the raw voxel map.
    // Unlike the SliceGrid (which only covers ±slice_half_thickness around the drone),
    // this checks ALL occupied voxels within ±obstacle_band_half_m_ of the drone's
    // altitude — so pillars that extend above/below the current slice are caught.
    //
    // Each raw occupied XY cell is inflated by obstacle_inflate_cells_ (= drone body +
    // safety margin) to prevent the planner from placing viewpoints adjacent to obstacles.
    //
    // Returns: blocked[r*cols+c] == true  →  cell is too close to a known obstacle.
    std::vector<bool> build_obstacle_mask(const SliceGrid &grid) const
    {
        const int N   = grid.rows * grid.cols;
        const int inf = obstacle_inflate_cells_;
        const double band = obstacle_band_half_for_z(grid.z_center);
        const double z_lo = grid.z_center - band;
        const double z_hi = grid.z_center + band;

        // Step 1: project occupied voxels onto the 2D grid (union across altitude band)
        std::vector<bool> raw(static_cast<size_t>(N), false);
        for (const auto &[key, log_odds] : voxels_) {
            if (sigmoid(log_odds) <= occupied_prob_) continue;
            const Vec3 c = key_to_center(key);
            if (c.z < z_lo || c.z > z_hi) continue;
            int row = 0, col = 0;
            if (!point_to_slice_cell(c.x, c.y, row, col)) continue;
            raw[static_cast<size_t>(row * grid.cols + col)] = true;
        }
        for (int r = 0; r < grid.rows; ++r) {
            for (int c = 0; c < grid.cols; ++c) {
                if (grid.data[static_cast<size_t>(r * grid.cols + c)] == 100) {
                    raw[static_cast<size_t>(r * grid.cols + c)] = true;
                }
            }
        }

        // Step 2: inflate raw occupied cells by obstacle_inflate_cells_ radius
        std::vector<bool> blocked(static_cast<size_t>(N), false);
        for (int r = 0; r < grid.rows; ++r) {
            for (int c = 0; c < grid.cols; ++c) {
                if (!raw[static_cast<size_t>(r * grid.cols + c)]) continue;
                for (int dr = -inf; dr <= inf; ++dr) {
                    for (int dc = -inf; dc <= inf; ++dc) {
                        if (dr * dr + dc * dc > inf * inf) continue;
                        const int nr = r + dr, nc = c + dc;
                        if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) continue;
                        blocked[static_cast<size_t>(nr * grid.cols + nc)] = true;
                    }
                }
            }
        }
        return blocked;
    }

    // Builds per-altitude-layer 2D passable maps for 3D A*.
    // Each layer zi is centred at z_ned_layers[zi] (NED, negative = above ground).
    // Obstacle detection uses obstacle_band_half_m_ (same as build_obstacle_mask).
    // Returns passable_3d[zi][r*cols+c] — true = obstacle-free at that layer.
    std::vector<std::vector<bool>> build_passable_volume_3d(
        const SliceGrid &grid,
        const std::vector<double> &z_ned_layers) const
    {
        const int n_layers = static_cast<int>(z_ned_layers.size());
        const int N = grid.rows * grid.cols;
        const int inf = obstacle_inflate_cells_;

        // Step 1: project each occupied voxel onto the Z layers it occupies
        std::vector<std::vector<bool>> raw(
            static_cast<size_t>(n_layers),
            std::vector<bool>(static_cast<size_t>(N), false));

        for (const auto &[key, log_odds] : voxels_) {
            if (sigmoid(log_odds) <= occupied_prob_) continue;
            const Vec3 vc = key_to_center(key);
            int row = 0, col = 0;
            if (!point_to_slice_cell(vc.x, vc.y, row, col)) continue;
            for (int zi = 0; zi < n_layers; ++zi) {
                const double z_center = z_ned_layers[static_cast<size_t>(zi)];
                const double band = obstacle_band_half_for_z(z_center);
                const double z_lo = z_center - band;
                const double z_hi = z_center + band;
                if (vc.z >= z_lo && vc.z <= z_hi) {
                    raw[static_cast<size_t>(zi)][static_cast<size_t>(row * grid.cols + col)] = true;
                }
            }
        }

        // Step 2: inflate each layer and apply world area mask
        std::vector<std::vector<bool>> passable_3d(
            static_cast<size_t>(n_layers),
            std::vector<bool>(static_cast<size_t>(N), true));

        for (int zi = 0; zi < n_layers; ++zi) {
            const auto &raw_zi  = raw[static_cast<size_t>(zi)];
            auto       &pass_zi = passable_3d[static_cast<size_t>(zi)];

            for (int r = 0; r < grid.rows; ++r) {
                for (int c = 0; c < grid.cols; ++c) {
                    if (!raw_zi[static_cast<size_t>(r * grid.cols + c)]) continue;
                    for (int dr = -inf; dr <= inf; ++dr) {
                        for (int dc = -inf; dc <= inf; ++dc) {
                            if (dr * dr + dc * dc > inf * inf) continue;
                            const int nr = r + dr, nc = c + dc;
                            if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) continue;
                            pass_zi[static_cast<size_t>(nr * grid.cols + nc)] = false;
                        }
                    }
                }
            }

            if (world_area_mask_enabled_) {
                for (int r = 0; r < grid.rows; ++r) {
                    for (int c = 0; c < grid.cols; ++c) {
                        double north, east;
                        slice_cell_to_world(r, c, north, east);
                        if (!in_allowed_area(north, east))
                            pass_zi[static_cast<size_t>(r * grid.cols + c)] = false;
                    }
                }
            }
            force_high_layer_room_connector(
                pass_zi, grid, z_ned_layers[static_cast<size_t>(zi)]);
        }
        return passable_3d;
    }

    void publish_obstacle_mask(const SliceGrid &grid, const std::vector<bool> &blocked)
    {
        if (obstacle_mask_pub_->get_subscription_count() == 0) return;
        OccupancyGrid msg{};
        msg.header.stamp = now();
        msg.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";
        msg.info.resolution = static_cast<float>(voxel_size_);
        msg.info.width  = static_cast<uint32_t>(grid.cols);
        msg.info.height = static_cast<uint32_t>(grid.rows);
        msg.info.origin.position.x = bounds_min_.y;
        msg.info.origin.position.y = bounds_min_.x;
        msg.info.origin.position.z = static_cast<float>(grid.z_center);
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(static_cast<size_t>(grid.rows * grid.cols));
        for (int i = 0; i < grid.rows * grid.cols; ++i) {
            msg.data[static_cast<size_t>(i)] = blocked[static_cast<size_t>(i)] ? 100 : 0;
        }
        obstacle_mask_pub_->publish(msg);
    }

    void publish_empty_frontier_path()
    {
        Path path_msg;
        path_msg.header.stamp = now();
        path_msg.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";
        path_msg.poses.clear();
        frontier_path_pub_->publish(path_msg);
    }

    void publish_frontier_goal(const SliceGrid &grid,
                               const std::vector<bool> &obstacle_mask_prebuilt)
    {
        frontier_clusters_count_ = 0;
        reachable_frontier_clusters_count_ = 0;
        reachable_frontier_cells_count_ = 0;
        frontier_route_available_ = false;

        if (!have_active_pose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Frontier: no active pose — skipping");
            publish_empty_frontier_path();
            return;
        }
        const Vec3 pose = active_position();

        int drone_row = 0;
        int drone_col = 0;
        if (!point_to_slice_cell(pose.x, pose.y, drone_row, drone_col)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Frontier: drone pos (%.2f,%.2f) out of grid bounds — skipping",
                pose.x, pose.y);
            publish_empty_frontier_path();
            return;
        }

        struct FrontierCluster {
            std::vector<std::pair<int, int>> cells;
            double centroid_row = 0.0;
            double centroid_col = 0.0;
            double target_north = 0.0;
            double target_east = 0.0;
            double coarse_score = -std::numeric_limits<double>::infinity();
            int rep_row = -1;
            int rep_col = -1;
            int reachable_cells = 0;
        };

        const double cluster_radius_cells =
            std::max(1.0, frontier_cluster_radius_m_ / voxel_size_);
        const double cluster_radius_sq = cluster_radius_cells * cluster_radius_cells;
        std::vector<uint8_t> visited(static_cast<size_t>(grid.rows * grid.cols), 0);
        std::vector<FrontierCluster> clusters;

        constexpr std::array<std::pair<int, int>, 8> kNeighbours{{
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},           {0, 1},
            {1, -1},  {1, 0},  {1, 1},
        }};

        for (int row = 0; row < grid.rows; ++row) {
            for (int col = 0; col < grid.cols; ++col) {
                const int idx = slice_index(grid, row, col);
                if (visited[idx] || !slice_is_frontier(grid, row, col)) {
                    continue;
                }

                std::queue<std::pair<int, int>> q;
                std::vector<std::pair<int, int>> cluster;
                visited[idx] = 1;
                q.push({row, col});

                while (!q.empty()) {
                    const auto [cr, cc] = q.front();
                    q.pop();
                    cluster.push_back({cr, cc});

                    for (const auto &[dr, dc] : kNeighbours) {
                        const int nr = cr + dr;
                        const int nc = cc + dc;
                        if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) {
                            continue;
                        }
                        const int nidx = slice_index(grid, nr, nc);
                        if (visited[nidx] || !slice_is_frontier(grid, nr, nc)) {
                            continue;
                        }
                        const double drow = static_cast<double>(nr - row);
                        const double dcol = static_cast<double>(nc - col);
                        if (drow * drow + dcol * dcol > cluster_radius_sq) {
                            continue;
                        }
                        visited[nidx] = 1;
                        q.push({nr, nc});
                    }
                }

                if (cluster.empty()) {
                    continue;
                }

                double centroid_row = 0.0;
                double centroid_col = 0.0;
                for (const auto &[cr, cc] : cluster) {
                    centroid_row += cr;
                    centroid_col += cc;
                }
                centroid_row /= static_cast<double>(cluster.size());
                centroid_col /= static_cast<double>(cluster.size());

                double best_rep_dist = std::numeric_limits<double>::infinity();
                int rep_row = -1;
                int rep_col = -1;
                for (const auto &[cr, cc] : cluster) {
                    const double dr = static_cast<double>(cr) - centroid_row;
                    const double dc = static_cast<double>(cc) - centroid_col;
                    const double d2 = dr * dr + dc * dc;
                    if (d2 < best_rep_dist) {
                        best_rep_dist = d2;
                        rep_row = cr;
                        rep_col = cc;
                    }
                }

                if (rep_row < 0 || rep_col < 0) {
                    continue;
                }

                double target_north = 0.0;
                double target_east = 0.0;
                slice_cell_to_world(rep_row, rep_col, target_north, target_east);

                const double north_err = target_north - pose.x;
                const double east_err  = target_east - pose.y;
                const double goal_dist = std::hypot(north_err, east_err);
                if (goal_dist < frontier_min_goal_distance_m_) {
                    continue;
                }
                // Reject clusters outside the known physical world (ghost free voxels
                // behind walls can produce frontier cells outside the room envelope).
                if (!in_allowed_area(target_north, target_east)) {
                    continue;
                }
                FrontierCluster summary;
                summary.cells = std::move(cluster);
                summary.centroid_row = centroid_row;
                summary.centroid_col = centroid_col;
                summary.target_north = target_north;
                summary.target_east = target_east;
                summary.rep_row = rep_row;
                summary.rep_col = rep_col;
                const double sort_dist =
                    std::max(goal_dist, frontier_min_goal_distance_m_);
                summary.coarse_score = static_cast<double>(summary.cells.size()) /
                                       std::sqrt(sort_dist);

                // Failure-penalty: if the drone hasn't moved >1 m since the last
                // published goal AND this cluster is within 2.5 m of that goal,
                // strongly deprioritise it so the planner explores elsewhere.
                if (std::isfinite(frontier_last_goal_n_)) {
                    const double drone_moved =
                        std::hypot(pose.x - frontier_last_drone_n_,
                                   pose.y - frontier_last_drone_e_);
                    const double d_last =
                        std::hypot(target_north - frontier_last_goal_n_,
                                   target_east  - frontier_last_goal_e_);
                    if (drone_moved < 1.0 && d_last < 2.5) {
                        summary.coarse_score -= 500.0;
                    }
                }

                clusters.push_back(std::move(summary));
            }
        }

        if (clusters.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Frontier: no frontier clusters found in slice at z=%.2f "
                "(drone row=%d col=%d) — map may be too sparse",
                grid.z_center, drone_row, drone_col);
            publish_empty_frontier_path();
            return;
        }
        frontier_clusters_count_ = static_cast<uint32_t>(clusters.size());

        // Precompute obstacle-safe reachability from the drone's cell.
        // Done once per planning tick; each viewpoint candidate checks the bitmap
        // in O(1) instead of running a full BFS per candidate.
        const auto passable    = compute_passable_map(grid, drone_row, drone_col);
        const auto reachable   = compute_reachable_from(
            passable, grid.rows, grid.cols, drone_row, drone_col);

        int reachable_clusters = 0;
        int reachable_frontier_cells = 0;
        for (auto &cluster : clusters) {
            int reachable_cells = 0;
            for (const auto &[cr, cc] : cluster.cells) {
                if (reachable[static_cast<size_t>(cr * grid.cols + cc)]) {
                    ++reachable_cells;
                }
            }
            cluster.reachable_cells = reachable_cells;
            reachable_frontier_cells += reachable_cells;
            if (reachable_cells > 0) {
                ++reachable_clusters;
            }
        }
        reachable_frontier_clusters_count_ = static_cast<uint32_t>(reachable_clusters);
        reachable_frontier_cells_count_ =
            static_cast<uint32_t>(std::max(0, reachable_frontier_cells));

        // DEBUG: print cluster positions and BFS path to first unreachable cluster.
        if (reachable_clusters == 0 && !clusters.empty()) {
            double drone_n = 0.0, drone_e = 0.0;
            slice_cell_to_world(drone_row, drone_col, drone_n, drone_e);
            // Find closest unreachable cluster by Euclidean distance.
            const FrontierCluster *nearest = nullptr;
            double nearest_dist_sq = std::numeric_limits<double>::infinity();
            for (const auto &cl : clusters) {
                const double dn = cl.target_north - drone_n;
                const double de = cl.target_east  - drone_e;
                const double d2 = dn * dn + de * de;
                if (d2 < nearest_dist_sq) { nearest_dist_sq = d2; nearest = &cl; }
            }
            if (nearest) {
                const int tr = nearest->rep_row, tc = nearest->rep_col;
                const bool cl_pass = tr >= 0 && tr < grid.rows && tc >= 0 && tc < grid.cols
                    ? passable[static_cast<size_t>(tr * grid.cols + tc)] : false;
                const bool cl_reach = tr >= 0 && tr < grid.rows && tc >= 0 && tc < grid.cols
                    ? reachable[static_cast<size_t>(tr * grid.cols + tc)] : false;
                const int8_t cl_slice = slice_cell(grid, tr, tc);
                double cl_n = 0.0, cl_e = 0.0;
                slice_cell_to_world(tr, tc, cl_n, cl_e);
                // Scan cells along straight N line from drone to cluster rep.
                std::string north_scan;
                const int step_sign = (tr >= drone_row) ? 1 : -1;
                for (int r = drone_row; r != tr + step_sign; r += step_sign) {
                    bool p = passable[static_cast<size_t>(r * grid.cols + drone_col)];
                    bool rv = reachable[static_cast<size_t>(r * grid.cols + drone_col)];
                    north_scan += (p ? (rv ? "R" : "p") : "X");
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "DBG clusters=0: drone(r=%d,c=%d n=%.1f e=%.1f) nearest_cl(r=%d,c=%d n=%.1f e=%.1f "
                    "slc=%d pass=%d reach=%d) N-scan[%s]",
                    drone_row, drone_col, drone_n, drone_e,
                    tr, tc, cl_n, cl_e,
                    static_cast<int>(cl_slice), cl_pass, cl_reach,
                    north_scan.c_str());
                // Also scan E direction from drone toward cluster east.
                std::string east_scan;
                const int estep = (tc >= drone_col) ? 1 : -1;
                for (int c = drone_col; c != tc + estep; c += estep) {
                    bool p = passable[static_cast<size_t>(drone_row * grid.cols + c)];
                    bool rv = reachable[static_cast<size_t>(drone_row * grid.cols + c)];
                    east_scan += (p ? (rv ? "R" : "p") : "X");
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5001,
                    "DBG E-scan[%s]", east_scan.c_str());
            }
        }

        std::sort(clusters.begin(), clusters.end(),
            [](const FrontierCluster &a, const FrontierCluster &b) {
                const bool a_reachable = a.reachable_cells > 0;
                const bool b_reachable = b.reachable_cells > 0;
                if (a_reachable != b_reachable) {
                    return a_reachable;
                }
                if (a_reachable && a.reachable_cells != b.reachable_cells) {
                    return a.reachable_cells > b.reachable_cells;
                }
                return a.coarse_score > b.coarse_score;
            });
        if (static_cast<int>(clusters.size()) > frontier_max_clusters_scored_) {
            clusters.resize(static_cast<size_t>(frontier_max_clusters_scored_));
        }
        {
            const int n_reachable = static_cast<int>(
                std::count(reachable.begin(), reachable.end(), true));
            int n_reachable_free = 0;
            int reach_path_room_a = 0;
            int reach_path_corridor = 0;
            int reach_path_room_b = 0;
            int reach_room_a = 0;
            int reach_corridor = 0;
            int reach_room_b = 0;
            for (int i = 0; i < grid.rows * grid.cols; ++i) {
                if (!reachable[static_cast<size_t>(i)]) {
                    continue;
                }
                const int r = i / grid.cols;
                const int c = i % grid.cols;
                double north = 0.0;
                double east = 0.0;
                slice_cell_to_world(r, c, north, east);
                if (north >= -8.5 && north <= 8.5 && east >= -8.5 && east <= 8.5) {
                    ++reach_path_room_a;
                } else if (north >= -2.5 && north <= 2.5 &&
                           east >= 7.5 && east <= 14.5) {
                    ++reach_path_corridor;
                } else if (north >= -8.5 && north <= 8.5 &&
                           east >= 13.5 && east <= 30.5) {
                    ++reach_path_room_b;
                }
                if (slice_cell(grid, r, c) == 0) {
                    ++n_reachable_free;
                    if (north >= -8.5 && north <= 8.5 && east >= -8.5 && east <= 8.5) {
                        ++reach_room_a;
                    } else if (north >= -2.5 && north <= 2.5 &&
                               east >= 7.5 && east <= 14.5) {
                        ++reach_corridor;
                    } else if (north >= -8.5 && north <= 8.5 &&
                               east >= 13.5 && east <= 30.5) {
                        ++reach_room_b;
                    }
                }
            }
            const double obstacle_band = obstacle_band_half_for_z(grid.z_center);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Frontier: clusters=%d reachable_clusters=%d reachable_path=%d reachable_free=%d z=%.2f band=%.2f "
                "reach_free A=%d C=%d B=%d reach_path A=%d C=%d B=%d",
                static_cast<int>(clusters.size()), reachable_clusters,
                n_reachable, n_reachable_free, grid.z_center, obstacle_band,
                reach_room_a, reach_corridor, reach_room_b,
                reach_path_room_a, reach_path_corridor, reach_path_room_b);
            const double alt_m = -grid.z_center;
            if (alt_m >= obstacle_band_high_alt_threshold_m_ &&
                ((reach_path_room_a > 0 && reach_path_room_b == 0) ||
                 (reach_path_room_b > 0 && reach_path_room_a == 0) ||
                 reach_path_corridor == 0)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Frontier: high-layer planner slice appears disconnected at alt=%.1fm "
                    "(A=%d corridor=%d B=%d). Check obstacle band/inflation around mezzanine.",
                    alt_m, reach_path_room_a, reach_path_corridor, reach_path_room_b);
            }
        }
        // Path-length distance map: BFS from drone through obstacle-safe cells.
        // Scores candidates by actual navigable distance, not Euclidean distance.
        // A goal behind a wall scores as far even if spatially close.
        const auto path_dist_map = compute_path_dist_map(
            passable, grid.rows, grid.cols, drone_row, drone_col);
        // Voronoi-inspired clearance map: distance (cells) from each free cell
        // to the nearest inflated planning obstacle / world boundary.  Used to
        // bias goal selection toward corridor centres, away from pillars/walls.
        const auto &obstacle_mask = obstacle_mask_prebuilt;
        const auto obs_dist_map = compute_obstacle_dist_map(grid, &obstacle_mask);
        const int min_obstacle_clearance_cells = std::max(
            2,
            static_cast<int>(std::round(frontier_min_obstacle_clearance_m_ / voxel_size_)));
        int cand_bfs_rejected = 0;
        // Obstacle mask passed in from publish_summary_and_cloud() — already built
        // once for this tick; no second full-voxel-map scan needed here.
        int cand_obstacle_rejected = 0;
        int cand_slice_rejected    = 0;  // slice_cell != 0 (occupied or unknown center)
        int cand_vp_rejected       = 0;  // viewpoint clearance (occupied neighbour)
        int cand_vol_rejected      = 0;  // volume_clear failed
        int cand_near_rejected     = 0;  // selected viewpoint is already reached
        int cand_weak_rejected     = 0;  // tiny frontier with no information gain

        const double yaw_now = current_yaw();
        double best_score = -std::numeric_limits<double>::infinity();
        double best_goal_north = 0.0;
        double best_goal_east = 0.0;
        double best_goal_yaw = 0.0;
        double best_info_gain = 0.0;
        int best_cluster_size = 0;
        int best_cand_row = drone_row;
        int best_cand_col = drone_col;

        auto zone_advance_bonus_for = [&](double frontier_east) -> double {
            if (!world_area_mask_enabled_) return 0.0;
            // The eastward bias is only a bootstrap to make early layers leave
            // Room A and enter Room B. On upper layers it hides remaining work in
            // Room A by continuing to prefer warehouse frontiers.
            const double altitude_m = -grid.z_center;
            if (altitude_m >= 5.0) return 0.0;
            if (frontier_east > 13.5) return 2.5;  // Room B
            if (frontier_east > 7.5)  return 1.2;  // Corridor
            return 0.0;
        };

        auto persistence_bonus_for = [&](double target_north, double target_east) -> double {
            if (!std::isfinite(frontier_last_goal_n_)) return 0.0;
            const double d_to_last =
                std::hypot(target_north - frontier_last_goal_n_,
                           target_east  - frontier_last_goal_e_);
            return d_to_last < frontier_cluster_radius_m_ * 1.5 ? 1.0 : 0.0;
        };

        auto viewpoint_utility = [&](double info_gain,
                                     int cluster_size,
                                     double clearance_m,
                                     double awareness,
                                     double path_m,
                                     double dist_z,
                                     double yaw_delta,
                                     double frontier_east,
                                     double cluster_target_n,
                                     double cluster_target_e,
                                     double failure_penalty) -> double {
            const double info_norm =
                std::clamp(std::log1p(info_gain) / std::log1p(250.0), 0.0, 1.0);
            const double cluster_norm =
                std::clamp(std::sqrt(static_cast<double>(cluster_size)) / 12.0, 0.0, 1.0);
            const double clearance_norm =
                std::clamp(clearance_m / 2.0, 0.0, 1.0);
            const double awareness_norm = std::clamp(awareness, 0.0, 1.0);
            const double travel_norm =
                std::clamp((path_m + 5.0 * dist_z) / 30.0, 0.0, 1.0);
            const double yaw_norm =
                std::clamp(yaw_delta / M_PI, 0.0, 1.0);
            const double progress_norm =
                std::clamp(path_m / 6.0, 0.0, 1.0);

            double peer_penalty = 0.0;
            if (have_peer_frontier_) {
                const double dx = cluster_target_n - peer_frontier_goal_.point.x;
                const double dy = cluster_target_e - peer_frontier_goal_.point.y;
                const double d  = std::hypot(dx, dy);
                if (d < static_cast<double>(peer_exclusion_radius_m_)) {
                    peer_penalty = static_cast<double>(peer_exclusion_penalty_) *
                                   (1.0 - d / static_cast<double>(peer_exclusion_radius_m_));
                }
            }

            return
                frontier_info_gain_weight_ * info_norm +
                frontier_cluster_weight_ * cluster_norm +
                frontier_clearance_weight_ * clearance_norm +
                frontier_awareness_weight_ * awareness_norm +
                frontier_progress_weight_ * progress_norm +
                zone_advance_bonus_for(frontier_east) +
                persistence_bonus_for(cluster_target_n, cluster_target_e) -
                4.0 * frontier_distance_weight_ * travel_norm -
                frontier_yaw_weight_ * yaw_norm -
                failure_penalty -
                peer_penalty;
        };

        for (const auto &cluster : clusters) {
            // === Voronoi-optimal viewpoint (Lecture 4 retraction principle) ===
            // Gradient ascent on obs_dist_map from the frontier representative cell
            // climbs to the nearest local maximum of obstacle distance — the Voronoi
            // ridge.  In a corridor this is the centreline; in a room it is the
            // equidistant skeleton.  This gives the safest navigable viewpoint rather
            // than an arbitrary ring position that may be close to a wall.
            {
                int vor_r = cluster.rep_row;
                int vor_c = cluster.rep_col;
                bool moved = true;
                for (int step = 0; step < 400 && moved; ++step) {
                    moved = false;
                    int best_d = obs_dist_map[static_cast<size_t>(vor_r * grid.cols + vor_c)];
                    int next_r = vor_r, next_c = vor_c;
                    for (int dr = -1; dr <= 1; ++dr) {
                        for (int dc = -1; dc <= 1; ++dc) {
                            if (!dr && !dc) continue;
                            const int nr = vor_r + dr, nc = vor_c + dc;
                            if (nr < 0 || nr >= grid.rows || nc < 0 || nc >= grid.cols) continue;
                            if (!reachable[static_cast<size_t>(nr * grid.cols + nc)]) continue;
                            if (slice_cell(grid, nr, nc) != 0) continue;
                            const int nd = obs_dist_map[static_cast<size_t>(nr * grid.cols + nc)];
                            if (nd > best_d) {
                                best_d = nd; next_r = nr; next_c = nc; moved = true;
                            }
                        }
                    }
                    vor_r = next_r; vor_c = next_c;
                }

                double vor_north = 0.0, vor_east = 0.0;
                slice_cell_to_world(vor_r, vor_c, vor_north, vor_east);
                const Vec3 vor_pos{vor_north, vor_east, grid.z_center};
                const bool vor_boundary_clear =
                    in_allowed_area_with_clearance(
                        vor_north, vor_east, frontier_world_boundary_clearance_m_);

                // Minimum clearance guard: Voronoi ascent may converge on a local
                // ridge still adjacent to a wall. Require a configurable clearance
                // to keep the selected viewpoint near passage centrelines.
                const int vor_obs_d =
                    obs_dist_map[static_cast<size_t>(vor_r * grid.cols + vor_c)];
                if (!obstacle_mask[static_cast<size_t>(vor_r * grid.cols + vor_c)] &&
                    vor_obs_d >= min_obstacle_clearance_cells &&
                    reachable[static_cast<size_t>(vor_r * grid.cols + vor_c)] &&
                    slice_cell(grid, vor_r, vor_c) == 0 &&
                    vor_boundary_clear &&
                    slice_viewpoint_clearance_ok(grid, vor_r, vor_c) &&
                    volume_clear(vor_pos, frontier_candidate_clearance_m_,
                                 frontier_candidate_vertical_half_m_))
                {
                    const double vor_yaw = std::atan2(
                        cluster.target_east  - vor_east,
                        cluster.target_north - vor_north);
                    const double vor_dist  = std::hypot(vor_north - pose.x,
                                                         vor_east  - pose.y);
                    if (vor_dist < frontier_min_goal_distance_m_) {
                        ++cand_near_rejected;
                    } else {
                        const double vor_info  = estimate_information_gain(vor_pos, vor_yaw);
                        // Count weak candidates for telemetry, but still score them —
                        // if no strong candidate exists the best weak one keeps the drone moving.
                        if (vor_info < 1.0 && cluster.cells.size() < 8) {
                            ++cand_weak_rejected;
                        }
                        const double vor_dist_z = std::abs(grid.z_center - pose.z);
                        const double vor_yaw_delta = std::abs(wrap_pi(vor_yaw - yaw_now));
                        const int    vor_clear_cells =
                            obs_dist_map[static_cast<size_t>(vor_r * grid.cols + vor_c)];
                        const int vor_path_cells =
                            path_dist_map[static_cast<size_t>(vor_r * grid.cols + vor_c)];
                        const double vor_path_m = (vor_path_cells < 9999)
                            ? vor_path_cells * voxel_size_
                            : vor_dist * 3.0;
                        // Fine failure penalty: same cluster-near-last-goal logic as
                        // coarse stage, applied in the detailed scoring pass.
                        double vor_failure_penalty = 0.0;
                        if (std::isfinite(frontier_last_goal_n_)) {
                            const double drone_moved =
                                std::hypot(pose.x - frontier_last_drone_n_,
                                           pose.y - frontier_last_drone_e_);
                            const double d_last =
                                std::hypot(cluster.target_north - frontier_last_goal_n_,
                                           cluster.target_east  - frontier_last_goal_e_);
                            if (drone_moved < 1.0 && d_last < 2.5) {
                                vor_failure_penalty = 50.0;
                            }
                        }
                        const double vor_clearance_m =
                            std::min(vor_clear_cells, 10) * voxel_size_;
                        const double vor_score = viewpoint_utility(
                            vor_info,
                            static_cast<int>(cluster.cells.size()),
                            vor_clearance_m,
                            local_awareness(grid, vor_r, vor_c),
                            vor_path_m,
                            vor_dist_z,
                            vor_yaw_delta,
                            cluster.target_east,
                            cluster.target_north,
                            cluster.target_east,
                            vor_failure_penalty);

                        if (vor_score > best_score) {
                            best_score        = vor_score;
                            best_goal_north   = vor_north;
                            best_goal_east    = vor_east;
                            best_goal_yaw     = vor_yaw;
                            best_info_gain    = vor_info;
                            best_cluster_size = static_cast<int>(cluster.cells.size());
                            best_cand_row     = vor_r;  // enable A* path to Voronoi viewpoint
                            best_cand_col     = vor_c;
                        }
                    }
                }
            }

            const int standoff_samples = std::max(1, frontier_view_standoff_samples_);
            const int angle_samples = std::max(4, frontier_view_angle_samples_);

            for (int si = 0; si < standoff_samples; ++si) {
                const double t = (standoff_samples <= 1)
                    ? 0.5
                    : static_cast<double>(si) / static_cast<double>(standoff_samples - 1);
                const double standoff =
                    frontier_view_standoff_min_m_ +
                    (frontier_view_standoff_max_m_ - frontier_view_standoff_min_m_) * t;

                for (int ai = 0; ai < angle_samples; ++ai) {
                    const double angle = 2.0 * M_PI * static_cast<double>(ai) /
                                         static_cast<double>(angle_samples);
                    const double cand_north =
                        cluster.target_north - standoff * std::cos(angle);
                    const double cand_east =
                        cluster.target_east - standoff * std::sin(angle);

                    int cand_row = 0;
                    int cand_col = 0;
                    if (!point_to_slice_cell(cand_north, cand_east, cand_row, cand_col)) {
                        continue;
                    }
                    if (!in_allowed_area(cand_north, cand_east)) continue;
                    if (!in_allowed_area_with_clearance(
                            cand_north, cand_east,
                            frontier_world_boundary_clearance_m_)) {
                        continue;
                    }
                    const double cand_direct_dist = std::hypot(cand_north - pose.x,
                                                               cand_east  - pose.y);
                    if (cand_direct_dist < frontier_min_goal_distance_m_) {
                        ++cand_near_rejected;
                        continue;
                    }
                    if (slice_cell(grid, cand_row, cand_col) != 0) {
                        ++cand_slice_rejected; continue;
                    }
                    if (!slice_viewpoint_clearance_ok(grid, cand_row, cand_col)) {
                        ++cand_vp_rejected; continue;
                    }
                    // Route reachability is a HARD veto again: the soft-signal variant
                    // let the planner select a Room A goal that is disconnected from the
                    // drone's current component at mezzanine heights, then A* failed to
                    // build a route → no waypoints → the drone stalled and runs never
                    // completed. Only score candidates the same-layer BFS can reach.
                    if (!reachable[static_cast<size_t>(cand_row * grid.cols + cand_col)]) {
                        ++cand_bfs_rejected;
                        continue;
                    }
                    // Obstacle veto: reject viewpoints inside the inflated obstacle mask.
                    // This catches pillars/walls even when the SliceGrid cell reads as
                    // free due to sparse ray coverage or voxel erosion.
                    if (obstacle_mask[static_cast<size_t>(cand_row * grid.cols + cand_col)]) {
                        ++cand_obstacle_rejected;
                        continue;
                    }
                    // Minimum clearance guard: even if obstacle_mask passes, require
                    // the candidate viewpoint to be far enough from confirmed
                    // obstacles. This is the practical grid version of the Lecture 4
                    // Voronoi/retraction idea: prefer passage centres over wall edges.
                    if (obs_dist_map[static_cast<size_t>(cand_row * grid.cols + cand_col)] <
                        min_obstacle_clearance_cells) {
                        ++cand_obstacle_rejected;
                        continue;
                    }
                    // LOS removed: safety is provided by obstacle_mask, volume_clear, and VFH.
                    // Checking LOS drone→cand and cand→cluster was rejecting ~50 candidates
                    // per tick in rooms with pillars, leaving zero scorable viewpoints.

                    const Vec3 candidate_pos{cand_north, cand_east, grid.z_center};
                    if (!volume_clear(candidate_pos,
                                      frontier_candidate_clearance_m_,
                                      frontier_candidate_vertical_half_m_)) {
                        ++cand_vol_rejected;
                        continue;
                    }
                    const double candidate_yaw = std::atan2(
                        cluster.target_east - cand_east,
                        cluster.target_north - cand_north);
                    const double info_gain =
                        estimate_information_gain(candidate_pos, candidate_yaw);
                    // Count weak candidates for telemetry, but still score them —
                    // if no strong candidate exists the best weak one keeps the drone moving.
                    if (info_gain < 1.0 && cluster.cells.size() < 8) {
                        ++cand_weak_rejected;
                    }
                    // Use navigable path length instead of Euclidean distance.
                    // Goals behind walls score as far even when spatially close.
                    // Candidate is guaranteed reachable here (hard veto above).
                    const int path_cells =
                        path_dist_map[static_cast<size_t>(cand_row * grid.cols + cand_col)];
                    const double dist_path = (path_cells < 9999)
                        ? path_cells * voxel_size_
                        : std::hypot(cand_north - pose.x, cand_east - pose.y) * 3.0;
                    const double dist_z = std::abs(grid.z_center - pose.z);
                    const double yaw_delta =
                        std::abs(wrap_pi(candidate_yaw - yaw_now));
                    // Voronoi clearance bonus: reward positions equidistant from
                    // obstacles (corridor centres), penalise positions near walls.
                    // Capped at 10 cells (2.0 m) to avoid over-rewarding open rooms.
                    const int raw_clearance_cells =
                        obs_dist_map[static_cast<size_t>(cand_row * grid.cols + cand_col)];
                    const double clearance_m =
                        std::min(raw_clearance_cells, 10) * voxel_size_;
                    // Local awareness: fraction of cells within 3-cell radius that
                    // are confirmed (not unknown).  Rewards viewpoints in well-mapped
                    // areas; penalises goals at the edge of the known map surrounded
                    // by unknown cells — safer and more predictable to navigate to.
                    const double awareness =
                        local_awareness(grid, cand_row, cand_col);
                    // Fine failure penalty on the standoff scoring pass.
                    double failure_penalty = 0.0;
                    if (std::isfinite(frontier_last_goal_n_)) {
                        const double drone_moved =
                            std::hypot(pose.x - frontier_last_drone_n_,
                                       pose.y - frontier_last_drone_e_);
                        const double d_last =
                            std::hypot(cluster.target_north - frontier_last_goal_n_,
                                       cluster.target_east  - frontier_last_goal_e_);
                        if (drone_moved < 1.0 && d_last < 2.5) {
                            failure_penalty = 50.0;
                        }
                    }
                    const double score = viewpoint_utility(
                        info_gain,
                        static_cast<int>(cluster.cells.size()),
                        clearance_m,
                        awareness,
                        dist_path,
                        dist_z,
                        yaw_delta,
                        cluster.target_east,
                        cluster.target_north,
                        cluster.target_east,
                        failure_penalty);

                    if (score > best_score) {
                        best_score = score;
                        best_goal_north = cand_north;
                        best_goal_east = cand_east;
                        best_goal_yaw = candidate_yaw;
                        best_info_gain = info_gain;
                        best_cluster_size = static_cast<int>(cluster.cells.size());
                        best_cand_row = cand_row;
                        best_cand_col = cand_col;
                    }
                }
            }
        }

        auto try_recovery_viewpoint = [&]() -> bool {
            const double altitude_m = -grid.z_center;
            const bool allow_unknown_recovery = altitude_m >= 6.5;
            const int relaxed_clearance_cells = std::max(
                2,
                static_cast<int>(std::round(
                    std::max(0.45, 0.6 * frontier_min_obstacle_clearance_m_) / voxel_size_)));
            const int min_path_cells = std::max(
                2, static_cast<int>(std::ceil(1.20 / std::max(0.05, voxel_size_))));
            const int max_path_cells = std::max(
                min_path_cells + 1,
                static_cast<int>(std::ceil(10.0 / std::max(0.05, voxel_size_))));

            double fallback_score = -std::numeric_limits<double>::infinity();
            double fallback_north = 0.0;
            double fallback_east = 0.0;
            double fallback_yaw = yaw_now;
            double fallback_info = 0.0;
            int fallback_row = drone_row;
            int fallback_col = drone_col;

            for (int row = 0; row < grid.rows; ++row) {
                for (int col = 0; col < grid.cols; ++col) {
                    const int idx = row * grid.cols + col;
                    const size_t sidx = static_cast<size_t>(idx);
                    if (sidx >= reachable.size() || !reachable[sidx]) {
                        continue;
                    }
                    const int8_t cell_state = slice_cell(grid, row, col);
                    if (cell_state == 100) {
                        continue;
                    }
                    if (!allow_unknown_recovery && cell_state != 0) {
                        continue;
                    }
                    double north = 0.0;
                    double east = 0.0;
                    slice_cell_to_world(row, col, north, east);
                    const bool connector_cell =
                        is_high_layer_room_connector(north, east, grid.z_center);
                    if (sidx >= obstacle_mask.size() ||
                        (obstacle_mask[sidx] && !connector_cell)) {
                        continue;
                    }
                    if (sidx >= obs_dist_map.size()) {
                        continue;
                    }
                    if (!connector_cell && obs_dist_map[sidx] < relaxed_clearance_cells) {
                        continue;
                    }
                    const int path_cells = path_dist_map[sidx];
                    if (path_cells < min_path_cells || path_cells > max_path_cells) {
                        continue;
                    }

                    if (!in_allowed_area_with_clearance(north, east, 0.35)) {
                        continue;
                    }
                    const Vec3 pos{north, east, grid.z_center};
                    if (!volume_clear(pos, 0.35, frontier_candidate_vertical_half_m_)) {
                        continue;
                    }

                    double target_n = north;
                    double target_e = east;
                    double target_dist_sq = std::numeric_limits<double>::infinity();
                    int target_cluster_size = 1;
                    for (const auto &cluster : clusters) {
                        if (cluster.reachable_cells <= 0) {
                            continue;
                        }
                        const double dn = cluster.target_north - north;
                        const double de = cluster.target_east - east;
                        const double d2 = dn * dn + de * de;
                        if (d2 < target_dist_sq) {
                            target_dist_sq = d2;
                            target_n = cluster.target_north;
                            target_e = cluster.target_east;
                            target_cluster_size =
                                static_cast<int>(std::max<size_t>(1, cluster.cells.size()));
                        }
                    }

                    const double yaw = std::atan2(target_e - east, target_n - north);
                    const double info = estimate_information_gain(pos, yaw);
                    const double clearance_m =
                        std::min(obs_dist_map[sidx], 10) * voxel_size_;
                    const double path_m = path_cells * voxel_size_;
                    const double info_norm =
                        std::clamp(std::log1p(info) / std::log1p(250.0), 0.0, 1.0);
                    const double clearance_norm =
                        std::clamp(clearance_m / 2.0, 0.0, 1.0);
                    const double progress_norm =
                        std::clamp(path_m / 6.0, 0.0, 1.0);
                    double high_layer_transfer_bonus = 0.0;
                    if (allow_unknown_recovery) {
                        if (connector_cell && east > 13.5) {
                            high_layer_transfer_bonus = 12.0;
                        } else if (connector_cell) {
                            high_layer_transfer_bonus = 7.0;
                        } else if (east > 13.5) {
                            high_layer_transfer_bonus = 8.0;
                        } else if (east > 7.5 && north > -2.5 && north < 2.5) {
                            high_layer_transfer_bonus = 4.0;
                        }
                    }
                    const double score =
                        4.0 * info_norm +
                        8.0 * clearance_norm +
                        2.0 * progress_norm -
                        0.20 * path_m -
                        0.15 * std::abs(wrap_pi(yaw - yaw_now)) +
                        high_layer_transfer_bonus;

                    if (score > fallback_score) {
                        fallback_score = score;
                        fallback_north = north;
                        fallback_east = east;
                        fallback_yaw = yaw;
                        fallback_info = info;
                        fallback_row = row;
                        fallback_col = col;
                        best_cluster_size = target_cluster_size;
                    }
                }
            }

            if (fallback_score == -std::numeric_limits<double>::infinity()) {
                return false;
            }

            best_score = fallback_score;
            best_goal_north = fallback_north;
            best_goal_east = fallback_east;
            best_goal_yaw = fallback_yaw;
            best_info_gain = fallback_info;
            best_cand_row = fallback_row;
            best_cand_col = fallback_col;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Frontier recovery viewpoint selected — normal candidates rejected; "
                "fallback N=%.2f E=%.2f gain=%.1f score=%.2f unknown_ok=%d",
                best_goal_north, best_goal_east, best_info_gain, best_score,
                allow_unknown_recovery ? 1 : 0);
            return true;
        };

        if (best_score == -std::numeric_limits<double>::infinity()) {
            if (!try_recovery_viewpoint()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Frontier: all viewpoint candidates rejected — no planned route "
                    "(slice=%d vp=%d bfs=%d obs=%d vol=%d near=%d weak=%d)",
                    cand_slice_rejected, cand_vp_rejected,
                    cand_bfs_rejected, cand_obstacle_rejected,
                    cand_vol_rejected, cand_near_rejected, cand_weak_rejected);
                publish_empty_frontier_path();
                return;
            }
        }

        const double selected_goal_dist = std::hypot(best_goal_north - pose.x,
                                                     best_goal_east  - pose.y);
        if (selected_goal_dist < frontier_min_goal_distance_m_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Frontier: selected viewpoint already reached (dist=%.2fm) — no route published",
                selected_goal_dist);
            publish_empty_frontier_path();
            return;
        }
        const bool weak_high_layer_goal =
            (-grid.z_center >= obstacle_band_high_alt_threshold_m_) &&
            best_info_gain < 1.0 && best_cluster_size < 8;
        if (weak_high_layer_goal) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Frontier: suppressing weak high-layer viewpoint (gain=%.1f cluster=%d); "
                "waiting for coverage/stagnation instead of sending a pointless route",
                best_info_gain, best_cluster_size);
            publish_empty_frontier_path();
            return;
        }
        if (best_info_gain < 1.0 && best_cluster_size < 8) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Frontier: best viewpoint is weak (gain=%.1f cluster=%d) — navigating anyway to avoid deadlock",
                best_info_gain, best_cluster_size);
        }

        // Candidate reachability was verified through the obstacle-safe reachable bitmap.
        // Nothing is published until A* builds a concrete path to the viewpoint.
        if (cand_bfs_rejected > 0 || cand_obstacle_rejected > 0 ||
            cand_vol_rejected > 0 || cand_near_rejected > 0 ||
            cand_weak_rejected > 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Frontier: rejections — slice=%d vp=%d bfs=%d obs=%d vol=%d near=%d weak=%d",
                cand_slice_rejected, cand_vp_rejected,
                cand_bfs_rejected, cand_obstacle_rejected,
                cand_vol_rejected, cand_near_rejected, cand_weak_rejected);
        }

        // Publish waypoint chain via 3D A* (falls back to 2D if 3D fails).
        // 3D A* searches a stack of altitude layers centred on the drone so it
        // can route over/under obstacles instead of being blocked in one plane.
        // VFH handles live collision avoidance while executing the route.
        {
            Path path_msg;
            path_msg.header.stamp    = now();
            path_msg.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";

            // ── Build Z layer set centred on the drone's NED altitude ──────────
            constexpr int kNumZLayers = 11;    // ±1 m around current altitude
            const double active_z_ned =
                have_active_pose() ? active_position().z : grid.z_center;
            const double z_step = voxel_size_;
            std::vector<double> z_ned_layers(kNumZLayers);
            for (int zi = 0; zi < kNumZLayers; ++zi) {
                // Clamp: not below 0.45 m AGL, not above map ceiling
                const double raw_z = active_z_ned + (zi - kNumZLayers / 2) * z_step;
                z_ned_layers[static_cast<size_t>(zi)] =
                    std::clamp(raw_z, bounds_min_.z, -0.45);
            }

            auto passable_3d = build_passable_volume_3d(grid, z_ned_layers);

            // Force drone start cell passable in all layers (inflation may block it)
            const int mid_zi = kNumZLayers / 2;
            for (int zi = 0; zi < kNumZLayers; ++zi) {
                auto &layer = passable_3d[static_cast<size_t>(zi)];
                for (int dr = -1; dr <= 1; ++dr)
                    for (int dc = -1; dc <= 1; ++dc) {
                        const int nr = drone_row + dr, nc = drone_col + dc;
                        if (nr >= 0 && nr < grid.rows && nc >= 0 && nc < grid.cols)
                            layer[static_cast<size_t>(nr * grid.cols + nc)] = true;
                    }
            }

            // ── Try 3D A* first ───────────────────────────────────────────────
            auto raw_path_3d = compute_astar_3d_path(
                passable_3d, grid.rows, grid.cols,
                drone_row, drone_col, mid_zi,
                best_cand_row, best_cand_col, mid_zi,
                obs_dist_map);

            const bool used_3d = raw_path_3d.size() >= 2;

            // ── 2D fallback if 3D fails ───────────────────────────────────────
            std::vector<std::pair<int,int>> raw_path_2d;
            if (!used_3d) {
                raw_path_2d = compute_astar_path(
                    passable, grid.rows, grid.cols,
                    drone_row, drone_col, best_cand_row, best_cand_col,
                    obs_dist_map);
            }

            if (!used_3d && raw_path_2d.size() < 2) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "A* path to viewpoint failed (3D=%zu 2D=%zu) — no route published",
                    raw_path_3d.size(), raw_path_2d.size());
                publish_empty_frontier_path();
                return;
            }

            // ── Build path message with LoS shortcutting ──────────────────────
            if (used_3d) {
                // LoS shortcut within same Z layer; Z-transition steps kept as-is.
                auto seg_ok_3d = [&](size_t ia, size_t ib) -> bool {
                    const auto &[r0, c0, zi0] = raw_path_3d[ia];
                    const auto &[r1, c1, zi1] = raw_path_3d[ib];
                    if (zi0 != zi1) return false;
                    const auto &layer = passable_3d[static_cast<size_t>(zi0)];
                    int dr = std::abs(r1 - r0), dc = std::abs(c1 - c0);
                    int sr = r0 < r1 ? 1 : -1, sc = c0 < c1 ? 1 : -1;
                    int err = dr - dc, r = r0, c = c0;
                    while (true) {
                        if (r < 0 || r >= grid.rows || c < 0 || c >= grid.cols) return false;
                        if (!layer[static_cast<size_t>(r * grid.cols + c)]) return false;
                        if (!shortcut_cell_clear_enough(r, c, obs_dist_map, grid.rows, grid.cols)) return false;
                        if (r == r1 && c == c1) break;
                        const int e2 = 2 * err;
                        if (e2 > -dc) { err -= dc; r += sr; }
                        if (e2 <  dr) { err += dr; c += sc; }
                    }
                    return true;
                };

                std::vector<size_t> final_idx;
                final_idx.reserve(32);
                final_idx.push_back(0);
                size_t cur = 0;
                const size_t Np = raw_path_3d.size();
                while (cur + 1 < Np) {
                    const auto &[cur_r, cur_c, cur_zi] = raw_path_3d[cur];
                    (void)cur_zi;
                    const size_t max_shortcut_cells = shortcut_cells_for_clearance(
                        cur_r, cur_c, obs_dist_map, grid.rows, grid.cols);
                    size_t lo = cur + 1;
                    size_t hi = std::min(Np - 1, cur + max_shortcut_cells);
                    size_t farthest = cur + 1;
                    while (lo <= hi) {
                        const size_t mid = (lo + hi) / 2;
                        if (seg_ok_3d(cur, mid)) { farthest = mid; lo = mid + 1; }
                        else { if (mid == 0) break; hi = mid - 1; }
                    }
                    final_idx.push_back(farthest);
                    cur = farthest;
                }

                std::vector<SmoothPathPoint> path_points;
                path_points.reserve(final_idx.size());
                for (size_t fi = 0; fi < final_idx.size(); ++fi) {
                    const auto &[r, c, zi] = raw_path_3d[final_idx[fi]];
                    double wn = 0.0, we = 0.0;
                    slice_cell_to_world(r, c, wn, we);
                    path_points.push_back({
                        wn, we, z_ned_layers[static_cast<size_t>(zi)],
                    });
                }

                const auto publish_points =
                    make_safe_bspline_path(path_points, grid, passable, obs_dist_map);
                for (size_t fi = 0; fi < publish_points.size(); ++fi) {
                    const auto &pt = publish_points[fi];
                    PoseStamped ps;
                    ps.header          = path_msg.header;
                    ps.pose.position.x = pt.north;
                    ps.pose.position.y = pt.east;
                    ps.pose.position.z = pt.z_ned;
                    if (fi + 1 == publish_points.size()) {
                        ps.pose.orientation.z = std::sin(best_goal_yaw * 0.5);
                        ps.pose.orientation.w = std::cos(best_goal_yaw * 0.5);
                    } else {
                        ps.pose.orientation.w = 1.0;
                    }
                    path_msg.poses.push_back(ps);
                }
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "3D A* path: %zu raw → %zu shortcut → %zu smoothed waypoints "
                    "(adaptive max_seg<=%.1fm, bspline=%s)",
                    raw_path_3d.size(), final_idx.size(), publish_points.size(),
                    path_shortcut_max_segment_m_,
                    publish_points.size() != path_points.size() ? "safe" : "fallback");
            } else {
                // 2D fallback: Bresenham LoS shortcutting at drone's current altitude.
                auto seg_ok = [&](size_t ia, size_t ib) -> bool {
                    int r0 = raw_path_2d[ia].first, c0 = raw_path_2d[ia].second;
                    int r1 = raw_path_2d[ib].first, c1 = raw_path_2d[ib].second;
                    int dr = std::abs(r1 - r0), dc = std::abs(c1 - c0);
                    int sr = r0 < r1 ? 1 : -1, sc = c0 < c1 ? 1 : -1;
                    int err = dr - dc, r = r0, c = c0;
                    while (true) {
                        if (r < 0 || r >= grid.rows || c < 0 || c >= grid.cols) return false;
                        if (!passable[static_cast<size_t>(r * grid.cols + c)]) return false;
                        if (!shortcut_cell_clear_enough(r, c, obs_dist_map, grid.rows, grid.cols)) return false;
                        if (r == r1 && c == c1) break;
                        const int e2 = 2 * err;
                        if (e2 > -dc) { err -= dc; r += sr; }
                        if (e2 <  dr) { err += dr; c += sc; }
                    }
                    return true;
                };

                std::vector<size_t> final_idx;
                final_idx.reserve(16);
                final_idx.push_back(0);
                size_t cur = 0;
                const size_t Np = raw_path_2d.size();
                while (cur + 1 < Np) {
                    const auto [cur_r, cur_c] = raw_path_2d[cur];
                    const size_t max_shortcut_cells = shortcut_cells_for_clearance(
                        cur_r, cur_c, obs_dist_map, grid.rows, grid.cols);
                    size_t lo = cur + 1;
                    size_t hi = std::min(Np - 1, cur + max_shortcut_cells);
                    size_t farthest = cur + 1;
                    while (lo <= hi) {
                        const size_t mid = (lo + hi) / 2;
                        if (seg_ok(cur, mid)) { farthest = mid; lo = mid + 1; }
                        else { if (mid == 0) break; hi = mid - 1; }
                    }
                    final_idx.push_back(farthest);
                    cur = farthest;
                }

                std::vector<SmoothPathPoint> path_points;
                path_points.reserve(final_idx.size());
                for (size_t fi = 0; fi < final_idx.size(); ++fi) {
                    const auto [r, c] = raw_path_2d[final_idx[fi]];
                    double wn = 0.0, we = 0.0;
                    slice_cell_to_world(r, c, wn, we);
                    path_points.push_back({wn, we, grid.z_center});
                }

                const auto publish_points =
                    make_safe_bspline_path(path_points, grid, passable, obs_dist_map);
                for (size_t fi = 0; fi < publish_points.size(); ++fi) {
                    const auto &pt = publish_points[fi];
                    PoseStamped ps;
                    ps.header          = path_msg.header;
                    ps.pose.position.x = pt.north;
                    ps.pose.position.y = pt.east;
                    ps.pose.position.z = pt.z_ned;
                    if (fi + 1 == publish_points.size()) {
                        ps.pose.orientation.z = std::sin(best_goal_yaw * 0.5);
                        ps.pose.orientation.w = std::cos(best_goal_yaw * 0.5);
                    } else {
                        ps.pose.orientation.w = 1.0;
                    }
                    path_msg.poses.push_back(ps);
                }
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "2D A* fallback: %zu raw → %zu shortcut → %zu smoothed waypoints "
                    "(adaptive max_seg<=%.1fm, bspline=%s)",
                    raw_path_2d.size(), final_idx.size(), publish_points.size(),
                    path_shortcut_max_segment_m_,
                    publish_points.size() != path_points.size() ? "safe" : "fallback");
            }

            frontier_path_pub_->publish(path_msg);
            frontier_route_available_ = true;

            PointStamped goal{};
            goal.header = path_msg.header;
            goal.point.x = best_goal_north;
            goal.point.y = best_goal_east;
            goal.point.z = grid.z_center;
            frontier_goal_pub_->publish(goal);

            PoseStamped goal_pose{};
            goal_pose.header = path_msg.header;
            goal_pose.pose.position.x = best_goal_north;
            goal_pose.pose.position.y = best_goal_east;
            goal_pose.pose.position.z = grid.z_center;
            goal_pose.pose.orientation.w = std::cos(best_goal_yaw * 0.5);
            goal_pose.pose.orientation.z = std::sin(best_goal_yaw * 0.5);
            frontier_goal_pose_pub_->publish(goal_pose);
        }

        // Record the published goal and the drone's current position so the
        // next planning tick can penalise this cluster if the drone gets stuck.
        frontier_last_goal_n_    = best_goal_north;
        frontier_last_goal_e_    = best_goal_east;
        frontier_last_drone_n_   = pose.x;
        frontier_last_drone_e_   = pose.y;
        frontier_last_best_score_ = best_score;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Frontier viewpoint → N=%.2f E=%.2f yaw=%.0f° gain=%.0f "
            "cluster=%d score=%.2f (rej: sl=%d vp=%d bfs=%d obs=%d vol=%d near=%d weak=%d)",
            best_goal_north,
            best_goal_east,
            best_goal_yaw * 180.0 / M_PI,
            best_info_gain,
            best_cluster_size,
            best_score,
            cand_slice_rejected, cand_vp_rejected,
            cand_bfs_rejected, cand_obstacle_rejected,
            cand_vol_rejected, cand_near_rejected, cand_weak_rejected);
    }

    void publish_return_path(const SliceGrid &grid,
                             const std::vector<bool> &obstacle_mask_prebuilt)
    {
        if (!have_active_pose()) {
            return;
        }

        const Vec3 pose = active_position();
        const double dist_home = std::hypot(pose.x - home_north_m_, pose.y - home_east_m_);
        if (dist_home < frontier_min_goal_distance_m_) {
            return;
        }

        int drone_row = 0, drone_col = 0;
        int home_row = 0, home_col = 0;
        if (!point_to_slice_cell(pose.x, pose.y, drone_row, drone_col) ||
            !point_to_slice_cell(home_north_m_, home_east_m_, home_row, home_col)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Return path: drone/home out of grid — skipping");
            return;
        }

        auto passable = compute_passable_map(grid, drone_row, drone_col);

        const int home_idx = home_row * grid.cols + home_col;
        if (!obstacle_mask_prebuilt[static_cast<size_t>(home_idx)] &&
            slice_cell(grid, home_row, home_col) != 100 &&
            in_allowed_area(home_north_m_, home_east_m_)) {
            // The takeoff cell can be transiently unknown on higher scan layers.
            // Allow it as a terminal cell, but never allow a confirmed obstacle.
            passable[static_cast<size_t>(home_idx)] = true;
        }

        const auto obs_dist_map = compute_obstacle_dist_map(grid, &obstacle_mask_prebuilt);
        const auto raw_path = compute_astar_path(
            passable, grid.rows, grid.cols,
            drone_row, drone_col, home_row, home_col,
            obs_dist_map);

        if (raw_path.size() < 2) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Return path: A* failed from (%d,%d) to home (%d,%d)",
                drone_row, drone_col, home_row, home_col);
            return;
        }

        Path path_msg;
        path_msg.header.stamp = now();
        path_msg.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";

        const int kStep = std::max(3, static_cast<int>(raw_path.size()) / 8);
        std::vector<size_t> kept;
        kept.reserve(12);
        for (size_t i = 0; i < raw_path.size(); ++i) {
            if (i % static_cast<size_t>(kStep) == 0 || i + 1 == raw_path.size()) {
                kept.push_back(i);
            }
        }

        auto seg_ok = [&](size_t ia, size_t ib) -> bool {
            int r0 = raw_path[ia].first, c0 = raw_path[ia].second;
            int r1 = raw_path[ib].first, c1 = raw_path[ib].second;
            int dr = std::abs(r1 - r0), dc = std::abs(c1 - c0);
            int sr = r0 < r1 ? 1 : -1, sc = c0 < c1 ? 1 : -1;
            int err = dr - dc, r = r0, c = c0;
            while (true) {
                if (r < 0 || r >= grid.rows || c < 0 || c >= grid.cols) {
                    return false;
                }
                if (!passable[static_cast<size_t>(r * grid.cols + c)]) {
                    return false;
                }
                if (!shortcut_cell_clear_enough(r, c, obs_dist_map, grid.rows, grid.cols)) {
                    return false;
                }
                if (r == r1 && c == c1) {
                    break;
                }
                const int e2 = 2 * err;
                if (e2 > -dc) { err -= dc; r += sr; }
                if (e2 <  dr) { err += dr; c += sc; }
            }
            return true;
        };

        std::vector<size_t> final_idx;
        final_idx.reserve(raw_path.size());
        final_idx.push_back(kept[0]);
        for (size_t k = 1; k < kept.size(); ++k) {
            if (seg_ok(kept[k - 1], kept[k])) {
                final_idx.push_back(kept[k]);
            } else {
                for (size_t j = kept[k - 1] + 1; j <= kept[k]; ++j) {
                    final_idx.push_back(j);
                }
            }
        }

        for (const size_t idx : final_idx) {
            const auto [r, c] = raw_path[idx];
            double wn = 0.0, we = 0.0;
            slice_cell_to_world(r, c, wn, we);
            PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = wn;
            ps.pose.position.y = we;
            ps.pose.position.z = grid.z_center;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
        }

        return_path_pub_->publish(path_msg);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "Return path: %zu raw → %zu validated waypoints to home (%.2f,%.2f)",
            raw_path.size(), final_idx.size(), home_north_m_, home_east_m_);
    }

    void apply_log_odds(const VoxelKey &key, double delta)
    {
        auto [it, inserted] = voxels_.try_emplace(key, 0.0f);
        double applied_delta = delta;

        if (delta < 0.0) {
            const double p = sigmoid(it->second);
            // Keep confirmed walls/stationary structure from disappearing too fast
            // because of pose jitter or slightly inconsistent depth rays.
            if (p >= occupied_retain_prob_) {
                applied_delta *= occupied_miss_scale_;
            }
        }

        double clamped = std::clamp(static_cast<double>(it->second) + applied_delta,
                                    log_odds_min_, log_odds_max_);
        it->second = static_cast<float>(clamped);
        ++voxels_touched_since_publish_;
        (void)inserted;
    }

    // March from `origin` to `hit` in half-voxel steps, decrementing log-odds
    // for each newly-entered free-space cell along the way, then increment the
    // endpoint cell as occupied. A fixed-step march (rather than e.g.
    // Amanatides–Woo) is simple and accurate enough at this voxel resolution.
    void cast_ray(const Vec3 &origin, const Vec3 &hit)
    {
        const double dx = hit.x - origin.x;
        const double dy = hit.y - origin.y;
        const double dz = hit.z - origin.z;
        const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-6) {
            return;
        }

        const double step = voxel_size_ * 0.5;
        VoxelKey prev_key = world_to_key(origin);
        bool have_prev = false;
        bool ray_stopped_early = false;
        for (double s = step; s < len; s += step) {
            const Vec3 p{origin.x + dx / len * s, origin.y + dy / len * s,
                         origin.z + dz / len * s};
            if (!in_bounds(p)) continue;
            if (!in_allowed_area(p.x, p.y)) {
                // Ray exited the allowed world zone — stop immediately.
                // Using `continue` here would punch through masked zone boundaries
                // and mark ghost free-space in distant rooms (e.g. Room B via corridor gap).
                ray_stopped_early = true;
                break;
            }

            const VoxelKey key = world_to_key(p);
            if (!have_prev || !(key == prev_key)) {
                // Stop marking free cells once the ray crosses a confirmed wall.
                // Prevents ghost free-space behind walls from depth camera keyhole.
                auto it = voxels_.find(key);
                if (it != voxels_.end() && sigmoid(it->second) >= occupied_prob_) {
                    ray_stopped_early = true;
                    break;
                }
                apply_log_odds(key, log_odds_miss_);
                prev_key = key;
                have_prev = true;
            }
        }
        // Only stamp the hit endpoint when the ray completed naturally.
        // If the loop broke early (zone boundary or mid-ray wall), the sensor
        // hit point is beyond the stopping position — applying it there would
        // create a phantom occupied voxel behind the real wall (keyhole effect).
        if (!ray_stopped_early && in_bounds(hit) && in_allowed_area(hit.x, hit.y)) {
            apply_log_odds(world_to_key(hit), log_odds_hit_);
        }
    }

    void on_lidar_scan(const LaserScan::SharedPtr &msg)
    {
        if (!have_active_pose()) return;

        const Vec3 drone_pos = active_position();
        const auto q = active_quat();
        // Yaw from NED quaternion (w,x,y,z)
        const float yaw = std::atan2(2.0f * (q[0]*q[3] + q[1]*q[2]),
                                      1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));

        const float angle_min = msg->angle_min;
        const float angle_inc = msg->angle_increment;
        const size_t n = msg->ranges.size();

        for (size_t i = 0; i < n; ++i) {
            const float r = msg->ranges[i];
            if (!std::isfinite(r) || r < 0.10f || r > static_cast<float>(max_range_)) continue;

            const float world_angle = yaw + angle_min + static_cast<float>(i) * angle_inc;
            const Vec3 hit{drone_pos.x + static_cast<double>(r) * std::cos(world_angle),
                           drone_pos.y + static_cast<double>(r) * std::sin(world_angle),
                           drone_pos.z};
            cast_ray(drone_pos, hit);
        }
    }

    void on_depth(const Image::SharedPtr &msg)
    {
        if (!have_info_ || !have_active_pose()) {
            return;
        }
        if (msg->encoding != "32FC1") {
            RCLCPP_WARN_ONCE(get_logger(), "Unexpected depth encoding '%s' (expected 32FC1)",
                             msg->encoding.c_str());
            return;
        }

        const auto &K = info_.k;
        const double fx = K[0], fy = K[4], cx = K[2], cy = K[5];
        if (fx == 0.0 || fy == 0.0) {
            return;
        }

        const auto *depth = reinterpret_cast<const float *>(msg->data.data());
        const int width  = static_cast<int>(msg->width);
        const int height = static_cast<int>(msg->height);

        const Vec3 drone_pos = active_position();
        const auto drone_q = active_quat();
        // Camera origin = drone position + mount offset rotated into world frame
        // — the ray-cast starting point for every pixel in this frame.
        const Vec3 cam_world = drone_pos + quat_rotate(drone_q, cam_offset_);

        for (int v = 0; v < height; v += pixel_stride_) {
            for (int u = 0; u < width; u += pixel_stride_) {
                const float d = depth[v * width + u];
                if (!std::isfinite(d) || d <= 0.0f || d > max_range_) {
                    continue;
                }

                // Pinhole back-projection in the optical frame (x right, y down, z forward).
                const double x_opt = (u - cx) * d / fx;
                const double y_opt = (v - cy) * d / fy;
                const double z_opt = d;

                // Optical frame -> body FRD for the neutral camera axes, then
                // apply the fixed 15° downward mount pitch used by x500_swarm.
                const Vec3 body_frd = pitch_about_body_y(
                    {z_opt, x_opt, y_opt}, camera_pitch_rad_);
                const Vec3 hit_world = drone_pos + quat_rotate(drone_q, cam_offset_ + body_frd);

                cast_ray(cam_world, hit_world);
            }
        }
    }

    void publish_summary_and_cloud()
    {
        if (voxels_.empty()) {
            return;
        }

        double entropy_sum = 0.0;
        double cent_x = 0.0, cent_y = 0.0, cent_z = 0.0, h_sum = 0.0;
        uint32_t observed_count = 0;
        uint32_t occupied_count = 0;
        uint32_t free_count = 0;
        for (const auto &[key, log_odds] : voxels_) {
            const double p = sigmoid(log_odds);
            const double h = binary_entropy(p);
            entropy_sum += h;
            const Vec3 c = key_to_center(key);
            cent_x += h * c.x;  cent_y += h * c.y;  cent_z += h * c.z;
            h_sum  += h;
            if (std::abs(p - 0.5) > observed_delta_) {
                ++observed_count;
                if (p <= occupied_prob_) {
                    ++free_count;
                }
            }
            if (p > occupied_prob_) {
                ++occupied_count;
            }
        }

        // Entropy-weighted centroid (Bernoulli entropy per voxel as weight).
        // Published in the world/odom frame so the controller can use it as
        // an attraction goal.  Suppressed when the centroid is too close to the
        // drone: publishing a centroid the drone is already sitting on causes an
        // infinite commit→arrive→commit loop at the same point.
        constexpr double kCentEps = 1e-9;
        const double centroid_n = cent_x / (h_sum + kCentEps);
        const double centroid_e = cent_y / (h_sum + kCentEps);
        bool publish_entropy_centroid = true;
        if (have_active_pose()) {
            const Vec3 p = active_position();
            const double dn = centroid_n - p.x;
            const double de = centroid_e - p.y;
            if (dn * dn + de * de <
                    frontier_min_goal_distance_m_ * frontier_min_goal_distance_m_) {
                // Suppress only the entropy fallback. The map summary, frontier path,
                // and clouds must still publish; otherwise the controller loses its
                // primary route source and holds position waiting for a fresh goal.
                publish_entropy_centroid = false;
            }
        }
        if (publish_entropy_centroid) {
            PointStamped centroid_msg{};
            centroid_msg.header.stamp    = now();
            centroid_msg.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";
            centroid_msg.point.x = centroid_n;
            centroid_msg.point.y = centroid_e;
            centroid_msg.point.z = cent_z / (h_sum + kCentEps);
            entropy_centroid_pub_->publish(centroid_msg);
        }

        // Accumulate drone path for 3D visualization (append current pose, publish full path).
        if (have_active_pose()) {
            const Vec3 pose_ned = active_position();
            const auto q = active_quat();
            PoseStamped pose{};
            pose.header.stamp    = now();
            pose.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";
            pose.pose.position.x = pose_ned.x;
            pose.pose.position.y = pose_ned.y;
            pose.pose.position.z = pose_ned.z;
            pose.pose.orientation.w = q[0];
            pose.pose.orientation.x = q[1];
            pose.pose.orientation.y = q[2];
            pose.pose.orientation.z = q[3];
            drone_path_msg_.header.stamp    = now();
            drone_path_msg_.header.frame_id = pose.header.frame_id;
            drone_path_msg_.poses.push_back(pose);
            // Limit stored history to 10000 poses (~166 min at 1 Hz)
            if (drone_path_msg_.poses.size() > 10000) {
                drone_path_msg_.poses.erase(drone_path_msg_.poses.begin());
            }
            path_pub_->publish(drone_path_msg_);
        }

        const SliceGrid slice = build_slice_grid();
        const auto obs_mask   = build_obstacle_mask(slice);
        const auto layer_stats = compute_layer_coverage(slice);
        const double dynamic_global_coverage =
            update_dynamic_global_coverage(layer_stats);
        publish_slice_map(slice);
        publish_obstacle_mask(slice, obs_mask);
        publish_frontier_goal(slice, obs_mask);
        publish_return_path(slice, obs_mask);

        MapUpdateSummary summary{};
        summary.header.stamp = now();
        summary.header.frame_id = slam_pose_fresh() ? "map_ned" : "odom";
        summary.drone_id = static_cast<uint8_t>(drone_id_);
        summary.entropy_mean = static_cast<float>(entropy_sum / voxels_.size());
        summary.coverage_fraction = static_cast<float>(dynamic_global_coverage);
        summary.voxels_updated = voxels_touched_since_publish_;
        summary.layer_coverage_fraction = static_cast<float>(
            std::clamp(layer_stats.fraction, 0.0, 1.0));
        summary.layer_altitude_m = static_cast<float>(layer_stats.altitude_m);
        summary.layer_observed_cells = layer_stats.observed_cells;
        summary.layer_total_cells = layer_stats.total_cells;
        summary.frontier_clusters = frontier_clusters_count_;
        summary.reachable_frontier_clusters = reachable_frontier_clusters_count_;
        summary.reachable_frontier_cells = reachable_frontier_cells_count_;
        summary.frontier_route_available = frontier_route_available_;
        summary_pub_->publish(summary);
        voxels_touched_since_publish_ = 0;

        publish_cloud(occupied_count);
        // Free cloud is large — publish at ~1 Hz regardless of main timer rate.
        if (++free_cloud_tick_ % std::max(1u, static_cast<uint32_t>(publish_rate_hz_)) == 0) {
            publish_free_cloud(free_count);
        }
    }

    void publish_cloud(uint32_t occupied_count)
    {
        PointCloud2 cloud;
        cloud.header.stamp = now();
        cloud.header.frame_id = "odom";
        cloud.height = 1;
        cloud.width = occupied_count;
        cloud.is_dense = true;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(occupied_count);

        sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> it_i(cloud, "intensity");

        for (const auto &[key, log_odds] : voxels_) {
            const double p = sigmoid(log_odds);
            if (p <= occupied_prob_) {
                continue;
            }
            const Vec3 c = key_to_center(key);
            *it_x = static_cast<float>(c.x);
            *it_y = static_cast<float>(c.y);
            *it_z = static_cast<float>(c.z);
            *it_i = static_cast<float>(p);  // colour-coded by occupancy confidence in RViz
            ++it_x; ++it_y; ++it_z; ++it_i;
        }

        cloud_pub_->publish(cloud);
    }

    void publish_free_cloud(uint32_t free_count)
    {
        if (free_cloud_pub_->get_subscription_count() == 0) {
            return;
        }

        PointCloud2 cloud;
        cloud.header.stamp = now();
        cloud.header.frame_id = "odom";
        cloud.height = 1;
        cloud.width = free_count;
        cloud.is_dense = true;
        cloud.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(free_count);

        sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

        for (const auto &[key, log_odds] : voxels_) {
            const double p = sigmoid(log_odds);
            if (p > occupied_prob_ || std::abs(p - 0.5) <= observed_delta_) {
                continue;
            }
            const Vec3 c = key_to_center(key);
            *it_x = static_cast<float>(c.x);
            *it_y = static_cast<float>(c.y);
            *it_z = static_cast<float>(c.z);
            ++it_x; ++it_y; ++it_z;
        }

        free_cloud_pub_->publish(cloud);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelMapper>());
    rclcpp::shutdown();
    return 0;
}
