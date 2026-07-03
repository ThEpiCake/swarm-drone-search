// Layered 2D occupancy map built from RPLIDAR A2M8 LaserScan data.
//
// Architecture:
//   - Space is sliced into horizontal layers, each layer_height_m thick.
//   - Every incoming scan is ray-cast into the layer that matches the drone's
//     current altitude (-pos_.z in NED).
//   - Ray casting: cells along each ray → FREE (0); endpoint → OCCUPIED (100).
//   - Obstacle inflation: each OCCUPIED cell inflates a disc of radius
//     inflate_cells * resolution to give the drone body clearance.
//   - The map at the drone's current layer is published as nav_msgs/OccupancyGrid
//     so the FrontierPlanner can find unexplored areas.
//
// Coordinate convention throughout this node (matches search_mission_controller):
//   pos_.x  = North (NED-X)
//   pos_.y  = East  (NED-Y)
//   pos_.z  = Down  (NED-Z, so altitude = -pos_.z)
//   pos_.heading = NED yaw: 0 = North, π/2 = East, clockwise positive
//
// RPLIDAR body-frame convention (ROS LaserScan standard):
//   angle = 0   → drone forward (+X body)
//   angle = +π/2 → drone left   (+Y body, which is West when heading=0)
//
// World-angle transform:  world_angle_NED = heading - body_angle
//   North component of ray: cos(world_angle)
//   East  component of ray: sin(world_angle)

#include <algorithm>
#include <cmath>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <swarm_msgs/msg/map_update_summary.hpp>

using nav_msgs::msg::OccupancyGrid;
using px4_msgs::msg::VehicleLocalPosition;
using sensor_msgs::msg::LaserScan;
using swarm_msgs::msg::MapUpdateSummary;

class LidarMapper : public rclcpp::Node
{
public:
    LidarMapper() : Node("lidar_mapper")
    {
        resolution_   = declare_parameter<float>("resolution_m",   0.15f);
        layer_height_ = declare_parameter<float>("layer_height_m", 0.30f);
        num_layers_   = declare_parameter<int>  ("num_layers",      14);
        map_origin_n_ = declare_parameter<float>("map_origin_n_m", -6.5f);
        map_origin_e_ = declare_parameter<float>("map_origin_e_m", -6.5f);
        map_size_n_   = declare_parameter<float>("map_size_n_m",   13.0f);
        map_size_e_   = declare_parameter<float>("map_size_e_m",   13.0f);
        inflate_cells_= declare_parameter<int>  ("inflate_cells",   3);

        grid_n_ = static_cast<int>(std::ceil(map_size_n_ / resolution_));
        grid_e_ = static_cast<int>(std::ceil(map_size_e_ / resolution_));

        // All cells start UNKNOWN (-1)
        layers_.assign(num_layers_,
            std::vector<int8_t>(grid_n_ * grid_e_, -1));

        auto qos_sub = rclcpp::QoS(1).best_effort();
        // Latched publisher so the frontier planner gets the latest map on subscribe
        auto qos_pub = rclcpp::QoS(1).transient_local().reliable();

        scan_sub_ = create_subscription<LaserScan>(
            "scan", qos_sub,
            [this](LaserScan::SharedPtr msg) {
                if (has_pos_) { integrate_scan(msg); }
            });

        pos_sub_ = create_subscription<VehicleLocalPosition>(
            "fmu/out/vehicle_local_position_v1", qos_sub,
            [this](VehicleLocalPosition::SharedPtr msg) {
                pos_     = *msg;
                has_pos_ = pos_.xy_valid && pos_.z_valid &&
                           std::isfinite(pos_.heading);
            });

        map_pub_      = create_publisher<OccupancyGrid>("occupancy_map", qos_pub);
        summary_pub_  = create_publisher<MapUpdateSummary>("map_update_summary", rclcpp::QoS(10));

        RCLCPP_INFO(get_logger(),
            "LidarMapper ready — grid %d×%d cells, %d layers "
            "(%.2f m/cell, %.2f m/layer)",
            grid_n_, grid_e_, num_layers_, resolution_, layer_height_);
    }

private:
    // ── parameters ───────────────────────────────────────────────────────────
    float resolution_, layer_height_;
    float map_origin_n_, map_origin_e_;
    float map_size_n_,   map_size_e_;
    int   num_layers_, grid_n_, grid_e_, inflate_cells_;

    // ── map storage ───────────────────────────────────────────────────────────
    // layers_[layer_idx][row * grid_e_ + col]
    //   row = North index (0 = map_origin_n_ side)
    //   col = East  index (0 = map_origin_e_ side)
    std::vector<std::vector<int8_t>> layers_;

    // ── sensor state ──────────────────────────────────────────────────────────
    VehicleLocalPosition pos_{};
    bool has_pos_ = false;
    int  last_published_layer_ = -1;

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::Subscription<LaserScan>::SharedPtr           scan_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Publisher<OccupancyGrid>::SharedPtr           map_pub_;
    rclcpp::Publisher<MapUpdateSummary>::SharedPtr        summary_pub_;

    // =========================================================================
    // Grid helpers
    // =========================================================================

    int layer_of(float altitude) const
    {
        return std::clamp(
            static_cast<int>(altitude / layer_height_), 0, num_layers_ - 1);
    }

    // Convert world-NED (n, e) to flat grid index. Returns -1 if out of bounds.
    int cell_idx(float n, float e) const
    {
        const int ni = static_cast<int>((n - map_origin_n_) / resolution_);
        const int ei = static_cast<int>((e - map_origin_e_) / resolution_);
        if (ni < 0 || ni >= grid_n_ || ei < 0 || ei >= grid_e_) return -1;
        return ni * grid_e_ + ei;
    }

    void mark_free(int layer, float n, float e)
    {
        const int idx = cell_idx(n, e);
        if (idx < 0) return;
        auto &c = layers_[layer][idx];
        if (c != 100) c = 0;  // never overwrite a confirmed obstacle
    }

    void mark_occupied(int layer, float n, float e)
    {
        const int ni = static_cast<int>((n - map_origin_n_) / resolution_);
        const int ei = static_cast<int>((e - map_origin_e_) / resolution_);
        if (ni < 0 || ni >= grid_n_ || ei < 0 || ei >= grid_e_) return;

        for (int dn = -inflate_cells_; dn <= inflate_cells_; ++dn) {
            for (int de = -inflate_cells_; de <= inflate_cells_; ++de) {
                if (dn*dn + de*de > inflate_cells_ * inflate_cells_) continue;
                const int nni = ni + dn, nei = ei + de;
                if (nni < 0 || nni >= grid_n_ || nei < 0 || nei >= grid_e_) continue;
                layers_[layer][nni * grid_e_ + nei] = 100;
            }
        }
    }

    // =========================================================================
    // Scan integration
    // =========================================================================

    void integrate_scan(const LaserScan::SharedPtr &scan)
    {
        const float altitude = -pos_.z;      // NED: z-down → altitude = -z
        const int   layer    = layer_of(altitude);
        const float heading  = pos_.heading; // NED yaw
        const float drone_n  = pos_.x;
        const float drone_e  = pos_.y;
        const float step     = resolution_ * 0.6f;  // trace step (smaller than cell)

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            const float body_angle  = scan->angle_min +
                                      static_cast<float>(i) * scan->angle_increment;
            // Transform: body frame (CCW positive) → NED world angle (CW positive)
            const float world_angle = heading - body_angle;
            const float cos_w = std::cos(world_angle);
            const float sin_w = std::sin(world_angle);

            const float r = scan->ranges[i];
            if (!std::isfinite(r) || r <= scan->range_min) continue;
            const bool  hit       = r < scan->range_max * 0.99f;
            const float trace_len = hit ? r : scan->range_max;

            // Mark free space along ray (skip cell 0 — drone's own cell)
            const int free_steps = static_cast<int>(trace_len / step);
            for (int s = 1; s < free_steps; ++s) {
                const float d = s * step;
                mark_free(layer, drone_n + cos_w * d, drone_e + sin_w * d);
            }

            // Mark obstacle at endpoint
            if (hit) {
                mark_occupied(layer, drone_n + cos_w * r, drone_e + sin_w * r);
            }
        }

        // Publish whenever the active layer changes or every scan
        publish_map(layer);
    }

    // =========================================================================
    // Publish
    // =========================================================================

    void publish_map(int layer)
    {
        OccupancyGrid grid;
        grid.header.stamp    = now();
        grid.header.frame_id = "map_ned";

        // nav_msgs/OccupancyGrid:
        //   width  = number of cells in the X (East) direction
        //   height = number of cells in the Y (North) direction
        //   data[row * width + col] = data[North_idx * grid_e_ + East_idx]
        //   origin = bottom-left corner (min East, min North)
        grid.info.resolution = resolution_;
        grid.info.width      = static_cast<uint32_t>(grid_e_);
        grid.info.height     = static_cast<uint32_t>(grid_n_);
        grid.info.origin.position.x = static_cast<double>(map_origin_e_);
        grid.info.origin.position.y = static_cast<double>(map_origin_n_);
        grid.info.origin.position.z = static_cast<double>(layer * layer_height_);
        grid.info.origin.orientation.w = 1.0;

        const auto &ld = layers_[layer];
        grid.data.assign(ld.begin(), ld.end());

        map_pub_->publish(grid);

        // Coverage fraction: (free + occupied) / total across all layers
        size_t known = 0;
        const size_t total = static_cast<size_t>(num_layers_) * grid_n_ * grid_e_;
        for (const auto &l : layers_) {
            for (int8_t c : l) {
                if (c >= 0) ++known;
            }
        }
        MapUpdateSummary summary{};
        summary.coverage_fraction = static_cast<float>(known) / static_cast<float>(total);
        summary_pub_->publish(summary);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMapper>());
    rclcpp::shutdown();
    return 0;
}
