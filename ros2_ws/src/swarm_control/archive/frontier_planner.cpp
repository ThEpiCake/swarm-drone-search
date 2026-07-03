// Frontier-based exploration planner.
//
// A FRONTIER is a FREE cell (value 0) that has at least one UNKNOWN neighbour
// (value -1) in the 4-connected grid.  Frontiers mark the boundary between
// explored and unexplored space — flying toward them maximally uncovers the map.
//
// Algorithm (every replan_period_s_ seconds):
//   1. Scan the occupancy grid for frontier cells.
//   2. Find the frontier cell nearest to the drone.
//   3. Cluster: collect all frontiers within cluster_radius_m_ of that cell.
//   4. Publish the cluster centroid as geometry_msgs/PointStamped
//      (point.x = North, point.y = East) on topic "frontier_goal".
//
// The search_mission_controller consumes this topic to replace fixed patrol
// waypoints with map-driven exploration goals.

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

using geometry_msgs::msg::PointStamped;
using nav_msgs::msg::OccupancyGrid;
using px4_msgs::msg::VehicleLocalPosition;

class FrontierPlanner : public rclcpp::Node
{
public:
    FrontierPlanner() : Node("frontier_planner")
    {
        min_frontier_cells_ = declare_parameter<int>  ("min_frontier_cells", 3);
        cluster_radius_m_   = declare_parameter<float>("cluster_radius_m",   1.5f);
        replan_period_s_    = declare_parameter<float>("replan_period_s",    2.0f);

        auto qos_sub = rclcpp::QoS(1).best_effort();
        // Match the transient_local publisher on the lidar_mapper side so we
        // always receive the latest map even if we subscribe after it was sent.
        auto qos_map = rclcpp::QoS(1).transient_local().reliable();

        pos_sub_ = create_subscription<VehicleLocalPosition>(
            "fmu/out/vehicle_local_position_v1", qos_sub,
            [this](VehicleLocalPosition::SharedPtr msg) {
                pos_     = *msg;
                has_pos_ = pos_.xy_valid;
            });

        map_sub_ = create_subscription<OccupancyGrid>(
            "occupancy_map", qos_map,
            [this](OccupancyGrid::SharedPtr msg) { last_map_ = msg; });

        goal_pub_ = create_publisher<PointStamped>("frontier_goal", rclcpp::QoS(10));

        replan_timer_ = create_wall_timer(
            std::chrono::duration<double>(replan_period_s_),
            [this]() { replan(); });

        RCLCPP_INFO(get_logger(),
            "FrontierPlanner ready — replanning every %.1f s, "
            "cluster radius %.1f m",
            replan_period_s_, cluster_radius_m_);
    }

private:
    // ── parameters ───────────────────────────────────────────────────────────
    int   min_frontier_cells_;
    float cluster_radius_m_;
    float replan_period_s_;

    // ── state ─────────────────────────────────────────────────────────────────
    VehicleLocalPosition   pos_{};
    bool                   has_pos_  = false;
    OccupancyGrid::SharedPtr last_map_;

    // ── ROS ──────────────────────────────────────────────────────────────────
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr        map_sub_;
    rclcpp::Publisher<PointStamped>::SharedPtr            goal_pub_;
    rclcpp::TimerBase::SharedPtr                          replan_timer_;

    // =========================================================================
    // Grid helpers
    // =========================================================================

    int8_t get_cell(const OccupancyGrid &g, int row, int col) const
    {
        if (row < 0 || row >= static_cast<int>(g.info.height) ||
            col < 0 || col >= static_cast<int>(g.info.width))
            return -1;  // treat out-of-bounds as UNKNOWN
        return g.data[static_cast<size_t>(row) * g.info.width + col];
    }

    bool is_frontier(const OccupancyGrid &g, int row, int col) const
    {
        if (get_cell(g, row, col) != 0) return false;  // must be FREE
        // 4-connected: at least one UNKNOWN neighbour
        const int dr[4] = {0,  0, 1, -1};
        const int dc[4] = {1, -1, 0,  0};
        for (int d = 0; d < 4; ++d) {
            if (get_cell(g, row + dr[d], col + dc[d]) == -1) return true;
        }
        return false;
    }

    // Cell (row, col) → NED world position.
    // Mapping:  row = North index, col = East index
    //   North = origin.y + (row + 0.5) * resolution
    //   East  = origin.x + (col + 0.5) * resolution
    void cell_to_ned(const OccupancyGrid &g, int row, int col,
                     float &n_out, float &e_out) const
    {
        const float res = g.info.resolution;
        e_out = static_cast<float>(g.info.origin.position.x) + (col + 0.5f) * res;
        n_out = static_cast<float>(g.info.origin.position.y) + (row + 0.5f) * res;
    }

    // =========================================================================
    // Line-of-sight helper (Bresenham) — returns false if any OCCUPIED cell
    // lies on the straight line from (r0,c0) to (r1,c1) in grid g.
    // =========================================================================

    bool line_clear(const OccupancyGrid &g, int r0, int c0, int r1, int c1) const
    {
        int dr = std::abs(r1 - r0), dc = std::abs(c1 - c0);
        int sr = (r0 < r1) ? 1 : -1, sc = (c0 < c1) ? 1 : -1;
        int err = dr - dc, r = r0, c = c0;
        while (r != r1 || c != c1) {
            if (get_cell(g, r, c) == 100) return false;
            const int e2 = 2 * err;
            if (e2 > -dc) { err -= dc; r += sr; }
            if (e2 <  dr) { err += dr; c += sc; }
        }
        return true;
    }

    // =========================================================================
    // Planning
    // =========================================================================

    void replan()
    {
        if (!has_pos_ || !last_map_) {
            RCLCPP_DEBUG(get_logger(), "replan: waiting for position/map");
            return;
        }

        const OccupancyGrid &g = *last_map_;
        const float drone_n = pos_.x;
        const float drone_e = pos_.y;
        const float res     = g.info.resolution;

        // Drone grid cell (for LOS check origin)
        const int drone_row = static_cast<int>(
            (drone_n - static_cast<float>(g.info.origin.position.y)) / res);
        const int drone_col = static_cast<int>(
            (drone_e - static_cast<float>(g.info.origin.position.x)) / res);

        struct FCell { int row; int col; float n; float e; };
        std::vector<FCell> frontiers;
        frontiers.reserve(512);

        const int rows = static_cast<int>(g.info.height);
        const int cols = static_cast<int>(g.info.width);

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                if (!is_frontier(g, r, c)) continue;
                float fn, fe;
                cell_to_ned(g, r, c, fn, fe);
                frontiers.push_back({r, c, fn, fe});
            }
        }

        if (frontiers.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                "No frontiers — map may be fully explored");
            return;
        }

        // Sort all frontiers by distance to drone (ascending)
        std::sort(frontiers.begin(), frontiers.end(),
            [&](const FCell &a, const FCell &b) {
                const float da = (a.n-drone_n)*(a.n-drone_n) + (a.e-drone_e)*(a.e-drone_e);
                const float db = (b.n-drone_n)*(b.n-drone_n) + (b.e-drone_e)*(b.e-drone_e);
                return da < db;
            });

        // Try up to 8 candidate seeds (nearest first); pick the first whose
        // cluster centroid has a clear line-of-sight from the drone.
        const float cr2 = cluster_radius_m_ * cluster_radius_m_;
        constexpr int kMaxCandidates = 8;
        int candidates_tried = 0;

        for (int seed = 0; seed < static_cast<int>(frontiers.size()) &&
                            candidates_tried < kMaxCandidates; ++seed)
        {
            // Build cluster around this seed
            float sum_n = 0.0f, sum_e = 0.0f;
            int   count = 0;
            for (const auto &f : frontiers) {
                const float dn = f.n - frontiers[seed].n;
                const float de = f.e - frontiers[seed].e;
                if (dn*dn + de*de <= cr2) {
                    sum_n += f.n; sum_e += f.e; ++count;
                }
            }
            if (count < min_frontier_cells_) {
                // Allow isolated frontier to be used as a single-cell cluster
                sum_n = frontiers[seed].n;
                sum_e = frontiers[seed].e;
                count = 1;
            }

            // Centroid-snap: find nearest frontier cell to centroid
            const float cen_n = sum_n / static_cast<float>(count);
            const float cen_e = sum_e / static_cast<float>(count);
            float best_d2 = std::numeric_limits<float>::max();
            int   best_i  = seed;
            for (int i = 0; i < static_cast<int>(frontiers.size()); ++i) {
                const float dn = frontiers[i].n - cen_n;
                const float de = frontiers[i].e - cen_e;
                const float d2 = dn*dn + de*de;
                if (d2 < best_d2) { best_d2 = d2; best_i = i; }
            }

            const int   goal_row = frontiers[best_i].row;
            const int   goal_col = frontiers[best_i].col;
            const float goal_n   = frontiers[best_i].n;
            const float goal_e   = frontiers[best_i].e;

            ++candidates_tried;

            // Line-of-sight check: reject if the straight path crosses an obstacle
            if (!line_clear(g, drone_row, drone_col, goal_row, goal_col)) {
                RCLCPP_DEBUG(get_logger(),
                    "Frontier (%.2f,%.2f) LOS blocked — trying next candidate",
                    goal_n, goal_e);
                continue;
            }

            // This goal is reachable — publish it
            PointStamped goal;
            goal.header.stamp    = now();
            goal.header.frame_id = "map_ned";
            goal.point.x = static_cast<double>(goal_n);
            goal.point.y = static_cast<double>(goal_e);
            goal.point.z = 0.0;
            goal_pub_->publish(goal);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Frontier goal → N=%.2f E=%.2f  (cluster %d cells, "
                "total frontiers=%zu, drone N=%.2f E=%.2f, tried %d)",
                goal_n, goal_e, count, frontiers.size(),
                drone_n, drone_e, candidates_tried);
            return;
        }

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "No reachable frontier found after %d candidates "
            "(total=%zu) — waiting for more map data",
            candidates_tried, frontiers.size());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierPlanner>());
    rclcpp::shutdown();
    return 0;
}
