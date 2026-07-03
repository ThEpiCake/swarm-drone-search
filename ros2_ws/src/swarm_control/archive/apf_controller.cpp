#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using sensor_msgs::msg::LaserScan;
using geometry_msgs::msg::PointStamped;

// ── 3-Mode operational state machine ─────────────────────────────────────────
//
//  PRIMING / REQUESTING  — offboard stream priming + ARM handshake (unchanged
//                          from Stage B's proven M1 pattern).
//
//  MODE_0  (Takeoff & Scan)
//    Climb to target_alt, then hover in place for scan_duration_s.
//    NO horizontal APF forces are applied.  Purpose: let PX4's EKF stabilise
//    after the climb transient, and give the 4 range sensors + RGB-D camera
//    time to observe the immediate surroundings before navigation begins.
//
//  MODE_1  (APF Cruise)
//    Full APF loop with radial saturation (thesis eq. 4.7–4.8).
//    Attraction toward goal (static or entropy centroid) + repulsion from the
//    4 horizontal range sensors.  The drone naturally decelerates as forces
//    cancel near the equilibrium point.
//
//  MODE_2  (RTL / Land)
//    Triggered by cruise timeout, goal proximity, or an explicit entropy
//    threshold (future).  Commands zero horizontal offset, issues NAV_LAND,
//    and disarms on touchdown.
//
enum class State {
    PRIMING,
    REQUESTING,
    MODE_0,   // takeoff + hover-scan
    MODE_1,   // APF cruise (radial saturation)
    MODE_2,   // RTL / land
    DONE,
};

static constexpr float kNaN          = std::numeric_limits<float>::quiet_NaN();
static constexpr float kLoopPeriod_s = 0.05f;  // 50ms timer → 20 Hz
// x500_swarm spawns with Gazebo ENU yaw=0 (faces East). In PX4 NED frame, yaw=0
// is North and yaw=π/2 is East. Holding this yaw keeps sensor axes consistent:
// FRONT=East, BACK=West, LEFT=North, RIGHT=South throughout all flight phases.
static constexpr float kYawEast_ = static_cast<float>(M_PI_2);

class ApfController : public rclcpp::Node
{
public:
    ApfController() : Node("apf_controller")
    {
        // ── Parameters ────────────────────────────────────────────────────────
        target_alt_     = declare_parameter<float>("target_altitude_m",    3.0f);
        goal_north_m_   = declare_parameter<float>("goal_north_m",        30.0f);
        goal_east_m_    = declare_parameter<float>("goal_east_m",          0.0f);
        rep_threshold_  = declare_parameter<float>("repulsion_threshold_m", 2.0f);
        k_att_          = declare_parameter<float>("attraction_gain",       0.2f);
        k_rep_          = declare_parameter<float>("repulsion_gain",        2.0f);
        // Radial saturation bound (thesis eq. 4.7–4.8): F_total used raw when
        // |F_total| < v_max_, capped otherwise.  Drone naturally decelerates as
        // attraction and repulsion cancel near equilibrium.
        v_max_          = declare_parameter<float>("v_max_m",               0.5f);
        // How long to hover in MODE_0 before engaging APF (wall-clock seconds).
        scan_s_         = declare_parameter<float>("scan_duration_s",      10.0f);
        cruise_s_       = declare_parameter<float>("cruise_duration_s",    30.0f);
        pos_tol_        = declare_parameter<float>("position_tol_m",        0.4f);
        arm_timeout_s_  = declare_parameter<float>("arm_timeout_s",        10.0f);

        // ── QoS ───────────────────────────────────────────────────────────────
        auto qos_pub = rclcpp::QoS(10);
        auto qos_sub = rclcpp::QoS(1).best_effort();

        // ── Publishers ────────────────────────────────────────────────────────
        ocm_pub_ = create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", qos_pub);
        sp_pub_  = create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint",   qos_pub);
        cmd_pub_ = create_publisher<VehicleCommand>("fmu/in/vehicle_command",           qos_pub);

        // ── Subscribers ───────────────────────────────────────────────────────
        status_sub_ = create_subscription<VehicleStatus>(
            "fmu/out/vehicle_status_v4", qos_sub,
            [this](VehicleStatus::SharedPtr msg) { status_ = *msg; });

        pos_sub_ = create_subscription<VehicleLocalPosition>(
            "fmu/out/vehicle_local_position_v1", qos_sub,
            [this](VehicleLocalPosition::SharedPtr msg) { pos_ = *msg; });

        scan_sub_ = create_subscription<LaserScan>(
            "scan", qos_sub,
            [this](LaserScan::SharedPtr msg) { last_scan_ = msg; });

        // Stage D: subscribe to entropy centroid from voxel_mapper.
        // Falls back to static goal parameters when no centroid has arrived.
        entropy_sub_ = create_subscription<PointStamped>(
            "entropy_centroid", rclcpp::QoS(10).best_effort(),
            [this](PointStamped::SharedPtr msg) {
                entropy_goal_x_    = static_cast<float>(msg->point.x);
                entropy_goal_y_    = static_cast<float>(msg->point.y);
                have_entropy_goal_ = true;
            });

        timer_ = create_wall_timer(50ms, [this]() { loop(); });

        RCLCPP_INFO(get_logger(),
            "ApfController ready — alt=%.1f m  goal=(%.1f,%.1f)  threshold=%.1f m  "
            "v_max=%.2f m  scan=%.0f s  cruise=%.0f s  [RPLIDAR 360° repulsion]",
            target_alt_, goal_north_m_, goal_east_m_, rep_threshold_,
            v_max_, scan_s_, cruise_s_);
    }

private:
    // ── State ─────────────────────────────────────────────────────────────────
    State    state_       = State::PRIMING;
    uint32_t tick_        = 0;
    uint32_t state_ticks_ = 0;
    float    scan_hold_x_ = 0.0f, scan_hold_y_ = 0.0f;  // hover point for MODE_0
    float    closest_front_seen_ = std::numeric_limits<float>::infinity();

    // ── Cached telemetry ──────────────────────────────────────────────────────
    VehicleStatus        status_{};
    VehicleLocalPosition pos_{};
    LaserScan::SharedPtr last_scan_;

    // ── Parameters ────────────────────────────────────────────────────────────
    float target_alt_, goal_north_m_, goal_east_m_, rep_threshold_;
    float k_att_, k_rep_, v_max_, scan_s_, cruise_s_, pos_tol_, arm_timeout_s_;

    // ── Entropy goal (Stage D) ─────────────────────────────────────────────────
    float entropy_goal_x_ = 0.0f, entropy_goal_y_ = 0.0f;
    bool  have_entropy_goal_ = false;

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::Publisher<OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  sp_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<LaserScan>::SharedPtr            scan_sub_;
    rclcpp::Subscription<PointStamped>::SharedPtr         entropy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Helpers ───────────────────────────────────────────────────────────────
    uint64_t timestamp_us() const
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void publish_offboard_mode()
    {
        OffboardControlMode ocm{};
        ocm.timestamp = timestamp_us();
        ocm.position  = true;
        ocm.velocity  = true;  // enables mixed position-z + velocity-xy in MODE_1
        ocm_pub_->publish(ocm);
    }

    void publish_setpoint(float x, float y, float z)
    {
        TrajectorySetpoint sp{};
        sp.timestamp    = timestamp_us();
        sp.position     = {x, y, z};
        sp.velocity     = {kNaN, kNaN, kNaN};
        sp.acceleration = {kNaN, kNaN, kNaN};
        sp.yaw          = kYawEast_;  // keep East-facing so sensor axes stay valid
        sp_pub_->publish(sp);
    }

    // MODE_1 APF: horizontal via velocity (bypasses EKF position drift),
    // altitude via position control (PX4 height controller owns z).
    void publish_apf_setpoint(float vx, float vy, float z)
    {
        TrajectorySetpoint sp{};
        sp.timestamp    = timestamp_us();
        sp.position     = {kNaN, kNaN, z};
        sp.velocity     = {vx, vy, kNaN};
        sp.acceleration = {kNaN, kNaN, kNaN};
        sp.yaw          = kYawEast_;  // maintain East-facing throughout flight
        sp_pub_->publish(sp);
    }

    void send_vehicle_command(uint16_t cmd, float p1 = 0, float p2 = 0,
                              float p3 = 0, float p4 = 0,
                              float p5 = 0, float p6 = 0, float p7 = 0)
    {
        VehicleCommand vc{};
        vc.timestamp        = timestamp_us();
        vc.command          = cmd;
        vc.param1           = p1;  vc.param2 = p2;  vc.param3 = p3;
        vc.param4           = p4;  vc.param5 = p5;  vc.param6 = p6;  vc.param7 = p7;
        vc.target_system    = 0;
        vc.target_component = 1;
        vc.source_system    = 1;
        vc.source_component = 1;
        vc.from_external    = true;
        cmd_pub_->publish(vc);
    }

    void set_offboard_mode() { send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f); }
    void arm()               { send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 21196.0f); }
    void land()              { send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND); }

    bool at_position(float x, float y, float z) const
    {
        if (!pos_.xy_valid || !pos_.z_valid) return false;
        float dx = pos_.x - x, dy = pos_.y - y, dz = pos_.z - z;
        return std::sqrt(dx*dx + dy*dy + dz*dz) < pos_tol_;
    }

    bool is_armed()    const { return status_.arming_state == VehicleStatus::ARMING_STATE_ARMED; }
    bool is_offboard() const { return status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD; }

    void transition(State next) { state_ticks_ = 0; state_ = next; }

    // Linear repulsion falloff: 0 at/above threshold, k_rep_ at distance ~0.
    float repulsion_mag(float d) const
    {
        if (!std::isfinite(d) || d >= rep_threshold_) return 0.0f;
        return k_rep_ * (rep_threshold_ - std::max(d, 0.05f)) / rep_threshold_;
    }

    // ── Main 20 Hz loop ───────────────────────────────────────────────────────
    void loop()
    {
        ++tick_;
        ++state_ticks_;
        publish_offboard_mode();

        const float z_target = -target_alt_;
        const float home_x = 0.0f, home_y = 0.0f;

        switch (state_) {

        // ── Offboard stream priming ───────────────────────────────────────────
        case State::PRIMING:
            publish_setpoint(home_x, home_y, 0.0f);
            if (state_ticks_ >= static_cast<uint32_t>(1.0f / kLoopPeriod_s)) { // 1 second at 20 Hz
                RCLCPP_INFO(get_logger(), "Stream primed — requesting OFFBOARD + ARM");
                set_offboard_mode();
                arm();
                transition(State::REQUESTING);
            }
            break;

        case State::REQUESTING:
            publish_setpoint(home_x, home_y, 0.0f);
            if (state_ticks_ % static_cast<uint32_t>(0.5f / kLoopPeriod_s) == 0) { set_offboard_mode(); arm(); } // Retry every 0.5 seconds
            if (is_offboard() && is_armed()) {
                RCLCPP_INFO(get_logger(), "Armed + OFFBOARD — climbing to %.1f m (MODE_0)", target_alt_);
                transition(State::MODE_0);
            }
            if (state_ticks_ > static_cast<uint32_t>(arm_timeout_s_ / kLoopPeriod_s)) {
                RCLCPP_ERROR(get_logger(), "Timeout waiting for ARM/OFFBOARD — aborting");
                transition(State::DONE);
            }
            break;

        // ── MODE 0: Takeoff & Scan ────────────────────────────────────────────
        // Climb to target altitude, then hold position for scan_s_ seconds.
        // No horizontal APF forces.  Lets EKF stabilise and sensors observe
        // the immediate surroundings before navigation begins.
        case State::MODE_0: {
            // Phase A: climb to altitude
            if (!at_position(home_x, home_y, z_target)) {
                publish_setpoint(home_x, home_y, z_target);
                if (state_ticks_ % static_cast<uint32_t>(1.0f / kLoopPeriod_s) == 0) { // Log every 1 second
                    RCLCPP_INFO(get_logger(), "MODE_0 climb: z=%.2f m (target %.1f m)",
                                -pos_.z, target_alt_);
                }
                break;
            }

            // Phase B: altitude reached — record hold point once
            if (scan_hold_x_ == 0.0f && scan_hold_y_ == 0.0f && pos_.xy_valid) {
                scan_hold_x_ = pos_.x;
                scan_hold_y_ = pos_.y;
                RCLCPP_INFO(get_logger(),
                    "MODE_0 altitude reached (z=%.2f m NED) — holding (%.2f,%.2f) for %.0f s scan",
                    pos_.z, scan_hold_x_, scan_hold_y_, scan_s_);
                closest_front_seen_ = std::numeric_limits<float>::infinity();
            }

            publish_setpoint(scan_hold_x_, scan_hold_y_, z_target);

            // Scan countdown status at ~1 Hz (every 1 second)
            if (state_ticks_ % static_cast<uint32_t>(1.0f / kLoopPeriod_s) == 0) {
                const int scan_rays = last_scan_ ? static_cast<int>(last_scan_->ranges.size()) : 0;
                RCLCPP_INFO(get_logger(),
                    "MODE_0 scan (%.0f/%.0f s): RPLIDAR rays=%d  entropy_goal=%s",
                    static_cast<float>(state_ticks_) / 20.0f, scan_s_,
                    scan_rays,
                    have_entropy_goal_ ? "YES" : "waiting...");
            }

            // Transition to MODE_1 after scan window
            // state_ticks_ counts from Mode 0 entry; altitude reach adds some ticks,
            // but scan_s_ is generous so we simply wait scan_s_ total ticks in this state.
            if (state_ticks_ >= static_cast<uint32_t>(scan_s_ / kLoopPeriod_s)) {
                RCLCPP_INFO(get_logger(),
                    "MODE_0 scan complete (%.0f s) — engaging APF cruise (MODE_1) "
                    "toward goal (%.1f,%.1f)  k_att=%.2f  k_rep=%.2f  v_max=%.2f m",
                    scan_s_, goal_north_m_, goal_east_m_, k_att_, k_rep_, v_max_);
                transition(State::MODE_1);
            }
            break;
        }

        // ── MODE 1: APF Cruise (Radial Saturation) ────────────────────────────
        case State::MODE_1: {
            // Dynamic goal: use entropy centroid if available, fall back to static goal.
            float gx = have_entropy_goal_ ? entropy_goal_x_ : goal_north_m_;
            float gy = have_entropy_goal_ ? entropy_goal_y_ : goal_east_m_;

            // Attraction: unit vector toward goal, scaled by k_att_.
            float ax = gx - pos_.x, ay = gy - pos_.y;
            float a_norm = std::hypot(ax, ay); // Use std::hypot for robustness
            if (a_norm > 1e-3f) { ax /= a_norm; ay /= a_norm; }

            // Repulsion: 360° RPLIDAR scan — angular-weighted sum (thesis §4.2.2).
            // world_angle_NED = heading - body_angle  (lidar_mapper convention)
            // Each ray within rep_threshold_ contributes a push away from the obstacle,
            // weighted by its angular width (da) so total force is resolution-independent.
            float rx = 0.0f, ry = 0.0f;
            float closest_scan_ = std::numeric_limits<float>::infinity();
            if (last_scan_) {
                const float hdg = std::isfinite(pos_.heading) ? pos_.heading : 0.0f;
                const float da  = last_scan_->angle_increment;
                for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
                    const float r = last_scan_->ranges[i]; // Current range reading
                    if (!std::isfinite(r) || r < last_scan_->range_min || r >= rep_threshold_)
                        continue;
                    closest_scan_ = std::min(closest_scan_, r);
                    const float body_a  = last_scan_->angle_min + static_cast<float>(i) * da;
                    const float obs_ned = hdg - body_a;   // NED world angle toward obstacle
                    const float mag     = repulsion_mag(r) * da;
                    rx -= std::cos(obs_ned) * mag;        // push AWAY from obstacle
                    ry -= std::sin(obs_ned) * mag;
                }
            }

            // Radial saturation (thesis eq. 4.7–4.8).
            float fx = k_att_ * ax + rx;
            float fy = k_att_ * ay + ry; // Total force components
            float f_mag = std::hypot(fx, fy); // Use std::hypot for robustness
            if (f_mag > v_max_) { fx = fx / f_mag * v_max_; fy = fy / f_mag * v_max_; }

            publish_apf_setpoint(fx, fy, z_target); // Ensure OCM is published in this state

            closest_front_seen_ = std::min(closest_front_seen_, closest_scan_);

            if (state_ticks_ % 20 == 0) {
                RCLCPP_INFO(get_logger(),
                    "MODE_1 APF: pos=(%.2f,%.2f)  goal=(%.2f,%.2f)%s  "
                    "closest=%.2f m  F=(%.3f,%.3f)  |F|=%.3f",
                    pos_.x, pos_.y, gx, gy,
                    have_entropy_goal_ ? "[ent]" : "[sta]",
                    closest_scan_, fx, fy, f_mag);
            }

            if (state_ticks_ >= static_cast<uint32_t>(cruise_s_ / kLoopPeriod_s)) {
                RCLCPP_INFO(get_logger(),
                    "MODE_1 cruise complete (%.0f s) — closest obstacle %.2f m "
                    "(threshold=%.1f m  v_max=%.2f m) — transitioning to MODE_2 (RTL)",
                    cruise_s_, closest_front_seen_, rep_threshold_, v_max_);
                transition(State::MODE_2);
            }
            break;
        }

        // ── MODE 2: RTL / Land ────────────────────────────────────────────────
        // Zero horizontal offset command, then NAV_LAND, then explicit disarm.
        case State::MODE_2:
            publish_setpoint(home_x, home_y, 0.0f);
            if (state_ticks_ == 1) {
                land();
            }
            if (pos_.z_valid && pos_.z > -0.3f) {
                RCLCPP_INFO(get_logger(), "MODE_2 ground reached (z=%.2f) — disarming", pos_.z);
                send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                transition(State::DONE); // Transition to DONE after disarming
            }
            if (!is_armed()) {
                RCLCPP_INFO(get_logger(), "MODE_2 landed and disarmed — mission complete");
                transition(State::DONE);
            }
            if (state_ticks_ > static_cast<uint32_t>(20.0f / kLoopPeriod_s)) { // 20 second landing timeout
                RCLCPP_WARN(get_logger(), "MODE_2 landing timeout — force disarming");
                send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                transition(State::DONE);
            }
            break;

        case State::DONE:
            if (state_ticks_ == 1) {
                RCLCPP_INFO(get_logger(), "APF run complete. Node idle.");
            }
            break;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApfController>());
    rclcpp::shutdown();
    return 0;
}
