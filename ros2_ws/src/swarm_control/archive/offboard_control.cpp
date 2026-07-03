#include <algorithm>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// ── State machine ─────────────────────────────────────────────────────────────
enum class State {
    PRIMING,      // publish setpoints for ≥1 s so PX4 accepts offboard mode
    REQUESTING,   // send OFFBOARD + ARM commands, wait for confirmation
    TAKEOFF,      // climb to target altitude (NED z negative = up)
    HOVER,        // hold position at target altitude for hover_duration_s_
    WAYPOINT,     // fly to target XY
    LANDING,      // issue NAV_LAND, wait for disarm
    DONE,
};

static constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // ── Parameters ────────────────────────────────────────────────────────
        target_alt_  = declare_parameter<float>("target_altitude_m", 3.0f);  // metres above ground
        wp_north_    = declare_parameter<float>("waypoint_north_m",  5.0f);  // NED x
        wp_east_     = declare_parameter<float>("waypoint_east_m",   0.0f);  // NED y
        pos_tol_     = declare_parameter<float>("position_tol_m",    0.4f);  // arrival radius
        hover_s_     = declare_parameter<float>("hover_duration_s", 15.0f);  // stable-hover window at target altitude

        // ── QoS ───────────────────────────────────────────────────────────────
        auto qos_pub = rclcpp::QoS(10);
        // PX4 publishes with BEST_EFFORT — match it on subscribers
        auto qos_sub = rclcpp::QoS(1).best_effort();

        // ── Publishers ────────────────────────────────────────────────────────
        // Relative names so ROS2 node namespace maps to the right px4_N/fmu/... topics
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

        // ── 20 Hz control loop ────────────────────────────────────────────────
        timer_ = create_wall_timer(50ms, [this]() { loop(); });

        RCLCPP_INFO(get_logger(),
            "OffboardControl ready — target alt=%.1f m, waypoint=(%.1f, %.1f)",
            target_alt_, wp_north_, wp_east_);
    }

private:
    // ── State ────────────────────────────────────────────────────────────────
    State state_          = State::PRIMING;
    uint32_t tick_        = 0;   // loop counter
    uint32_t state_ticks_ = 0;   // ticks spent in current state
    float hover_drift_max_ = 0.0f;  // peak lateral drift observed during HOVER, metres

    // ── Cached telemetry ─────────────────────────────────────────────────────
    VehicleStatus        status_{};
    VehicleLocalPosition pos_{};

    // ── Parameters ───────────────────────────────────────────────────────────
    float target_alt_, wp_north_, wp_east_, pos_tol_, hover_s_;

    // ── ROS handles ──────────────────────────────────────────────────────────
    rclcpp::Publisher<OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  sp_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr        status_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── Helpers ──────────────────────────────────────────────────────────────
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
        ocm_pub_->publish(ocm);
    }

    // Publish a position setpoint (NED). Unused axes → NaN.
    void publish_setpoint(float x, float y, float z, float yaw = 0.0f)
    {
        TrajectorySetpoint sp{};
        sp.timestamp    = timestamp_us();
        sp.position     = {x, y, z};
        sp.velocity     = {kNaN, kNaN, kNaN};
        sp.acceleration = {kNaN, kNaN, kNaN};
        sp.yaw          = yaw;
        sp_pub_->publish(sp);
    }

    void send_vehicle_command(uint16_t cmd, float p1 = 0, float p2 = 0,
                              float p3 = 0, float p4 = 0,
                              float p5 = 0, float p6 = 0, float p7 = 0)
    {
        VehicleCommand vc{};
        vc.timestamp        = timestamp_us();
        vc.command          = cmd;
        vc.param1           = p1;
        vc.param2           = p2;
        vc.param3           = p3;
        vc.param4           = p4;
        vc.param5           = p5;
        vc.param6           = p6;
        vc.param7           = p7;
        vc.target_system    = 0;  // 0 = broadcast; works for any MAV_SYS_ID (multi-instance safe)
        vc.target_component = 1;
        vc.source_system    = 1;
        vc.source_component = 1;
        vc.from_external    = true;
        cmd_pub_->publish(vc);
    }

    void set_offboard_mode()
    {
        // MAV_CMD_DO_SET_MODE (176): p1=custom-mode flag (1), p2=PX4 offboard mode (6)
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    }

    void arm()
    {
        // MAV_CMD_COMPONENT_ARM_DISARM (400): p1=1 arm, p2=21196 force-arm (bypasses preflight in sim)
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 21196.0f);
    }

    void land()
    {
        // MAV_CMD_NAV_LAND (21)
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    }

    bool at_position(float x, float y, float z) const
    {
        if (!pos_.xy_valid || !pos_.z_valid) return false;
        float dx = pos_.x - x;
        float dy = pos_.y - y;
        float dz = pos_.z - z;
        return std::sqrt(dx*dx + dy*dy + dz*dz) < pos_tol_;
    }

    bool is_armed() const
    {
        return status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    }

    bool is_offboard() const
    {
        return status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }

    void transition(State next)
    {
        state_ticks_ = 0;
        state_ = next;
    }

    // ── Main 20 Hz loop ──────────────────────────────────────────────────────
    void loop()
    {
        ++tick_;
        ++state_ticks_;

        // PX4 requires continuous offboard_control_mode heartbeat while in offboard
        publish_offboard_mode();

        // NED: z = -(altitude above ground)
        const float z_target = -target_alt_;
        // Takeoff position = current XY (hover in place), target alt
        // We use 0,0 as spawn origin
        const float home_x = 0.0f, home_y = 0.0f;

        switch (state_) {

        case State::PRIMING:
            // Stream a stationary setpoint at ground level while we prime the DDS stream.
            // PX4 needs ≥0.5 s of continuous setpoints before accepting the mode switch.
            publish_setpoint(home_x, home_y, 0.0f);
            if (state_ticks_ >= 20) {  // 1 s at 20 Hz
                RCLCPP_INFO(get_logger(), "Stream primed — requesting OFFBOARD + ARM");
                set_offboard_mode();
                arm();
                transition(State::REQUESTING);
            }
            break;

        case State::REQUESTING:
            // Retry commands at 2 Hz until both confirmed
            publish_setpoint(home_x, home_y, 0.0f);
            if (state_ticks_ % 10 == 0) {
                set_offboard_mode();
                arm();
            }
            if (is_offboard() && is_armed()) {
                RCLCPP_INFO(get_logger(), "Armed + OFFBOARD confirmed — climbing to %.1f m", target_alt_);
                transition(State::TAKEOFF);
            }
            if (state_ticks_ > 200) {  // 10 s timeout
                RCLCPP_ERROR(get_logger(), "Timeout waiting for ARM/OFFBOARD — aborting");
                transition(State::DONE);
            }
            break;

        case State::TAKEOFF:
            publish_setpoint(home_x, home_y, z_target);
            if (at_position(home_x, home_y, z_target)) {
                RCLCPP_INFO(get_logger(),
                    "Altitude reached (z=%.2f m NED) — holding hover for %.0f s",
                    pos_.z, hover_s_);
                transition(State::HOVER);
            }
            break;

        case State::HOVER:
            // Hold position at target altitude — this is the "stable hover" window
            // Stage A's checklist measures (duration + lateral drift).
            publish_setpoint(home_x, home_y, z_target);
            if (state_ticks_ == 1) {
                hover_drift_max_ = 0.0f;
            } else if (pos_.xy_valid) {
                float dx = pos_.x - home_x;
                float dy = pos_.y - home_y;
                hover_drift_max_ = std::max(hover_drift_max_, std::sqrt(dx*dx + dy*dy));
            }
            if (state_ticks_ >= static_cast<uint32_t>(hover_s_ * 20.0f)) {  // hover_s_ seconds at 20 Hz
                RCLCPP_INFO(get_logger(),
                    "Hover complete — held %.0f s, max lateral drift=%.2f m — flying to waypoint (%.1f, %.1f)",
                    hover_s_, hover_drift_max_, wp_north_, wp_east_);
                transition(State::WAYPOINT);
            }
            break;

        case State::WAYPOINT:
            publish_setpoint(wp_north_, wp_east_, z_target);
            if (at_position(wp_north_, wp_east_, z_target)) {
                RCLCPP_INFO(get_logger(), "Waypoint reached — landing");
                land();
                transition(State::LANDING);
            }
            break;

        case State::LANDING:
            // Descend via position setpoint to ground (NED z=0), then disarm explicitly.
            // NAV_LAND doesn't cut motors while offboard setpoints are active, so we
            // command ground level ourselves and disarm once the drone is within 30 cm.
            publish_setpoint(wp_north_, wp_east_, 0.0f);
            if (pos_.z_valid && pos_.z > -0.3f) {  // within 30 cm of ground in NED
                RCLCPP_INFO(get_logger(), "Ground reached (z=%.2f) — disarming", pos_.z);
                send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                transition(State::DONE);
            }
            if (!is_armed()) {
                RCLCPP_INFO(get_logger(), "Landed and disarmed — M1 complete");
                transition(State::DONE);
            }
            if (state_ticks_ > 400) {  // 20 s landing timeout
                RCLCPP_WARN(get_logger(), "Landing timeout — disarming");
                send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
                transition(State::DONE);
            }
            break;

        case State::DONE:
            // Stop publishing — let PX4 handle itself
            if (state_ticks_ == 1) {
                RCLCPP_INFO(get_logger(), "Mission complete. Node idle.");
            }
            break;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
