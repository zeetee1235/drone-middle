#include "px4_offboard_control/control_core.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sprint_drone_msgs/msg/marker_list.hpp"
#include "sprint_drone_msgs/msg/velocity_profile.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// PX4 MAVLink command IDs (from px4_msgs VehicleCommand constants)
static constexpr uint32_t CMD_DO_SET_MODE = 176;
static constexpr uint32_t CMD_ARM_DISARM = 400;
// custom_mode = 6 → PX4_CUSTOM_MAIN_MODE_OFFBOARD
static constexpr float PX4_CUSTOM_MODE_OFFBOARD = 6.0f;

// Radius at which target node is considered reached
static constexpr double TARGET_REACHED_RADIUS_M = 1.0;
// Altitude below which landing is considered complete (NED: -z)
static constexpr double LANDED_ALTITUDE_M = 0.3;
// PX4 requires a steady offboard setpoint stream before accepting Offboard mode.
static constexpr int ARM_SEQ_OFFBOARD_START_TICK = 25;  // 1.25 s at 20 Hz
static constexpr int ARM_SEQ_RETRY_PERIOD_TICKS = 10;   // 0.50 s at 20 Hz

class Px4OffboardControlNode final : public rclcpp::Node {
public:
    Px4OffboardControlNode()
        : Node("px4_offboard_control_node")
    {
        declareParams();
        params_ = loadParams();

        sub_mission_state_ = create_subscription<std_msgs::msg::String>(
            "/mission/state", rclcpp::QoS(10),
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
                onMissionState(msg->data);
            });

        sub_velocity_profile_ = create_subscription<sprint_drone_msgs::msg::VelocityProfile>(
            "/planner/velocity_profile", rclcpp::QoS(10),
            [this](sprint_drone_msgs::msg::VelocityProfile::ConstSharedPtr msg) {
                updatePlannerTarget(*msg);
            });

        sub_candidates_ = create_subscription<sprint_drone_msgs::msg::MarkerList>(
            "/markers/candidates", rclcpp::QoS(10),
            [this](sprint_drone_msgs::msg::MarkerList::ConstSharedPtr msg) {
                marker_candidate_ = !msg->markers.empty();
            });

        sub_vertiport_error_ = create_subscription<geometry_msgs::msg::Vector3>(
            "/vertiport/center_error", rclcpp::QoS(10),
            [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
                vertiport_error_ = {msg->x, msg->y, msg->z};
                vertiport_error_valid_ = true;
            });

        const auto px4_out_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4_out_qos,
            [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) {
                onAttitude(*msg);
            });

        sub_battery_ = create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status", px4_out_qos,
            [this](px4_msgs::msg::BatteryStatus::ConstSharedPtr msg) {
                battery_pct_ = msg->remaining * 100.0f;
            });

        sub_local_pos_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4_out_qos,
            [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
                altitude_m_ = static_cast<double>(-msg->z);  // NED: z negative = up
            });

        sub_vehicle_control_mode_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", px4_out_qos,
            [this](px4_msgs::msg::VehicleControlMode::ConstSharedPtr msg) {
                px4_offboard_enabled_ = msg->flag_control_offboard_enabled;
                px4_armed_ = msg->flag_armed;
            });

        sub_pose_grid_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/localization/pose_grid", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                pose_grid_ = {
                    msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z,
                };
            });

        sub_target_node_ = create_subscription<geometry_msgs::msg::Point>(
            "/mission/target_node", rclcpp::QoS(10),
            [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
                target_node_ = {msg->x, msg->y, msg->z};
                has_target_ = true;
            });

        pub_offboard_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", rclcpp::QoS(10));
        pub_trajectory_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", rclcpp::QoS(10));
        pub_vehicle_cmd_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::QoS(10));
        pub_mode_ = create_publisher<std_msgs::msg::String>(
            "/controller/mode", rclcpp::QoS(10));
        pub_sway_ = create_publisher<std_msgs::msg::Float32>(
            "/sway/metric", rclcpp::QoS(10));
        pub_battery_ = create_publisher<std_msgs::msg::Float32>(
            "/battery/percent", rclcpp::QoS(10));
        pub_target_reached_ = create_publisher<std_msgs::msg::Bool>(
            "/mission/target_reached", rclcpp::QoS(10));
        pub_landing_complete_ = create_publisher<std_msgs::msg::Bool>(
            "/landing/complete", rclcpp::QoS(10));
        pub_takeoff_complete_ = create_publisher<std_msgs::msg::Bool>(
            "/mission/takeoff_complete", rclcpp::QoS(10));

        timer_ = create_wall_timer(50ms, [this]() { controlLoop(); });
    }

private:
    // ── parameter helpers ─────────────────────────────────────────────────

    void declareParams() {
        declare_parameter("target_altitude_m", 2.0);
        declare_parameter("sway_threshold", 0.3);
        declare_parameter("vision_servo_gain_xy", 0.003);
        declare_parameter("vision_servo_max_xy", 0.3);
        declareModeParams("sprint",       {3.0, 0.3, 2.0, 5.0, 25.0});
        declareModeParams("approach",     {1.0, 0.25, 1.0, 2.0, 15.0});
        declareModeParams("anti_sway",    {0.5, 0.2, 0.5, 1.0, 10.0});
        declareModeParams("hover_confirm",{0.1, 0.15, 0.2, 0.5, 5.0});
        declareModeParams("return",       {2.0, 0.3, 1.5, 3.0, 20.0});
        declareModeParams("vision_servo", {0.3, 0.2, 0.3, 1.0, 8.0});
        declareModeParams("landing",      {0.1, 0.2, 0.2, 0.5, 5.0});
        declareModeParams("hold",         {0.1, 0.1, 0.2, 0.5, 5.0});
        declareModeParams("takeoff",      {0.3, 0.4, 0.5, 1.0, 10.0});
    }

    void declareModeParams(const std::string& name, const px4_offboard_control::ModeLimits& d) {
        declare_parameter(name + ".v_max_xy",   d.v_max_xy);
        declare_parameter(name + ".v_max_z",    d.v_max_z);
        declare_parameter(name + ".a_max_xy",   d.a_max_xy);
        declare_parameter(name + ".jerk_max",   d.jerk_max);
        declare_parameter(name + ".tilt_max_deg", d.tilt_max_deg);
    }

    px4_offboard_control::ModeLimits loadMode(const std::string& name) const {
        return {
            get_parameter(name + ".v_max_xy").as_double(),
            get_parameter(name + ".v_max_z").as_double(),
            get_parameter(name + ".a_max_xy").as_double(),
            get_parameter(name + ".jerk_max").as_double(),
            get_parameter(name + ".tilt_max_deg").as_double(),
        };
    }

    px4_offboard_control::ControllerParams loadParams() const {
        auto p = px4_offboard_control::defaultParams();
        p.target_altitude_m       = get_parameter("target_altitude_m").as_double();
        p.sway_threshold          = get_parameter("sway_threshold").as_double();
        p.vision_servo_gain_xy    = get_parameter("vision_servo_gain_xy").as_double();
        p.vision_servo_max_xy     = get_parameter("vision_servo_max_xy").as_double();
        p.limits[px4_offboard_control::ControlMode::Takeoff]      = loadMode("takeoff");
        p.limits[px4_offboard_control::ControlMode::Sprint]        = loadMode("sprint");
        p.limits[px4_offboard_control::ControlMode::Approach]      = loadMode("approach");
        p.limits[px4_offboard_control::ControlMode::AntiSway]      = loadMode("anti_sway");
        p.limits[px4_offboard_control::ControlMode::HoverConfirm]  = loadMode("hover_confirm");
        p.limits[px4_offboard_control::ControlMode::Return]        = loadMode("return");
        p.limits[px4_offboard_control::ControlMode::VisionServo]   = loadMode("vision_servo");
        p.limits[px4_offboard_control::ControlMode::Landing]       = loadMode("landing");
        p.limits[px4_offboard_control::ControlMode::Hold]          = loadMode("hold");
        return p;
    }

    // ── callbacks ─────────────────────────────────────────────────────────

    void onMissionState(const std::string& state) {
        if (state == mission_state_) return;
        if (isActiveMissionState(state) && !arm_done_ && !arm_seq_active_) {
            arm_seq_tick_ = 0;
            arm_seq_active_ = true;
        }
        mission_state_ = state;
    }

    bool isActiveMissionState(const std::string& state) const {
        return state != "INIT" && state != "LANDED" && state != "ABORT";
    }

    void onAttitude(const px4_msgs::msg::VehicleAttitude& att) {
        // q = [w, x, y, z]
        const double w = att.q[0], x = att.q[1], y = att.q[2], z = att.q[3];
        const double roll  = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
        sway_metric_ = std::sqrt(roll * roll + pitch * pitch);
    }

    void updatePlannerTarget(const sprint_drone_msgs::msg::VelocityProfile& msg) {
        if (msg.waypoints.empty() || msg.target_speeds_mps.empty()) return;
        planner_target_ = {
            msg.waypoints[0].x,
            msg.waypoints[0].y,
            msg.waypoints[0].z,
        };
        planner_speed_ = msg.target_speeds_mps[0];
        has_planner_target_ = true;
    }

    // ── control loop ──────────────────────────────────────────────────────

    void controlLoop() {
        stepArmSequence();

        px4_offboard_control::ControllerInput input;
        input.mission_state             = mission_state_;
        input.planner_velocity_mps      = computeDirectedVelocity();
        input.vertiport_center_error_px = vertiport_error_;
        input.current_altitude_m        = altitude_m_;
        input.sway_metric               = sway_metric_;
        input.marker_candidate          = marker_candidate_;
        input.vertiport_error_valid     = vertiport_error_valid_;
        input.dt_sec                    = 0.05;

        const auto output = px4_offboard_control::stepController(input, state_, params_);

        publishOffboardControlMode();
        publishTrajectorySetpoint(output);
        publishControllerMode(output.mode);

        publishSway();
        publishBattery();
        checkTargetReached();
        checkLandingComplete(output.mode);
        publishTakeoffComplete();
    }

    // Direction from current grid pose toward planner waypoint, scaled to target speed.
    // Grid frame: x = east, y = north (negative = south)
    // NED frame:  x = north, y = east
    px4_offboard_control::Vec3 computeDirectedVelocity() const {
        if (!has_planner_target_) return {};
        const double dx = planner_target_.x - pose_grid_.x;  // east  in grid
        const double dy = planner_target_.y - pose_grid_.y;  // north in grid
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < 1e-3) return {};
        const double s = static_cast<double>(planner_speed_) / dist;
        return {dy * s, dx * s, 0.0};  // NED: vx=north=grid_y, vy=east=grid_x
    }

    // ── ARM / OFFBOARD activation sequence ───────────────────────────────

    void stepArmSequence() {
        if (!arm_seq_active_) return;
        ++arm_seq_tick_;

        if (arm_seq_tick_ >= ARM_SEQ_OFFBOARD_START_TICK
            && !px4_offboard_enabled_
            && ((arm_seq_tick_ - ARM_SEQ_OFFBOARD_START_TICK) % ARM_SEQ_RETRY_PERIOD_TICKS == 0)) {
            sendVehicleCommand(CMD_DO_SET_MODE, 1.0f, PX4_CUSTOM_MODE_OFFBOARD);
        }

        if (px4_offboard_enabled_ && !px4_armed_
            && ((arm_seq_tick_ - ARM_SEQ_OFFBOARD_START_TICK) % ARM_SEQ_RETRY_PERIOD_TICKS == 0)) {
            sendVehicleCommand(CMD_ARM_DISARM, 1.0f);
        }

        if (px4_offboard_enabled_ && px4_armed_) {
            arm_seq_active_ = false;
            arm_done_ = true;
        }
    }

    void sendVehicleCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f) {
        px4_msgs::msg::VehicleCommand cmd;
        cmd.timestamp        = timestampUs();
        cmd.command          = command;
        cmd.param1           = param1;
        cmd.param2           = param2;
        cmd.target_system    = 1;
        cmd.target_component = 1;
        cmd.source_system    = 1;
        cmd.source_component = 1;
        cmd.from_external    = true;
        pub_vehicle_cmd_->publish(cmd);
    }

    // ── publish helpers ───────────────────────────────────────────────────

    void publishOffboardControlMode() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp    = timestampUs();
        msg.position     = false;
        msg.velocity     = true;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;
        pub_offboard_mode_->publish(msg);
    }

    void publishTrajectorySetpoint(const px4_offboard_control::ControllerOutput& output) {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp   = timestampUs();
        msg.position    = {NAN, NAN, NAN};
        msg.velocity    = {
            static_cast<float>(output.velocity_mps.x),
            static_cast<float>(output.velocity_mps.y),
            static_cast<float>(output.velocity_mps.z),
        };
        msg.yaw = NAN;
        pub_trajectory_->publish(msg);
    }

    void publishControllerMode(px4_offboard_control::ControlMode mode) {
        std_msgs::msg::String msg;
        msg.data = px4_offboard_control::toString(mode);
        pub_mode_->publish(msg);
    }

    void publishSway() {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(sway_metric_);
        pub_sway_->publish(msg);
    }

    void publishBattery() {
        std_msgs::msg::Float32 msg;
        msg.data = battery_pct_;
        pub_battery_->publish(msg);
    }

    void checkTargetReached() {
        if (!has_target_) return;
        const double dx = target_node_.x - pose_grid_.x;
        const double dy = target_node_.y - pose_grid_.y;
        const bool reached = std::sqrt(dx * dx + dy * dy) < TARGET_REACHED_RADIUS_M;
        std_msgs::msg::Bool msg;
        msg.data = reached;
        pub_target_reached_->publish(msg);
    }

    void checkLandingComplete(px4_offboard_control::ControlMode mode) {
        if (mode != px4_offboard_control::ControlMode::Landing) return;
        std_msgs::msg::Bool msg;
        msg.data = (altitude_m_ < LANDED_ALTITUDE_M);
        pub_landing_complete_->publish(msg);
    }

    void publishTakeoffComplete() {
        std_msgs::msg::Bool msg;
        msg.data = (altitude_m_ >= params_.target_altitude_m - 0.1);
        pub_takeoff_complete_->publish(msg);
    }

    uint64_t timestampUs() const {
        return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
    }

    // ── state ─────────────────────────────────────────────────────────────

    px4_offboard_control::ControllerParams params_;
    px4_offboard_control::ControllerState  state_;
    std::string mission_state_ = "INIT";

    // ARM/OFFBOARD sequence
    int  arm_seq_tick_   = 0;
    bool arm_seq_active_ = false;
    bool arm_done_       = false;
    bool px4_offboard_enabled_ = false;
    bool px4_armed_ = false;

    // Sensor data
    double sway_metric_  = 0.0;
    float  battery_pct_  = 100.0f;
    double altitude_m_   = 0.0;

    // Grid localization
    px4_offboard_control::Vec3 pose_grid_;

    // Mission target (from mission_manager)
    px4_offboard_control::Vec3 target_node_;
    bool has_target_ = false;

    // Planner target (from VelocityProfile waypoint[0])
    px4_offboard_control::Vec3 planner_target_;
    float planner_speed_ = 0.0f;
    bool has_planner_target_ = false;

    // Vertiport servo
    px4_offboard_control::Vec3 vertiport_error_;
    bool vertiport_error_valid_ = false;

    // Other sensor flags
    bool marker_candidate_ = false;

    // ── ROS handles ───────────────────────────────────────────────────────

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                   sub_mission_state_;
    rclcpp::Subscription<sprint_drone_msgs::msg::VelocityProfile>::SharedPtr sub_velocity_profile_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr                  sub_sway_;
    rclcpp::Subscription<sprint_drone_msgs::msg::MarkerList>::SharedPtr      sub_candidates_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr             sub_vertiport_error_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr          sub_attitude_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr            sub_battery_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr     sub_local_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr       sub_vehicle_control_mode_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr         sub_pose_grid_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr               sub_target_node_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  pub_offboard_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   pub_trajectory_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       pub_vehicle_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr               pub_mode_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr              pub_sway_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr              pub_battery_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 pub_target_reached_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 pub_landing_complete_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 pub_takeoff_complete_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OffboardControlNode>());
    rclcpp::shutdown();
    return 0;
}
