#include "mission_manager/mission_core.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sprint_drone_msgs/msg/marker_list.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

mission_manager::MarkerInfo toCoreMarker(
    const sprint_drone_msgs::msg::MarkerInfo& msg,
    double stamp_sec)
{
    mission_manager::MarkerInfo marker;
    marker.id = msg.id;
    marker.grid_x_m = msg.grid_position.x;
    marker.grid_y_m = msg.grid_position.y;
    marker.home_x_m = msg.home_position.x;
    marker.home_y_m = msg.home_position.y;
    marker.confidence = msg.confidence;
    marker.timestamp_sec = stamp_sec;
    return marker;
}

double nowSec(const rclcpp::Time& time) {
    return time.seconds();
}

geometry_msgs::msg::Point point(double x, double y, double z = 0.0) {
    geometry_msgs::msg::Point msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    return msg;
}

}  // namespace

class MissionManagerNode final : public rclcpp::Node {
public:
    MissionManagerNode()
        : Node("mission_manager_node")
    {
        declare_parameter("hover_confirm_duration_sec", 3.0);
        declare_parameter("battery_return_threshold_pct", 20.0);
        declare_parameter("max_drift_before_stabilize_m", 0.5);
        declare_parameter("home_approach_radius_m", 5.0);
        declare_parameter("vertiport_detect_radius_m", 3.0);
        declare_parameter("required_marker_count", 4);
        declare_parameter("auto_takeoff_complete", false);
        declare_parameter("auto_home_initialized", true);
        declare_parameter("auto_perception_stable", true);

        mission_manager::MissionParams params;
        params.required_marker_count = get_parameter("required_marker_count").as_int();
        params.hover_confirm_duration_sec = get_parameter("hover_confirm_duration_sec").as_double();
        params.battery_return_threshold_pct =
            get_parameter("battery_return_threshold_pct").as_double();
        params.max_drift_before_stabilize_m =
            get_parameter("max_drift_before_stabilize_m").as_double();
        params.home_approach_radius_m = get_parameter("home_approach_radius_m").as_double();
        params.vertiport_detect_radius_m = get_parameter("vertiport_detect_radius_m").as_double();
        core_ = std::make_unique<mission_manager::MissionCore>(params);

        sub_candidates_ = create_subscription<sprint_drone_msgs::msg::MarkerList>(
            "/markers/candidates",
            rclcpp::QoS(10),
            [this](sprint_drone_msgs::msg::MarkerList::ConstSharedPtr msg) {
                marker_candidate_ = !msg->markers.empty();
            });

        sub_confirmed_ = create_subscription<sprint_drone_msgs::msg::MarkerList>(
            "/markers/confirmed",
            rclcpp::QoS(10),
            [this](sprint_drone_msgs::msg::MarkerList::ConstSharedPtr msg) {
                updateConfirmedMarker(*msg);
            });

        sub_battery_ = create_subscription<std_msgs::msg::Float32>(
            "/battery/percent",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
                battery_percent_ = msg->data;
            });

        sub_drift_ = create_subscription<std_msgs::msg::Float32>(
            "/localization/drift_metric",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
                drift_metric_m_ = msg->data;
            });

        sub_target_reached_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/target_reached",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                target_reached_ = msg->data;
            });

        sub_vertiport_acquired_ = create_subscription<std_msgs::msg::Bool>(
            "/vertiport/acquired",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                vertiport_acquired_ = msg->data;
            });

        sub_landing_complete_ = create_subscription<std_msgs::msg::Bool>(
            "/landing/complete",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                landing_complete_ = msg->data;
            });

        sub_takeoff_complete_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/takeoff_complete",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                takeoff_complete_ = msg->data;
            });

        pub_state_ = create_publisher<std_msgs::msg::String>("/mission/state", rclcpp::QoS(10));
        pub_target_ = create_publisher<geometry_msgs::msg::Point>(
            "/mission/target_node", rclcpp::QoS(10));

        srv_start_ = create_service<std_srvs::srv::Trigger>(
            "/mission/start",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                start_requested_ = true;
                response->success = true;
                response->message = "mission start requested";
            });

        srv_abort_ = create_service<std_srvs::srv::Trigger>(
            "/mission/abort",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                abort_requested_ = true;
                response->success = true;
                response->message = "mission abort requested";
            });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { tick(); });
    }

private:
    void updateConfirmedMarker(const sprint_drone_msgs::msg::MarkerList& msg) {
        if (msg.markers.empty()) {
            latest_confirmed_marker_.reset();
            return;
        }

        const auto best = std::max_element(
            msg.markers.begin(),
            msg.markers.end(),
            [](const auto& a, const auto& b) {
                return a.confidence < b.confidence;
            });

        const double stamp_sec = static_cast<double>(msg.header.stamp.sec)
            + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        latest_confirmed_marker_ = toCoreMarker(*best, stamp_sec);
    }

    void tick() {
        mission_manager::TickInput input;
        input.now_sec = nowSec(get_clock()->now());
        input.mission_start = start_requested_;
        input.abort = abort_requested_;
        input.takeoff_complete =
            get_parameter("auto_takeoff_complete").as_bool() || takeoff_complete_;
        input.home_initialized = get_parameter("auto_home_initialized").as_bool();
        input.marker_candidate = marker_candidate_;
        input.perception_stable =
            get_parameter("auto_perception_stable").as_bool()
            && drift_metric_m_ <= get_parameter("max_drift_before_stabilize_m").as_double();
        input.confirmed_marker = latest_confirmed_marker_;
        input.target_reached = target_reached_;
        input.vertiport_acquired = vertiport_acquired_;
        input.landing_complete = landing_complete_;
        input.battery_percent = battery_percent_;
        input.drift_metric_m = drift_metric_m_;

        core_->tick(input);
        start_requested_ = false;
        abort_requested_ = false;

        publishSnapshot();
    }

    void publishSnapshot() {
        const auto snapshot = core_->snapshot();

        std_msgs::msg::String state_msg;
        state_msg.data = mission_manager::toString(snapshot.phase);
        pub_state_->publish(state_msg);

        if (snapshot.active_target_marker.has_value()) {
            const auto& target = *snapshot.active_target_marker;
            pub_target_->publish(point(target.grid_x_m, target.grid_y_m, 2.0));
        } else if (snapshot.phase == mission_manager::MissionPhase::ReturnHome
                   || snapshot.phase == mission_manager::MissionPhase::EmergencyReturn) {
            pub_target_->publish(point(0.0, 0.0, 2.0));
        }
    }

    std::unique_ptr<mission_manager::MissionCore> core_;
    bool start_requested_ = false;
    bool abort_requested_ = false;
    bool marker_candidate_ = false;
    bool target_reached_ = false;
    bool vertiport_acquired_ = false;
    bool landing_complete_ = false;
    bool takeoff_complete_ = false;
    double battery_percent_ = 100.0;
    double drift_metric_m_ = 0.0;
    std::optional<mission_manager::MarkerInfo> latest_confirmed_marker_;

    rclcpp::Subscription<sprint_drone_msgs::msg::MarkerList>::SharedPtr sub_candidates_;
    rclcpp::Subscription<sprint_drone_msgs::msg::MarkerList>::SharedPtr sub_confirmed_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_battery_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_drift_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_target_reached_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_vertiport_acquired_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_landing_complete_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_takeoff_complete_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_target_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_abort_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionManagerNode>());
    rclcpp::shutdown();
    return 0;
}
