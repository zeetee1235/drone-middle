#include "sprint_planner/time_cost.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sprint_drone_msgs/msg/velocity_profile.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ── Grid layout (grid frame: origin = vertiport, x east, y north/south) ──
// Vertiport: Gazebo (2, 19).  First marker column: Gazebo x=4 → grid x=2.
// Marker grid (grid frame): x = 2, 5, 8, …, 26  |  y = 0, -3, …, -15
static constexpr double GRID_SPACING_M  = 3.0;
static constexpr double GRID_COL_OFFSET = 2.0;   // metres from vertiport to first column
static constexpr int    GRID_ROWS = 6;
static constexpr int    GRID_COLS = 9;

struct Pt {
    double x = 0.0;
    double y = 0.0;
};

// Row-by-row serpentine: even rows go east (+x), odd rows go west (-x).
// 54 waypoints covering the full 24×15 m mission area.
static std::vector<Pt> buildSerpentine() {
    std::vector<Pt> pts;
    pts.reserve(GRID_ROWS * GRID_COLS);
    for (int row = 0; row < GRID_ROWS; ++row) {
        const double y = -row * GRID_SPACING_M;
        if (row % 2 == 0) {
            for (int col = 0; col < GRID_COLS; ++col) {
                pts.push_back({GRID_COL_OFFSET + col * GRID_SPACING_M, y});
            }
        } else {
            for (int col = GRID_COLS - 1; col >= 0; --col) {
                pts.push_back({GRID_COL_OFFSET + col * GRID_SPACING_M, y});
            }
        }
    }
    return pts;
}

static double dist2D(const Pt& a, const Pt& b) {
    const double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

class SprintPlannerNode final : public rclcpp::Node {
public:
    SprintPlannerNode() : Node("sprint_planner_node") {
        declare_parameter("v_max_mps",          3.0);
        declare_parameter("a_max_mps2",         2.0);
        declare_parameter("a_brake_mps2",       2.5);
        declare_parameter("t_turn_90_sec",      1.5);
        declare_parameter("t_turn_180_sec",     2.5);
        declare_parameter("t_marker_check_sec", 2.0);
        declare_parameter("t_hover_sec",        3.0);
        declare_parameter("t_stabilize_sec",    0.5);
        declare_parameter("waypoint_advance_m", 1.5);

        serpentine_ = buildSerpentine();

        sub_state_ = create_subscription<std_msgs::msg::String>(
            "/mission/state", rclcpp::QoS(10),
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
                const std::string& prev = mission_state_;
                mission_state_ = msg->data;
                // Reset serpentine index when re-entering grid search
                if (mission_state_ == "GRID_SEARCH" && prev != "GRID_SEARCH") {
                    search_idx_ = nearestSerpentineIndex();
                }
            });

        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/localization/pose_grid", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                pose_ = {msg->pose.position.x, msg->pose.position.y};
                has_pose_ = true;
            });

        sub_target_ = create_subscription<geometry_msgs::msg::Point>(
            "/mission/target_node", rclcpp::QoS(10),
            [this](geometry_msgs::msg::Point::ConstSharedPtr msg) {
                mission_target_ = {msg->x, msg->y};
                has_target_ = true;
            });

        sub_target_reached_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/target_reached", rclcpp::QoS(10),
            [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
                rescue_target_reached_ = msg->data;
            });

        pub_profile_ = create_publisher<sprint_drone_msgs::msg::VelocityProfile>(
            "/planner/velocity_profile", rclcpp::QoS(10));

        // 10 Hz publish rate — fast enough for smooth direction updates
        timer_ = create_wall_timer(100ms, [this]() { planAndPublish(); });
    }

private:
    // ── planning tick ─────────────────────────────────────────────────────

    void planAndPublish() {
        const auto params = loadParams();
        Pt target;
        float speed;
        std::string mode;

        if (mission_state_ == "GRID_SEARCH") {
            advanceSerpentineIfClose();
            target = currentSerpentineWaypoint();
            speed  = static_cast<float>(params.v_max_mps);
            mode   = "GRID_SEARCH";
        } else if (mission_state_ == "RESCUE_VISIT") {
            target = has_target_ ? mission_target_ : pose_;
            // Hold position once target is reached (hover phase); planner resumes
            // after mission_manager advances to the next rescue marker
            speed  = rescue_target_reached_ ? 0.0f : static_cast<float>(params.v_max_mps);
            mode   = "RESCUE";
        } else if (mission_state_ == "MARKER_APPROACH") {
            target = has_target_ ? mission_target_ : pose_;
            speed  = static_cast<float>(params.v_max_mps);
            mode   = "RESCUE";
        } else if (mission_state_ == "RETURN_HOME" || mission_state_ == "EMERGENCY_RETURN") {
            target = {0.0, 0.0};  // home = vertiport = grid origin
            speed  = static_cast<float>(params.v_max_mps);
            mode   = "RETURN";
        } else if (mission_state_ == "VERTIPORT_ACQUIRE" || mission_state_ == "VISION_SERVO_LAND") {
            target = has_target_ ? mission_target_ : pose_;
            speed  = static_cast<float>(params.v_max_mps * 0.3);
            mode   = "SERVO";
        } else {
            // HOVER_CONFIRM, ANTI_SWAY, INIT, LANDED, etc. — hold current position
            target = pose_;
            speed  = 0.0f;
            mode   = "HOLD";
        }

        publish(target, speed, mode, params);
    }

    // ── serpentine helpers ────────────────────────────────────────────────

    std::size_t nearestSerpentineIndex() const {
        if (!has_pose_) return 0;
        std::size_t best = 0;
        double best_d = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < serpentine_.size(); ++i) {
            const double d = dist2D(pose_, serpentine_[i]);
            if (d < best_d) { best_d = d; best = i; }
        }
        return best;
    }

    void advanceSerpentineIfClose() {
        if (!has_pose_ || serpentine_.empty()) return;
        if (search_idx_ >= serpentine_.size()) return;
        const double advance_m = get_parameter("waypoint_advance_m").as_double();
        if (dist2D(pose_, serpentine_[search_idx_]) < advance_m) {
            ++search_idx_;
        }
    }

    Pt currentSerpentineWaypoint() const {
        if (serpentine_.empty()) return pose_;
        const auto idx = std::min(search_idx_, serpentine_.size() - 1);
        return serpentine_[idx];
    }

    // ── publishing ────────────────────────────────────────────────────────

    void publish(const Pt& target, float speed, const std::string& mode,
                 const sprint_planner::DroneParams& params)
    {
        sprint_drone_msgs::msg::VelocityProfile msg;
        msg.header.stamp    = now();
        msg.header.frame_id = "grid";
        msg.planner_mode    = mode;

        geometry_msgs::msg::Point wp;
        wp.x = target.x;
        wp.y = target.y;
        wp.z = params.v_max_mps > 0 ? 2.0 : 2.0;  // altitude (informational)
        msg.waypoints.push_back(wp);
        msg.target_speeds_mps.push_back(speed);

        // Estimate remaining distance for cost/time
        const double dist = has_pose_ ? dist2D(pose_, target) : 0.0;
        const auto timing = sprint_planner::timeCostStraight(dist, 0.0, 0.0, params);
        msg.segment_durations_sec.push_back(static_cast<float>(timing.total_sec));
        msg.total_time_sec = static_cast<float>(timing.total_sec);
        msg.cost           = static_cast<float>(dist);

        pub_profile_->publish(msg);
    }

    // ── parameter loading ─────────────────────────────────────────────────

    sprint_planner::DroneParams loadParams() const {
        sprint_planner::DroneParams p;
        p.v_max_mps          = get_parameter("v_max_mps").as_double();
        p.a_max_mps2         = get_parameter("a_max_mps2").as_double();
        p.a_brake_mps2       = get_parameter("a_brake_mps2").as_double();
        p.t_turn_90_sec      = get_parameter("t_turn_90_sec").as_double();
        p.t_turn_180_sec     = get_parameter("t_turn_180_sec").as_double();
        p.t_marker_check_sec = get_parameter("t_marker_check_sec").as_double();
        p.t_hover_sec        = get_parameter("t_hover_sec").as_double();
        p.t_stabilize_sec    = get_parameter("t_stabilize_sec").as_double();
        return p;
    }

    // ── state ─────────────────────────────────────────────────────────────

    std::vector<Pt> serpentine_;
    std::size_t search_idx_ = 0;

    std::string mission_state_ = "INIT";
    Pt pose_;
    Pt mission_target_;
    bool has_pose_              = false;
    bool has_target_            = false;
    bool rescue_target_reached_ = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr       sub_target_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             sub_target_reached_;
    rclcpp::Publisher<sprint_drone_msgs::msg::VelocityProfile>::SharedPtr pub_profile_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SprintPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
