#include "visual_odometry/localization_core.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sprint_drone_msgs/msg/grid_intersections.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw_rad) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw_rad * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw_rad * 0.5);
    return q;
}

double stampToSec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

class VisualOdometryNode final : public rclcpp::Node {
public:
    VisualOdometryNode()
        : Node("visual_odometry_node")
    {
        declare_parameter("target_altitude_m", 2.0);
        declare_parameter("camera_fov_deg", 110.0);
        declare_parameter("grid_spacing_m", 3.0);
        declare_parameter("grid_snap_radius_m", 0.4);
        declare_parameter("max_corners", 160);
        declare_parameter("quality_level", 0.01);
        declare_parameter("min_distance_px", 10.0);
        declare_parameter("lk_window_px", 21);
        declare_parameter("min_tracked_points", 12);

        params_.target_altitude_m = get_parameter("target_altitude_m").as_double();
        params_.camera_fov_deg = get_parameter("camera_fov_deg").as_double();
        params_.grid_spacing_m = get_parameter("grid_spacing_m").as_double();
        params_.grid_snap_radius_m = get_parameter("grid_snap_radius_m").as_double();

        pose_grid_ = {0.0, 0.0, 0.0};
        home_in_grid_ = {0.0, 0.0, 0.0};

        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);

        sub_image_ = create_subscription<sensor_msgs::msg::Image>(
            "/drone/camera/down/image_raw",
            sensor_qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg); });

        sub_grid_ = create_subscription<sprint_drone_msgs::msg::GridIntersections>(
            "/grid/intersections",
            sensor_qos,
            [this](sprint_drone_msgs::msg::GridIntersections::ConstSharedPtr msg) {
                latest_grid_confidence_ = msg->confidence;
            });

        pub_flow_ = create_publisher<geometry_msgs::msg::Vector3>(
            "/optical_flow/velocity", rclcpp::QoS(10));
        pub_pose_grid_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/localization/pose_grid", rclcpp::QoS(10));
        pub_pose_home_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/localization/pose_home", rclcpp::QoS(10));
        pub_drift_ = create_publisher<std_msgs::msg::Float32>(
            "/localization/drift_metric", rclcpp::QoS(10));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "cv_bridge conversion failed: %s", e.what());
            return;
        }

        cv::Mat gray;
        if (cv_ptr->image.channels() == 1) {
            gray = cv_ptr->image;
        } else {
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        }

        const double now_sec = stampToSec(msg->header.stamp);
        if (prev_gray_.empty() || prev_points_.empty() || last_stamp_sec_ <= 0.0) {
            resetTracking(gray, now_sec);
            publishPoses(msg->header);
            return;
        }

        const double dt = now_sec - last_stamp_sec_;
        if (dt <= 0.0 || dt > 1.0) {
            resetTracking(gray, now_sec);
            publishPoses(msg->header);
            return;
        }

        std::vector<cv::Point2f> next_points;
        std::vector<unsigned char> status;
        std::vector<float> error;
        const int window_px = static_cast<int>(
            std::max<int64_t>(3, get_parameter("lk_window_px").as_int()));
        cv::calcOpticalFlowPyrLK(
            prev_gray_,
            gray,
            prev_points_,
            next_points,
            status,
            error,
            cv::Size(window_px, window_px),
            3);

        std::vector<double> dx_values;
        std::vector<double> dy_values;
        std::vector<cv::Point2f> kept_points;
        for (std::size_t i = 0; i < status.size(); ++i) {
            if (!status[i]) {
                continue;
            }
            dx_values.push_back(static_cast<double>(next_points[i].x - prev_points_[i].x));
            dy_values.push_back(static_cast<double>(next_points[i].y - prev_points_[i].y));
            kept_points.push_back(next_points[i]);
        }

        const int min_points = get_parameter("min_tracked_points").as_int();
        if (static_cast<int>(kept_points.size()) < min_points) {
            resetTracking(gray, now_sec);
            publishPoses(msg->header);
            return;
        }

        const double mean_dx = mean(dx_values);
        const double mean_dy = mean(dy_values);
        visual_odometry::FlowMeasurement flow;
        flow.mean_dx_px = mean_dx;
        flow.mean_dy_px = mean_dy;
        flow.dt_sec = dt;
        flow.image_width_px = gray.cols;
        flow.image_height_px = gray.rows;
        flow.altitude_m = params_.target_altitude_m;

        const auto velocity = visual_odometry::pixelFlowToVelocity(flow, params_.camera_fov_deg);
        const auto predicted = visual_odometry::integratePose(pose_grid_, velocity, dt);
        auto corrected = predicted;
        double drift = 0.0;

        if (latest_grid_confidence_ > 0.5F) {
            const auto snap = visual_odometry::snapPoseToGrid(
                predicted,
                params_.grid_spacing_m,
                params_.grid_snap_radius_m);
            corrected = snap.pose;
            drift = snap.snapped ? snap.correction_m : 0.0;
        }

        pose_grid_ = corrected;
        publishVelocity(velocity);
        publishDrift(drift);
        publishPoses(msg->header);

        prev_gray_ = gray.clone();
        prev_points_ = kept_points;
        last_stamp_sec_ = now_sec;
        if (static_cast<int>(prev_points_.size()) < min_points * 2) {
            detectFeatures(prev_gray_, prev_points_);
        }
    }

    void resetTracking(const cv::Mat& gray, double stamp_sec) {
        prev_gray_ = gray.clone();
        prev_points_.clear();
        detectFeatures(prev_gray_, prev_points_);
        last_stamp_sec_ = stamp_sec;
    }

    void detectFeatures(const cv::Mat& gray, std::vector<cv::Point2f>& points) const {
        cv::goodFeaturesToTrack(
            gray,
            points,
            get_parameter("max_corners").as_int(),
            get_parameter("quality_level").as_double(),
            get_parameter("min_distance_px").as_double());
    }

    static double mean(const std::vector<double>& values) {
        if (values.empty()) {
            return 0.0;
        }
        return std::accumulate(values.begin(), values.end(), 0.0)
            / static_cast<double>(values.size());
    }

    void publishVelocity(const visual_odometry::Vec2& velocity) {
        geometry_msgs::msg::Vector3 msg;
        msg.x = velocity.x;
        msg.y = velocity.y;
        msg.z = 0.0;
        pub_flow_->publish(msg);
    }

    void publishDrift(double drift) {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(drift);
        pub_drift_->publish(msg);
    }

    void publishPoses(const std_msgs::msg::Header& header) {
        publishPose(pub_pose_grid_, header, pose_grid_, "grid");
        const auto pose_home = visual_odometry::transformGridToHome(pose_grid_, home_in_grid_);
        publishPose(pub_pose_home_, header, pose_home, "home");
    }

    void publishPose(
        const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
        const std_msgs::msg::Header& header,
        const visual_odometry::Pose2& pose,
        const char* frame_id)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header = header;
        msg.header.frame_id = frame_id;
        msg.pose.position.x = pose.x;
        msg.pose.position.y = pose.y;
        msg.pose.position.z = params_.target_altitude_m;
        msg.pose.orientation = yawToQuaternion(pose.yaw_rad);
        pub->publish(msg);
    }

    visual_odometry::LocalizerParams params_;
    visual_odometry::Pose2 pose_grid_;
    visual_odometry::Pose2 home_in_grid_;
    float latest_grid_confidence_ = 0.0F;
    double last_stamp_sec_ = -1.0;

    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_points_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sprint_drone_msgs::msg::GridIntersections>::SharedPtr sub_grid_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_flow_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_grid_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_home_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_drift_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
