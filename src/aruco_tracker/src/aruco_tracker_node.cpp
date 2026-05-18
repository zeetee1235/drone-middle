#include "aruco_tracker/confidence_filter.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sprint_drone_msgs/msg/marker_detection.hpp"
#include "sprint_drone_msgs/msg/marker_detection_list.hpp"
#include "sprint_drone_msgs/msg/marker_info.hpp"
#include "sprint_drone_msgs/msg/marker_list.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"

namespace {

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

double sideLength(const cv::Point2f& a, const cv::Point2f& b) {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

double markerSizePx(const std::vector<cv::Point2f>& corners) {
    if (corners.size() != 4) {
        return 0.0;
    }
    return 0.25 * (
        sideLength(corners[0], corners[1])
        + sideLength(corners[1], corners[2])
        + sideLength(corners[2], corners[3])
        + sideLength(corners[3], corners[0]));
}

cv::Point2f markerCenter(const std::vector<cv::Point2f>& corners) {
    cv::Point2f center{0.0F, 0.0F};
    if (corners.empty()) {
        return center;
    }
    for (const auto& p : corners) {
        center += p;
    }
    center.x /= static_cast<float>(corners.size());
    center.y /= static_cast<float>(corners.size());
    return center;
}

double markerYawDeg(const std::vector<cv::Point2f>& corners) {
    if (corners.size() < 2) {
        return 0.0;
    }
    const auto& a = corners[0];
    const auto& b = corners[1];
    return std::atan2(static_cast<double>(b.y - a.y), static_cast<double>(b.x - a.x))
        * 180.0 / CV_PI;
}

}  // namespace

class ArucoTrackerNode final : public rclcpp::Node {
public:
    ArucoTrackerNode()
        : Node("aruco_tracker_node")
    {
        declare_parameter("confirm_threshold", 0.75);
        declare_parameter("sway_threshold", 0.3);
        declare_parameter("confidence_increment", 0.15);
        declare_parameter("confidence_decay", 0.8);
        declare_parameter("missing_decay", 0.92);
        declare_parameter("pos_stable_px", 10.0);
        declare_parameter("size_stable_ratio", 0.2);
        declare_parameter("expected_grid_pixel_width", 110.0);

        aruco_tracker::ConfidenceParams params;
        params.confirm_threshold = get_parameter("confirm_threshold").as_double();
        params.sway_threshold = get_parameter("sway_threshold").as_double();
        params.confidence_increment = get_parameter("confidence_increment").as_double();
        params.confidence_decay = get_parameter("confidence_decay").as_double();
        params.missing_decay = get_parameter("missing_decay").as_double();
        params.pos_stable_px = get_parameter("pos_stable_px").as_double();
        params.size_stable_ratio = get_parameter("size_stable_ratio").as_double();
        filter_ = std::make_unique<aruco_tracker::ConfidenceFilter>(params);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_image_ = create_subscription<sensor_msgs::msg::Image>(
            "/drone/camera/down/image_raw",
            sensor_qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg); });

        sub_sway_ = create_subscription<std_msgs::msg::Float32>(
            "/sway/metric",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Float32::ConstSharedPtr msg) { sway_metric_ = msg->data; });

        pub_detections_ = create_publisher<sprint_drone_msgs::msg::MarkerDetectionList>(
            "/aruco/detections", sensor_qos);
        pub_candidates_ = create_publisher<sprint_drone_msgs::msg::MarkerList>(
            "/markers/candidates", rclcpp::QoS(10));
        pub_confirmed_ = create_publisher<sprint_drone_msgs::msg::MarkerList>(
            "/markers/confirmed", rclcpp::QoS(10));
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

        const cv::Mat& image = cv_ptr->image;
        if (image.empty()) {
            return;
        }

        cv::Mat gray;
        if (image.channels() == 1) {
            gray = image;
        } else {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

        const double stamp_sec = static_cast<double>(msg->header.stamp.sec)
            + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

        sprint_drone_msgs::msg::MarkerDetectionList detection_msg;
        detection_msg.header = msg->header;

        for (std::size_t i = 0; i < ids.size(); ++i) {
            const auto center = markerCenter(corners[i]);
            const auto size = markerSizePx(corners[i]);
            const auto yaw = markerYawDeg(corners[i]);

            aruco_tracker::MarkerObservation observation;
            observation.id = ids[i];
            observation.center_x_px = center.x;
            observation.center_y_px = center.y;
            observation.size_px = size;
            observation.yaw_deg = yaw;
            observation.timestamp_sec = stamp_sec;

            const auto& state = filter_->update(observation, sway_metric_);

            auto detection = makeDetection(
                msg->header,
                state,
                image.cols,
                image.rows);
            detection_msg.detections.push_back(detection);
        }

        filter_->decayMissing(stamp_sec);
        pub_detections_->publish(detection_msg);
        publishMarkerLists(msg->header, image.cols, image.rows);
    }

    sprint_drone_msgs::msg::MarkerDetection makeDetection(
        const std_msgs::msg::Header& header,
        const aruco_tracker::MarkerState& state,
        int width,
        int height) const
    {
        sprint_drone_msgs::msg::MarkerDetection msg;
        msg.header = header;
        msg.id = state.id;
        msg.pixel_center = makePoint(state.center_x_px, state.center_y_px);
        msg.grid_position = pixelToMetric(state.center_x_px, state.center_y_px, width, height);
        msg.size_px = static_cast<float>(state.size_px);
        msg.yaw_deg = static_cast<float>(state.yaw_deg);
        msg.confidence = static_cast<float>(state.confidence);
        return msg;
    }

    sprint_drone_msgs::msg::MarkerInfo makeMarkerInfo(
        const std_msgs::msg::Header& header,
        const aruco_tracker::MarkerState& state,
        int width,
        int height) const
    {
        sprint_drone_msgs::msg::MarkerInfo msg;
        msg.header = header;
        msg.id = state.id;
        msg.grid_position = pixelToMetric(state.center_x_px, state.center_y_px, width, height);
        msg.home_position = msg.grid_position;
        msg.confidence = static_cast<float>(state.confidence);
        msg.status = aruco_tracker::toString(state.status);
        return msg;
    }

    geometry_msgs::msg::Point pixelToMetric(
        double x_px,
        double y_px,
        int width,
        int height) const
    {
        const double expected_grid_px = std::max(
            1.0, get_parameter("expected_grid_pixel_width").as_double());
        const double meters_per_px = 3.0 / expected_grid_px;
        return makePoint(
            (x_px - static_cast<double>(width) * 0.5) * meters_per_px,
            (y_px - static_cast<double>(height) * 0.5) * meters_per_px,
            0.0);
    }

    void publishMarkerLists(
        const std_msgs::msg::Header& header,
        int width,
        int height)
    {
        sprint_drone_msgs::msg::MarkerList candidates;
        candidates.header = header;
        for (const auto& state : filter_->candidateStates()) {
            candidates.markers.push_back(makeMarkerInfo(header, state, width, height));
        }
        pub_candidates_->publish(candidates);

        sprint_drone_msgs::msg::MarkerList confirmed;
        confirmed.header = header;
        for (const auto& state : filter_->confirmedStates()) {
            confirmed.markers.push_back(makeMarkerInfo(header, state, width, height));
        }
        pub_confirmed_->publish(confirmed);
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::unique_ptr<aruco_tracker::ConfidenceFilter> filter_;
    double sway_metric_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_sway_;
    rclcpp::Publisher<sprint_drone_msgs::msg::MarkerDetectionList>::SharedPtr pub_detections_;
    rclcpp::Publisher<sprint_drone_msgs::msg::MarkerList>::SharedPtr pub_candidates_;
    rclcpp::Publisher<sprint_drone_msgs::msg::MarkerList>::SharedPtr pub_confirmed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
