#include "vertiport_detector/descent_filter.hpp"

#include <memory>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

// Detect the largest orange blob in the HSV image and return its enclosing circle.
// Returns false if no candidate found.
bool detectVertiport(
    const cv::Mat& bgr,
    int h_lo, int h_hi,
    int s_lo, int v_lo,
    double& cx, double& cy, double& radius)
{
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv,
        cv::Scalar(h_lo, s_lo, v_lo),
        cv::Scalar(h_hi, 255, 255),
        mask);

    // Morphological cleanup to reduce noise
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        return false;
    }

    // Pick the largest contour by area
    std::size_t best = 0;
    double best_area = 0.0;
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area > best_area) {
            best_area = area;
            best = i;
        }
    }

    cv::Point2f center;
    float r = 0.0F;
    cv::minEnclosingCircle(contours[best], center, r);

    cx = static_cast<double>(center.x);
    cy = static_cast<double>(center.y);
    radius = static_cast<double>(r);
    return true;
}

}  // namespace

class VertiportDetectorNode final : public rclcpp::Node {
public:
    VertiportDetectorNode()
        : Node("vertiport_detector_node")
    {
        declare_parameter("confirm_threshold", 0.7);
        declare_parameter("confidence_increment", 0.15);
        declare_parameter("missing_decay", 0.85);
        declare_parameter("align_threshold_m", 0.3);
        declare_parameter("altitude_m", 2.0);
        declare_parameter("camera_fov_deg", 110.0);
        // HSV orange bounds (OpenCV hue: 0-179)
        declare_parameter("hsv_h_lo", 5);
        declare_parameter("hsv_h_hi", 25);
        declare_parameter("hsv_s_lo", 80);
        declare_parameter("hsv_v_lo", 80);
        // minimum blob area as fraction of image area to reject tiny false positives
        declare_parameter("min_area_ratio", 0.005);

        vertiport_detector::VertiportParams params;
        params.confirm_threshold = get_parameter("confirm_threshold").as_double();
        params.confidence_increment = get_parameter("confidence_increment").as_double();
        params.missing_decay = get_parameter("missing_decay").as_double();
        params.align_threshold_m = get_parameter("align_threshold_m").as_double();
        params.altitude_m = get_parameter("altitude_m").as_double();
        params.camera_fov_deg = get_parameter("camera_fov_deg").as_double();
        filter_ = std::make_unique<vertiport_detector::DescentFilter>(params);

        const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);

        sub_image_ = create_subscription<sensor_msgs::msg::Image>(
            "/drone/camera/down/image_raw",
            sensor_qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg); });

        sub_altitude_ = create_subscription<std_msgs::msg::Float32>(
            "/drone/altitude_m",
            rclcpp::QoS(10),
            [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
                altitude_m_ = static_cast<double>(msg->data);
            });

        pub_center_error_ = create_publisher<geometry_msgs::msg::Vector3>(
            "/vertiport/center_error", rclcpp::QoS(10));
        pub_acquired_ = create_publisher<std_msgs::msg::Bool>(
            "/vertiport/acquired", rclcpp::QoS(10));
        pub_can_descend_ = create_publisher<std_msgs::msg::Bool>(
            "/vertiport/can_descend", rclcpp::QoS(10));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
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

        // Lazily update image dimensions in the filter params on first frame
        if (!dims_set_) {
            filter_->reset();
            vertiport_detector::VertiportParams params;
            params.image_width_px = image.cols;
            params.image_height_px = image.rows;
            params.altitude_m = altitude_m_;
            params.camera_fov_deg = get_parameter("camera_fov_deg").as_double();
            params.confirm_threshold = get_parameter("confirm_threshold").as_double();
            params.confidence_increment = get_parameter("confidence_increment").as_double();
            params.missing_decay = get_parameter("missing_decay").as_double();
            params.align_threshold_m = get_parameter("align_threshold_m").as_double();
            filter_ = std::make_unique<vertiport_detector::DescentFilter>(params);
            dims_set_ = true;
        }

        const int h_lo = get_parameter("hsv_h_lo").as_int();
        const int h_hi = get_parameter("hsv_h_hi").as_int();
        const int s_lo = get_parameter("hsv_s_lo").as_int();
        const int v_lo = get_parameter("hsv_v_lo").as_int();
        const double min_area = get_parameter("min_area_ratio").as_double()
            * static_cast<double>(image.cols * image.rows);

        double cx = 0.0;
        double cy = 0.0;
        double radius = 0.0;
        const bool found = detectVertiport(image, h_lo, h_hi, s_lo, v_lo, cx, cy, radius);

        vertiport_detector::VertiportObservation obs;
        obs.valid = found && (radius * radius * M_PI >= min_area);
        obs.center_x_px = cx;
        obs.center_y_px = cy;
        obs.radius_px = radius;
        obs.timestamp_sec = static_cast<double>(msg->header.stamp.sec)
            + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

        const auto state = filter_->update(obs);
        publish(state);
    }

    void publish(const vertiport_detector::VertiportState& state) {
        geometry_msgs::msg::Vector3 err;
        err.x = state.center_error_x_m;
        err.y = state.center_error_y_m;
        err.z = 0.0;
        pub_center_error_->publish(err);

        std_msgs::msg::Bool acquired;
        acquired.data = state.acquired;
        pub_acquired_->publish(acquired);

        std_msgs::msg::Bool can_descend;
        can_descend.data = state.can_descend;
        pub_can_descend_->publish(can_descend);
    }

    std::unique_ptr<vertiport_detector::DescentFilter> filter_;
    double altitude_m_ = 2.0;
    bool dims_set_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_altitude_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_center_error_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_acquired_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_can_descend_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VertiportDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
