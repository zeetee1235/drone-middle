#include "grid_detector/grid_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sprint_drone_msgs/msg/grid_intersections.hpp"
#include "sprint_drone_msgs/msg/grid_lines.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

int oddKernel(int value) {
    if (value < 1) {
        return 1;
    }
    return (value % 2 == 0) ? value + 1 : value;
}

}  // namespace

class GridDetectorNode final : public rclcpp::Node {
public:
    GridDetectorNode() : Node("grid_detector_node") {
        declare_parameter("hough_threshold", 80);
        declare_parameter("min_line_length", 30);
        declare_parameter("max_line_gap", 10);
        declare_parameter("angle_tolerance_deg", 10.0);
        declare_parameter("expected_grid_pixel_width", 110.0);
        declare_parameter("min_intersections_valid", 2);
        declare_parameter("blur_kernel_size", 5);
        declare_parameter("canny_threshold_low", 50.0);
        declare_parameter("canny_threshold_high", 150.0);
        declare_parameter("merge_axis_tolerance_px", 8.0);
        declare_parameter("merge_gap_tolerance_px", 20.0);
        declare_parameter("intersection_padding_px", 12.0);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort);

        sub_image_ = create_subscription<sensor_msgs::msg::Image>(
            "/drone/camera/down/image_raw",
            qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg); });

        pub_lines_ = create_publisher<sprint_drone_msgs::msg::GridLines>(
            "/grid/lines", qos);
        pub_intersections_ = create_publisher<sprint_drone_msgs::msg::GridIntersections>(
            "/grid/intersections", qos);
        pub_heading_ = create_publisher<std_msgs::msg::Float32>(
            "/grid/heading_angle", qos);
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

        cv::Mat blurred;
        const int blur_kernel = oddKernel(get_parameter("blur_kernel_size").as_int());
        if (blur_kernel > 1) {
            cv::GaussianBlur(gray, blurred, cv::Size(blur_kernel, blur_kernel), 0.0);
        } else {
            blurred = gray;
        }

        cv::Mat edges;
        cv::Canny(
            blurred,
            edges,
            get_parameter("canny_threshold_low").as_double(),
            get_parameter("canny_threshold_high").as_double());

        std::vector<cv::Vec4i> hough_lines;
        cv::HoughLinesP(
            edges,
            hough_lines,
            1.0,
            CV_PI / 180.0,
            get_parameter("hough_threshold").as_int(),
            get_parameter("min_line_length").as_int(),
            get_parameter("max_line_gap").as_int());

        std::vector<grid_detector::LineSegment> raw_lines;
        raw_lines.reserve(hough_lines.size());
        for (const auto& line : hough_lines) {
            raw_lines.push_back({
                {static_cast<double>(line[0]), static_cast<double>(line[1])},
                {static_cast<double>(line[2]), static_cast<double>(line[3])},
            });
        }

        grid_detector::GridGeometryParams params;
        params.angle_tolerance_deg = get_parameter("angle_tolerance_deg").as_double();
        params.merge_axis_tolerance_px = get_parameter("merge_axis_tolerance_px").as_double();
        params.merge_gap_tolerance_px = get_parameter("merge_gap_tolerance_px").as_double();
        params.intersection_padding_px = get_parameter("intersection_padding_px").as_double();
        const auto min_intersections = std::max<int64_t>(
            0, get_parameter("min_intersections_valid").as_int());
        params.min_intersections_valid = static_cast<std::size_t>(min_intersections);

        const auto result = grid_detector::analyzeGridGeometry(raw_lines, params);
        publishResult(*msg, image.cols, image.rows, result);
    }

    void publishResult(
        const sensor_msgs::msg::Image& image_msg,
        int image_width,
        int image_height,
        const grid_detector::GridGeometryResult& result)
    {
        sprint_drone_msgs::msg::GridLines lines_msg;
        lines_msg.header = image_msg.header;
        lines_msg.dominant_heading_deg = static_cast<float>(result.dominant_heading_deg);
        lines_msg.confidence = static_cast<float>(result.confidence);

        for (const auto& line : result.merged_lines) {
            lines_msg.line_start_pts.push_back(
                makePoint(line.segment.start.x, line.segment.start.y));
            lines_msg.line_end_pts.push_back(
                makePoint(line.segment.end.x, line.segment.end.y));
            lines_msg.angles_deg.push_back(static_cast<float>(line.angle_deg));
        }
        pub_lines_->publish(lines_msg);

        sprint_drone_msgs::msg::GridIntersections intersections_msg;
        intersections_msg.header = image_msg.header;
        intersections_msg.confidence = static_cast<float>(result.confidence);

        const double expected_grid_px = std::max(
            1.0, get_parameter("expected_grid_pixel_width").as_double());
        const double meters_per_px = 3.0 / expected_grid_px;
        const double cx = static_cast<double>(image_width) * 0.5;
        const double cy = static_cast<double>(image_height) * 0.5;

        for (const auto& point : result.intersections) {
            intersections_msg.pixel_positions.push_back(makePoint(point.x, point.y));
            intersections_msg.world_positions.push_back(makePoint(
                (point.x - cx) * meters_per_px,
                (point.y - cy) * meters_per_px,
                0.0));
        }
        pub_intersections_->publish(intersections_msg);

        std_msgs::msg::Float32 heading_msg;
        heading_msg.data = static_cast<float>(result.dominant_heading_deg);
        pub_heading_->publish(heading_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sprint_drone_msgs::msg::GridLines>::SharedPtr pub_lines_;
    rclcpp::Publisher<sprint_drone_msgs::msg::GridIntersections>::SharedPtr pub_intersections_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_heading_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
