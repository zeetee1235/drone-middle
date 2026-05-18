#pragma once

#include <utility>

namespace visual_odometry {

struct LocalizerParams {
    double target_altitude_m = 2.0;
    double camera_fov_deg = 110.0;
    double grid_spacing_m = 3.0;
    double grid_snap_radius_m = 0.4;
    double home_grid_yaw_rad = 0.0;
};

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Pose2 {
    double x = 0.0;
    double y = 0.0;
    double yaw_rad = 0.0;
};

struct FlowMeasurement {
    double mean_dx_px = 0.0;
    double mean_dy_px = 0.0;
    double dt_sec = 0.0;
    int image_width_px = 640;
    int image_height_px = 480;
    double altitude_m = 2.0;
};

struct SnapResult {
    Pose2 pose;
    bool snapped = false;
    double correction_m = 0.0;
};

double metersPerPixel(int image_width_px, double altitude_m, double horizontal_fov_deg);
Vec2 pixelFlowToVelocity(const FlowMeasurement& flow, double horizontal_fov_deg);
Pose2 integratePose(const Pose2& pose, const Vec2& velocity_body_mps, double dt_sec);
SnapResult snapPoseToGrid(const Pose2& pose, double spacing_m, double radius_m);

Pose2 transformHomeToGrid(const Pose2& home_pose, const Pose2& home_in_grid);
Pose2 transformGridToHome(const Pose2& grid_pose, const Pose2& home_in_grid);
double driftMetric(const Pose2& predicted, const Pose2& corrected);

}  // namespace visual_odometry
