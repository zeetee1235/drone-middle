#include "visual_odometry/localization_core.hpp"

#include <cmath>
#include <stdexcept>

namespace visual_odometry {

namespace {

constexpr double kPi = 3.14159265358979323846;

void validatePositive(const char* name, double value) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        throw std::invalid_argument(std::string(name) + " must be positive and finite");
    }
}

double normalizeYaw(double yaw) {
    while (yaw > kPi) {
        yaw -= 2.0 * kPi;
    }
    while (yaw < -kPi) {
        yaw += 2.0 * kPi;
    }
    return yaw;
}

Vec2 rotate(const Vec2& v, double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    return {
        c * v.x - s * v.y,
        s * v.x + c * v.y,
    };
}

}  // namespace

double metersPerPixel(int image_width_px, double altitude_m, double horizontal_fov_deg) {
    if (image_width_px <= 0) {
        throw std::invalid_argument("image_width_px must be positive");
    }
    validatePositive("altitude_m", altitude_m);
    validatePositive("horizontal_fov_deg", horizontal_fov_deg);

    const double fov_rad = horizontal_fov_deg * kPi / 180.0;
    const double ground_width_m = 2.0 * altitude_m * std::tan(fov_rad * 0.5);
    return ground_width_m / static_cast<double>(image_width_px);
}

Vec2 pixelFlowToVelocity(const FlowMeasurement& flow, double horizontal_fov_deg) {
    validatePositive("dt_sec", flow.dt_sec);
    const double m_per_px = metersPerPixel(
        flow.image_width_px,
        flow.altitude_m,
        horizontal_fov_deg);

    // If the image moves right, the vehicle moved left relative to the ground.
    return {
        -flow.mean_dx_px * m_per_px / flow.dt_sec,
        -flow.mean_dy_px * m_per_px / flow.dt_sec,
    };
}

Pose2 integratePose(const Pose2& pose, const Vec2& velocity_body_mps, double dt_sec) {
    validatePositive("dt_sec", dt_sec);
    const auto velocity_world = rotate(velocity_body_mps, pose.yaw_rad);
    return {
        pose.x + velocity_world.x * dt_sec,
        pose.y + velocity_world.y * dt_sec,
        normalizeYaw(pose.yaw_rad),
    };
}

SnapResult snapPoseToGrid(const Pose2& pose, double spacing_m, double radius_m) {
    validatePositive("spacing_m", spacing_m);
    validatePositive("radius_m", radius_m);

    const double gx = std::round(pose.x / spacing_m) * spacing_m;
    const double gy = std::round(pose.y / spacing_m) * spacing_m;
    const double dx = gx - pose.x;
    const double dy = gy - pose.y;
    const double correction = std::sqrt(dx * dx + dy * dy);

    if (correction <= radius_m) {
        return {{gx, gy, pose.yaw_rad}, true, correction};
    }
    return {pose, false, correction};
}

Pose2 transformHomeToGrid(const Pose2& home_pose, const Pose2& home_in_grid) {
    const auto rotated = rotate({home_pose.x, home_pose.y}, home_in_grid.yaw_rad);
    return {
        home_in_grid.x + rotated.x,
        home_in_grid.y + rotated.y,
        normalizeYaw(home_in_grid.yaw_rad + home_pose.yaw_rad),
    };
}

Pose2 transformGridToHome(const Pose2& grid_pose, const Pose2& home_in_grid) {
    const Vec2 delta{
        grid_pose.x - home_in_grid.x,
        grid_pose.y - home_in_grid.y,
    };
    const auto rotated = rotate(delta, -home_in_grid.yaw_rad);
    return {
        rotated.x,
        rotated.y,
        normalizeYaw(grid_pose.yaw_rad - home_in_grid.yaw_rad),
    };
}

double driftMetric(const Pose2& predicted, const Pose2& corrected) {
    const double dx = corrected.x - predicted.x;
    const double dy = corrected.y - predicted.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace visual_odometry
