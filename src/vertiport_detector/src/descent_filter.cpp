#include "vertiport_detector/descent_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vertiport_detector {

namespace {

constexpr double kPi = 3.14159265358979323846;

double metersPerPixel(int image_width_px, double altitude_m, double fov_deg) {
    const double fov_rad = fov_deg * kPi / 180.0;
    const double ground_width_m = 2.0 * altitude_m * std::tan(fov_rad * 0.5);
    return ground_width_m / static_cast<double>(image_width_px);
}

}  // namespace

std::pair<double, double> computeCenterError(
    const VertiportObservation& obs,
    int image_width_px,
    int image_height_px,
    double altitude_m,
    double camera_fov_deg)
{
    if (image_width_px <= 0 || image_height_px <= 0) {
        throw std::invalid_argument("image dimensions must be positive");
    }
    if (!(altitude_m > 0.0) || !(camera_fov_deg > 0.0)) {
        throw std::invalid_argument("altitude_m and camera_fov_deg must be positive");
    }
    const double mpp = metersPerPixel(image_width_px, altitude_m, camera_fov_deg);
    const double cx = static_cast<double>(image_width_px) * 0.5;
    const double cy = static_cast<double>(image_height_px) * 0.5;
    return {
        (obs.center_x_px - cx) * mpp,
        (obs.center_y_px - cy) * mpp,
    };
}

DescentFilter::DescentFilter(VertiportParams params)
    : params_(params)
{
    if (params_.confirm_threshold <= 0.0 || params_.confirm_threshold > 1.0) {
        throw std::invalid_argument("confirm_threshold must be in (0, 1]");
    }
    if (params_.image_width_px <= 0 || params_.image_height_px <= 0) {
        throw std::invalid_argument("image dimensions must be positive");
    }
}

VertiportState DescentFilter::update(const VertiportObservation& obs) {
    if (!obs.valid) {
        decayMissing();
        return state_;
    }

    const auto [ex, ey] = computeCenterError(
        obs,
        params_.image_width_px,
        params_.image_height_px,
        params_.altitude_m,
        params_.camera_fov_deg);

    state_.center_error_x_m = ex;
    state_.center_error_y_m = ey;
    state_.confidence = std::min(1.0, state_.confidence + params_.confidence_increment);
    state_.acquired = state_.confidence >= params_.confirm_threshold;

    const double err = std::sqrt(ex * ex + ey * ey);
    state_.can_descend = state_.acquired && err <= params_.align_threshold_m;

    return state_;
}

void DescentFilter::decayMissing() {
    state_.confidence *= params_.missing_decay;
    state_.acquired = state_.confidence >= params_.confirm_threshold;
    state_.can_descend = false;
    // center error is stale when there's no detection; leave last known value but block descent
}

void DescentFilter::reset() {
    state_ = {};
}

VertiportState DescentFilter::state() const {
    return state_;
}

}  // namespace vertiport_detector
