#pragma once

#include <utility>

namespace vertiport_detector {

struct VertiportParams {
    int image_width_px = 640;
    int image_height_px = 480;
    double altitude_m = 2.0;
    double camera_fov_deg = 110.0;
    double confirm_threshold = 0.7;
    double confidence_increment = 0.15;
    double missing_decay = 0.85;
    // radial center error must be below this to allow descent
    double align_threshold_m = 0.3;
};

struct VertiportObservation {
    double center_x_px = 0.0;
    double center_y_px = 0.0;
    double radius_px = 0.0;
    double timestamp_sec = 0.0;
    bool valid = false;
};

struct VertiportState {
    // lateral (x = image-right) and longitudinal (y = image-down) offset in meters
    double center_error_x_m = 0.0;
    double center_error_y_m = 0.0;
    double confidence = 0.0;
    bool acquired = false;
    bool can_descend = false;
};

// Pixel offset of the detected vertiport center from the image center, converted to meters.
std::pair<double, double> computeCenterError(
    const VertiportObservation& obs,
    int image_width_px,
    int image_height_px,
    double altitude_m,
    double camera_fov_deg);

class DescentFilter {
public:
    explicit DescentFilter(VertiportParams params = {});

    // Feed one detection frame. If obs.valid == false, acts as a missing-frame decay.
    VertiportState update(const VertiportObservation& obs);

    // Call explicitly when no image is available this cycle.
    void decayMissing();

    void reset();

    VertiportState state() const;

private:
    VertiportParams params_;
    VertiportState state_;
};

}  // namespace vertiport_detector
