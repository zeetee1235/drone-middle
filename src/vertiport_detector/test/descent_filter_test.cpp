#include "vertiport_detector/descent_filter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

// 640x480, FOV 110deg: mpp = 2*2*tan(55deg)/640 ≈ 0.00893 m/px
vertiport_detector::VertiportParams defaultParams() {
    vertiport_detector::VertiportParams p;
    p.image_width_px = 640;
    p.image_height_px = 480;
    p.altitude_m = 2.0;
    p.camera_fov_deg = 110.0;
    p.confirm_threshold = 0.6;
    p.confidence_increment = 0.25;
    p.missing_decay = 0.5;
    p.align_threshold_m = 0.3;
    return p;
}

vertiport_detector::VertiportObservation centeredObs(double t = 0.0) {
    return {320.0, 240.0, 80.0, t, true};
}

void testCenteredDetectionBuildsConfidenceAndAcquires() {
    vertiport_detector::DescentFilter filter(defaultParams());

    filter.update(centeredObs(0.0));
    filter.update(centeredObs(0.1));
    filter.update(centeredObs(0.2));

    const auto s = filter.state();
    assert(s.acquired);
    assert(s.can_descend);
    // exactly centered → zero error
    assert(std::abs(s.center_error_x_m) < 1e-9);
    assert(std::abs(s.center_error_y_m) < 1e-9);
}

void testOffCenterBlocksDescent() {
    vertiport_detector::DescentFilter filter(defaultParams());

    // 200px right of center → ~1.78m offset, well above align_threshold_m=0.3
    vertiport_detector::VertiportObservation off{520.0, 240.0, 80.0, 0.0, true};
    filter.update(off);
    filter.update(off);
    filter.update(off);

    const auto s = filter.state();
    assert(s.acquired);
    assert(!s.can_descend);
    assert(s.center_error_x_m > 1.0);
}

void testMissingFramesDecayConfidence() {
    vertiport_detector::DescentFilter filter(defaultParams());

    // acquire first
    filter.update(centeredObs(0.0));
    filter.update(centeredObs(0.1));
    filter.update(centeredObs(0.2));
    assert(filter.state().acquired);

    // missing_decay=0.5: after 4 decays confidence < threshold
    filter.decayMissing();
    filter.decayMissing();
    filter.decayMissing();
    filter.decayMissing();

    assert(!filter.state().acquired);
    assert(!filter.state().can_descend);
}

void testInvalidObsFallsThroughToDecay() {
    vertiport_detector::DescentFilter filter(defaultParams());

    filter.update(centeredObs(0.0));
    filter.update(centeredObs(0.1));
    filter.update(centeredObs(0.2));
    assert(filter.state().acquired);

    // valid=false observation should decay, not update center error
    vertiport_detector::VertiportObservation no_det{0.0, 0.0, 0.0, 0.3, false};
    filter.update(no_det);
    filter.update(no_det);
    filter.update(no_det);
    filter.update(no_det);

    assert(!filter.state().acquired);
}

void testResetClearsState() {
    vertiport_detector::DescentFilter filter(defaultParams());

    filter.update(centeredObs(0.0));
    filter.update(centeredObs(0.1));
    filter.update(centeredObs(0.2));
    assert(filter.state().acquired);

    filter.reset();
    const auto s = filter.state();
    assert(!s.acquired);
    assert(!s.can_descend);
    assert(s.confidence == 0.0);
}

void testComputeCenterErrorMath() {
    // 640x480, alt=2m, FOV=110deg
    // ground_width = 2 * 2 * tan(55°) ≈ 5.712m → mpp ≈ 0.008925 m/px
    // obs at (420, 240): error_x = (420-320)*mpp ≈ 0.893m, error_y = 0
    vertiport_detector::VertiportObservation obs{420.0, 240.0, 0.0, 0.0, true};
    const auto [ex, ey] = vertiport_detector::computeCenterError(obs, 640, 480, 2.0, 110.0);
    assert(std::abs(ey) < 1e-9);
    assert(ex > 0.8 && ex < 1.0);
}

}  // namespace

int main() {
    testCenteredDetectionBuildsConfidenceAndAcquires();
    testOffCenterBlocksDescent();
    testMissingFramesDecayConfidence();
    testInvalidObsFallsThroughToDecay();
    testResetClearsState();
    testComputeCenterErrorMath();
    std::cout << "vertiport_detector_core_test passed\n";
    return 0;
}
