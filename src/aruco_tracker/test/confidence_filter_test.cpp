#include "aruco_tracker/confidence_filter.hpp"

#include <cassert>
#include <iostream>

namespace {

aruco_tracker::MarkerObservation obsAt(double t, double x = 100.0, double y = 120.0) {
    return {7, x, y, 40.0, 0.0, t};
}

void testStableObservationsConfirmMarker() {
    aruco_tracker::ConfidenceParams params;
    params.confirm_threshold = 0.6;
    params.confidence_increment = 0.2;
    aruco_tracker::ConfidenceFilter filter(params);

    filter.update(obsAt(0.0), 0.0);
    filter.update(obsAt(0.1, 102.0, 121.0), 0.0);
    filter.update(obsAt(0.2, 101.0, 120.0), 0.0);

    const auto confirmed = filter.confirmedStates();
    assert(confirmed.size() == 1);
    assert(confirmed[0].id == 7);
    assert(confirmed[0].status == aruco_tracker::MarkerStatus::Confirmed);
}

void testHighSwaySlowsConfidenceGrowth() {
    aruco_tracker::ConfidenceParams params;
    params.confirm_threshold = 0.6;
    params.confidence_increment = 0.2;
    params.sway_threshold = 0.3;
    aruco_tracker::ConfidenceFilter filter(params);

    filter.update(obsAt(0.0), 1.0);
    filter.update(obsAt(0.1), 1.0);
    filter.update(obsAt(0.2), 1.0);

    assert(filter.confirmedStates().empty());
    assert(!filter.candidateStates().empty());
}

void testMissingDecayCanDemoteConfirmedMarker() {
    aruco_tracker::ConfidenceParams params;
    params.confirm_threshold = 0.6;
    params.confidence_increment = 0.3;
    params.missing_decay = 0.5;
    aruco_tracker::ConfidenceFilter filter(params);

    filter.update(obsAt(0.0), 0.0);
    filter.update(obsAt(0.1), 0.0);
    assert(filter.confirmedStates().size() == 1);

    filter.decayMissing(1.0);
    assert(filter.confirmedStates().empty());
}

}  // namespace

int main() {
    testStableObservationsConfirmMarker();
    testHighSwaySlowsConfidenceGrowth();
    testMissingDecayCanDemoteConfirmedMarker();
    std::cout << "aruco_tracker_core_test passed\n";
    return 0;
}
