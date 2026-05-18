#include "sprint_planner/time_cost.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

void testBrakingDistance() {
    assert(near(sprint_planner::brakingDistance(3.0, 2.5), 1.8));
}

void testTrapezoidProfile() {
    sprint_planner::DroneParams p;
    const auto timing = sprint_planner::timeCostStraight(12.0, 0.0, 0.0, p);

    assert(timing.reaches_v_max);
    assert(near(timing.peak_speed_mps, 3.0));
    assert(near(timing.accel_sec, 1.5));
    assert(near(timing.brake_sec, 1.2));
    assert(near(timing.cruise_sec, 2.65));
    assert(near(timing.total_sec, 5.35));
}

void testTriangularProfile() {
    sprint_planner::DroneParams p;
    const auto timing = sprint_planner::timeCostStraight(3.0, 0.0, 0.0, p);

    assert(!timing.reaches_v_max);
    assert(timing.peak_speed_mps > 0.0);
    assert(timing.peak_speed_mps < p.v_max_mps);
    assert(timing.total_sec > 0.0);
}

void testLongerStraightCanBeatShorterZigzag() {
    sprint_planner::DroneParams p;
    sprint_planner::CostWeights w;
    w.coverage = 0.0;
    w.marker_likelihood = 0.0;

    sprint_planner::RouteCandidate short_zigzag;
    short_zigzag.name = "short_zigzag";
    short_zigzag.segments = {
        {3.0, 0.0, 0.5, 90.0, false, false, false},
        {3.0, 0.5, 0.5, 90.0, false, false, false},
        {3.0, 0.5, 0.0, 90.0, false, false, true},
    };

    sprint_planner::RouteCandidate longer_straight;
    longer_straight.name = "longer_straight";
    longer_straight.segments = {
        {12.0, 0.0, 0.0, 90.0, false, false, true},
    };

    const std::vector<sprint_planner::RouteCandidate> candidates{
        short_zigzag,
        longer_straight,
    };

    const auto best = sprint_planner::chooseBestRoute(candidates, p, w);
    assert(best == 1);

    const auto zigzag_cost = sprint_planner::evaluateRoute(short_zigzag, p, w);
    const auto straight_cost = sprint_planner::evaluateRoute(longer_straight, p, w);
    std::cout << "short_zigzag cost=" << zigzag_cost.weighted_cost
              << " longer_straight cost=" << straight_cost.weighted_cost << '\n';
}

}  // namespace

int main() {
    testBrakingDistance();
    testTrapezoidProfile();
    testTriangularProfile();
    testLongerStraightCanBeatShorterZigzag();
    std::cout << "sprint_planner_core_test passed\n";
    return 0;
}
