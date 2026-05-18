#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace sprint_planner {

struct DroneParams {
    double v_max_mps = 3.0;
    double a_max_mps2 = 2.0;
    double a_brake_mps2 = 2.5;
    double t_turn_90_sec = 1.5;
    double t_turn_180_sec = 2.5;
    double t_marker_check_sec = 2.0;
    double t_hover_sec = 3.0;
    double t_stabilize_sec = 0.5;
};

struct CostWeights {
    double time = 1.0;
    double turn = 0.5;
    double stop = 0.35;
    double risk = 1.0;
    double altitude = 1.0;
    double coverage = 2.0;
    double marker_likelihood = 1.5;
};

struct Segment {
    double length_m = 0.0;
    double v0_mps = 0.0;
    double vf_mps = 0.0;
    double turn_degrees_after = 0.0;
    bool marker_check = false;
    bool confirmed_hover = false;
    bool forced_stop = false;
};

struct RouteCandidate {
    std::string name;
    std::vector<Segment> segments;
    double coverage_gain = 0.0;
    double marker_likelihood = 0.0;
    double risk = 0.0;
    double altitude_penalty = 0.0;
};

struct SegmentTiming {
    double total_sec = 0.0;
    double peak_speed_mps = 0.0;
    double accel_sec = 0.0;
    double cruise_sec = 0.0;
    double brake_sec = 0.0;
    bool reaches_v_max = false;
};

struct RouteCostBreakdown {
    double motion_time_sec = 0.0;
    double turn_time_sec = 0.0;
    double marker_time_sec = 0.0;
    double hover_time_sec = 0.0;
    double stabilize_time_sec = 0.0;
    double weighted_cost = 0.0;
    int turn_count = 0;
    int stop_count = 0;
};

double brakingDistance(double speed_mps, double brake_mps2);
double accelerationDistance(double from_mps, double to_mps, double accel_mps2);
double clampSpeed(double value_mps, const DroneParams& params);
double turnCostSeconds(double turn_degrees, const DroneParams& params);

SegmentTiming timeCostStraight(
    double length_m,
    double v0_mps,
    double vf_mps,
    const DroneParams& params);

RouteCostBreakdown evaluateRoute(
    const RouteCandidate& route,
    const DroneParams& params,
    const CostWeights& weights = {});

std::size_t chooseBestRoute(
    const std::vector<RouteCandidate>& candidates,
    const DroneParams& params,
    const CostWeights& weights = {});

}  // namespace sprint_planner
