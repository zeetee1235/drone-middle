#include "sprint_planner/time_cost.hpp"

#include <algorithm>
#include <limits>

namespace sprint_planner {

namespace {

void validatePositive(const char* name, double value) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        throw std::invalid_argument(std::string(name) + " must be positive and finite");
    }
}

double normalizedTurn(double turn_degrees) {
    double a = std::fmod(std::abs(turn_degrees), 360.0);
    if (a > 180.0) {
        a = 360.0 - a;
    }
    return a;
}

}  // namespace

double brakingDistance(double speed_mps, double brake_mps2) {
    validatePositive("brake_mps2", brake_mps2);
    const double v = std::max(0.0, speed_mps);
    return (v * v) / (2.0 * brake_mps2);
}

double accelerationDistance(double from_mps, double to_mps, double accel_mps2) {
    validatePositive("accel_mps2", accel_mps2);
    const double v0 = std::max(0.0, from_mps);
    const double v1 = std::max(v0, to_mps);
    return (v1 * v1 - v0 * v0) / (2.0 * accel_mps2);
}

double clampSpeed(double value_mps, const DroneParams& params) {
    validatePositive("v_max_mps", params.v_max_mps);
    return std::clamp(value_mps, 0.0, params.v_max_mps);
}

double turnCostSeconds(double turn_degrees, const DroneParams& params) {
    const double turn = normalizedTurn(turn_degrees);
    if (turn < 1.0) {
        return 0.0;
    }
    validatePositive("t_turn_90_sec", params.t_turn_90_sec);
    validatePositive("t_turn_180_sec", params.t_turn_180_sec);

    if (turn <= 90.0) {
        return params.t_turn_90_sec * (turn / 90.0);
    }

    const double extra = (turn - 90.0) / 90.0;
    return params.t_turn_90_sec
         + (params.t_turn_180_sec - params.t_turn_90_sec) * extra;
}

SegmentTiming timeCostStraight(
    double length_m,
    double v0_mps,
    double vf_mps,
    const DroneParams& params)
{
    if (length_m < 0.0 || !std::isfinite(length_m)) {
        throw std::invalid_argument("length_m must be non-negative and finite");
    }
    validatePositive("v_max_mps", params.v_max_mps);
    validatePositive("a_max_mps2", params.a_max_mps2);
    validatePositive("a_brake_mps2", params.a_brake_mps2);

    SegmentTiming result;
    if (length_m == 0.0) {
        result.peak_speed_mps = std::max(v0_mps, vf_mps);
        return result;
    }

    const double v0 = clampSpeed(v0_mps, params);
    const double vf = clampSpeed(vf_mps, params);
    const double vmax = params.v_max_mps;

    const double d_accel = (vmax * vmax - v0 * v0) / (2.0 * params.a_max_mps2);
    const double d_brake = (vmax * vmax - vf * vf) / (2.0 * params.a_brake_mps2);

    if (d_accel + d_brake <= length_m) {
        const double cruise_dist = length_m - d_accel - d_brake;
        result.reaches_v_max = true;
        result.peak_speed_mps = vmax;
        result.accel_sec = (vmax - v0) / params.a_max_mps2;
        result.cruise_sec = cruise_dist / vmax;
        result.brake_sec = (vmax - vf) / params.a_brake_mps2;
        result.total_sec = result.accel_sec + result.cruise_sec + result.brake_sec;
        return result;
    }

    const double numerator = 2.0 * params.a_max_mps2 * params.a_brake_mps2 * length_m
        + params.a_max_mps2 * vf * vf
        + params.a_brake_mps2 * v0 * v0;
    const double denominator = params.a_max_mps2 + params.a_brake_mps2;
    const double peak = std::sqrt(std::max(0.0, numerator / denominator));

    result.reaches_v_max = false;
    result.peak_speed_mps = std::min(peak, vmax);
    result.accel_sec = std::max(0.0, (result.peak_speed_mps - v0) / params.a_max_mps2);
    result.brake_sec = std::max(0.0, (result.peak_speed_mps - vf) / params.a_brake_mps2);
    result.total_sec = result.accel_sec + result.brake_sec;
    return result;
}

RouteCostBreakdown evaluateRoute(
    const RouteCandidate& route,
    const DroneParams& params,
    const CostWeights& weights)
{
    RouteCostBreakdown out;

    for (const auto& segment : route.segments) {
        const auto timing = timeCostStraight(
            segment.length_m, segment.v0_mps, segment.vf_mps, params);
        out.motion_time_sec += timing.total_sec;

        const double turn_time = turnCostSeconds(segment.turn_degrees_after, params);
        if (turn_time > 0.0) {
            out.turn_time_sec += turn_time;
            out.turn_count += 1;
        }

        if (segment.marker_check) {
            out.marker_time_sec += params.t_marker_check_sec;
            out.stabilize_time_sec += params.t_stabilize_sec;
        }
        if (segment.confirmed_hover) {
            out.hover_time_sec += params.t_hover_sec;
        }
        if (segment.forced_stop || segment.vf_mps < 0.05) {
            out.stop_count += 1;
        }
    }

    const double raw_time = out.motion_time_sec
        + out.turn_time_sec
        + out.marker_time_sec
        + out.hover_time_sec
        + out.stabilize_time_sec;

    out.weighted_cost = weights.time * raw_time
        + weights.turn * static_cast<double>(out.turn_count)
        + weights.stop * static_cast<double>(out.stop_count)
        + weights.risk * route.risk
        + weights.altitude * route.altitude_penalty
        - weights.coverage * route.coverage_gain
        - weights.marker_likelihood * route.marker_likelihood;

    return out;
}

std::size_t chooseBestRoute(
    const std::vector<RouteCandidate>& candidates,
    const DroneParams& params,
    const CostWeights& weights)
{
    if (candidates.empty()) {
        throw std::invalid_argument("candidates must not be empty");
    }

    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t best_index = 0;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto breakdown = evaluateRoute(candidates[i], params, weights);
        if (breakdown.weighted_cost < best_cost) {
            best_cost = breakdown.weighted_cost;
            best_index = i;
        }
    }
    return best_index;
}

}  // namespace sprint_planner
