#include "px4_offboard_control/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_control {

namespace {

constexpr double TAKEOFF_ALTITUDE_GATE_M = 0.3;

double clamp(double value, double limit) {
    const double lim = std::abs(limit);
    return std::clamp(value, -lim, lim);
}

double normXY(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

Vec3 scaleXY(const Vec3& v, double max_xy) {
    const double n = normXY(v);
    if (n <= max_xy || n <= 1e-9) {
        return v;
    }
    const double s = max_xy / n;
    return {v.x * s, v.y * s, v.z};
}

double altitudeHoldVelocityNed(double current_altitude_m, double target_altitude_m, double max_vz) {
    // PX4 trajectory velocity uses NED: z < 0 climbs, z > 0 descends.
    const double error_m = current_altitude_m - target_altitude_m;
    if (std::abs(error_m) < 0.08) {
        return 0.0;
    }
    return clamp(error_m, max_vz);
}

}  // namespace

std::string toString(ControlMode mode) {
    switch (mode) {
    case ControlMode::Takeoff:
        return "TAKEOFF";
    case ControlMode::Sprint:
        return "SPRINT";
    case ControlMode::Approach:
        return "APPROACH";
    case ControlMode::AntiSway:
        return "ANTI_SWAY";
    case ControlMode::HoverConfirm:
        return "HOVER_CONFIRM";
    case ControlMode::Return:
        return "RETURN";
    case ControlMode::VisionServo:
        return "VISION_SERVO";
    case ControlMode::Landing:
        return "LANDING";
    case ControlMode::Hold:
        return "HOLD";
    }
    return "HOLD";
}

ControlMode modeFromMissionState(const std::string& mission_state) {
    if (mission_state == "TAKEOFF" || mission_state == "HOME_INIT") {
        return ControlMode::Takeoff;
    }
    if (mission_state == "GRID_SEARCH" || mission_state == "RESCUE_VISIT") {
        return ControlMode::Sprint;
    }
    if (mission_state == "MARKER_APPROACH" || mission_state == "VERTIPORT_ACQUIRE") {
        return ControlMode::Approach;
    }
    if (mission_state == "ANTI_SWAY") {
        return ControlMode::AntiSway;
    }
    if (mission_state == "HOVER_CONFIRM" || mission_state == "MARKER_SAVE") {
        return ControlMode::HoverConfirm;
    }
    if (mission_state == "RETURN_HOME" || mission_state == "EMERGENCY_RETURN") {
        return ControlMode::Return;
    }
    if (mission_state == "VISION_SERVO_LAND") {
        return ControlMode::VisionServo;
    }
    if (mission_state == "LANDED") {
        return ControlMode::Landing;
    }
    return ControlMode::Hold;
}

ControllerParams defaultParams() {
    ControllerParams params;
    params.target_altitude_m = 2.0;
    params.sway_threshold = 0.3;
    params.vision_servo_gain_xy = 0.003;
    params.vision_servo_max_xy = 0.3;
    params.limits = {
        {ControlMode::Takeoff, {0.3, 0.4, 0.5, 1.0, 10.0}},
        {ControlMode::Sprint, {3.0, 0.3, 2.0, 5.0, 25.0}},
        {ControlMode::Approach, {1.0, 0.25, 1.0, 2.0, 15.0}},
        {ControlMode::AntiSway, {0.5, 0.2, 0.5, 1.0, 10.0}},
        {ControlMode::HoverConfirm, {0.1, 0.15, 0.2, 0.5, 5.0}},
        {ControlMode::Return, {2.0, 0.3, 1.5, 3.0, 20.0}},
        {ControlMode::VisionServo, {0.3, 0.2, 0.3, 1.0, 8.0}},
        {ControlMode::Landing, {0.1, 0.2, 0.2, 0.5, 5.0}},
        {ControlMode::Hold, {0.1, 0.1, 0.2, 0.5, 5.0}},
    };
    return params;
}

Vec3 clampVelocity(const Vec3& velocity, const ModeLimits& limits) {
    Vec3 out = scaleXY(velocity, limits.v_max_xy);
    out.z = clamp(out.z, limits.v_max_z);
    return out;
}

Vec3 rateLimitVelocity(const Vec3& desired, const Vec3& previous, const ModeLimits& limits, double dt_sec) {
    if (!(dt_sec > 0.0) || !std::isfinite(dt_sec)) {
        throw std::invalid_argument("dt_sec must be positive and finite");
    }
    const double max_delta_xy = limits.a_max_xy * dt_sec;
    Vec3 delta{desired.x - previous.x, desired.y - previous.y, desired.z - previous.z};
    const double delta_xy = normXY(delta);
    if (delta_xy > max_delta_xy && delta_xy > 1e-9) {
        const double s = max_delta_xy / delta_xy;
        delta.x *= s;
        delta.y *= s;
    }
    delta.z = clamp(delta.z, limits.v_max_z * dt_sec);
    return {
        previous.x + delta.x,
        previous.y + delta.y,
        previous.z + delta.z,
    };
}

Vec3 visionServoVelocity(const Vec3& center_error_px, const ControllerParams& params) {
    return {
        clamp(params.vision_servo_gain_xy * center_error_px.x, params.vision_servo_max_xy),
        clamp(params.vision_servo_gain_xy * center_error_px.y, params.vision_servo_max_xy),
        0.0,
    };
}

ControllerOutput stepController(
    const ControllerInput& input,
    ControllerState& state,
    const ControllerParams& params)
{
    ControlMode requested = modeFromMissionState(input.mission_state);

    if (requested != ControlMode::Landing
        && input.current_altitude_m < params.target_altitude_m - TAKEOFF_ALTITUDE_GATE_M) {
        requested = ControlMode::Takeoff;
    }

    if ((requested == ControlMode::Sprint || requested == ControlMode::Approach)
        && input.sway_metric > params.sway_threshold) {
        requested = ControlMode::AntiSway;
    }
    if (input.marker_candidate && requested == ControlMode::Sprint) {
        requested = ControlMode::Approach;
    }

    const auto limit_iter = params.limits.find(requested);
    const ModeLimits limits = limit_iter == params.limits.end()
        ? defaultParams().limits.at(ControlMode::Hold)
        : limit_iter->second;

    Vec3 desired = input.planner_velocity_mps;
    desired.z = altitudeHoldVelocityNed(
        input.current_altitude_m,
        params.target_altitude_m,
        limits.v_max_z);

    if (requested == ControlMode::Takeoff) {
        desired.x = 0.0;
        desired.y = 0.0;
    } else if (requested == ControlMode::HoverConfirm
               || requested == ControlMode::AntiSway
               || requested == ControlMode::Hold) {
        desired.x = 0.0;
        desired.y = 0.0;
    } else if (requested == ControlMode::VisionServo) {
        const double vz = desired.z;
        desired = input.vertiport_error_valid
            ? visionServoVelocity(input.vertiport_center_error_px, params)
            : Vec3{0.0, 0.0, 0.0};
        desired.z = vz;
    } else if (requested == ControlMode::Landing) {
        desired = {0.0, 0.0, 0.15};
    }

    const auto clamped = clampVelocity(desired, limits);
    const auto limited = rateLimitVelocity(clamped, state.last_velocity_mps, limits, input.dt_sec);
    state.mode = requested;
    state.last_velocity_mps = limited;

    return {
        requested,
        limited,
        params.target_altitude_m,
        requested != ControlMode::Landing,
    };
}

}  // namespace px4_offboard_control
