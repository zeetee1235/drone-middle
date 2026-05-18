#pragma once

#include <map>
#include <string>

namespace px4_offboard_control {

enum class ControlMode {
    Takeoff,
    Sprint,
    Approach,
    AntiSway,
    HoverConfirm,
    Return,
    VisionServo,
    Landing,
    Hold,
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ModeLimits {
    double v_max_xy = 1.0;
    double v_max_z = 0.3;
    double a_max_xy = 1.0;
    double jerk_max = 2.0;
    double tilt_max_deg = 15.0;
};

struct ControllerParams {
    double target_altitude_m = 2.0;
    double sway_threshold = 0.3;
    double vision_servo_gain_xy = 0.003;
    double vision_servo_max_xy = 0.3;
    std::map<ControlMode, ModeLimits> limits;
};

struct ControllerInput {
    std::string mission_state;
    Vec3 planner_velocity_mps;
    Vec3 vertiport_center_error_px;
    double current_altitude_m = 0.0;
    double sway_metric = 0.0;
    bool marker_candidate = false;
    bool vertiport_error_valid = false;
    double dt_sec = 0.05;
};

struct ControllerState {
    ControlMode mode = ControlMode::Hold;
    Vec3 last_velocity_mps;
};

struct ControllerOutput {
    ControlMode mode = ControlMode::Hold;
    Vec3 velocity_mps;
    double target_altitude_m = 2.0;
    bool offboard_enabled = true;
};

std::string toString(ControlMode mode);
ControlMode modeFromMissionState(const std::string& mission_state);
ControllerParams defaultParams();
Vec3 clampVelocity(const Vec3& velocity, const ModeLimits& limits);
Vec3 rateLimitVelocity(const Vec3& desired, const Vec3& previous, const ModeLimits& limits, double dt_sec);
Vec3 visionServoVelocity(const Vec3& center_error_px, const ControllerParams& params);
ControllerOutput stepController(
    const ControllerInput& input,
    ControllerState& state,
    const ControllerParams& params = defaultParams());

}  // namespace px4_offboard_control
