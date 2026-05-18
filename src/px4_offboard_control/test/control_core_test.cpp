#include "px4_offboard_control/control_core.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

void testModeMapping() {
    assert(px4_offboard_control::modeFromMissionState("GRID_SEARCH")
        == px4_offboard_control::ControlMode::Sprint);
    assert(px4_offboard_control::modeFromMissionState("HOVER_CONFIRM")
        == px4_offboard_control::ControlMode::HoverConfirm);
    assert(px4_offboard_control::modeFromMissionState("VISION_SERVO_LAND")
        == px4_offboard_control::ControlMode::VisionServo);
}

void testVelocityClampPreservesDirection() {
    px4_offboard_control::ModeLimits limits;
    limits.v_max_xy = 2.0;
    limits.v_max_z = 0.4;

    const auto out = px4_offboard_control::clampVelocity({3.0, 4.0, 1.0}, limits);
    assert(near(std::sqrt(out.x * out.x + out.y * out.y), 2.0));
    assert(near(out.z, 0.4));
}

void testAntiSwayOverridesSprint() {
    auto params = px4_offboard_control::defaultParams();
    px4_offboard_control::ControllerState state;
    px4_offboard_control::ControllerInput input;
    input.mission_state = "GRID_SEARCH";
    input.planner_velocity_mps = {3.0, 0.0, 0.0};
    input.current_altitude_m = params.target_altitude_m;
    input.sway_metric = 1.0;
    input.dt_sec = 0.1;

    const auto out = px4_offboard_control::stepController(input, state, params);
    assert(out.mode == px4_offboard_control::ControlMode::AntiSway);
    assert(near(out.velocity_mps.x, 0.0));
    assert(near(out.velocity_mps.y, 0.0));
}

void testMarkerCandidateSwitchesSprintToApproach() {
    auto params = px4_offboard_control::defaultParams();
    px4_offboard_control::ControllerState state;
    px4_offboard_control::ControllerInput input;
    input.mission_state = "GRID_SEARCH";
    input.planner_velocity_mps = {3.0, 0.0, 0.0};
    input.current_altitude_m = params.target_altitude_m;
    input.marker_candidate = true;
    input.dt_sec = 1.0;

    const auto out = px4_offboard_control::stepController(input, state, params);
    assert(out.mode == px4_offboard_control::ControlMode::Approach);
    assert(out.velocity_mps.x <= 1.0);
}

void testVisionServoClamp() {
    auto params = px4_offboard_control::defaultParams();
    const auto vel = px4_offboard_control::visionServoVelocity({200.0, -200.0, 0.0}, params);
    assert(near(vel.x, 0.3));
    assert(near(vel.y, -0.3));
}

void testTakeoffUsesNedClimbVelocity() {
    auto params = px4_offboard_control::defaultParams();
    px4_offboard_control::ControllerState state;
    px4_offboard_control::ControllerInput input;
    input.mission_state = "TAKEOFF";
    input.current_altitude_m = 0.7;
    input.dt_sec = 1.0;

    const auto out = px4_offboard_control::stepController(input, state, params);
    assert(out.mode == px4_offboard_control::ControlMode::Takeoff);
    assert(out.velocity_mps.z < 0.0);
}

void testGridSearchBelowTargetAltitudeStaysInTakeoff() {
    auto params = px4_offboard_control::defaultParams();
    px4_offboard_control::ControllerState state;
    px4_offboard_control::ControllerInput input;
    input.mission_state = "GRID_SEARCH";
    input.planner_velocity_mps = {3.0, 0.0, 0.0};
    input.current_altitude_m = 0.9;
    input.dt_sec = 1.0;

    const auto out = px4_offboard_control::stepController(input, state, params);
    assert(out.mode == px4_offboard_control::ControlMode::Takeoff);
    assert(near(out.velocity_mps.x, 0.0));
    assert(near(out.velocity_mps.y, 0.0));
    assert(out.velocity_mps.z < 0.0);
}

void testGridSearchAtTargetAltitudeCanSprint() {
    auto params = px4_offboard_control::defaultParams();
    px4_offboard_control::ControllerState state;
    px4_offboard_control::ControllerInput input;
    input.mission_state = "GRID_SEARCH";
    input.planner_velocity_mps = {3.0, 0.0, 0.0};
    input.current_altitude_m = params.target_altitude_m;
    input.dt_sec = 1.0;

    const auto out = px4_offboard_control::stepController(input, state, params);
    assert(out.mode == px4_offboard_control::ControlMode::Sprint);
    assert(out.velocity_mps.x > 0.0);
}

void testLandingUsesNedDescendVelocity() {
    auto params = px4_offboard_control::defaultParams();
    px4_offboard_control::ControllerState state;
    px4_offboard_control::ControllerInput input;
    input.mission_state = "LANDED";
    input.current_altitude_m = 2.0;
    input.dt_sec = 1.0;

    const auto out = px4_offboard_control::stepController(input, state, params);
    assert(out.mode == px4_offboard_control::ControlMode::Landing);
    assert(out.velocity_mps.z > 0.0);
}

}  // namespace

int main() {
    testModeMapping();
    testVelocityClampPreservesDirection();
    testAntiSwayOverridesSprint();
    testMarkerCandidateSwitchesSprintToApproach();
    testVisionServoClamp();
    testTakeoffUsesNedClimbVelocity();
    testGridSearchBelowTargetAltitudeStaysInTakeoff();
    testGridSearchAtTargetAltitudeCanSprint();
    testLandingUsesNedDescendVelocity();
    std::cout << "px4_offboard_control_core_test passed\n";
    return 0;
}
