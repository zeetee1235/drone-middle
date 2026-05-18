#include "mission_manager/mission_core.hpp"

#include <cassert>
#include <iostream>
#include <vector>

namespace {

mission_manager::MarkerInfo marker(int id, double t) {
    mission_manager::MarkerInfo info;
    info.id = id;
    info.grid_x_m = static_cast<double>(id);
    info.grid_y_m = static_cast<double>(id + 1);
    info.home_x_m = static_cast<double>(id + 2);
    info.home_y_m = static_cast<double>(id + 3);
    info.confidence = 0.9;
    info.timestamp_sec = t;
    return info;
}

mission_manager::TickInput tickAt(double t) {
    mission_manager::TickInput input;
    input.now_sec = t;
    return input;
}

void bootToGridSearch(mission_manager::MissionCore& core) {
    mission_manager::TickInput input;
    input.now_sec = 0.0;
    input.mission_start = true;
    assert(core.tick(input) == mission_manager::MissionPhase::Takeoff);

    input = {};
    input.now_sec = 1.0;
    input.takeoff_complete = true;
    assert(core.tick(input) == mission_manager::MissionPhase::HomeInit);

    input = {};
    input.now_sec = 2.0;
    input.home_initialized = true;
    assert(core.tick(input) == mission_manager::MissionPhase::GridSearch);
}

void confirmOneMarker(mission_manager::MissionCore& core, int id, double start_time) {
    mission_manager::TickInput input;
    input.now_sec = start_time;
    input.marker_candidate = true;
    assert(core.tick(input) == mission_manager::MissionPhase::MarkerApproach);

    input = {};
    input.now_sec = start_time + 0.1;
    input.perception_stable = true;
    assert(core.tick(input) == mission_manager::MissionPhase::AntiSway);

    input = {};
    input.now_sec = start_time + 0.2;
    input.confirmed_marker = marker(id, input.now_sec);
    assert(core.tick(input) == mission_manager::MissionPhase::HoverConfirm);

    input = {};
    input.now_sec = start_time + 2.0;
    assert(core.tick(input) == mission_manager::MissionPhase::HoverConfirm);

    input = {};
    input.now_sec = start_time + 3.3;
    assert(core.tick(input) == mission_manager::MissionPhase::MarkerSave);
}

void testStartupAndMarkerSave() {
    mission_manager::MissionCore core;
    bootToGridSearch(core);
    confirmOneMarker(core, 3, 10.0);

    auto snapshot = core.snapshot();
    assert(snapshot.confirmed_markers.size() == 1);
    assert(snapshot.confirmed_markers[0].id == 3);

    mission_manager::TickInput input;
    input.now_sec = 14.0;
    assert(core.tick(input) == mission_manager::MissionPhase::GridSearch);
}

void testRescueOrderIsDescending() {
    mission_manager::MissionCore core;
    bootToGridSearch(core);

    confirmOneMarker(core, 3, 10.0);
    assert(core.tick(tickAt(14.0)) == mission_manager::MissionPhase::GridSearch);
    confirmOneMarker(core, 8, 20.0);
    assert(core.tick(tickAt(24.0)) == mission_manager::MissionPhase::GridSearch);
    confirmOneMarker(core, 1, 30.0);
    assert(core.tick(tickAt(34.0)) == mission_manager::MissionPhase::GridSearch);
    confirmOneMarker(core, 5, 40.0);

    mission_manager::TickInput input;
    input.now_sec = 44.0;
    assert(core.tick(input) == mission_manager::MissionPhase::RescueRoutePlan);
    input.now_sec = 44.1;
    assert(core.tick(input) == mission_manager::MissionPhase::RescueVisit);

    const auto snapshot = core.snapshot();
    const std::vector<int> expected{8, 5, 3, 1};
    assert(snapshot.rescue_order == expected);
    assert(snapshot.active_rescue_marker_id.has_value());
    assert(*snapshot.active_rescue_marker_id == 8);
}

void testRescueVisitRequiresHoverThenReturnsHome() {
    mission_manager::MissionCore core;
    bootToGridSearch(core);

    for (const auto [id, t] : std::vector<std::pair<int, double>>{
             {3, 10.0}, {8, 20.0}, {1, 30.0}, {5, 40.0}}) {
        confirmOneMarker(core, id, t);
        mission_manager::TickInput advance;
        advance.now_sec = t + 4.0;
        core.tick(advance);
    }
    mission_manager::TickInput input;
    input.now_sec = 50.0;
    core.tick(input);
    input.now_sec = 50.1;
    assert(core.tick(input) == mission_manager::MissionPhase::RescueVisit);

    double t = 60.0;
    for (int expected_id : {8, 5, 3, 1}) {
        input = {};
        input.now_sec = t;
        input.target_reached = true;
        assert(core.tick(input) == mission_manager::MissionPhase::RescueVisit);

        input.now_sec = t + 3.1;
        input.target_reached = true;
        const auto phase = core.tick(input);
        if (expected_id != 1) {
            assert(phase == mission_manager::MissionPhase::RescueVisit);
        }
        t += 10.0;
    }

    assert(core.phase() == mission_manager::MissionPhase::ReturnHome);
}

void testEmergencyReturnAndLanding() {
    mission_manager::MissionCore core;
    bootToGridSearch(core);

    mission_manager::TickInput input;
    input.now_sec = 5.0;
    input.battery_percent = 19.0;
    assert(core.tick(input) == mission_manager::MissionPhase::EmergencyReturn);

    input = {};
    input.now_sec = 6.0;
    input.target_reached = true;
    assert(core.tick(input) == mission_manager::MissionPhase::VertiportAcquire);

    input = {};
    input.now_sec = 7.0;
    input.vertiport_acquired = true;
    assert(core.tick(input) == mission_manager::MissionPhase::VisionServoLand);

    input = {};
    input.now_sec = 8.0;
    input.landing_complete = true;
    assert(core.tick(input) == mission_manager::MissionPhase::Landed);
}

}  // namespace

int main() {
    testStartupAndMarkerSave();
    testRescueOrderIsDescending();
    testRescueVisitRequiresHoverThenReturnsHome();
    testEmergencyReturnAndLanding();
    std::cout << "mission_manager_core_test passed\n";
    return 0;
}
