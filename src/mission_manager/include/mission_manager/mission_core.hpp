#pragma once

#include <optional>
#include <string>
#include <vector>

namespace mission_manager {

enum class MissionPhase {
    Init,
    Takeoff,
    HomeInit,
    GridSearch,
    MarkerApproach,
    AntiSway,
    HoverConfirm,
    MarkerSave,
    RescueRoutePlan,
    RescueVisit,
    ReturnHome,
    VertiportAcquire,
    VisionServoLand,
    Landed,
    EmergencyReturn,
    Abort,
};

struct MissionParams {
    int required_marker_count = 4;
    double hover_confirm_duration_sec = 3.0;
    double battery_return_threshold_pct = 20.0;
    double max_drift_before_stabilize_m = 0.5;
    double home_approach_radius_m = 5.0;
    double vertiport_detect_radius_m = 3.0;
};

struct MarkerInfo {
    int id = -1;
    double grid_x_m = 0.0;
    double grid_y_m = 0.0;
    double home_x_m = 0.0;
    double home_y_m = 0.0;
    double confidence = 0.0;
    double timestamp_sec = 0.0;
};

struct TickInput {
    double now_sec = 0.0;
    bool mission_start = false;
    bool abort = false;
    bool takeoff_complete = false;
    bool home_initialized = false;
    bool marker_candidate = false;
    bool perception_stable = false;
    std::optional<MarkerInfo> confirmed_marker;
    bool target_reached = false;
    bool vertiport_acquired = false;
    bool landing_complete = false;
    double battery_percent = 100.0;
    double drift_metric_m = 0.0;
};

struct MissionSnapshot {
    MissionPhase phase = MissionPhase::Init;
    std::vector<MarkerInfo> confirmed_markers;
    std::vector<int> rescue_order;
    std::vector<int> rescue_visited;
    std::optional<int> active_rescue_marker_id;
    std::optional<MarkerInfo> active_target_marker;
};

std::string toString(MissionPhase phase);

class MissionCore {
public:
    explicit MissionCore(MissionParams params = {});

    MissionPhase phase() const;
    MissionSnapshot snapshot() const;
    MissionPhase tick(const TickInput& input);
    void reset();

private:
    bool isTerminal() const;
    bool shouldEmergencyReturn(const TickInput& input) const;
    void saveMarker(const MarkerInfo& marker);
    void buildRescueOrder();
    std::optional<MarkerInfo> markerById(int id) const;
    void startHover(double now_sec);
    bool hoverElapsed(double now_sec) const;
    void updateRescueVisit(const TickInput& input);

    MissionParams params_;
    MissionPhase phase_ = MissionPhase::Init;
    std::vector<MarkerInfo> confirmed_markers_;
    std::vector<int> rescue_order_;
    std::vector<int> rescue_visited_;
    std::optional<MarkerInfo> pending_marker_;
    std::optional<double> hover_start_sec_;
    std::optional<double> rescue_hover_start_sec_;
    std::size_t rescue_index_ = 0;
};

}  // namespace mission_manager
