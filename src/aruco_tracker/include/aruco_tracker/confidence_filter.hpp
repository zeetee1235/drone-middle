#pragma once

#include <map>
#include <string>
#include <vector>

namespace aruco_tracker {

enum class MarkerStatus {
    Candidate,
    Tracking,
    Confirmed,
    Visited,
};

struct ConfidenceParams {
    double confirm_threshold = 0.75;
    double sway_threshold = 0.3;
    double confidence_increment = 0.15;
    double confidence_decay = 0.8;
    double missing_decay = 0.92;
    double pos_stable_px = 10.0;
    double size_stable_ratio = 0.2;
};

struct MarkerObservation {
    int id = -1;
    double center_x_px = 0.0;
    double center_y_px = 0.0;
    double size_px = 0.0;
    double yaw_deg = 0.0;
    double timestamp_sec = 0.0;
};

struct MarkerState {
    int id = -1;
    double confidence = 0.0;
    double center_x_px = 0.0;
    double center_y_px = 0.0;
    double size_px = 0.0;
    double yaw_deg = 0.0;
    double last_seen_sec = 0.0;
    MarkerStatus status = MarkerStatus::Candidate;
};

std::string toString(MarkerStatus status);
double observationDistancePx(const MarkerObservation& obs, const MarkerState& state);
bool isObservationStable(
    const MarkerObservation& obs,
    const MarkerState& state,
    const ConfidenceParams& params);

class ConfidenceFilter {
public:
    explicit ConfidenceFilter(ConfidenceParams params = {});

    const MarkerState& update(
        const MarkerObservation& observation,
        double sway_metric);

    void decayMissing(double current_time_sec);
    void markVisited(int marker_id);
    std::vector<MarkerState> states() const;
    std::vector<MarkerState> confirmedStates() const;
    std::vector<MarkerState> candidateStates() const;

private:
    ConfidenceParams params_;
    std::map<int, MarkerState> states_;
};

}  // namespace aruco_tracker
