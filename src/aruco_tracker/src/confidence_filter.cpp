#include "aruco_tracker/confidence_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_tracker {

std::string toString(MarkerStatus status) {
    switch (status) {
    case MarkerStatus::Candidate:
        return "Candidate";
    case MarkerStatus::Tracking:
        return "Tracking";
    case MarkerStatus::Confirmed:
        return "Confirmed";
    case MarkerStatus::Visited:
        return "Visited";
    }
    return "Candidate";
}

double observationDistancePx(const MarkerObservation& obs, const MarkerState& state) {
    const double dx = obs.center_x_px - state.center_x_px;
    const double dy = obs.center_y_px - state.center_y_px;
    return std::sqrt(dx * dx + dy * dy);
}

bool isObservationStable(
    const MarkerObservation& obs,
    const MarkerState& state,
    const ConfidenceParams& params)
{
    if (state.size_px <= 1.0) {
        return true;
    }
    const double pos_delta = observationDistancePx(obs, state);
    const double size_delta_ratio = std::abs(obs.size_px - state.size_px) / state.size_px;
    return pos_delta <= params.pos_stable_px
        && size_delta_ratio <= params.size_stable_ratio;
}

ConfidenceFilter::ConfidenceFilter(ConfidenceParams params)
    : params_(params)
{
    if (params_.confirm_threshold <= 0.0 || params_.confirm_threshold > 1.0) {
        throw std::invalid_argument("confirm_threshold must be in (0, 1]");
    }
}

const MarkerState& ConfidenceFilter::update(
    const MarkerObservation& observation,
    double sway_metric)
{
    auto& state = states_[observation.id];
    const bool first_seen = state.id < 0;
    if (first_seen) {
        state.id = observation.id;
        state.center_x_px = observation.center_x_px;
        state.center_y_px = observation.center_y_px;
        state.size_px = observation.size_px;
        state.yaw_deg = observation.yaw_deg;
        state.last_seen_sec = observation.timestamp_sec;
        state.confidence = std::min(1.0, params_.confidence_increment);
        state.status = MarkerStatus::Candidate;
        return state;
    }

    const bool stable = isObservationStable(observation, state, params_);
    const double sway_weight = (sway_metric > params_.sway_threshold) ? 0.2 : 1.0;

    if (stable) {
        state.confidence = std::min(
            1.0,
            state.confidence + sway_weight * params_.confidence_increment);
    } else {
        state.confidence *= params_.confidence_decay;
        state.confidence = std::min(
            1.0,
            state.confidence + 0.5 * sway_weight * params_.confidence_increment);
    }

    state.center_x_px = observation.center_x_px;
    state.center_y_px = observation.center_y_px;
    state.size_px = observation.size_px;
    state.yaw_deg = observation.yaw_deg;
    state.last_seen_sec = observation.timestamp_sec;

    if (state.status != MarkerStatus::Visited) {
        if (state.confidence >= params_.confirm_threshold) {
            state.status = MarkerStatus::Confirmed;
        } else if (state.confidence >= params_.confirm_threshold * 0.5) {
            state.status = MarkerStatus::Tracking;
        } else {
            state.status = MarkerStatus::Candidate;
        }
    }

    return state;
}

void ConfidenceFilter::decayMissing(double current_time_sec) {
    for (auto& [id, state] : states_) {
        (void)id;
        if (state.status == MarkerStatus::Visited) {
            continue;
        }
        if (current_time_sec <= state.last_seen_sec) {
            continue;
        }
        state.confidence *= params_.missing_decay;
        if (state.confidence < params_.confirm_threshold) {
            state.status = state.confidence >= params_.confirm_threshold * 0.5
                ? MarkerStatus::Tracking
                : MarkerStatus::Candidate;
        }
    }
}

void ConfidenceFilter::markVisited(int marker_id) {
    auto iter = states_.find(marker_id);
    if (iter == states_.end()) {
        return;
    }
    iter->second.status = MarkerStatus::Visited;
    iter->second.confidence = 1.0;
}

std::vector<MarkerState> ConfidenceFilter::states() const {
    std::vector<MarkerState> out;
    out.reserve(states_.size());
    for (const auto& [id, state] : states_) {
        (void)id;
        out.push_back(state);
    }
    return out;
}

std::vector<MarkerState> ConfidenceFilter::confirmedStates() const {
    std::vector<MarkerState> out;
    for (const auto& [id, state] : states_) {
        (void)id;
        if (state.status == MarkerStatus::Confirmed || state.status == MarkerStatus::Visited) {
            out.push_back(state);
        }
    }
    return out;
}

std::vector<MarkerState> ConfidenceFilter::candidateStates() const {
    std::vector<MarkerState> out;
    for (const auto& [id, state] : states_) {
        (void)id;
        if (state.status == MarkerStatus::Candidate || state.status == MarkerStatus::Tracking) {
            out.push_back(state);
        }
    }
    return out;
}

}  // namespace aruco_tracker
