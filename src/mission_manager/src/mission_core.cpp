#include "mission_manager/mission_core.hpp"

#include <algorithm>
#include <stdexcept>

namespace mission_manager {

std::string toString(MissionPhase phase) {
    switch (phase) {
    case MissionPhase::Init:
        return "INIT";
    case MissionPhase::Takeoff:
        return "TAKEOFF";
    case MissionPhase::HomeInit:
        return "HOME_INIT";
    case MissionPhase::GridSearch:
        return "GRID_SEARCH";
    case MissionPhase::MarkerApproach:
        return "MARKER_APPROACH";
    case MissionPhase::AntiSway:
        return "ANTI_SWAY";
    case MissionPhase::HoverConfirm:
        return "HOVER_CONFIRM";
    case MissionPhase::MarkerSave:
        return "MARKER_SAVE";
    case MissionPhase::RescueRoutePlan:
        return "RESCUE_ROUTE_PLAN";
    case MissionPhase::RescueVisit:
        return "RESCUE_VISIT";
    case MissionPhase::ReturnHome:
        return "RETURN_HOME";
    case MissionPhase::VertiportAcquire:
        return "VERTIPORT_ACQUIRE";
    case MissionPhase::VisionServoLand:
        return "VISION_SERVO_LAND";
    case MissionPhase::Landed:
        return "LANDED";
    case MissionPhase::EmergencyReturn:
        return "EMERGENCY_RETURN";
    case MissionPhase::Abort:
        return "ABORT";
    }
    return "INIT";
}

MissionCore::MissionCore(MissionParams params)
    : params_(params)
{
    if (params_.required_marker_count <= 0) {
        throw std::invalid_argument("required_marker_count must be positive");
    }
    if (params_.hover_confirm_duration_sec <= 0.0) {
        throw std::invalid_argument("hover_confirm_duration_sec must be positive");
    }
}

MissionPhase MissionCore::phase() const {
    return phase_;
}

MissionSnapshot MissionCore::snapshot() const {
    MissionSnapshot out;
    out.phase = phase_;
    out.confirmed_markers = confirmed_markers_;
    out.rescue_order = rescue_order_;
    out.rescue_visited = rescue_visited_;

    if (phase_ == MissionPhase::RescueVisit && rescue_index_ < rescue_order_.size()) {
        out.active_rescue_marker_id = rescue_order_[rescue_index_];
        out.active_target_marker = markerById(rescue_order_[rescue_index_]);
    }
    return out;
}

void MissionCore::reset() {
    phase_ = MissionPhase::Init;
    confirmed_markers_.clear();
    rescue_order_.clear();
    rescue_visited_.clear();
    pending_marker_.reset();
    hover_start_sec_.reset();
    rescue_hover_start_sec_.reset();
    rescue_index_ = 0;
}

MissionPhase MissionCore::tick(const TickInput& input) {
    if (input.abort) {
        phase_ = MissionPhase::Abort;
        return phase_;
    }

    if (shouldEmergencyReturn(input)) {
        phase_ = MissionPhase::EmergencyReturn;
    }

    switch (phase_) {
    case MissionPhase::Init:
        if (input.mission_start) {
            phase_ = MissionPhase::Takeoff;
        }
        break;

    case MissionPhase::Takeoff:
        if (input.takeoff_complete) {
            phase_ = MissionPhase::HomeInit;
        }
        break;

    case MissionPhase::HomeInit:
        if (input.home_initialized) {
            phase_ = MissionPhase::GridSearch;
        }
        break;

    case MissionPhase::GridSearch:
        if (static_cast<int>(confirmed_markers_.size()) >= params_.required_marker_count) {
            phase_ = MissionPhase::RescueRoutePlan;
        } else if (input.marker_candidate) {
            phase_ = MissionPhase::MarkerApproach;
        }
        break;

    case MissionPhase::MarkerApproach:
        if (input.perception_stable || input.confirmed_marker.has_value()) {
            phase_ = MissionPhase::AntiSway;
        }
        break;

    case MissionPhase::AntiSway:
        if (input.confirmed_marker.has_value()) {
            pending_marker_ = input.confirmed_marker;
            startHover(input.now_sec);
            phase_ = MissionPhase::HoverConfirm;
        }
        break;

    case MissionPhase::HoverConfirm:
        if (input.confirmed_marker.has_value()) {
            pending_marker_ = input.confirmed_marker;
        }
        if (pending_marker_.has_value() && hoverElapsed(input.now_sec)) {
            saveMarker(*pending_marker_);
            pending_marker_.reset();
            hover_start_sec_.reset();
            phase_ = MissionPhase::MarkerSave;
        }
        break;

    case MissionPhase::MarkerSave:
        if (static_cast<int>(confirmed_markers_.size()) >= params_.required_marker_count) {
            phase_ = MissionPhase::RescueRoutePlan;
        } else {
            phase_ = MissionPhase::GridSearch;
        }
        break;

    case MissionPhase::RescueRoutePlan:
        buildRescueOrder();
        rescue_index_ = 0;
        rescue_visited_.clear();
        rescue_hover_start_sec_.reset();
        phase_ = rescue_order_.empty() ? MissionPhase::ReturnHome : MissionPhase::RescueVisit;
        break;

    case MissionPhase::RescueVisit:
        updateRescueVisit(input);
        break;

    case MissionPhase::ReturnHome:
        if (input.target_reached) {
            phase_ = MissionPhase::VertiportAcquire;
        }
        break;

    case MissionPhase::VertiportAcquire:
        if (input.vertiport_acquired) {
            phase_ = MissionPhase::VisionServoLand;
        }
        break;

    case MissionPhase::VisionServoLand:
        if (input.landing_complete) {
            phase_ = MissionPhase::Landed;
        }
        break;

    case MissionPhase::EmergencyReturn:
        if (input.vertiport_acquired) {
            phase_ = MissionPhase::VisionServoLand;
        } else if (input.target_reached) {
            phase_ = MissionPhase::VertiportAcquire;
        }
        break;

    case MissionPhase::Landed:
    case MissionPhase::Abort:
        break;
    }

    return phase_;
}

bool MissionCore::isTerminal() const {
    return phase_ == MissionPhase::Landed || phase_ == MissionPhase::Abort;
}

bool MissionCore::shouldEmergencyReturn(const TickInput& input) const {
    if (isTerminal() || phase_ == MissionPhase::Init) {
        return false;
    }
    if (phase_ == MissionPhase::EmergencyReturn
        || phase_ == MissionPhase::VertiportAcquire
        || phase_ == MissionPhase::VisionServoLand) {
        return false;
    }
    return input.battery_percent <= params_.battery_return_threshold_pct;
}

void MissionCore::saveMarker(const MarkerInfo& marker) {
    auto iter = std::find_if(
        confirmed_markers_.begin(),
        confirmed_markers_.end(),
        [&](const MarkerInfo& existing) { return existing.id == marker.id; });

    if (iter == confirmed_markers_.end()) {
        confirmed_markers_.push_back(marker);
    } else if (marker.confidence >= iter->confidence) {
        *iter = marker;
    }
}

void MissionCore::buildRescueOrder() {
    rescue_order_.clear();
    rescue_order_.reserve(confirmed_markers_.size());
    for (const auto& marker : confirmed_markers_) {
        rescue_order_.push_back(marker.id);
    }
    std::sort(rescue_order_.begin(), rescue_order_.end(), std::greater<int>());
}

std::optional<MarkerInfo> MissionCore::markerById(int id) const {
    auto iter = std::find_if(
        confirmed_markers_.begin(),
        confirmed_markers_.end(),
        [&](const MarkerInfo& marker) { return marker.id == id; });
    if (iter == confirmed_markers_.end()) {
        return std::nullopt;
    }
    return *iter;
}

void MissionCore::startHover(double now_sec) {
    hover_start_sec_ = now_sec;
}

bool MissionCore::hoverElapsed(double now_sec) const {
    return hover_start_sec_.has_value()
        && now_sec - *hover_start_sec_ >= params_.hover_confirm_duration_sec;
}

void MissionCore::updateRescueVisit(const TickInput& input) {
    if (rescue_index_ >= rescue_order_.size()) {
        phase_ = MissionPhase::ReturnHome;
        return;
    }

    // Start hover timer on first arrival only — do not reset if drone overshoots
    if (input.target_reached && !rescue_hover_start_sec_.has_value()) {
        rescue_hover_start_sec_ = input.now_sec;
    }

    if (!rescue_hover_start_sec_.has_value()) {
        return;
    }

    if (input.now_sec - *rescue_hover_start_sec_ < params_.hover_confirm_duration_sec) {
        return;
    }

    rescue_visited_.push_back(rescue_order_[rescue_index_]);
    rescue_index_ += 1;
    rescue_hover_start_sec_.reset();

    if (rescue_index_ >= rescue_order_.size()) {
        phase_ = MissionPhase::ReturnHome;
    }
}

}  // namespace mission_manager
