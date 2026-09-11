#include "hdmap_dynamic_tracker/signal_state_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hdmap_dynamic_tracker {

double ioniq6StoppingDistance(
    double speed_mps,
    const SignalStateManagerConfig& config,
    const std::vector<BrakingDistanceSample>& calibration) {
    const StopRampParameters ramp{speed_mps, config.yellow_decision_deceleration_mps2,
        config.braking_distance_factor, config.latency_budget_s, config.stop_margin_m, std::nullopt};
    if (!validStopRamp(ramp, calibration)) return std::numeric_limits<double>::infinity();
    return stopRampStartDistance(ramp, calibration);
}

const char* signalApproachStateName(SignalApproachState state) noexcept {
    switch (state) {
    case SignalApproachState::Unknown:
        return "UNKNOWN";
    case SignalApproachState::Go:
        return "GO";
    case SignalApproachState::StopRequired:
        return "STOP_REQUIRED";
    case SignalApproachState::Hold:
        return "HOLD";
    case SignalApproachState::Committed:
        return "COMMITTED";
    case SignalApproachState::Clearing:
        return "CLEARING";
    }
    return "UNKNOWN";
}

SignalStateManager::SignalStateManager(
    SignalStateManagerConfig config,
    std::vector<BrakingDistanceSample> braking_calibration)
    : config_(config), braking_calibration_(std::move(braking_calibration)) {
    if (!std::isfinite(config_.yellow_decision_deceleration_mps2) ||
        config_.yellow_decision_deceleration_mps2 <= 0.0 ||
        config_.yellow_decision_deceleration_mps2 > kMaximumDesignDecelerationMps2 ||
        !std::isfinite(config_.braking_distance_factor) ||
        config_.braking_distance_factor < 1.0 ||
        !std::isfinite(config_.latency_budget_s) || config_.latency_budget_s < 0.0 ||
        !std::isfinite(config_.stop_margin_m) || config_.stop_margin_m < 0.0 ||
        !std::isfinite(config_.hold_speed_mps) || config_.hold_speed_mps < 0.0 ||
        !std::isfinite(config_.hold_distance_m) || config_.hold_distance_m < 0.0 ||
        (!braking_calibration_.empty() &&
            !validBrakingDistanceTable(braking_calibration_))) {
        throw std::invalid_argument("Invalid signal state-manager parameters");
    }
}

void SignalStateManager::reset() noexcept {
    state_ = SignalApproachState::Unknown;
    controller_id_ = 0;
    approach_id_ = 0;
    approach_selected_ = false;
    decision_speed_mps_ = 0.0;
}

void SignalStateManager::decideYellow(
    const SignalStateInput& input,
    double required_stopping_distance_m) {
    decision_speed_mps_ = input.ego_speed_mps;
    state_ = input.front_axle_to_stopline_m >= required_stopping_distance_m
        ? SignalApproachState::StopRequired
        : SignalApproachState::Committed;
}

SignalStateOutput SignalStateManager::output(
    bool transitioned,
    double required_stopping_distance_m,
    double static_speed_cap_mps) const {
    const bool stop = state_ == SignalApproachState::StopRequired ||
        state_ == SignalApproachState::Hold;
    const bool proceed = state_ == SignalApproachState::Committed ||
        state_ == SignalApproachState::Clearing;
    return {state_, transitioned, stop, required_stopping_distance_m,
        decision_speed_mps_,
        proceed ? static_speed_cap_mps : 0.0};
}

SignalStateOutput SignalStateManager::update(const SignalStateInput& input) {
    const auto previous_state = state_;
    double required_stopping_distance_m = 0.0;

    if (controller_id_ != input.controller_id ||
        (input.on_approach && approach_selected_ && approach_id_ != input.approach_id)) {
        reset();
    }
    controller_id_ = input.controller_id;

    if (!input.on_approach) {
        if (state_ == SignalApproachState::Go ||
            state_ == SignalApproachState::Committed) {
            state_ = SignalApproachState::Clearing;
        } else if (state_ == SignalApproachState::StopRequired ||
            state_ == SignalApproachState::Hold) {
            state_ = SignalApproachState::Unknown;
            approach_selected_ = false;
        }
        return output(state_ != previous_state, 0.0, input.static_speed_cap_mps);
    }

    approach_id_ = input.approach_id;
    approach_selected_ = true;
    required_stopping_distance_m = ioniq6StoppingDistance(
        input.ego_speed_mps, config_, braking_calibration_);

    if (state_ == SignalApproachState::Committed) {
        return output(false, required_stopping_distance_m, input.static_speed_cap_mps);
    }
    if (state_ == SignalApproachState::Clearing) {
        state_ = SignalApproachState::Committed;
        return output(true, required_stopping_distance_m, input.static_speed_cap_mps);
    }

    if (state_ == SignalApproachState::StopRequired ||
        state_ == SignalApproachState::Hold) {
        if (input.permitted) {
            state_ = SignalApproachState::Go;
            decision_speed_mps_ = 0.0;
        } else if (input.ego_speed_mps <= config_.hold_speed_mps &&
            input.front_axle_to_stopline_m <= config_.hold_distance_m) {
            state_ = SignalApproachState::Hold;
        } else {
            state_ = SignalApproachState::StopRequired;
        }
        return output(state_ != previous_state, required_stopping_distance_m,
            input.static_speed_cap_mps);
    }

    if (input.signal_state == 2U) {
        decideYellow(input, required_stopping_distance_m);
    } else if (input.permitted) {
        state_ = SignalApproachState::Go;
        decision_speed_mps_ = 0.0;
    } else {
        state_ = SignalApproachState::StopRequired;
        decision_speed_mps_ = input.ego_speed_mps;
    }
    return output(state_ != previous_state, required_stopping_distance_m,
        input.static_speed_cap_mps);
}

}  // namespace hdmap_dynamic_tracker
