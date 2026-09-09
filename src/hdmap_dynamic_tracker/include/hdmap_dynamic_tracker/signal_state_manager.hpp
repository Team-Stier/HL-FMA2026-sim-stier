#pragma once

#include "hdmap_dynamic_tracker/tracker_core.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace hdmap_dynamic_tracker {

enum class SignalApproachState : std::uint8_t {
    Unknown,
    Go,
    StopRequired,
    Hold,
    Committed,
    Clearing,
};

struct SignalStateManagerConfig {
    double yellow_decision_deceleration_mps2 = 3.0;
    double braking_distance_factor = 1.0;
    double latency_budget_s = 0.25;
    double stop_margin_m = 1.0;
    double hold_speed_mps = 0.2;
    double hold_distance_m = 1.0;
};

struct SignalStateInput {
    std::int32_t controller_id = 0;
    std::size_t approach_id = 0;
    std::uint8_t signal_state = 0;
    bool on_approach = false;
    bool permitted = false;
    double ego_speed_mps = 0.0;
    double front_axle_to_stopline_m = 0.0;
    double static_speed_cap_mps = 0.0;
};

struct SignalStateOutput {
    SignalApproachState state = SignalApproachState::Unknown;
    bool transitioned = false;
    bool apply_stop_cap = false;
    double required_stopping_distance_m = 0.0;
    double decision_speed_mps = 0.0;
    double target_speed_cap_mps = 0.0;
};

double ioniq6StoppingDistance(
    double speed_mps,
    const SignalStateManagerConfig& config,
    const std::vector<BrakingDistanceSample>& calibration = {});

const char* signalApproachStateName(SignalApproachState state) noexcept;

class SignalStateManager {
public:
    SignalStateManager(
        SignalStateManagerConfig config,
        std::vector<BrakingDistanceSample> braking_calibration = {});

    SignalStateOutput update(const SignalStateInput& input);
    void reset() noexcept;

private:
    SignalStateOutput output(
        bool transitioned,
        double required_stopping_distance_m,
        double static_speed_cap_mps) const;
    void decideYellow(const SignalStateInput& input, double required_stopping_distance_m);

    SignalStateManagerConfig config_;
    std::vector<BrakingDistanceSample> braking_calibration_;
    SignalApproachState state_ = SignalApproachState::Unknown;
    std::int32_t controller_id_ = 0;
    std::size_t approach_id_ = 0;
    bool approach_selected_ = false;
    double decision_speed_mps_ = 0.0;
};

}  // namespace hdmap_dynamic_tracker
