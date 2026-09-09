#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace hdmap_dynamic_tracker {

struct ObjectObservation {
    std::uint32_t id = 0;
    double x = 0.0;
    double y = 0.0;
    double min_z = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
};

struct ObjectPrediction {
    std::uint32_t id = 0;
    double x = 0.0;
    double y = 0.0;
    double min_z = 0.0;
    double heading = 0.0;
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
    double position_sigma_m = 0.0;
    double direction_uncertainty_m = 0.0;
    double age_since_observation_s = 0.0;
    double speed_mps = 0.0;
    double course_rad = 0.0;
    double turn_rate_radps = 0.0;
    double acceleration_mps2 = 0.0;
    bool motion_state_converged = false;
};

struct PredictorConfig {
    double process_acceleration_sigma_mps2 = 2.0;
    double position_measurement_sigma_m = 0.5;
    double initial_velocity_sigma_mps = 5.0;
    double track_retention_s = 0.5;
    double minimum_frame_dt_s = 1.0e-4;
    double maximum_frame_dt_s = 0.25;
    // Retained for source compatibility with the original predictor config.
    // The CTRA implementation no longer rejects observations with these gates.
    double maximum_position_innovation_m = 15.0;
    std::size_t minimum_velocity_observations = 4;
    double minimum_velocity_observation_span_s = 0.15;
    double minimum_velocity_displacement_m = 1.0;
    double maximum_velocity_innovation_mps = 3.0;
    // CTRA fields are appended so existing positional aggregate initialization
    // of the original predictor configuration keeps its field order.
    double process_jerk_sigma_mps3 = 2.0;
    double process_turn_acceleration_sigma_radps2 = 0.6;
    double speed_measurement_sigma_mps = 1.0;
    double course_measurement_sigma_rad = 0.20;
    double initial_acceleration_sigma_mps2 = 2.0;
    double initial_turn_rate_sigma_radps = 0.5;
    double maximum_prediction_step_s = 0.05;
};

class MotionPredictor {
public:
    virtual ~MotionPredictor() = default;

    virtual void reset() = 0;
    virtual void updateFrame(double stamp_s, const std::vector<ObjectObservation>& observations) = 0;
    virtual std::vector<ObjectPrediction> predict(double stamp_s, double future_s) const = 0;
    virtual std::vector<std::vector<ObjectPrediction>> predictSequence(
        double stamp_s,
        const std::vector<double>& future_times_s) const {
        std::vector<std::vector<ObjectPrediction>> output;
        output.reserve(future_times_s.size());
        for (const double future_s : future_times_s) {
            output.push_back(predict(stamp_s, future_s));
        }
        return output;
    }
};

std::unique_ptr<MotionPredictor> makeEkfPredictor(const PredictorConfig& config);

}  // namespace hdmap_dynamic_tracker
