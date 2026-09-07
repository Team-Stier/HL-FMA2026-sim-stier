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
};

struct PredictorConfig {
    double process_acceleration_sigma_mps2 = 2.0;
    double position_measurement_sigma_m = 0.5;
    double initial_velocity_sigma_mps = 5.0;
    double track_retention_s = 0.5;
    double minimum_frame_dt_s = 1.0e-4;
    double maximum_frame_dt_s = 0.25;
    double maximum_position_innovation_m = 15.0;
    std::size_t minimum_velocity_observations = 4;
    double minimum_velocity_observation_span_s = 0.15;
    double minimum_velocity_displacement_m = 1.0;
    double maximum_velocity_innovation_mps = 3.0;
};

class MotionPredictor {
public:
    virtual ~MotionPredictor() = default;

    virtual void reset() = 0;
    virtual void updateFrame(double stamp_s, const std::vector<ObjectObservation>& observations) = 0;
    virtual std::vector<ObjectPrediction> predict(double stamp_s, double future_s) const = 0;
};

std::unique_ptr<MotionPredictor> makeEkfPredictor(const PredictorConfig& config);

}  // namespace hdmap_dynamic_tracker
