#pragma once

#include "hdmap_dynamic_tracker/motion_predictor.hpp"

#include <optional>
#include <string>
#include <vector>

namespace hdmap_dynamic_tracker {

struct Point2d {
    double x = 0.0;
    double y = 0.0;
};

struct EgoSpeedResult {
    double speed_mps = 0.0;
    bool history_reset = false;
};

struct EgoMotionLimits {
    double maximum_speed_mps = 60.0;
    double maximum_vertical_speed_mps = 10.0;
    double maximum_yaw_rate_radps = 3.0;
    double minimum_frame_dt_s = 1.0e-4;
    double maximum_frame_dt_s = 1.0;
};

class EgoSpeedEstimator {
public:
    explicit EgoSpeedEstimator(EgoMotionLimits limits = {});

    EgoSpeedResult update(double stamp_s, double x, double y, double z, double yaw);
    void reset();

private:
    EgoMotionLimits limits_;
    bool initialized_ = false;
    double stamp_s_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double yaw_ = 0.0;
};

// Independent tracker braking contract; no dependency on another consumer's configuration.
inline constexpr double kMaximumDesignDecelerationMps2 = 3.0;

struct StopRampParameters {
    double entry_speed_mps = 0.0;
    double design_deceleration_mps2 = 0.0;
    double braking_distance_factor = 1.0;
    double latency_budget_s = 0.0;
    double stop_margin_m = 0.0;
    // Distance travelled from measured deceleration onset to full stop. The
    // command/actuation latency is represented separately by latency_budget_s.
    // This scalar assumes constant deceleration; pass a full calibration table
    // to the ramp helpers to bound every measured speed bin.
    std::optional<double> calibrated_braking_distance_m;
};

struct BrakingDistanceSample {
    double speed_mps = 0.0;
    double braking_distance_m = 0.0;
};

bool validBrakingDistanceTable(const std::vector<BrakingDistanceSample>& samples);
std::optional<double> conservativeBrakingDistance(
    double speed_mps,
    const std::vector<BrakingDistanceSample>& samples);
bool validStopRamp(const StopRampParameters& parameters,
    const std::vector<BrakingDistanceSample>& calibration = {});
double stopRampProfileLength(const StopRampParameters& parameters,
    const std::vector<BrakingDistanceSample>& calibration = {});
double stopRampStartDistance(const StopRampParameters& parameters,
    const std::vector<BrakingDistanceSample>& calibration = {});
double stopRampCap(double distance_to_stop_m, const StopRampParameters& parameters,
    const std::vector<BrakingDistanceSample>& calibration = {});

std::optional<double> parseSpeedLimitMps(const std::string& text);
std::optional<double> parsePositiveMetres(const std::string& text);
std::vector<int> parseIntegerCsv(const std::string& text);

double normalizeAngle(double angle);
double predictionUncertaintyInflation(
    double position_sigma_m,
    double direction_uncertainty_m,
    double sigma_multiplier);
double yawIndependentRotationInflation(double length_m, double width_m);
std::vector<Point2d> orientedBox(const ObjectPrediction& object, double inflation_m = 0.0);
std::vector<Point2d> sweptFootprint(
    const ObjectPrediction& start,
    const ObjectPrediction& end,
    double inflation_m = 0.0);

}  // namespace hdmap_dynamic_tracker
