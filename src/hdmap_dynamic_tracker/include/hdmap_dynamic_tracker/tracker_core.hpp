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

class EgoSpeedEstimator {
public:
    EgoSpeedEstimator(double maximum_dt_s, double maximum_speed_mps);

    EgoSpeedResult update(double stamp_s, double x, double y);
    void reset();

private:
    double maximum_dt_s_;
    double maximum_speed_mps_;
    bool initialized_ = false;
    double stamp_s_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
};

struct StopRampParameters {
    double entry_speed_mps = 0.0;
    double design_deceleration_mps2 = 0.0;
    double braking_distance_factor = 1.0;
    double latency_budget_s = 0.0;
    double stop_margin_m = 0.0;
};

bool validStopRamp(const StopRampParameters& parameters);
double stopRampStartDistance(const StopRampParameters& parameters);
double stopRampCap(double distance_to_stop_m, const StopRampParameters& parameters);

std::optional<double> parseSpeedLimitMps(const std::string& text);
std::optional<double> parsePositiveMetres(const std::string& text);
std::vector<int> parseIntegerCsv(const std::string& text);

double normalizeAngle(double angle);
std::vector<Point2d> orientedBox(const ObjectPrediction& object, double inflation_m = 0.0);
std::vector<Point2d> sweptFootprint(
    const ObjectPrediction& start,
    const ObjectPrediction& end,
    double inflation_m = 0.0);

}  // namespace hdmap_dynamic_tracker
