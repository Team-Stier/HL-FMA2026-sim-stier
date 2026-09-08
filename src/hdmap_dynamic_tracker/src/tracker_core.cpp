#include "hdmap_dynamic_tracker/tracker_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace hdmap_dynamic_tracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

double cross(const Point2d& origin, const Point2d& a, const Point2d& b) {
    return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<Point2d> convexHull(std::vector<Point2d> points) {
    std::sort(points.begin(), points.end(), [](const Point2d& left, const Point2d& right) {
        return left.x < right.x || (left.x == right.x && left.y < right.y);
    });
    points.erase(std::unique(points.begin(), points.end(), [](const Point2d& left, const Point2d& right) {
        return left.x == right.x && left.y == right.y;
    }), points.end());
    if (points.size() <= 2) {
        return points;
    }

    std::vector<Point2d> hull;
    hull.reserve(points.size() * 2);
    for (const auto& point : points) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const auto lower_size = hull.size();
    for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator) {
        while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), *iterator) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(*iterator);
    }
    hull.pop_back();
    return hull;
}

std::string compactLower(std::string value) {
    value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char character) {
        return std::isspace(character) != 0;
    }), value.end());
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return value;
}

}  // namespace

EgoSpeedEstimator::EgoSpeedEstimator(double maximum_dt_s, double maximum_speed_mps)
    : maximum_dt_s_(maximum_dt_s), maximum_speed_mps_(maximum_speed_mps) {
    if (!std::isfinite(maximum_dt_s_) || maximum_dt_s_ <= 0.0 ||
        !std::isfinite(maximum_speed_mps_) || maximum_speed_mps_ <= 0.0) {
        throw std::invalid_argument("Ego speed estimator limits must be finite and positive");
    }
}

EgoSpeedResult EgoSpeedEstimator::update(double stamp_s, double x, double y) {
    if (!std::isfinite(stamp_s) || !std::isfinite(x) || !std::isfinite(y)) {
        reset();
        return {0.0, true};
    }
    if (!initialized_) {
        initialized_ = true;
        stamp_s_ = stamp_s;
        x_ = x;
        y_ = y;
        return {};
    }

    const double dt = stamp_s - stamp_s_;
    const double distance = std::hypot(x - x_, y - y_);
    stamp_s_ = stamp_s;
    x_ = x;
    y_ = y;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > maximum_dt_s_) {
        return {0.0, true};
    }
    const double speed = distance / dt;
    if (!std::isfinite(speed) || speed > maximum_speed_mps_) {
        return {0.0, true};
    }
    return {speed, false};
}

void EgoSpeedEstimator::reset() {
    initialized_ = false;
    stamp_s_ = 0.0;
    x_ = 0.0;
    y_ = 0.0;
}

bool validBrakingDistanceTable(const std::vector<BrakingDistanceSample>& samples) {
    if (samples.empty()) {
        return false;
    }
    double previous_speed = 0.0;
    double previous_distance = 0.0;
    for (const auto& sample : samples) {
        if (!std::isfinite(sample.speed_mps) || sample.speed_mps <= previous_speed ||
            !std::isfinite(sample.braking_distance_m) ||
            sample.braking_distance_m <= 0.0 ||
            sample.braking_distance_m < previous_distance) {
            return false;
        }
        previous_speed = sample.speed_mps;
        previous_distance = sample.braking_distance_m;
    }
    return true;
}

std::optional<double> conservativeBrakingDistance(
    double speed_mps,
    const std::vector<BrakingDistanceSample>& samples) {
    if (!std::isfinite(speed_mps) || speed_mps < 0.0 ||
        !validBrakingDistanceTable(samples)) {
        return std::nullopt;
    }
    if (speed_mps == 0.0) {
        return 0.0;
    }
    const auto sample = std::lower_bound(
        samples.begin(), samples.end(), speed_mps,
        [](const BrakingDistanceSample& candidate, double speed) {
            return candidate.speed_mps < speed;
        });
    if (sample == samples.end()) {
        return std::nullopt;
    }
    // Use the next measured speed bin rather than extrapolating or interpolating
    // below it. With a monotone worst-case table this never understates a measured
    // braking distance between calibration speeds.
    return sample->braking_distance_m;
}

bool validStopRamp(const StopRampParameters& parameters) {
    if (!std::isfinite(parameters.entry_speed_mps) || parameters.entry_speed_mps < 0.0 ||
        !std::isfinite(parameters.design_deceleration_mps2) ||
        parameters.design_deceleration_mps2 <= 0.0 ||
        !std::isfinite(parameters.braking_distance_factor) ||
        parameters.braking_distance_factor < 1.0 ||
        !std::isfinite(parameters.latency_budget_s) || parameters.latency_budget_s < 0.0 ||
        !std::isfinite(parameters.stop_margin_m) || parameters.stop_margin_m < 0.0 ||
        (parameters.calibrated_braking_distance_m &&
            (!std::isfinite(*parameters.calibrated_braking_distance_m) ||
                *parameters.calibrated_braking_distance_m < 0.0 ||
                (parameters.entry_speed_mps > 0.0 &&
                    *parameters.calibrated_braking_distance_m == 0.0)))) {
        return false;
    }
    const double theoretical_linear_distance =
        parameters.entry_speed_mps * parameters.entry_speed_mps /
        parameters.design_deceleration_mps2;
    const double measured_linear_distance = parameters.calibrated_braking_distance_m
        ? 2.0 * *parameters.calibrated_braking_distance_m
        : 0.0;
    const double braking_span = parameters.braking_distance_factor *
        std::max(theoretical_linear_distance, measured_linear_distance);
    const double latency_span = parameters.entry_speed_mps * parameters.latency_budget_s;
    return std::isfinite(theoretical_linear_distance) &&
        std::isfinite(measured_linear_distance) && std::isfinite(braking_span) &&
        std::isfinite(latency_span) && std::isfinite(braking_span + latency_span) &&
        std::isfinite(braking_span + latency_span + parameters.stop_margin_m);
}

double stopRampProfileLength(const StopRampParameters& parameters) {
    if (!validStopRamp(parameters)) {
        return 0.0;
    }
    const double theoretical_linear_distance =
        parameters.entry_speed_mps * parameters.entry_speed_mps /
        parameters.design_deceleration_mps2;
    const double measured_linear_distance = parameters.calibrated_braking_distance_m
        ? 2.0 * *parameters.calibrated_braking_distance_m
        : 0.0;
    return parameters.braking_distance_factor *
        std::max(theoretical_linear_distance, measured_linear_distance) +
        parameters.entry_speed_mps * parameters.latency_budget_s;
}

double stopRampStartDistance(const StopRampParameters& parameters) {
    if (!validStopRamp(parameters)) {
        return 0.0;
    }
    return parameters.stop_margin_m + stopRampProfileLength(parameters);
}

double stopRampCap(double distance_to_stop_m, const StopRampParameters& parameters) {
    if (!std::isfinite(distance_to_stop_m) || !validStopRamp(parameters)) {
        return 0.0;
    }
    const double profile_length = stopRampProfileLength(parameters);
    if (profile_length <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }
    const double ratio = std::clamp(
        (distance_to_stop_m - parameters.stop_margin_m) / profile_length, 0.0, 1.0);
    return parameters.entry_speed_mps * ratio;
}

std::optional<double> parseSpeedLimitMps(const std::string& text) {
    const std::string compact = compactLower(text);
    if (compact.empty()) {
        return std::nullopt;
    }
    char* end = nullptr;
    const double value = std::strtod(compact.c_str(), &end);
    if (end == compact.c_str() || !std::isfinite(value) || value < 0.0) {
        return std::nullopt;
    }
    const std::string unit(end);
    if (unit.empty() || unit == "m/s" || unit == "mps" || unit == "m/sec") {
        return value;
    }
    if (unit == "km/h" || unit == "kmh" || unit == "kph") {
        return value / 3.6;
    }
    return std::nullopt;
}

std::optional<double> parsePositiveMetres(const std::string& text) {
    const std::string compact = compactLower(text);
    if (compact.empty()) {
        return std::nullopt;
    }
    char* end = nullptr;
    const double value = std::strtod(compact.c_str(), &end);
    if (end == compact.c_str() || !std::isfinite(value) || value <= 0.0) {
        return std::nullopt;
    }
    const std::string unit(end);
    if (!unit.empty() && unit != "m" && unit != "meter" && unit != "metres" && unit != "meters") {
        return std::nullopt;
    }
    return value;
}

std::vector<int> parseIntegerCsv(const std::string& text) {
    std::vector<int> result;
    if (text.find_first_not_of(" \t\r\n") == std::string::npos) {
        return result;
    }
    std::size_t begin = 0;
    while (begin <= text.size()) {
        const auto comma = text.find(',', begin);
        const auto token = text.substr(begin, comma == std::string::npos ? std::string::npos : comma - begin);
        const auto first = token.find_first_not_of(" \t\r\n");
        const auto last = token.find_last_not_of(" \t\r\n");
        if (first == std::string::npos) {
            return {};
        }
        const auto trimmed = token.substr(first, last - first + 1);
        char* end = nullptr;
        const long value = std::strtol(trimmed.c_str(), &end, 10);
        if (end == trimmed.c_str() || *end != '\0' ||
            value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
            return {};
        }
        result.push_back(static_cast<int>(value));
        if (comma == std::string::npos) {
            break;
        }
        begin = comma + 1;
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

double normalizeAngle(double angle) {
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    return std::remainder(angle, 2.0 * kPi);
}

double predictionUncertaintyInflation(
    double position_sigma_m,
    double direction_uncertainty_m,
    double sigma_multiplier) {
    if (!std::isfinite(position_sigma_m) || position_sigma_m < 0.0 ||
        !std::isfinite(direction_uncertainty_m) || direction_uncertainty_m < 0.0 ||
        !std::isfinite(sigma_multiplier) || sigma_multiplier <= 0.0) {
        throw std::invalid_argument(
            "Prediction uncertainty values must be finite and non-negative, with a positive multiplier");
    }
    const double inflation = sigma_multiplier * position_sigma_m + direction_uncertainty_m;
    if (!std::isfinite(inflation)) {
        throw std::overflow_error("Prediction uncertainty inflation overflowed");
    }
    return inflation;
}

double trajectorySweepDiscretizationInflation(
    const std::vector<ObjectPrediction>& samples,
    double maximum_sample_interval_s) {
    if (!std::isfinite(maximum_sample_interval_s) || maximum_sample_interval_s <= 0.0) {
        throw std::invalid_argument("Trajectory sample interval must be finite and positive");
    }
    double maximum_planar_acceleration_mps2 = 0.0;
    for (const auto& sample : samples) {
        if (!std::isfinite(sample.speed_mps) || sample.speed_mps < 0.0 ||
            !std::isfinite(sample.turn_rate_radps) ||
            !std::isfinite(sample.acceleration_mps2)) {
            throw std::invalid_argument("Trajectory motion state must be finite");
        }
        const double tangential_acceleration_mps2 = std::abs(sample.acceleration_mps2);
        const double normal_acceleration_mps2 =
            sample.speed_mps * std::abs(sample.turn_rate_radps);
        maximum_planar_acceleration_mps2 = std::max(
            maximum_planar_acceleration_mps2,
            std::hypot(tangential_acceleration_mps2, normal_acceleration_mps2));
    }
    // For a twice-differentiable trajectory with |p''| <= M, deviation from
    // its endpoint chord is bounded by M*h^2/8. The small factor protects the
    // bound from floating-point rounding at rasterization boundaries.
    const double inflation = 1.01 * maximum_planar_acceleration_mps2 *
        maximum_sample_interval_s * maximum_sample_interval_s / 8.0;
    if (!std::isfinite(inflation)) {
        throw std::overflow_error("Trajectory discretization inflation overflowed");
    }
    return inflation;
}

double yawIndependentRotationInflation(double length_m, double width_m) {
    if (!std::isfinite(length_m) || length_m <= 0.0 ||
        !std::isfinite(width_m) || width_m <= 0.0) {
        throw std::invalid_argument("Object dimensions must be finite and positive");
    }
    const double half_length = length_m * 0.5;
    const double half_width = width_m * 0.5;
    return std::hypot(half_length, half_width) - std::min(half_length, half_width);
}

std::vector<Point2d> orientedBox(const ObjectPrediction& object, double inflation_m) {
    const double half_length = std::max(0.0, object.length * 0.5 + inflation_m);
    const double half_width = std::max(0.0, object.width * 0.5 + inflation_m);
    const double cosine = std::cos(object.heading);
    const double sine = std::sin(object.heading);
    std::vector<Point2d> points;
    points.reserve(4);
    for (const auto& local : std::vector<Point2d>{{-half_length, -half_width},
             {half_length, -half_width}, {half_length, half_width}, {-half_length, half_width}}) {
        points.push_back({
            object.x + cosine * local.x - sine * local.y,
            object.y + sine * local.x + cosine * local.y});
    }
    return points;
}

std::vector<Point2d> sweptFootprint(
    const ObjectPrediction& start,
    const ObjectPrediction& end,
    double inflation_m) {
    std::vector<ObjectPrediction> samples;
    samples.reserve(3);
    for (int index = 0; index < 3; ++index) {
        const double fraction = static_cast<double>(index) / 2.0;
        ObjectPrediction sample = start;
        sample.x = start.x + fraction * (end.x - start.x);
        sample.y = start.y + fraction * (end.y - start.y);
        sample.heading = normalizeAngle(
            start.heading + fraction * normalizeAngle(end.heading - start.heading));
        sample.length = std::max(start.length, end.length);
        sample.width = std::max(start.width, end.width);
        samples.push_back(sample);
    }
    return sweptFootprint(samples, inflation_m);
}

std::vector<Point2d> sweptFootprint(
    const std::vector<ObjectPrediction>& samples,
    double inflation_m) {
    std::vector<Point2d> footprint_points;
    footprint_points.reserve(samples.size() * 4U);
    for (const auto& sample : samples) {
        const auto box = orientedBox(sample, inflation_m);
        footprint_points.insert(footprint_points.end(), box.begin(), box.end());
    }
    return convexHull(std::move(footprint_points));
}

}  // namespace hdmap_dynamic_tracker
