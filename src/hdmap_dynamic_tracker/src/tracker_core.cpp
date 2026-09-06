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

bool validStopRamp(const StopRampParameters& parameters) {
    return std::isfinite(parameters.entry_speed_mps) && parameters.entry_speed_mps >= 0.0 &&
        std::isfinite(parameters.design_deceleration_mps2) && parameters.design_deceleration_mps2 > 0.0 &&
        std::isfinite(parameters.braking_distance_factor) && parameters.braking_distance_factor >= 1.0 &&
        std::isfinite(parameters.latency_budget_s) && parameters.latency_budget_s >= 0.0 &&
        std::isfinite(parameters.stop_margin_m) && parameters.stop_margin_m >= 0.0;
}

double stopRampStartDistance(const StopRampParameters& parameters) {
    if (!validStopRamp(parameters)) {
        return 0.0;
    }
    const double linear_base = parameters.entry_speed_mps * parameters.entry_speed_mps /
        parameters.design_deceleration_mps2;
    const double ramp = parameters.braking_distance_factor * linear_base;
    return parameters.entry_speed_mps * parameters.latency_budget_s + parameters.stop_margin_m + ramp;
}

double stopRampCap(double distance_to_stop_m, const StopRampParameters& parameters) {
    if (!validStopRamp(parameters)) {
        return 0.0;
    }
    const double linear_base = parameters.entry_speed_mps * parameters.entry_speed_mps /
        parameters.design_deceleration_mps2;
    const double ramp = parameters.braking_distance_factor * linear_base;
    if (ramp <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }
    const double ratio = std::clamp(
        (distance_to_stop_m - parameters.stop_margin_m) / ramp, 0.0, 1.0);
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
    std::vector<Point2d> samples;
    samples.reserve(12);
    for (int index = 0; index < 3; ++index) {
        const double fraction = static_cast<double>(index) / 2.0;
        ObjectPrediction sample = start;
        sample.x = start.x + fraction * (end.x - start.x);
        sample.y = start.y + fraction * (end.y - start.y);
        sample.heading = normalizeAngle(
            start.heading + fraction * normalizeAngle(end.heading - start.heading));
        sample.length = std::max(start.length, end.length);
        sample.width = std::max(start.width, end.width);
        const auto box = orientedBox(sample, inflation_m);
        samples.insert(samples.end(), box.begin(), box.end());
    }
    return convexHull(std::move(samples));
}

}  // namespace hdmap_dynamic_tracker
