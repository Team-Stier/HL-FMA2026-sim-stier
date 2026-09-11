#include "control/controller_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

bool finiteNonNegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

Point2 interpolate(const Point2 &first, const Point2 &second, double ratio) {
    return {first.x + ratio * (second.x - first.x),
            first.y + ratio * (second.y - first.y)};
}

Point2 capturedPointToMap(const Point2 &point, const Pose2 &capture) {
    const double cosine = std::cos(capture.yaw);
    const double sine = std::sin(capture.yaw);
    return {capture.x + cosine * point.x - sine * point.y,
            capture.y + sine * point.x + cosine * point.y};
}

Point2 mapPointToCurrent(const Point2 &point, const Pose2 &current) {
    const double dx = point.x - current.x;
    const double dy = point.y - current.y;
    const double cosine = std::cos(current.yaw);
    const double sine = std::sin(current.yaw);
    return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

double signedCurvature(const Point2 &first, const Point2 &second,
                       const Point2 &third) {
    const double a = std::hypot(second.x - first.x, second.y - first.y);
    const double b = std::hypot(third.x - second.x, third.y - second.y);
    const double c = std::hypot(third.x - first.x, third.y - first.y);
    const double denominator = a * b * c;
    if (!(denominator > 1e-9) || !std::isfinite(denominator)) {
        return 0.0;
    }
    return 2.0 * ((second.x - first.x) * (third.y - first.y) -
                  (second.y - first.y) * (third.x - first.x)) /
           denominator;
}

}  // namespace

double normalizeAngle(double angle) noexcept {
    if (!std::isfinite(angle)) {
        return angle;
    }
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

ReferenceBuilder::ReferenceBuilder(ReferenceConfig config) : config_(config) {
    if (!(config_.duplicate_epsilon_m > 0.0) ||
        !std::isfinite(config_.curvature_window_m) ||
        !(config_.curvature_window_m > 0.0) ||
        !(config_.heading_rejection_rad > 0.0) ||
        config_.heading_rejection_rad > kPi ||
        !finiteNonNegative(config_.continuity_weight) ||
        !std::isfinite(config_.maximum_projection_distance_m) ||
        !(config_.maximum_projection_distance_m > 0.0) ||
        config_.minimum_steps < 2) {
        throw std::invalid_argument("invalid reference configuration");
    }
}

void ReferenceBuilder::reset() noexcept {
    has_previous_projection_ = false;
    previous_projection_map_ = {};
    previous_stamp_seconds_ = 0.0;
}

PreparedReference ReferenceBuilder::prepare(
    const PathSnapshot &path, const Pose2 &current, double speed_mps,
    double dt_seconds, std::size_t requested_steps) {
    PreparedReference result;
    if (!std::isfinite(current.x) || !std::isfinite(current.y) ||
        !std::isfinite(current.yaw) || !finiteNonNegative(speed_mps) ||
        !(dt_seconds > 0.0) || !std::isfinite(dt_seconds) ||
        requested_steps < config_.minimum_steps ||
        !std::isfinite(path.capture_pose_map.x) ||
        !std::isfinite(path.capture_pose_map.y) ||
        !std::isfinite(path.capture_pose_map.yaw)) {
        result.error = "invalid state, dt, or horizon";
        return result;
    }

    std::vector<Point2> map_points;
    std::vector<Point2> current_points;
    for (const Point2 &raw : path.points_at_capture) {
        if (!std::isfinite(raw.x) || !std::isfinite(raw.y)) {
            result.error = "non-finite path point";
            return result;
        }
        const Point2 map = capturedPointToMap(raw, path.capture_pose_map);
        const Point2 local = mapPointToCurrent(map, current);
        if (current_points.empty() ||
            std::hypot(local.x - current_points.back().x,
                       local.y - current_points.back().y) >=
                config_.duplicate_epsilon_m) {
            map_points.push_back(map);
            current_points.push_back(local);
        }
    }
    // A single origin pose is the existing Path contract for a stop request.
    if (path.points_at_capture.size() == 1 &&
        std::hypot(path.points_at_capture.front().x,
                   path.points_at_capture.front().y) <= config_.duplicate_epsilon_m &&
        std::hypot(current_points.front().x, current_points.front().y) <=
            config_.maximum_projection_distance_m) {
        result.valid = true;
        result.stop_only = true;
        result.hold_requested = true;
        return result;
    }
    if (current_points.size() < 2) {
        result.error = "path has fewer than two distinct finite points";
        return result;
    }

    double best_cost = std::numeric_limits<double>::infinity();
    double best_ratio = 0.0;
    double best_heading = 0.0;
    std::size_t best_segment = 0;
    bool found = false;
    for (std::size_t index = 0; index + 1 < current_points.size(); ++index) {
        const double dx = current_points[index + 1].x - current_points[index].x;
        const double dy = current_points[index + 1].y - current_points[index].y;
        const double length_squared = dx * dx + dy * dy;
        if (!(length_squared > 0.0)) {
            continue;
        }
        const double heading = std::atan2(dy, dx);
        const double heading_error = std::abs(normalizeAngle(heading));
        if (heading_error > config_.heading_rejection_rad) {
            continue;
        }
        const double ratio = clamp(
            -(current_points[index].x * dx + current_points[index].y * dy) /
                length_squared,
            0.0, 1.0);
        const Point2 projection = interpolate(current_points[index],
                                             current_points[index + 1], ratio);
        if (std::hypot(projection.x, projection.y) >
            config_.maximum_projection_distance_m) {
            continue;
        }
        const Point2 projection_map = interpolate(map_points[index],
                                                  map_points[index + 1], ratio);
        const bool use_continuity = has_previous_projection_ &&
                                    path.stamp_seconds >= previous_stamp_seconds_;
        const double continuity = use_continuity
            ? std::hypot(projection_map.x - previous_projection_map_.x,
                         projection_map.y - previous_projection_map_.y)
            : 0.0;
        const double cost = std::hypot(projection.x, projection.y) +
                            0.5 * heading_error +
                            config_.continuity_weight * continuity;
        if (cost < best_cost) {
            best_cost = cost;
            best_ratio = ratio;
            best_heading = heading;
            best_segment = index;
            found = true;
        }
    }
    if (!found) {
        result.error = "no nearby path segment aligned with vehicle direction";
        return result;
    }

    const Point2 projection = interpolate(current_points[best_segment],
                                         current_points[best_segment + 1],
                                         best_ratio);
    result.lateral_error_m = std::sin(best_heading) * projection.x -
                             std::cos(best_heading) * projection.y;
    result.heading_error_rad = normalizeAngle(-best_heading);

    std::vector<Point2> forward{projection};
    if (std::hypot(current_points[best_segment + 1].x - projection.x,
                   current_points[best_segment + 1].y - projection.y) >=
        config_.duplicate_epsilon_m) {
        forward.push_back(current_points[best_segment + 1]);
    }
    forward.insert(forward.end(), current_points.begin() + best_segment + 2,
                   current_points.end());
    if (forward.size() < 2) {
        if (std::hypot(projection.x, projection.y) <= config_.duplicate_epsilon_m) {
            result.valid = true;
            result.stop_only = true;
        } else {
            result.error = "no usable path remains ahead";
        }
        return result;
    }

    std::vector<double> arc(forward.size(), 0.0);
    for (std::size_t index = 1; index < forward.size(); ++index) {
        arc[index] = arc[index - 1] +
                     std::hypot(forward[index].x - forward[index - 1].x,
                                forward[index].y - forward[index - 1].y);
    }
    result.remaining_length_m = arc.back();
    if (!std::isfinite(result.remaining_length_m)) {
        result.error = "non-finite path length";
        return result;
    }
    const double travel_per_step = speed_mps * dt_seconds;
    if (!std::isfinite(travel_per_step)) {
        result.error = "non-finite preview distance";
        return result;
    }
    const double available_steps = travel_per_step > 0.0
        ? result.remaining_length_m / travel_per_step
        : static_cast<double>(requested_steps);
    const std::size_t steps = available_steps >= requested_steps
        ? requested_steps : static_cast<std::size_t>(std::floor(available_steps));
    if (steps < config_.minimum_steps) {
        result.valid = true;
        result.stop_only = true;
        return result;
    }

    auto sample = [&](double station) {
        const auto upper = std::upper_bound(arc.begin(), arc.end(), station);
        const std::size_t segment = std::min(
            static_cast<std::size_t>(upper - arc.begin() - 1), arc.size() - 2);
        const double length = arc[segment + 1] - arc[segment];
        return interpolate(forward[segment], forward[segment + 1],
                           length > 0.0 ? (station - arc[segment]) / length : 0.0);
    };
    // The geometric stencil is independent of the MPC's temporal grid.
    // In particular 0.5 m must not stand in for v * 0.05 s at low speed.
    const double window = std::min(config_.curvature_window_m,
                                    result.remaining_length_m / 2.0);
    result.curvature.reserve(steps);
    for (std::size_t step = 0; step < steps; ++step) {
        const double station = travel_per_step * static_cast<double>(step);
        const double center = clamp(station, window,
                                     result.remaining_length_m - window);
        result.curvature.push_back(signedCurvature(
            sample(center - window), sample(center), sample(center + window)));
    }

    previous_projection_map_ = interpolate(map_points[best_segment],
                                           map_points[best_segment + 1],
                                           best_ratio);
    previous_stamp_seconds_ = path.stamp_seconds;
    has_previous_projection_ = true;
    result.valid = true;
    return result;
}

LateralMpc::LateralMpc(MpcConfig config) : config_(config) {
    if (!(config_.wheelbase_m > 0.0) || !(config_.dt_seconds > 0.0) ||
        config_.horizon_steps < 2 || !(config_.lateral_error_weight > 0.0) ||
        !(config_.heading_error_weight > 0.0) || config_.steering_weight < 0.0 ||
        config_.steering_rate_weight < 0.0 ||
        !(config_.terminal_multiplier > 0.0) ||
        !(config_.maximum_steering_rad > 0.0) ||
        !(config_.maximum_steering_rate_radps > 0.0) ||
        config_.maximum_iterations == 0 || !(config_.gradient_step > 0.0) ||
        !(config_.convergence_tolerance > 0.0) ||
        !(config_.maximum_solve_time_ms > 0.0)) {
        configuration_error_ = "invalid MPC configuration";
    }
}

MpcResult LateralMpc::solve(
    double lateral_error, double heading_error, double speed_mps,
    const std::vector<double> &curvature, double previous_steering,
    const std::vector<double> *warm_start) const {
    MpcResult result;
    const auto started = std::chrono::steady_clock::now();
    if (!configuration_error_.empty()) {
        result.reason = configuration_error_;
        return result;
    }
    if (!std::isfinite(lateral_error) || !std::isfinite(heading_error) ||
        !finiteNonNegative(speed_mps) || !std::isfinite(previous_steering)) {
        result.reason = "non-finite MPC input";
        return result;
    }
    const std::size_t count = std::min(config_.horizon_steps, curvature.size());
    if (count < 2) {
        result.reason = "reference horizon has fewer than two steps";
        return result;
    }
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::isfinite(curvature[index])) {
            result.reason = "non-finite reference curvature";
            return result;
        }
    }

    std::vector<double> steering(count, previous_steering);
    if (warm_start != nullptr && warm_start->size() >= count) {
        std::copy_n(warm_start->begin(), count, steering.begin());
    }
    const double rate_step = config_.maximum_steering_rate_radps *
                             config_.dt_seconds;
    auto project = [&]() {
        double prior = clamp(previous_steering, -config_.maximum_steering_rad,
                             config_.maximum_steering_rad);
        for (double &value : steering) {
            value = clamp(value, -config_.maximum_steering_rad,
                          config_.maximum_steering_rad);
            value = clamp(value, prior - rate_step, prior + rate_step);
            prior = value;
        }
    };
    project();

    const double a01 = speed_mps * config_.dt_seconds;
    const double b1 = speed_mps * config_.dt_seconds / config_.wheelbase_m;
    std::vector<double> ey(count + 1), epsi(count + 1);
    std::vector<double> lambda_y(count + 1), lambda_psi(count + 1);
    std::vector<double> gradient(count);
    bool converged = false;
    std::size_t iterations = 0;
    for (; iterations < config_.maximum_iterations; ++iterations) {
        ey[0] = lateral_error;
        epsi[0] = heading_error;
        for (std::size_t step = 0; step < count; ++step) {
            ey[step + 1] = ey[step] + a01 * epsi[step];
            epsi[step + 1] = epsi[step] + b1 * steering[step] -
                              speed_mps * curvature[step] * config_.dt_seconds;
        }
        lambda_y[count] = 2.0 * config_.terminal_multiplier *
                          config_.lateral_error_weight * ey[count];
        lambda_psi[count] = 2.0 * config_.terminal_multiplier *
                            config_.heading_error_weight * epsi[count];
        for (std::size_t step = count; step-- > 0;) {
            const double prior = step == 0 ? previous_steering
                                           : steering[step - 1];
            double rate_gradient = 2.0 * config_.steering_rate_weight *
                                   (steering[step] - prior);
            if (step + 1 < count) {
                rate_gradient -= 2.0 * config_.steering_rate_weight *
                                 (steering[step + 1] - steering[step]);
            }
            gradient[step] = 2.0 * config_.steering_weight * steering[step] +
                             rate_gradient + b1 * lambda_psi[step + 1];
            lambda_y[step] = 2.0 * config_.lateral_error_weight * ey[step] +
                             lambda_y[step + 1];
            lambda_psi[step] =
                2.0 * config_.heading_error_weight * epsi[step] +
                a01 * lambda_y[step + 1] + lambda_psi[step + 1];
        }
        const std::vector<double> previous_iteration = steering;
        for (std::size_t step = 0; step < count; ++step) {
            steering[step] -= config_.gradient_step * gradient[step];
        }
        project();
        double maximum_change = 0.0;
        for (std::size_t step = 0; step < count; ++step) {
            maximum_change = std::max(
                maximum_change,
                std::abs(steering[step] - previous_iteration[step]));
        }
        result.solve_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        if (result.solve_time_ms > config_.maximum_solve_time_ms) {
            result.reason = "MPC solve time limit exceeded";
            return result;
        }
        if (maximum_change < config_.convergence_tolerance) {
            converged = true;
            ++iterations;
            break;
        }
    }
    result.iterations = iterations;
    result.solution = steering;
    double prior = previous_steering;
    for (double value : steering) {
        if (!std::isfinite(value)) {
            result.reason = "non-finite MPC solution";
            return result;
        }
        result.maximum_constraint_violation = std::max(
            result.maximum_constraint_violation,
            std::max(0.0, std::abs(value) - config_.maximum_steering_rad));
        result.maximum_constraint_violation = std::max(
            result.maximum_constraint_violation,
            std::max(0.0, std::abs(value - prior) - rate_step));
        prior = value;
    }
    if (result.maximum_constraint_violation > config_.constraint_tolerance) {
        result.reason = "MPC solution violates constraints";
        return result;
    }
    if (!converged) {
        result.reason = "MPC iteration limit reached";
        return result;
    }
    result.steering_rad = steering.front();
    result.reason = "ok";
    result.success = true;
    return result;
}

SpeedPolicyResult limitTargetSpeed(
    const SpeedPolicyConfig &config, double cruise_speed,
    double speed_limit, double maximum_curvature,
    double remaining_path, double requested_preview) {
    SpeedPolicyResult result;
    if (!finiteNonNegative(cruise_speed) || !finiteNonNegative(speed_limit) ||
        !finiteNonNegative(maximum_curvature) ||
        !finiteNonNegative(remaining_path) ||
        !finiteNonNegative(requested_preview) ||
        !finiteNonNegative(config.limit_margin_mps) ||
        !(config.comfortable_deceleration_mps2 > 0.0) ||
        !finiteNonNegative(config.path_end_buffer_m) ||
        !(config.maximum_lateral_acceleration_mps2 > 0.0)) {
        result.error = "invalid speed policy input";
        return result;
    }
    double speed = std::min(
        cruise_speed, std::max(0.0, speed_limit - config.limit_margin_mps));
    if (maximum_curvature > 1e-8) {
        speed = std::min(
            speed,
            std::sqrt(config.maximum_lateral_acceleration_mps2 /
                      maximum_curvature));
    }
    const double path_end_speed = std::sqrt(
        2.0 * config.comfortable_deceleration_mps2 *
        std::max(0.0, remaining_path - config.path_end_buffer_m));
    result.short_path_limited = path_end_speed < speed;
    speed = std::min(speed, path_end_speed);
    result.valid = std::isfinite(speed);
    result.target_speed_mps = std::max(0.0, speed);
    if (!result.valid) {
        result.error = "non-finite speed policy output";
    }
    return result;
}

LongitudinalController::LongitudinalController(LongitudinalConfig config)
    : config_(config) {
    if (!finiteNonNegative(config_.kp) || !finiteNonNegative(config_.ki) ||
        !finiteNonNegative(config_.kd) ||
        !finiteNonNegative(config_.integral_limit) ||
        !(config_.maximum_acceleration_mps2 > 0.0) ||
        !(config_.maximum_deceleration_mps2 > 0.0) ||
        !(config_.acceleration_rate_mps3 > 0.0)) {
        throw std::invalid_argument("invalid longitudinal configuration");
    }
}

double LongitudinalController::update(double target_speed, double measured_speed,
                                      double dt_seconds) {
    if (!finiteNonNegative(target_speed) || !finiteNonNegative(measured_speed) ||
        !(dt_seconds > 0.0) || !std::isfinite(dt_seconds)) {
        throw std::invalid_argument("invalid longitudinal input");
    }
    const double error = target_speed - measured_speed;
    const double candidate_integral = clamp(
        integral_ + error * dt_seconds, -config_.integral_limit,
        config_.integral_limit);
    const double derivative = has_previous_speed_
        ? (measured_speed - previous_speed_) / dt_seconds
        : 0.0;
    const double raw = config_.kp * error + config_.ki * candidate_integral -
                       config_.kd * derivative;
    double acceleration = clamp(raw, -config_.maximum_deceleration_mps2,
                                config_.maximum_acceleration_mps2);
    if (raw == acceleration) {
        integral_ = candidate_integral;
    }
    const double maximum_change = config_.acceleration_rate_mps3 * dt_seconds;
    acceleration = clamp(acceleration,
                         previous_acceleration_ - maximum_change,
                         previous_acceleration_ + maximum_change);
    previous_speed_ = measured_speed;
    previous_acceleration_ = acceleration;
    has_previous_speed_ = true;
    return acceleration;
}

void LongitudinalController::setAppliedAcceleration(double acceleration_mps2) noexcept {
    previous_acceleration_ = std::isfinite(acceleration_mps2)
        ? clamp(acceleration_mps2, -config_.maximum_deceleration_mps2,
                config_.maximum_acceleration_mps2)
        : -config_.maximum_deceleration_mps2;
}

void LongitudinalController::reset() noexcept {
    integral_ = 0.0;
    previous_speed_ = 0.0;
    has_previous_speed_ = false;
}

}  // namespace control
