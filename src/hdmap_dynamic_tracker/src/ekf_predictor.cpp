#include "hdmap_dynamic_tracker/motion_predictor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace hdmap_dynamic_tracker {
namespace {

constexpr std::size_t kStateSize = 6U;
constexpr std::size_t kX = 0U;
constexpr std::size_t kY = 1U;
constexpr std::size_t kSpeed = 2U;
constexpr std::size_t kCourse = 3U;
constexpr std::size_t kTurnRate = 4U;
constexpr std::size_t kAcceleration = 5U;
constexpr double kRequiredPredictionHorizonS = 6.0;
constexpr double kTimestampToleranceS = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

using State = std::array<double, kStateSize>;
using Matrix6 = std::array<double, kStateSize * kStateSize>;

double& matrixAt(Matrix6& matrix, std::size_t row, std::size_t column) {
    return matrix[row * kStateSize + column];
}

double matrixAt(const Matrix6& matrix, std::size_t row, std::size_t column) {
    return matrix[row * kStateSize + column];
}

Matrix6 identityMatrix() {
    Matrix6 result{};
    for (std::size_t index = 0; index < kStateSize; ++index) {
        matrixAt(result, index, index) = 1.0;
    }
    return result;
}

Matrix6 multiply(const Matrix6& left, const Matrix6& right) {
    Matrix6 result{};
    for (std::size_t row = 0; row < kStateSize; ++row) {
        for (std::size_t column = 0; column < kStateSize; ++column) {
            for (std::size_t inner = 0; inner < kStateSize; ++inner) {
                matrixAt(result, row, column) +=
                    matrixAt(left, row, inner) * matrixAt(right, inner, column);
            }
        }
    }
    return result;
}

Matrix6 transpose(const Matrix6& matrix) {
    Matrix6 result{};
    for (std::size_t row = 0; row < kStateSize; ++row) {
        for (std::size_t column = 0; column < kStateSize; ++column) {
            matrixAt(result, column, row) = matrixAt(matrix, row, column);
        }
    }
    return result;
}

double wrapAngle(double angle) {
    if (!std::isfinite(angle)) {
        return 0.0;
    }
    return std::remainder(angle, 2.0 * kPi);
}

double square(double value) {
    return value * value;
}

bool finiteNonNegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

void addOuterProduct(Matrix6& matrix, const State& vector, double variance) {
    for (std::size_t row = 0; row < kStateSize; ++row) {
        for (std::size_t column = 0; column < kStateSize; ++column) {
            matrixAt(matrix, row, column) += vector[row] * variance * vector[column];
        }
    }
}

void symmetrizeAndValidate(Matrix6& covariance) {
    for (std::size_t row = 0; row < kStateSize; ++row) {
        for (std::size_t column = row; column < kStateSize; ++column) {
            const double symmetric = 0.5 *
                (matrixAt(covariance, row, column) + matrixAt(covariance, column, row));
            if (!std::isfinite(symmetric)) {
                throw std::overflow_error("EKF covariance overflow");
            }
            matrixAt(covariance, row, column) = symmetric;
            matrixAt(covariance, column, row) = symmetric;
        }
        if (matrixAt(covariance, row, row) < 0.0) {
            if (matrixAt(covariance, row, row) < -1.0e-9) {
                throw std::overflow_error("EKF covariance lost positive semidefiniteness");
            }
            matrixAt(covariance, row, row) = 0.0;
        }
    }
}

struct Track {
    // CTRA state: map x/y, speed magnitude, course, course turn rate, acceleration.
    // Course is deliberately separate from the packet's body heading.
    State state{};
    Matrix6 covariance{};
    double state_stamp_s = 0.0;
    double observation_stamp_s = 0.0;
    double last_observation_x = 0.0;
    double last_observation_y = 0.0;
    double reported_speed_mps = 0.0;
    double observed_speed_mps = 0.0;
    double observed_velocity_x_mps = 0.0;
    double observed_velocity_y_mps = 0.0;
    bool velocity_from_position_history = false;
    bool velocity_candidate_initialized = false;
    std::size_t consistent_velocity_observations = 0U;
    double velocity_confirmation_start_stamp_s = 0.0;
    double velocity_confirmation_start_x = 0.0;
    double velocity_confirmation_start_y = 0.0;
    bool observed_course_initialized = false;
    double previous_observed_course_rad = 0.0;
    double previous_observed_speed_mps = 0.0;
    double body_heading_rate_radps = 0.0;
    bool body_heading_rate_initialized = false;
    double min_z = 0.0;
    double heading = 0.0;
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
};

class EkfPredictor final : public MotionPredictor {
public:
    explicit EkfPredictor(PredictorConfig config) : config_(config) {
        const double position_variance = square(config_.position_measurement_sigma_m);
        const double velocity_variance = square(config_.initial_velocity_sigma_mps);
        const double acceleration_variance = square(config_.initial_acceleration_sigma_mps2);
        const double minimum_frame_dt_squared = square(config_.minimum_frame_dt_s);
        const double maximum_velocity_measurement_variance =
            2.0 * position_variance / minimum_frame_dt_squared;
        const double maximum_propagation_s =
            std::max(kRequiredPredictionHorizonS, config_.maximum_frame_dt_s);
        const double conservative_position_variance = position_variance +
            square(maximum_propagation_s) * velocity_variance +
            0.25 * square(square(maximum_propagation_s)) * acceleration_variance;

        if (!finiteNonNegative(config_.process_acceleration_sigma_mps2) ||
            !finiteNonNegative(config_.process_jerk_sigma_mps3) ||
            !finiteNonNegative(config_.process_turn_acceleration_sigma_radps2) ||
            !finitePositive(config_.position_measurement_sigma_m) ||
            !finitePositive(config_.speed_measurement_sigma_mps) ||
            !finitePositive(config_.course_measurement_sigma_rad) ||
            !finitePositive(config_.initial_velocity_sigma_mps) ||
            !finitePositive(config_.initial_acceleration_sigma_mps2) ||
            !finitePositive(config_.initial_turn_rate_sigma_radps) ||
            !finiteNonNegative(config_.track_retention_s) ||
            !finitePositive(config_.minimum_frame_dt_s) ||
            !finitePositive(config_.maximum_frame_dt_s) ||
            config_.minimum_frame_dt_s > config_.maximum_frame_dt_s ||
            !finitePositive(config_.maximum_prediction_step_s) ||
            !finitePositive(config_.maximum_position_innovation_m) ||
            !finitePositive(config_.maximum_abs_acceleration_mps2) ||
            !finitePositive(config_.maximum_abs_turn_rate_radps) ||
            config_.minimum_velocity_observations < 2U ||
            !finitePositive(config_.minimum_velocity_observation_span_s) ||
            !finitePositive(config_.minimum_velocity_displacement_m) ||
            !finitePositive(config_.maximum_velocity_innovation_mps) ||
            !std::isfinite(position_variance) ||
            !std::isfinite(velocity_variance) ||
            !std::isfinite(acceleration_variance) ||
            !std::isfinite(square(config_.process_acceleration_sigma_mps2)) ||
            !std::isfinite(square(config_.process_jerk_sigma_mps3)) ||
            !std::isfinite(square(config_.process_turn_acceleration_sigma_radps2)) ||
            !std::isfinite(square(config_.speed_measurement_sigma_mps)) ||
            !std::isfinite(square(config_.course_measurement_sigma_rad)) ||
            !std::isfinite(square(config_.initial_turn_rate_sigma_radps)) ||
            !std::isfinite(maximum_velocity_measurement_variance) ||
            !std::isfinite(conservative_position_variance)) {
            throw std::invalid_argument("Invalid CTRA EKF predictor configuration");
        }
    }

    void reset() override {
        tracks_.clear();
        frame_stamp_s_ = 0.0;
        initialized_ = false;
    }

    void updateFrame(double stamp_s, const std::vector<ObjectObservation>& observations) override {
        if (!std::isfinite(stamp_s)) {
            reset();
            return;
        }
        if (initialized_) {
            const double frame_dt = stamp_s - frame_stamp_s_;
            if (frame_dt < config_.minimum_frame_dt_s ||
                frame_dt > config_.maximum_frame_dt_s) {
                reset();
            }
        }
        initialized_ = true;
        frame_stamp_s_ = stamp_s;

        std::unordered_set<std::uint32_t> observed_ids;
        for (const auto& observation : observations) {
            if (!validObservation(observation) || !observed_ids.insert(observation.id).second) {
                continue;
            }
            auto iterator = tracks_.find(observation.id);
            if (iterator == tracks_.end()) {
                tracks_.emplace(observation.id, initializeTrack(observation, stamp_s));
                continue;
            }

            Track& track = iterator->second;
            predictTrack(track, stamp_s - track.state_stamp_s);
            const double innovation = std::hypot(
                observation.x - track.state[kX], observation.y - track.state[kY]);
            if (!std::isfinite(innovation) || innovation > config_.maximum_position_innovation_m) {
                track = initializeTrack(observation, stamp_s);
                continue;
            }

            updateCoordinate(track, kX, observation.x,
                square(config_.position_measurement_sigma_m), false);
            updateCoordinate(track, kY, observation.y,
                square(config_.position_measurement_sigma_m), false);

            const double observation_dt = stamp_s - track.observation_stamp_s;
            if (observation_dt > 0.0) {
                updateBodyHeading(track, observation.heading, observation_dt);
                updateMotionMeasurements(track, observation, stamp_s, observation_dt);
            }

            // Occupancy starts at the accepted raw packet position. Retaining only
            // the smoothed posterior centre can leave the first future bin behind.
            track.state[kX] = observation.x;
            track.state[kY] = observation.y;
            track.observation_stamp_s = stamp_s;
            track.last_observation_x = observation.x;
            track.last_observation_y = observation.y;
            track.reported_speed_mps = observation.speed;
            track.min_z = observation.min_z;
            track.heading = observation.heading;
            track.length = observation.length;
            track.width = observation.width;
            track.height = observation.height;
        }

        for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
            if (observed_ids.count(iterator->first) == 0U) {
                predictTrack(iterator->second, stamp_s - iterator->second.state_stamp_s);
            }
            if (stamp_s - iterator->second.observation_stamp_s > config_.track_retention_s) {
                iterator = tracks_.erase(iterator);
            } else {
                ++iterator;
            }
        }
    }

    std::vector<ObjectPrediction> predict(double stamp_s, double future_s) const override {
        auto sequence = predictSequence(stamp_s, {future_s});
        if (sequence.size() != 1U) {
            return {};
        }
        return std::move(sequence.front());
    }

    std::vector<std::vector<ObjectPrediction>> predictSequence(
        double stamp_s,
        const std::vector<double>& future_times_s) const override {
        if (!initialized_ || !std::isfinite(stamp_s) || future_times_s.empty()) {
            return {};
        }
        double previous_future_s = -1.0;
        for (const double future_s : future_times_s) {
            if (!std::isfinite(future_s) || future_s < 0.0 ||
                future_s < previous_future_s) {
                return {};
            }
            previous_future_s = future_s;
        }

        std::vector<std::vector<ObjectPrediction>> output(future_times_s.size());
        for (auto& predictions : output) {
            predictions.reserve(tracks_.size());
        }
        for (const auto& entry : tracks_) {
            Track predicted = entry.second;
            for (std::size_t index = 0; index < future_times_s.size(); ++index) {
                const double future_s = future_times_s[index];
                const double propagation_step_s =
                    std::max(0.0, stamp_s + future_s - predicted.state_stamp_s);
                predictTrack(predicted, propagation_step_s);
                output[index].push_back(makePrediction(
                    entry.first, entry.second, predicted, stamp_s, future_s));
            }
        }
        return output;
    }

private:
    ObjectPrediction makePrediction(
        std::uint32_t id,
        const Track& observed,
        const Track& predicted,
        double stamp_s,
        double future_s) const {
        const double x_variance = matrixAt(predicted.covariance, kX, kX);
        const double y_variance = matrixAt(predicted.covariance, kY, kY);
        const double xy_covariance = 0.5 *
            (matrixAt(predicted.covariance, kX, kY) +
                matrixAt(predicted.covariance, kY, kX));
        const double discriminant = std::max(0.0,
            square(x_variance - y_variance) + 4.0 * square(xy_covariance));
        const double maximum_position_variance =
            0.5 * (x_variance + y_variance + std::sqrt(discriminant));
        if (!std::isfinite(maximum_position_variance)) {
            throw std::overflow_error("EKF prediction covariance overflow");
        }

        const double prediction_age_s =
            std::max(0.0, stamp_s - observed.observation_stamp_s) + future_s;
        const double position_derived_speed_mps = std::hypot(
            observed.observed_velocity_x_mps,
            observed.observed_velocity_y_mps);
        const double model_speed_mps = std::max(0.0, predicted.state[kSpeed]);
        double direction_uncertainty_m = 0.0;
        if (observed.velocity_from_position_history) {
            const double observed_course = std::atan2(
                observed.observed_velocity_y_mps,
                observed.observed_velocity_x_mps);
            const double vector_residual_mps = std::hypot(
                position_derived_speed_mps * std::cos(observed_course) -
                    observed.state[kSpeed] * std::cos(observed.state[kCourse]),
                position_derived_speed_mps * std::sin(observed_course) -
                    observed.state[kSpeed] * std::sin(observed.state[kCourse]));
            direction_uncertainty_m = vector_residual_mps * prediction_age_s;
        } else {
            // Until course convergence, the packet only bounds speed magnitude.
            // Cover every possible direction instead of guessing from body yaw.
            direction_uncertainty_m =
                (std::max(observed.reported_speed_mps, position_derived_speed_mps) +
                    model_speed_mps) * prediction_age_s;
        }
        if (!std::isfinite(direction_uncertainty_m)) {
            throw std::overflow_error("EKF direction uncertainty overflow");
        }

        ObjectPrediction prediction;
        prediction.id = id;
        prediction.x = predicted.state[kX];
        prediction.y = predicted.state[kY];
        prediction.min_z = observed.min_z;
        prediction.heading = wrapAngle(
            observed.heading + observed.body_heading_rate_radps * prediction_age_s);
        prediction.length = observed.length;
        prediction.width = observed.width;
        prediction.height = observed.height;
        prediction.position_sigma_m = std::sqrt(std::max(0.0, maximum_position_variance));
        prediction.direction_uncertainty_m = direction_uncertainty_m;
        prediction.age_since_observation_s =
            std::max(0.0, stamp_s - observed.observation_stamp_s);
        prediction.speed_mps = model_speed_mps;
        prediction.course_rad = predicted.state[kCourse];
        prediction.turn_rate_radps = predicted.state[kTurnRate];
        prediction.acceleration_mps2 = predicted.state[kAcceleration];
        prediction.motion_state_converged = observed.velocity_from_position_history;
        return prediction;
    }

    static bool validObservation(const ObjectObservation& observation) {
        return std::isfinite(observation.x) && std::isfinite(observation.y) &&
            std::isfinite(observation.min_z) && std::isfinite(observation.heading) &&
            finiteNonNegative(observation.speed) && finitePositive(observation.length) &&
            finitePositive(observation.width) && finitePositive(observation.height);
    }

    Track initializeTrack(const ObjectObservation& observation, double stamp_s) const {
        Track track;
        // A scalar speed and body heading do not establish a velocity vector.
        // The neutral mean is paired with an omnidirectional reach bound until
        // position history establishes course.
        track.state = {observation.x, observation.y, 0.0, 0.0, 0.0, 0.0};
        matrixAt(track.covariance, kX, kX) = square(config_.position_measurement_sigma_m);
        matrixAt(track.covariance, kY, kY) = square(config_.position_measurement_sigma_m);
        matrixAt(track.covariance, kSpeed, kSpeed) = square(config_.initial_velocity_sigma_mps);
        matrixAt(track.covariance, kCourse, kCourse) = square(kPi);
        matrixAt(track.covariance, kTurnRate, kTurnRate) =
            square(config_.initial_turn_rate_sigma_radps);
        matrixAt(track.covariance, kAcceleration, kAcceleration) =
            square(config_.initial_acceleration_sigma_mps2);
        track.state_stamp_s = stamp_s;
        track.observation_stamp_s = stamp_s;
        track.last_observation_x = observation.x;
        track.last_observation_y = observation.y;
        track.reported_speed_mps = observation.speed;
        track.previous_observed_speed_mps = observation.speed;
        track.min_z = observation.min_z;
        track.heading = observation.heading;
        track.length = observation.length;
        track.width = observation.width;
        track.height = observation.height;
        return track;
    }

    void updateBodyHeading(Track& track, double heading, double dt) const {
        const double measured_rate = std::clamp(
            wrapAngle(heading - track.heading) / dt,
            -2.0 * config_.maximum_abs_turn_rate_radps,
            2.0 * config_.maximum_abs_turn_rate_radps);
        if (track.body_heading_rate_initialized) {
            track.body_heading_rate_radps =
                0.75 * track.body_heading_rate_radps + 0.25 * measured_rate;
        } else {
            track.body_heading_rate_radps = measured_rate;
            track.body_heading_rate_initialized = true;
        }
    }

    void updateMotionMeasurements(
        Track& track,
        const ObjectObservation& observation,
        double stamp_s,
        double dt) const {
        const double displacement_x = observation.x - track.last_observation_x;
        const double displacement_y = observation.y - track.last_observation_y;
        const double displacement_m = std::hypot(displacement_x, displacement_y);
        const double derived_speed_mps = displacement_m / dt;
        const bool has_course = displacement_m > std::numeric_limits<double>::epsilon();
        const double observed_course_rad = has_course ?
            std::atan2(displacement_y, displacement_x) : track.state[kCourse];
        const double observed_speed_mps = has_course ?
            std::max(observation.speed, derived_speed_mps) : observation.speed;
        const double observed_vx_mps = has_course ?
            observed_speed_mps * std::cos(observed_course_rad) : 0.0;
        const double observed_vy_mps = has_course ?
            observed_speed_mps * std::sin(observed_course_rad) : 0.0;

        const bool scalar_speed_agrees =
            std::abs(derived_speed_mps - observation.speed) <=
                config_.maximum_velocity_innovation_mps;
        const bool vector_agrees = !track.velocity_candidate_initialized ||
            std::hypot(observed_vx_mps - track.observed_velocity_x_mps,
                observed_vy_mps - track.observed_velocity_y_mps) <=
                config_.maximum_velocity_innovation_mps;

        if (!scalar_speed_agrees || (!has_course && observation.speed >
            std::numeric_limits<double>::epsilon())) {
            track.velocity_candidate_initialized = false;
            track.consistent_velocity_observations = 0U;
            track.velocity_from_position_history = false;
            track.observed_velocity_x_mps = observed_vx_mps;
            track.observed_velocity_y_mps = observed_vy_mps;
            track.observed_speed_mps = observed_speed_mps;
            return;
        }

        if (!track.velocity_candidate_initialized || !vector_agrees) {
            track.velocity_candidate_initialized = true;
            track.consistent_velocity_observations = 1U;
            track.velocity_confirmation_start_stamp_s = track.observation_stamp_s;
            track.velocity_confirmation_start_x = track.last_observation_x;
            track.velocity_confirmation_start_y = track.last_observation_y;
            track.velocity_from_position_history = false;
        } else if (track.consistent_velocity_observations <
            std::numeric_limits<std::size_t>::max()) {
            ++track.consistent_velocity_observations;
        }

        if (has_course) {
            updateCoordinate(track, kCourse, observed_course_rad,
                square(config_.course_measurement_sigma_rad), true);
        }
        updateCoordinate(track, kSpeed, observed_speed_mps,
            square(config_.speed_measurement_sigma_mps), false);

        if (track.observed_course_initialized && has_course) {
            const double measured_turn_rate = std::clamp(
                wrapAngle(observed_course_rad - track.previous_observed_course_rad) / dt,
                -config_.maximum_abs_turn_rate_radps,
                config_.maximum_abs_turn_rate_radps);
            const double turn_rate_variance = std::max(
                square(config_.process_turn_acceleration_sigma_radps2),
                2.0 * square(config_.course_measurement_sigma_rad) /
                    square(std::max(dt, 0.10)));
            updateCoordinate(track, kTurnRate, measured_turn_rate,
                turn_rate_variance, false);
        }
        const double measured_acceleration = std::clamp(
            (observed_speed_mps - track.previous_observed_speed_mps) / dt,
            -config_.maximum_abs_acceleration_mps2,
            config_.maximum_abs_acceleration_mps2);
        const double acceleration_variance = std::max(
            square(config_.process_acceleration_sigma_mps2),
            2.0 * square(config_.speed_measurement_sigma_mps) /
                square(std::max(dt, 0.10)));
        updateCoordinate(track, kAcceleration, measured_acceleration,
            acceleration_variance, false);

        track.state[kSpeed] = std::max(0.0, track.state[kSpeed]);
        track.state[kTurnRate] = std::clamp(track.state[kTurnRate],
            -config_.maximum_abs_turn_rate_radps,
            config_.maximum_abs_turn_rate_radps);
        track.state[kAcceleration] = std::clamp(track.state[kAcceleration],
            -config_.maximum_abs_acceleration_mps2,
            config_.maximum_abs_acceleration_mps2);

        track.observed_velocity_x_mps = observed_vx_mps;
        track.observed_velocity_y_mps = observed_vy_mps;
        track.observed_speed_mps = observed_speed_mps;
        if (has_course) {
            track.previous_observed_course_rad = observed_course_rad;
            track.observed_course_initialized = true;
        }
        track.previous_observed_speed_mps = observed_speed_mps;

        const double confirmation_span_s =
            stamp_s - track.velocity_confirmation_start_stamp_s;
        const double confirmation_displacement_m = std::hypot(
            observation.x - track.velocity_confirmation_start_x,
            observation.y - track.velocity_confirmation_start_y);
        track.velocity_from_position_history =
            track.consistent_velocity_observations >=
                config_.minimum_velocity_observations &&
            confirmation_span_s + kTimestampToleranceS >=
                config_.minimum_velocity_observation_span_s &&
            confirmation_displacement_m >= config_.minimum_velocity_displacement_m;
    }

    void predictTrack(Track& track, double dt) const {
        if (dt <= 0.0) {
            return;
        }
        const double requested_steps = std::ceil(dt / config_.maximum_prediction_step_s);
        if (!std::isfinite(requested_steps) || requested_steps > 1000000.0) {
            throw std::overflow_error("EKF prediction interval is too large");
        }
        const std::size_t step_count =
            std::max<std::size_t>(1U, static_cast<std::size_t>(requested_steps));
        const double step_s = dt / static_cast<double>(step_count);
        for (std::size_t step = 0; step < step_count; ++step) {
            propagateStep(track, step_s);
        }
        track.state_stamp_s += dt;
    }

    void propagateStep(Track& track, double dt) const {
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double speed = track.state[kSpeed];
        const double course = track.state[kCourse];
        const double turn_rate = track.state[kTurnRate];
        const double acceleration = track.state[kAcceleration];
        const double unconstrained_next_speed = speed + acceleration * dt;
        const double motion_dt = acceleration < 0.0 && unconstrained_next_speed < 0.0 ?
            std::clamp(speed / -acceleration, 0.0, dt) : dt;
        const double motion_dt2 = motion_dt * motion_dt;
        const double midpoint_course = course + 0.5 * turn_rate * motion_dt;
        const double cosine = std::cos(midpoint_course);
        const double sine = std::sin(midpoint_course);
        const double travel = speed * motion_dt + 0.5 * acceleration * motion_dt2;

        Matrix6 transition = identityMatrix();
        matrixAt(transition, kX, kSpeed) = motion_dt * cosine;
        matrixAt(transition, kX, kCourse) = -travel * sine;
        matrixAt(transition, kX, kTurnRate) = -0.5 * motion_dt * travel * sine;
        matrixAt(transition, kX, kAcceleration) = 0.5 * motion_dt2 * cosine;
        matrixAt(transition, kY, kSpeed) = motion_dt * sine;
        matrixAt(transition, kY, kCourse) = travel * cosine;
        matrixAt(transition, kY, kTurnRate) = 0.5 * motion_dt * travel * cosine;
        matrixAt(transition, kY, kAcceleration) = 0.5 * motion_dt2 * sine;
        matrixAt(transition, kSpeed, kAcceleration) = dt;
        matrixAt(transition, kCourse, kTurnRate) = dt;

        track.state[kX] += travel * cosine;
        track.state[kY] += travel * sine;
        track.state[kSpeed] = std::max(0.0, unconstrained_next_speed);
        track.state[kCourse] = wrapAngle(course + turn_rate * dt);
        if (unconstrained_next_speed < 0.0) {
            track.state[kAcceleration] = 0.0;
        }

        Matrix6 covariance = multiply(
            multiply(transition, track.covariance), transpose(transition));

        const State unmodelled_acceleration{
            0.5 * dt2 * cosine,
            0.5 * dt2 * sine,
            dt,
            0.0,
            0.0,
            0.0};
        addOuterProduct(covariance, unmodelled_acceleration,
            square(config_.process_acceleration_sigma_mps2));

        const State jerk{
            dt3 * cosine / 6.0,
            dt3 * sine / 6.0,
            0.5 * dt2,
            0.0,
            0.0,
            dt};
        addOuterProduct(covariance, jerk, square(config_.process_jerk_sigma_mps3));

        const State turn_acceleration{
            0.0,
            0.0,
            0.0,
            0.5 * dt2,
            dt,
            0.0};
        addOuterProduct(covariance, turn_acceleration,
            square(config_.process_turn_acceleration_sigma_radps2));
        symmetrizeAndValidate(covariance);
        track.covariance = covariance;
    }

    void updateCoordinate(
        Track& track,
        std::size_t coordinate,
        double measurement,
        double measurement_variance,
        bool angular) const {
        const double innovation_variance =
            matrixAt(track.covariance, coordinate, coordinate) + measurement_variance;
        if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
            throw std::overflow_error("EKF innovation covariance is invalid");
        }
        State gain{};
        for (std::size_t row = 0; row < kStateSize; ++row) {
            gain[row] = matrixAt(track.covariance, row, coordinate) / innovation_variance;
        }
        double residual = measurement - track.state[coordinate];
        if (angular) {
            residual = wrapAngle(residual);
        }
        for (std::size_t row = 0; row < kStateSize; ++row) {
            track.state[row] += gain[row] * residual;
        }
        track.state[kCourse] = wrapAngle(track.state[kCourse]);

        Matrix6 residual_projection = identityMatrix();
        for (std::size_t row = 0; row < kStateSize; ++row) {
            matrixAt(residual_projection, row, coordinate) -= gain[row];
        }
        Matrix6 covariance = multiply(
            multiply(residual_projection, track.covariance), transpose(residual_projection));
        for (std::size_t row = 0; row < kStateSize; ++row) {
            for (std::size_t column = 0; column < kStateSize; ++column) {
                matrixAt(covariance, row, column) +=
                    gain[row] * measurement_variance * gain[column];
            }
        }
        symmetrizeAndValidate(covariance);
        track.covariance = covariance;
    }

    PredictorConfig config_;
    std::unordered_map<std::uint32_t, Track> tracks_;
    bool initialized_ = false;
    double frame_stamp_s_ = 0.0;
};

}  // namespace

std::unique_ptr<MotionPredictor> makeEkfPredictor(const PredictorConfig& config) {
    return std::make_unique<EkfPredictor>(config);
}

}  // namespace hdmap_dynamic_tracker
