#include "hdmap_dynamic_tracker/motion_predictor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace hdmap_dynamic_tracker {
namespace {

using Matrix4 = std::array<double, 16>;
constexpr double kRequiredPredictionHorizonS = 6.0;
constexpr double kTimestampToleranceS = 1.0e-9;

double& matrixAt(Matrix4& matrix, std::size_t row, std::size_t column) {
    return matrix[row * 4 + column];
}

double matrixAt(const Matrix4& matrix, std::size_t row, std::size_t column) {
    return matrix[row * 4 + column];
}

Matrix4 multiply(const Matrix4& left, const Matrix4& right) {
    Matrix4 result{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t inner = 0; inner < 4; ++inner) {
                matrixAt(result, row, column) += matrixAt(left, row, inner) * matrixAt(right, inner, column);
            }
        }
    }
    return result;
}

Matrix4 transpose(const Matrix4& matrix) {
    Matrix4 result{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            matrixAt(result, column, row) = matrixAt(matrix, row, column);
        }
    }
    return result;
}

struct Track {
    std::array<double, 4> state{};  // x, y, vx, vy
    Matrix4 covariance{};
    double state_stamp_s = 0.0;
    double observation_stamp_s = 0.0;
    double last_observation_x = 0.0;
    double last_observation_y = 0.0;
    double min_z = 0.0;
    double heading = 0.0;
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
};

class EkfPredictor final : public MotionPredictor {
public:
    explicit EkfPredictor(PredictorConfig config) : config_(config) {
        const double process_variance = config_.process_acceleration_sigma_mps2 *
            config_.process_acceleration_sigma_mps2;
        const double position_variance = config_.position_measurement_sigma_m *
            config_.position_measurement_sigma_m;
        const double velocity_variance = config_.initial_velocity_sigma_mps *
            config_.initial_velocity_sigma_mps;
        const double minimum_frame_dt_squared =
            config_.minimum_frame_dt_s * config_.minimum_frame_dt_s;
        const double maximum_velocity_measurement_variance =
            2.0 * position_variance / minimum_frame_dt_squared;
        const double maximum_propagation_s =
            kRequiredPredictionHorizonS;
        const double propagation_squared = maximum_propagation_s * maximum_propagation_s;
        const double propagation_fourth = propagation_squared * propagation_squared;
        const double propagated_position_variance = position_variance +
            propagation_squared * velocity_variance +
            0.25 * propagation_fourth * process_variance;
        if (!std::isfinite(config_.process_acceleration_sigma_mps2) ||
            config_.process_acceleration_sigma_mps2 < 0.0 ||
            !std::isfinite(config_.position_measurement_sigma_m) ||
            config_.position_measurement_sigma_m <= 0.0 ||
            !std::isfinite(config_.initial_velocity_sigma_mps) ||
            config_.initial_velocity_sigma_mps <= 0.0 ||
            !std::isfinite(config_.minimum_frame_dt_s) ||
            config_.minimum_frame_dt_s <= 0.0 ||
            !std::isfinite(process_variance) ||
            !std::isfinite(position_variance) || position_variance <= 0.0 ||
            !std::isfinite(velocity_variance) || velocity_variance <= 0.0 ||
            !std::isfinite(minimum_frame_dt_squared) ||
            minimum_frame_dt_squared <= 0.0 ||
            !std::isfinite(maximum_velocity_measurement_variance) ||
            !std::isfinite(propagated_position_variance)) {
            throw std::invalid_argument("Invalid EKF predictor configuration");
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
        initialized_ = true;
        frame_stamp_s_ = stamp_s;

        std::unordered_set<std::uint32_t> observed_ids;
        for (const auto& observation : observations) {
            observed_ids.insert(observation.id);
            auto iterator = tracks_.find(observation.id);
            if (iterator == tracks_.end()) {
                tracks_.emplace(observation.id, initializeTrack(observation, stamp_s));
                continue;
            }
            predictTrack(iterator->second, stamp_s - iterator->second.state_stamp_s);
            updatePosition(iterator->second, 0, observation.x);
            updatePosition(iterator->second, 1, observation.y);
            const double observation_dt = stamp_s - iterator->second.observation_stamp_s;
            if (observation_dt > 0.0) {
                const double displacement_x =
                    observation.x - iterator->second.last_observation_x;
                const double displacement_y =
                    observation.y - iterator->second.last_observation_y;
                const double displacement = std::hypot(displacement_x, displacement_y);
                const double velocity_variance = std::max(
                    config_.position_measurement_sigma_m *
                        config_.position_measurement_sigma_m * 2.0 /
                        (observation_dt * observation_dt),
                    0.25);
                if (displacement > std::numeric_limits<double>::epsilon()) {
                    const double speed = std::max(observation.speed, displacement / observation_dt);
                    updateCoordinate(iterator->second, 2,
                        displacement_x / displacement * speed, velocity_variance);
                    updateCoordinate(iterator->second, 3,
                        displacement_y / displacement * speed, velocity_variance);
                }
            }
            // Occupancy prediction must start at the accepted raw observation. A
            // smoothed posterior position can remain metres behind after a valid
            // innovation and leave every later future bin on the wrong trajectory.
            iterator->second.state[0] = observation.x;
            iterator->second.state[1] = observation.y;
            iterator->second.observation_stamp_s = stamp_s;
            iterator->second.last_observation_x = observation.x;
            iterator->second.last_observation_y = observation.y;
            iterator->second.min_z = observation.min_z;
            iterator->second.heading = observation.heading;
            iterator->second.length = observation.length;
            iterator->second.width = observation.width;
            iterator->second.height = observation.height;
        }

        for (auto& [id, track] : tracks_) {
            if (observed_ids.count(id) == 0U) {
                predictTrack(track, stamp_s - track.state_stamp_s);
            }
        }
    }

    std::vector<ObjectPrediction> predict(double stamp_s, double future_s) const override {
        std::vector<ObjectPrediction> output;
        if (!initialized_ || !std::isfinite(stamp_s) || !std::isfinite(future_s) || future_s < 0.0) {
            return output;
        }
        output.reserve(tracks_.size());
        for (const auto& entry : tracks_) {
            const auto& track = entry.second;
            const double dt = std::max(0.0, stamp_s + future_s - track.state_stamp_s);
            const double process_variance = config_.process_acceleration_sigma_mps2 *
                config_.process_acceleration_sigma_mps2;
            const double x_variance = matrixAt(track.covariance, 0, 0) +
                2.0 * dt * matrixAt(track.covariance, 0, 2) +
                dt * dt * matrixAt(track.covariance, 2, 2) +
                0.25 * dt * dt * dt * dt * process_variance;
            const double y_variance = matrixAt(track.covariance, 1, 1) +
                2.0 * dt * matrixAt(track.covariance, 1, 3) +
                dt * dt * matrixAt(track.covariance, 3, 3) +
                0.25 * dt * dt * dt * dt * process_variance;
            if (!std::isfinite(x_variance) || !std::isfinite(y_variance)) {
                throw std::overflow_error("EKF prediction covariance overflow");
            }
            output.push_back({entry.first,
                track.state[0] + track.state[2] * dt,
                track.state[1] + track.state[3] * dt,
                track.min_z,
                track.heading,
                track.length,
                track.width,
                track.height,
                std::sqrt(std::max(0.0, std::max(x_variance, y_variance))),
                0.0,
                std::max(0.0, stamp_s - track.observation_stamp_s)});
        }
        return output;
    }

private:
    Track initializeTrack(const ObjectObservation& observation, double stamp_s) const {
        Track track;
        // The packet heading is body orientation, not a guaranteed velocity direction.
        // Keep velocity neutral until position history passes the convergence gate.
        track.state = {observation.x, observation.y, 0.0, 0.0};
        const double position_variance = config_.position_measurement_sigma_m *
            config_.position_measurement_sigma_m;
        const double velocity_variance = config_.initial_velocity_sigma_mps *
            config_.initial_velocity_sigma_mps;
        matrixAt(track.covariance, 0, 0) = position_variance;
        matrixAt(track.covariance, 1, 1) = position_variance;
        matrixAt(track.covariance, 2, 2) = velocity_variance;
        matrixAt(track.covariance, 3, 3) = velocity_variance;
        track.state_stamp_s = stamp_s;
        track.observation_stamp_s = stamp_s;
        track.last_observation_x = observation.x;
        track.last_observation_y = observation.y;
        track.min_z = observation.min_z;
        track.heading = observation.heading;
        track.length = observation.length;
        track.width = observation.width;
        track.height = observation.height;
        return track;
    }

    void predictTrack(Track& track, double dt) const {
        if (dt <= 0.0) {
            return;
        }
        Matrix4 transition{1.0, 0.0, dt, 0.0,
                           0.0, 1.0, 0.0, dt,
                           0.0, 0.0, 1.0, 0.0,
                           0.0, 0.0, 0.0, 1.0};
        track.state[0] += track.state[2] * dt;
        track.state[1] += track.state[3] * dt;
        track.covariance = multiply(multiply(transition, track.covariance), transpose(transition));

        const double variance = config_.process_acceleration_sigma_mps2 *
            config_.process_acceleration_sigma_mps2;
        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt2 * dt2;
        matrixAt(track.covariance, 0, 0) += 0.25 * dt4 * variance;
        matrixAt(track.covariance, 0, 2) += 0.5 * dt3 * variance;
        matrixAt(track.covariance, 2, 0) += 0.5 * dt3 * variance;
        matrixAt(track.covariance, 2, 2) += dt2 * variance;
        matrixAt(track.covariance, 1, 1) += 0.25 * dt4 * variance;
        matrixAt(track.covariance, 1, 3) += 0.5 * dt3 * variance;
        matrixAt(track.covariance, 3, 1) += 0.5 * dt3 * variance;
        matrixAt(track.covariance, 3, 3) += dt2 * variance;
        track.state_stamp_s += dt;
    }

    void updatePosition(Track& track, std::size_t coordinate, double measurement) const {
        const double measurement_variance = config_.position_measurement_sigma_m *
            config_.position_measurement_sigma_m;
        updateCoordinate(track, coordinate, measurement, measurement_variance);
    }

    void updateCoordinate(
        Track& track,
        std::size_t coordinate,
        double measurement,
        double measurement_variance) const {
        const double innovation_variance = matrixAt(track.covariance, coordinate, coordinate) +
            measurement_variance;
        std::array<double, 4> gain{};
        for (std::size_t row = 0; row < 4; ++row) {
            gain[row] = matrixAt(track.covariance, row, coordinate) / innovation_variance;
        }
        const double residual = measurement - track.state[coordinate];
        for (std::size_t row = 0; row < 4; ++row) {
            track.state[row] += gain[row] * residual;
        }
        Matrix4 residual_projection{1.0, 0.0, 0.0, 0.0,
                                    0.0, 1.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0, 0.0,
                                    0.0, 0.0, 0.0, 1.0};
        for (std::size_t row = 0; row < 4; ++row) {
            matrixAt(residual_projection, row, coordinate) -= gain[row];
        }
        Matrix4 covariance = multiply(
            multiply(residual_projection, track.covariance), transpose(residual_projection));
        for (std::size_t row = 0; row < 4; ++row) {
            for (std::size_t column = 0; column < 4; ++column) {
                matrixAt(covariance, row, column) +=
                    gain[row] * measurement_variance * gain[column];
            }
        }
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
