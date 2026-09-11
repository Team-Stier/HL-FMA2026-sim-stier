#include "hdmap_dynamic_tracker/motion_predictor.hpp"
#include "hdmap_dynamic_tracker/signal_state_manager.hpp"
#include "hdmap_dynamic_tracker/tracker_core.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace hdmap_dynamic_tracker;

namespace {

void expectNear(double actual, double expected, double tolerance = 1.0e-6) {
    assert(std::abs(actual - expected) <= tolerance);
}

std::pair<double, double> xBounds(const std::vector<Point2d>& polygon) {
    assert(!polygon.empty());
    double minimum = polygon.front().x;
    double maximum = polygon.front().x;
    for (const auto& point : polygon) {
        minimum = std::min(minimum, point.x);
        maximum = std::max(maximum, point.x);
    }
    return {minimum, maximum};
}

bool containsPoint(const std::vector<Point2d>& polygon, const Point2d& point) {
    assert(polygon.size() >= 3U);
    assert(std::isfinite(point.x));
    assert(std::isfinite(point.y));
    double orientation = 0.0;
    for (std::size_t index = 0; index < polygon.size(); ++index) {
        const auto& start = polygon[index];
        const auto& end = polygon[(index + 1) % polygon.size()];
        assert(std::isfinite(start.x));
        assert(std::isfinite(start.y));
        assert(std::isfinite(end.x));
        assert(std::isfinite(end.y));
        const double cross = (end.x - start.x) * (point.y - start.y) -
            (end.y - start.y) * (point.x - start.x);
        if (std::abs(cross) <= 1.0e-8) {
            continue;
        }
        if (orientation == 0.0) {
            orientation = cross;
        } else if (orientation * cross < 0.0) {
            return false;
        }
    }
    return true;
}

void testEgoSpeed() {
    EgoSpeedEstimator estimator;
    expectNear(estimator.update(1.0, 0.0, 0.0, 0.0, 0.0).speed_mps, 0.0);
    expectNear(estimator.update(1.1, 0.3, 0.4, 0.0, 0.0).speed_mps, 5.0);
    expectNear(estimator.update(1.6, 0.9, 1.2, 0.0, 0.0).speed_mps, 2.0);
    const auto stationary = estimator.update(2.0, 0.9, 1.2, 0.0, 0.0);
    assert(!stationary.history_reset);
    expectNear(stationary.speed_mps, 0.0);
    const auto jump = estimator.update(2.1, 99.9, 1.2, 0.0, 0.0);
    assert(jump.history_reset);
    expectNear(jump.speed_mps, 0.0); // The old implementation published 990 m/s.
    expectNear(estimator.update(2.2, 100.0, 1.2, 0.0, 0.0).speed_mps, 1.0);
    const auto gap = estimator.update(4.0, 101.0, 1.2, 0.0, 0.0);
    assert(gap.history_reset);
    expectNear(gap.speed_mps, 0.0);
    expectNear(estimator.update(4.1, 101.1, 1.2, 0.0, 0.0).speed_mps, 1.0);
    assert(estimator.update(4.1, 102.0, 1.2, 0.0, 0.0).history_reset);
    assert(estimator.update(4.0, 102.0, 1.2, 0.0, 0.0).history_reset);
    assert(estimator.update(4.000001, 102.0, 1.2, 0.0, 0.0).history_reset);

    // Long, physically possible movement is not an absolute-distance respawn.
    estimator.reset();
    estimator.update(10.0, 0.0, 0.0, 0.0, 0.0);
    const auto fast = estimator.update(10.6, 30.0, 0.0, 3.0, 0.3);
    assert(!fast.history_reset);
    expectNear(fast.speed_mps, 50.0);

    // Recorded upper-to-lower road jump: XY speed alone is below the 60 m/s bound.
    estimator.reset();
    estimator.update(20.0, 1447.083496, 865.331848, 52.923733, -1.529468);
    const auto height_jump = estimator.update(
        20.0794, 1448.383057, 861.187134, 48.970463, -1.131964);
    assert(height_jump.history_reset);
    expectNear(height_jump.speed_mps, 0.0);

    const double pi = std::acos(-1.0);
    estimator.reset();
    estimator.update(30.0, 0.0, 0.0, 0.0, pi - 0.01);
    assert(!estimator.update(30.1, 0.1, 0.0, 0.0, -pi + 0.01).history_reset);
    const auto heading_jump = estimator.update(30.14, 0.2, 0.0, 0.0, -pi + 0.61);
    assert(heading_jump.history_reset);
    expectNear(heading_jump.speed_mps, 0.0);
    expectNear(estimator.update(30.24, 0.3, 0.0, 0.0, -pi + 0.62).speed_mps, 1.0);

    const auto non_finite = estimator.update(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0);
    assert(non_finite.history_reset);
    expectNear(non_finite.speed_mps, 0.0);
    expectNear(estimator.update(40.0, 0.0, 0.0, 0.0, 0.0).speed_mps, 0.0);
    assert(estimator.update(40.1, 0.0, 0.0,
        std::numeric_limits<double>::infinity(), 0.0).history_reset);

    EgoMotionLimits invalid;
    invalid.maximum_frame_dt_s = invalid.minimum_frame_dt_s;
    bool rejected = false;
    try { EgoSpeedEstimator bad(invalid); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}

void testParsersAndRamp() {
    expectNear(*parseSpeedLimitMps("8.0 m/s"), 8.0);
    expectNear(*parseSpeedLimitMps("36 km/h"), 10.0);
    assert(!parseSpeedLimitMps("fast"));
    assert(!parseSpeedLimitMps("-1 m/s"));
    expectNear(*parsePositiveMetres("0.75"), 0.75);
    assert(!parsePositiveMetres("0"));
    const auto values = parseIntegerCsv("5, 3,5");
    assert((values == std::vector<int>{3, 5}));
    assert(parseIntegerCsv("3,bad").empty());
    assert(parseIntegerCsv("3,,5").empty());

    const std::vector<BrakingDistanceSample> calibration{
        {2.0, 1.2}, {4.0, 4.5}, {8.0, 18.0}};
    assert(validBrakingDistanceTable(calibration));
    expectNear(*conservativeBrakingDistance(0.0, calibration), 0.0);
    expectNear(*conservativeBrakingDistance(1.0, calibration), 1.2);
    expectNear(*conservativeBrakingDistance(2.0, calibration), 1.2);
    expectNear(*conservativeBrakingDistance(3.0, calibration), 4.5);
    expectNear(*conservativeBrakingDistance(8.0, calibration), 18.0);
    assert(!conservativeBrakingDistance(8.01, calibration));
    assert(!conservativeBrakingDistance(-1.0, calibration));
    assert(!validBrakingDistanceTable({}));
    assert(!validBrakingDistanceTable({{2.0, 1.0}, {2.0, 2.0}}));
    assert(!validBrakingDistanceTable({{2.0, 2.0}, {4.0, 1.0}}));
    assert(!validBrakingDistanceTable({{2.0, 0.0}}));

    StopRampParameters ramp{8.0, 2.0, 1.0, 0.0, 7.0, std::nullopt};
    expectNear(stopRampProfileLength(ramp), 16.0);
    expectNear(stopRampStartDistance(ramp), 23.0);
    expectNear(stopRampCap(0.0, ramp), 0.0);
    expectNear(stopRampCap(15.0, ramp), std::sqrt(32.0));
    expectNear(stopRampCap(23.0, ramp), 8.0);
    for (int metre = -5; metre <= 50; ++metre) {
        const double cap = stopRampCap(static_cast<double>(metre), ramp);
        assert(cap >= 0.0 && cap <= ramp.entry_speed_mps);
        if (metre > -5) {
            assert(cap >= stopRampCap(static_cast<double>(metre - 1), ramp));
        }
    }
    expectNear(stopRampCap(7.0, ramp), 0.0);

    StopRampParameters longer = ramp;
    longer.braking_distance_factor = 2.0;
    expectNear(stopRampProfileLength(longer), 32.0);
    expectNear(stopRampStartDistance(longer), 39.0);
    expectNear(stopRampCap(23.0, longer), std::sqrt(32.0));

    StopRampParameters calibrated = ramp;
    calibrated.calibrated_braking_distance_m = 20.0;
    expectNear(stopRampProfileLength(calibrated), 20.0);
    expectNear(stopRampStartDistance(calibrated), 27.0);
    expectNear(stopRampCap(17.0, calibrated), std::sqrt(32.0));
    calibrated.calibrated_braking_distance_m = 10.0;
    expectNear(stopRampProfileLength(calibrated), 16.0);

    assert(stopRampCap(std::numeric_limits<double>::quiet_NaN(), ramp) == 0.0);
    assert(stopRampCap(std::numeric_limits<double>::infinity(), ramp) == 0.0);
    calibrated.calibrated_braking_distance_m = 0.0;
    assert(!validStopRamp(calibrated));
    ramp.design_deceleration_mps2 = 0.0;
    assert(!validStopRamp(ramp));
    expectNear(stopRampCap(100.0, ramp), 0.0);
    ramp = StopRampParameters{
        std::numeric_limits<double>::max(), 0.1, 1.0, 1.0, 1.0, std::nullopt};
    assert(!validStopRamp(ramp));
    ramp = StopRampParameters{
        std::sqrt(std::numeric_limits<double>::max() / 2.0), 1.0, 1.0, 0.0,
        std::numeric_limits<double>::max(), std::nullopt};
    assert(!validStopRamp(ramp));
}

void testStopRampRespectsBrakingAndLatency() {
    for (double latency : {0.0, 0.25, 1.0}) {
        for (double factor : {1.0, 1.5}) {
            for (const auto calibration : {std::optional<double>{}, std::optional<double>{20.0}}) {
                StopRampParameters ramp{8.0, 3.0, factor, latency, 7.0, calibration};
                const double braking_distance = factor * std::max(64.0 / 6.0,
                    calibration.value_or(0.0));
                const double coefficient = braking_distance / 64.0;
                const double start = stopRampStartDistance(ramp);
                expectNear(stopRampCap(start, ramp), 8.0);
                double previous_speed = 0.0;
                constexpr double step = 0.01;
                for (double remaining = 0.0; remaining <= start + 2.0; remaining += step) {
                    const double speed = stopRampCap(remaining, ramp);
                    assert(speed + 1.0e-9 >= previous_speed);
                    if (speed > 0.0) {
                        assert(coefficient * speed * speed + latency * speed + 7.0 <=
                            remaining + 1.0e-8);
                    }
                    const double required_brake =
                        (speed * speed - previous_speed * previous_speed) / (2.0 * step);
                    assert(required_brake <= 3.0 + 1.0e-7);
                    previous_speed = speed;
                }
            }
        }
    }
    // Scaling only the entry calibration misses a worse low-speed bin.
    const std::vector<BrakingDistanceSample> table{{2.0, 10.0}, {10.0, 20.0}};
    for (double latency : {0.0, 0.25}) {
        StopRampParameters ramp{10.0, 3.0, 1.0, latency, 7.0, std::nullopt};
        const double start = stopRampStartDistance(ramp, table);
        expectNear(start, 43.0 + 10.0 * latency);
        double previous_speed = 0.0;
        constexpr double step = 0.01;
        for (double distance = 0.0; distance <= start + 1.0; distance += step) {
            const double speed = stopRampCap(distance, ramp, table);
            assert(speed + 1.0e-9 >= previous_speed);
            if (speed > 1.0e-9) {
                const auto measured = conservativeBrakingDistance(speed, table);
                assert(measured);
                assert(std::max(speed * speed / 6.0, *measured) + speed * latency + 7.0 <=
                    distance + 1.0e-8);
            }
            assert((speed * speed - previous_speed * previous_speed) / (2.0 * step) <=
                3.0 + 1.0e-7);
            previous_speed = speed;
        }
        expectNear(stopRampCap(start, ramp, table), 10.0);
        expectNear(stopRampCap(16.99, ramp, table), 0.0); // Lower bin still needs 10 m.
        ramp.entry_speed_mps = 10.01;
        assert(!validStopRamp(ramp, table));
        expectNear(stopRampCap(100.0, ramp, table), 0.0);
        ramp.entry_speed_mps = 8.0;
        assert(!validStopRamp(ramp, {{2.0, 10.0}, {2.0, 20.0}}));
    }
    StopRampParameters unsupported{8.0, 11.0, 1.0, 0.25, 7.0, std::nullopt};
    assert(!validStopRamp(unsupported));
    expectNear(stopRampCap(100.0, unsupported), 0.0);
    SignalStateManagerConfig invalid;
    invalid.yellow_decision_deceleration_mps2 = 3.01;
    bool rejected = false;
    try { SignalStateManager manager(invalid); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}

void testYellowSignalStateManager() {
    SignalStateManagerConfig config;
    config.yellow_decision_deceleration_mps2 = 3.0;
    config.braking_distance_factor = 1.0;
    config.latency_budget_s = 0.25;
    config.stop_margin_m = 1.0;
    config.hold_speed_mps = 0.2;
    config.hold_distance_m = 1.0;

    const double fifty_kph_mps = 50.0 / 3.6;
    expectNear(ioniq6StoppingDistance(fifty_kph_mps, config),
        fifty_kph_mps * fifty_kph_mps / 6.0 +
            fifty_kph_mps * 0.25 + 1.0);
    expectNear(ioniq6StoppingDistance(fifty_kph_mps, config), 36.622427983539097);

    SignalStateInput input;
    input.controller_id = 27;
    input.approach_id = 4;
    input.signal_state = 2;
    input.on_approach = true;
    input.ego_speed_mps = fifty_kph_mps;
    input.front_axle_to_stopline_m = 40.0;
    input.static_speed_cap_mps = fifty_kph_mps;

    SignalStateManager stop_manager(config);
    input.signal_state = 3;
    input.permitted = true;
    auto result = stop_manager.update(input);
    assert(result.state == SignalApproachState::Go);

    input.signal_state = 2;
    input.permitted = false;
    result = stop_manager.update(input);
    assert(result.state == SignalApproachState::StopRequired);
    assert(result.apply_stop_cap);
    assert(result.transitioned);

    input.front_axle_to_stopline_m = 20.0;
    result = stop_manager.update(input);
    assert(result.state == SignalApproachState::StopRequired);
    assert(result.apply_stop_cap);

    input.signal_state = 1;

    input.ego_speed_mps = 0.1;
    input.front_axle_to_stopline_m = 0.8;
    result = stop_manager.update(input);
    assert(result.state == SignalApproachState::Hold);
    assert(result.apply_stop_cap);

    input.signal_state = 3;
    input.permitted = true;
    result = stop_manager.update(input);
    assert(result.state == SignalApproachState::Go);
    assert(!result.apply_stop_cap);

    SignalStateManager committed_manager(config);
    input.ego_speed_mps = fifty_kph_mps;
    input.front_axle_to_stopline_m = 30.0;
    input.signal_state = 3;
    input.permitted = true;
    result = committed_manager.update(input);
    assert(result.state == SignalApproachState::Go);

    input.signal_state = 2;
    input.permitted = false;
    result = committed_manager.update(input);
    assert(result.state == SignalApproachState::Committed);
    assert(!result.apply_stop_cap);
    expectNear(result.target_speed_cap_mps, fifty_kph_mps);

    input.signal_state = 1;
    input.front_axle_to_stopline_m = 100.0;
    result = committed_manager.update(input);
    assert(result.state == SignalApproachState::Committed);
    assert(!result.apply_stop_cap);

    input.on_approach = false;
    result = committed_manager.update(input);
    assert(result.state == SignalApproachState::Clearing);
    assert(!result.apply_stop_cap);
    result = committed_manager.update(input);
    assert(result.state == SignalApproachState::Clearing);

    input.controller_id = 28;
    result = committed_manager.update(input);
    assert(result.state == SignalApproachState::Unknown);

    const std::vector<BrakingDistanceSample> calibration{
        {10.0, 20.0}, {15.0, 40.0}};
    expectNear(ioniq6StoppingDistance(fifty_kph_mps, config, calibration),
        40.0 + (fifty_kph_mps * fifty_kph_mps - 100.0) / 6.0 +
            fifty_kph_mps * 0.25 + 1.0);
    assert(std::isinf(ioniq6StoppingDistance(16.0, config, calibration)));
}

void testGeometry() {
    const auto huge_angle = normalizeAngle(std::numeric_limits<double>::max());
    assert(std::isfinite(huge_angle));
    assert(std::abs(huge_angle) <= std::acos(-1.0));

    ObjectPrediction object;
    object.x = 10.0;
    object.y = 20.0;
    object.heading = 0.0;
    object.length = 4.0;
    object.width = 2.0;
    const auto box = orientedBox(object);
    assert(box.size() == 4);
    expectNear(box.front().x, 8.0);
    expectNear(box.front().y, 19.0);

    ObjectPrediction end = object;
    end.x = 15.0;
    const auto swept = sweptFootprint(object, end);
    assert(swept.size() == 4);
    double minimum_x = swept.front().x;
    double maximum_x = swept.front().x;
    for (const auto& point : swept) {
        minimum_x = std::min(minimum_x, point.x);
        maximum_x = std::max(maximum_x, point.x);
    }
    expectNear(minimum_x, 8.0);
    expectNear(maximum_x, 17.0);

    object.heading = std::acos(-1.0) * 0.5;
    const auto rotated = orientedBox(object);
    expectNear(rotated.front().x, 11.0);
    expectNear(rotated.front().y, 18.0);

    expectNear(predictionUncertaintyInflation(36.0, 4.0, 2.0), 76.0);
    assert(predictionUncertaintyInflation(36.0, 0.0, 2.0) > 5.0);
    const double yaw_inflation = yawIndependentRotationInflation(4.0, 2.0);
    expectNear(yaw_inflation, std::sqrt(5.0) - 1.0);
    object.heading = 0.0;
    const auto yaw_independent_envelope = orientedBox(object, yaw_inflation);
    for (int step = 0; step <= 720; ++step) {
        object.heading = 2.0 * std::acos(-1.0) * static_cast<double>(step) / 720.0;
        for (const auto& corner : orientedBox(object)) {
            assert(containsPoint(yaw_independent_envelope, corner));
        }
    }
}

void testPredictor() {
    PredictorConfig config;
    auto predictor = makeEkfPredictor(config);
    ObjectObservation observation;
    observation.id = 7;
    observation.x = 0.0;
    observation.y = 0.0;
    observation.heading = 0.0;
    observation.speed = 10.0;
    observation.length = 4.0;
    observation.width = 2.0;
    observation.height = 1.5;
    predictor->updateFrame(10.0, {observation});
    auto prediction = predictor->predict(10.0, 0.5);
    assert(prediction.size() == 1);
    expectNear(prediction.front().x, 0.0);
    expectNear(prediction.front().direction_uncertainty_m, 0.0);

    observation.x = 1.0;
    predictor->updateFrame(10.1, {observation});
    prediction = predictor->predict(10.1, 0.0);
    assert(prediction.size() == 1);
    assert(prediction.front().x > 0.0 && prediction.front().x < 1.01);
    expectNear(prediction.front().direction_uncertainty_m, 0.0);
    expectNear(predictor->predict(10.1, 0.5).front().direction_uncertainty_m, 0.0);

    predictor->updateFrame(10.2, {});
    assert(predictor->predict(10.2, 0.0).size() == 1);
    predictor->updateFrame(10.3, {});
    assert(predictor->predict(10.3, 0.0).size() == 1);
    predictor->updateFrame(10.8, {});
    assert(predictor->predict(10.8, 0.0).size() == 1);

    predictor->updateFrame(11.0, {observation});
    observation.x = 1.0;
    observation.y = 1.0;
    observation.heading = 0.0;  // Orientation is not forced to be the velocity direction.
    predictor->updateFrame(11.1, {observation});
    const auto off_heading = predictor->predict(11.1, 0.5);
    assert(off_heading.front().y > 1.0);

    for (int index = 1; index <= 2000; ++index) {
        const double stamp = 11.1 + static_cast<double>(index) * 0.05;
        observation.x += 0.05;
        observation.y += 0.025;
        predictor->updateFrame(stamp, {observation});
        const auto soak = predictor->predict(stamp, 6.0);
        assert(soak.size() == 1);
        assert(std::isfinite(soak.front().x));
        assert(std::isfinite(soak.front().y));
        assert(std::isfinite(soak.front().position_sigma_m));
        assert(soak.front().position_sigma_m >= 0.0);
    }
}

void testVelocityUsesNextPositionSample() {
    PredictorConfig config;
    auto predictor = makeEkfPredictor(config);
    ObjectObservation observation;
    observation.id = 8;
    observation.speed = 10.0;
    observation.length = 4.0;
    observation.width = 2.0;
    observation.height = 1.5;
    predictor->updateFrame(15.0, {observation});
    observation.x = 0.5;
    predictor->updateFrame(15.05, {observation});
    const auto prediction = predictor->predict(15.05, 1.0).at(0);
    assert(prediction.x > observation.x);
    expectNear(prediction.direction_uncertainty_m, 0.0);
}

void testPredictorRejectsUnsafeNumerics() {
    PredictorConfig huge_sigma;
    huge_sigma.position_measurement_sigma_m = 1.0e200;
    bool rejected = false;
    try {
        static_cast<void>(makeEkfPredictor(huge_sigma));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    PredictorConfig unsafe_velocity_variance;
    unsafe_velocity_variance.position_measurement_sigma_m = 1.0e150;
    rejected = false;
    try {
        static_cast<void>(makeEkfPredictor(unsafe_velocity_variance));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);

    PredictorConfig config;
    auto predictor = makeEkfPredictor(config);
    ObjectObservation observation;
    observation.id = 9;
    observation.speed = 10.0;
    observation.length = 4.0;
    observation.width = 2.0;
    observation.height = 1.5;
    predictor->updateFrame(0.0, {observation});
    observation.x = 1.0e-300;
    const double tiny_dt = std::numeric_limits<double>::denorm_min();
    predictor->updateFrame(tiny_dt, {observation});
    const auto prediction = predictor->predict(tiny_dt, 6.0).at(0);
    assert(std::isfinite(prediction.x));
    assert(std::isfinite(prediction.y));
    assert(std::isfinite(prediction.position_sigma_m));
}

void testFutureEnvelopeContainsColdStartTrajectory() {
    PredictorConfig config;
    auto predictor = makeEkfPredictor(config);

    ObjectObservation observation;
    observation.id = 42;
    observation.x = 0.0;
    observation.y = 0.0;
    observation.heading = 0.0;
    observation.speed = 10.0;
    observation.length = 4.0;
    observation.width = 2.0;
    observation.height = 1.5;
    predictor->updateFrame(20.0, {observation});
    observation.x = 0.5;
    predictor->updateFrame(20.05, {observation});

    for (std::size_t bin = 1; bin <= 12; ++bin) {
        const double start_time = static_cast<double>(bin - 1) * 0.5;
        const double end_time = static_cast<double>(bin) * 0.5;
        auto start = predictor->predict(20.05, start_time).at(0);
        const auto end = predictor->predict(20.05, end_time).at(0);
        if (bin == 1) {
            start.x = observation.x;
            start.y = observation.y;
        }
        const double inflation = predictionUncertaintyInflation(
            std::max(start.position_sigma_m, end.position_sigma_m),
            std::max(start.direction_uncertainty_m, end.direction_uncertainty_m), 2.0) +
            yawIndependentRotationInflation(observation.length, observation.width);
        const auto [minimum_x, maximum_x] = xBounds(sweptFootprint(start, end, inflation));
        const double actual_start_x = observation.x + observation.speed * start_time;
        const double actual_end_x = observation.x + observation.speed * end_time;
        assert(minimum_x <= actual_start_x - observation.length * 0.5);
        assert(maximum_x >= actual_end_x + observation.length * 0.5);
    }

    const auto six_seconds = predictor->predict(20.05, 6.0).at(0);
    expectNear(six_seconds.direction_uncertainty_m, 0.0);
    const double full_inflation = predictionUncertaintyInflation(
        six_seconds.position_sigma_m, six_seconds.direction_uncertainty_m, 2.0);
    assert(full_inflation > 5.0);
    assert(six_seconds.x + observation.length * 0.5 + full_inflation >= 62.5);
}

void testAcceptedPositionInnovationStaysAnchored() {
    PredictorConfig config;
    config.process_acceleration_sigma_mps2 = 0.0;
    auto predictor = makeEkfPredictor(config);

    ObjectObservation observation;
    observation.id = 77;
    observation.speed = 1.0;
    observation.length = 4.0;
    observation.width = 2.0;
    observation.height = 1.5;
    for (int index = 0; index < 200; ++index) {
        observation.x = static_cast<double>(index) * 0.05;
        predictor->updateFrame(70.0 + static_cast<double>(index) * 0.05, {observation});
    }

    // The 14 m innovation is inside the configured 15 m gate. The following
    // normal sample used to overwrite the temporary speed bound while the
    // posterior centre remained near the old trajectory.
    observation.x = 24.0;
    predictor->updateFrame(80.0, {observation});
    observation.x = 24.05;
    predictor->updateFrame(80.05, {observation});
    expectNear(predictor->predict(80.05, 0.0).at(0).x, observation.x);

    for (std::size_t bin = 1; bin <= 12; ++bin) {
        const double start_time = static_cast<double>(bin - 1) * 0.5;
        const double end_time = static_cast<double>(bin) * 0.5;
        const auto start = predictor->predict(80.05, start_time).at(0);
        const auto end = predictor->predict(80.05, end_time).at(0);
        const double inflation = predictionUncertaintyInflation(
            std::max(start.position_sigma_m, end.position_sigma_m),
            std::max(start.direction_uncertainty_m, end.direction_uncertainty_m), 2.0) +
            yawIndependentRotationInflation(observation.length, observation.width);
        const auto envelope = sweptFootprint(start, end, inflation);
        for (int sample = 0; sample <= 40; ++sample) {
            const double future_s = start_time +
                (end_time - start_time) * static_cast<double>(sample) / 40.0;
            ObjectPrediction actual;
            actual.x = observation.x + observation.speed * future_s;
            actual.y = observation.y;
            actual.heading = observation.heading;
            actual.length = observation.length;
            actual.width = observation.width;
            for (const auto& corner : orientedBox(actual)) {
                assert(containsPoint(envelope, corner));
            }
        }
    }
}


}  // namespace

int main() {
    testEgoSpeed();
    testParsersAndRamp();
    testStopRampRespectsBrakingAndLatency();
    testYellowSignalStateManager();
    testGeometry();
    testPredictor();
    testVelocityUsesNextPositionSample();
    testPredictorRejectsUnsafeNumerics();
    testFutureEnvelopeContainsColdStartTrajectory();
    testAcceptedPositionInnovationStaysAnchored();
    std::cout << "tracker_core_test passed\n";
    return 0;
}
