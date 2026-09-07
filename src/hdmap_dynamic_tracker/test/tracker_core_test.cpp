#include "hdmap_dynamic_tracker/motion_predictor.hpp"
#include "hdmap_dynamic_tracker/tracker_core.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
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
    EgoSpeedEstimator estimator(0.25, 300.0 / 3.6);
    expectNear(estimator.update(1.0, 0.0, 0.0).speed_mps, 0.0);
    expectNear(estimator.update(1.1, 0.3, 0.4).speed_mps, 5.0);
    const auto repeated = estimator.update(1.1, 1.0, 1.0);
    expectNear(repeated.speed_mps, 0.0);
    assert(repeated.history_reset);
    expectNear(estimator.update(1.2, 1.0, 1.2).speed_mps, 2.0);
    const auto gap = estimator.update(2.0, 1.0, 1.2);
    assert(gap.history_reset);
    expectNear(gap.speed_mps, 0.0);
    expectNear(estimator.update(2.1, 1.0, 1.3).speed_mps, 1.0);
    const auto jump = estimator.update(2.2, 100.0, 1.3);
    assert(jump.history_reset);
    expectNear(jump.speed_mps, 0.0);
    const auto non_finite = estimator.update(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    assert(non_finite.history_reset);
    expectNear(non_finite.speed_mps, 0.0);
    expectNear(estimator.update(3.0, 0.0, 0.0).speed_mps, 0.0);
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

    StopRampParameters ramp{8.0, 2.0, 1.0, 0.25, 1.0};
    expectNear(stopRampStartDistance(ramp), 35.0);
    expectNear(stopRampCap(0.0, ramp), 0.0);
    expectNear(stopRampCap(17.0, ramp), 4.0);
    expectNear(stopRampCap(33.0, ramp), 8.0);
    assert(stopRampCap(10.0, ramp) <= stopRampCap(20.0, ramp));
    ramp.design_deceleration_mps2 = 0.0;
    assert(!validStopRamp(ramp));
    expectNear(stopRampCap(100.0, ramp), 0.0);
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
    config.maximum_frame_dt_s = 0.25;
    config.track_retention_s = 0.5;
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
    expectNear(prediction.front().direction_uncertainty_m, 5.0);

    observation.x = 1.0;
    predictor->updateFrame(10.1, {observation});
    prediction = predictor->predict(10.1, 0.0);
    assert(prediction.size() == 1);
    assert(prediction.front().x > 0.0 && prediction.front().x < 1.01);
    expectNear(prediction.front().direction_uncertainty_m, 0.0);
    assert(predictor->predict(10.1, 0.5).front().direction_uncertainty_m >= 5.0);

    predictor->updateFrame(10.2, {});
    assert(predictor->predict(10.2, 0.0).size() == 1);
    predictor->updateFrame(10.3, {});
    assert(predictor->predict(10.3, 0.0).size() == 1);
    predictor->updateFrame(10.8, {});  // Frame gap resets every track.
    assert(predictor->predict(10.8, 0.0).empty());

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

void testVelocityDirectionRequiresConvergence() {
    PredictorConfig config;
    auto predictor = makeEkfPredictor(config);
    ObjectObservation observation;
    observation.id = 8;
    observation.speed = 10.0;
    observation.length = 4.0;
    observation.width = 2.0;
    observation.height = 1.5;
    predictor->updateFrame(15.0, {observation});

    // One plausible delta is not enough to resolve velocity direction.
    observation.x = 0.5;
    predictor->updateFrame(15.05, {observation});
    assert(predictor->predict(15.05, 1.0).at(0).direction_uncertainty_m >= 10.0);

    // Consistent deltas still have to satisfy both sample-count and time-span gates.
    for (int index = 2; index <= 4; ++index) {
        observation.x = static_cast<double>(index) * 0.5;
        predictor->updateFrame(15.0 + static_cast<double>(index) * 0.05, {observation});
    }
    const auto converged = predictor->predict(15.2, 1.0).at(0);
    assert(converged.direction_uncertainty_m < 10.0);

    // A sudden orthogonal delta invalidates the old direction immediately.
    observation.y = 0.5;
    predictor->updateFrame(15.25, {observation});
    assert(predictor->predict(15.25, 1.0).at(0).direction_uncertainty_m >= 10.0);

    // Even consistent low-speed deltas stay unresolved below the displacement floor.
    predictor->reset();
    observation.x = 0.0;
    observation.y = 0.0;
    observation.speed = 2.0;
    predictor->updateFrame(16.0, {observation});
    for (int index = 1; index <= 4; ++index) {
        observation.x = static_cast<double>(index) * 0.1;
        predictor->updateFrame(16.0 + static_cast<double>(index) * 0.05, {observation});
    }
    assert(predictor->predict(16.2, 1.0).at(0).direction_uncertainty_m >= 2.0);

    // A position-derived speed above the scalar API speed is also part of the
    // arbitrary-direction reach bound; otherwise the opposite side can be missed.
    predictor->reset();
    observation.x = 0.0;
    observation.y = 0.0;
    observation.speed = 10.0;
    predictor->updateFrame(17.0, {observation});
    observation.x = 10.0;
    predictor->updateFrame(17.1, {observation});
    const auto mismatch_now = predictor->predict(17.1, 0.0).at(0);
    const auto mismatch_future = predictor->predict(17.1, 1.0).at(0);
    const double estimated_displacement = std::hypot(
        mismatch_future.x - mismatch_now.x, mismatch_future.y - mismatch_now.y);
    assert(mismatch_future.direction_uncertainty_m + 1.0e-6 >=
        100.0 + estimated_displacement);
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
    config.maximum_frame_dt_s = 0.25;
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
    assert(six_seconds.direction_uncertainty_m > 0.0);
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

void testFutureEnvelopeContainsObservedDirectionChange() {
    auto predictor = makeEkfPredictor(PredictorConfig{});
    ObjectObservation observation;
    observation.id = 99;
    observation.heading = 0.0;
    observation.speed = 10.0;
    observation.length = 4.0;
    observation.width = 2.0;
    observation.height = 1.5;
    for (int index = 0; index < 40; ++index) {
        observation.x = static_cast<double>(index) * 0.5;
        observation.y = 0.0;
        predictor->updateFrame(40.0 + static_cast<double>(index) * 0.05, {observation});
    }
    observation.x = 19.5;
    observation.y = 0.5;
    observation.heading = std::acos(-1.0) * 0.5;
    predictor->updateFrame(42.0, {observation});

    for (std::size_t bin = 1; bin <= 12; ++bin) {
        const double start_time = static_cast<double>(bin - 1) * 0.5;
        const double end_time = static_cast<double>(bin) * 0.5;
        auto start = predictor->predict(42.0, start_time).at(0);
        const auto end = predictor->predict(42.0, end_time).at(0);
        if (bin == 1) {
            start.x = observation.x;
            start.y = observation.y;
        }
        const double inflation = predictionUncertaintyInflation(
            std::max(start.position_sigma_m, end.position_sigma_m),
            std::max(start.direction_uncertainty_m, end.direction_uncertainty_m), 2.0) +
            yawIndependentRotationInflation(observation.length, observation.width);
        const auto envelope = sweptFootprint(start, end, inflation);
        for (int sample = 0; sample <= 40; ++sample) {
            const double future_s = start_time +
                (end_time - start_time) * static_cast<double>(sample) / 40.0;
            ObjectPrediction actual;
            actual.x = observation.x;
            actual.y = observation.y + observation.speed * future_s;
            actual.heading = observation.heading;
            actual.length = observation.length;
            actual.width = observation.width;
            for (const auto& corner : orientedBox(actual)) {
                assert(containsPoint(envelope, corner));
            }
        }
    }

    predictor->reset();
    observation.x = 0.0;
    observation.y = 0.0;
    predictor->updateFrame(50.0, {observation});
    predictor->updateFrame(50.05, {observation});
    const auto unresolved = predictor->predict(50.05, 6.0).at(0);
    assert(unresolved.direction_uncertainty_m >= 60.0 - 1.0e-6);
}

}  // namespace

int main() {
    testEgoSpeed();
    testParsersAndRamp();
    testGeometry();
    testPredictor();
    testVelocityDirectionRequiresConvergence();
    testPredictorRejectsUnsafeNumerics();
    testFutureEnvelopeContainsColdStartTrajectory();
    testAcceptedPositionInnovationStaysAnchored();
    testFutureEnvelopeContainsObservedDirectionChange();
    std::cout << "tracker_core_test passed\n";
    return 0;
}
