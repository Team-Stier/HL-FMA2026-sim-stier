#include "hdmap_dynamic_tracker/motion_predictor.hpp"
#include "hdmap_dynamic_tracker/tracker_core.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace hdmap_dynamic_tracker;

namespace {

void expectNear(double actual, double expected, double tolerance = 1.0e-6) {
    assert(std::abs(actual - expected) <= tolerance);
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

}  // namespace

int main() {
    testEgoSpeed();
    testParsersAndRamp();
    testGeometry();
    testPredictor();
    std::cout << "tracker_core_test passed\n";
    return 0;
}
