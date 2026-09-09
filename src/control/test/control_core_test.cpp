#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "control/controller_core.hpp"

namespace {

void testStraightAndDuplicates() {
    control::PathSnapshot path;
    path.stamp_seconds = 1.0;
    path.points_at_capture = {{-2.0, 1.0}, {0.0, 1.0}, {0.0, 1.0},
                              {5.0, 1.0}, {10.0, 1.0}, {20.0, 1.0}};
    control::ReferenceBuilder builder({});
    const auto result = builder.prepare(path, {}, 5.0, 0.1, 12);
    assert(result.valid);
    assert(std::abs(result.lateral_error_m + 1.0) < 1e-9);
    assert(std::abs(result.heading_error_rad) < 1e-9);
    assert(std::abs(result.curvature.front()) < 1e-9);
}

void testInvalidAndShortPaths() {
    control::ReferenceBuilder builder({});
    control::PathSnapshot path;
    path.stamp_seconds = 1.0;
    assert(!builder.prepare(path, {}, 2.0, 0.1, 10).valid);
    path.points_at_capture = {{0.0, 0.0}, {0.0, 0.0},
        {std::numeric_limits<double>::quiet_NaN(), 2.0}};
    assert(!builder.prepare(path, {}, 2.0, 0.1, 10).valid);
    path.points_at_capture = {{0.0, 0.0}, {0.2, 0.0}};
    assert(!builder.prepare(path, {}, 2.0, 0.1, 10).valid);
}

void testCapturePoseCompensationAndYawWrap() {
    control::PathSnapshot path;
    path.stamp_seconds = 1.0;
    path.capture_pose_map = {10.0, 20.0, 0.0};
    for (int index = 0; index <= 30; ++index) {
        path.points_at_capture.push_back({static_cast<double>(index), 0.0});
    }
    control::ReferenceBuilder builder({});
    const auto moved = builder.prepare(path, {12.0, 20.0, 0.0}, 5.0, 0.1, 10);
    assert(moved.valid);
    assert(std::abs(moved.remaining_length_m - 28.0) < 1e-9);

    builder.reset();
    control::PathSnapshot reverse;
    reverse.stamp_seconds = 2.0;
    for (int index = 20; index >= -20; --index) {
        reverse.points_at_capture.push_back({static_cast<double>(index), 0.0});
    }
    const auto wrapped = builder.prepare(
        reverse, {0.0, 0.0, 3.14159265358979323846}, 4.0, 0.1, 10);
    assert(wrapped.valid);
    assert(std::abs(wrapped.heading_error_rad) < 1e-8);
}

void testCurveAndMpcDirection() {
    control::PathSnapshot path;
    path.stamp_seconds = 1.0;
    for (int index = 0; index <= 40; ++index) {
        const double angle = 0.02 * index;
        path.points_at_capture.push_back(
            {20.0 * std::sin(angle), 20.0 * (1.0 - std::cos(angle))});
    }
    control::ReferenceBuilder builder({});
    const auto curve = builder.prepare(path, {}, 8.0, 0.1, 15);
    assert(curve.valid);
    assert(curve.curvature.at(3) > 0.0);

    control::MpcConfig config;
    config.maximum_iterations = 1000;
    control::LateralMpc mpc(config);
    const auto result = mpc.solve(-1.0, 0.0, 5.0,
                                  std::vector<double>(20, 0.0), 0.0, nullptr);
    assert(result.success);
    assert(result.steering_rad > 0.0);
    assert(std::abs(result.steering_rad) <=
           config.maximum_steering_rate_radps * config.dt_seconds + 1e-8);
    assert(!mpc.solve(std::numeric_limits<double>::quiet_NaN(), 0.0, 5.0,
                      std::vector<double>(20, 0.0), 0.0, nullptr).success);
}

void testSpeedPolicyAndLongitudinalUnits() {
    const auto normal = control::limitTargetSpeed({}, 20.0, 10.0, 0.0,
                                                  50.0, 20.0);
    assert(normal.valid);
    assert(std::abs(normal.target_speed_mps - 9.5) < 1e-9);
    const auto short_path = control::limitTargetSpeed({}, 20.0, 10.0, 0.0,
                                                      5.0, 20.0);
    assert(short_path.valid && short_path.short_path_limited);
    assert(short_path.target_speed_mps > 0.0);
    assert(short_path.target_speed_mps < normal.target_speed_mps);

    control::LongitudinalController longitudinal({});
    const double acceleration = longitudinal.update(10.0, 5.0, 0.05);
    assert(acceleration > 0.0 && acceleration <= 2.0);
    longitudinal.reset();
    const double braking = longitudinal.update(0.0, 5.0, 0.05);
    assert(braking < 0.0 && braking >= -3.0);
}

}  // namespace

int main() {
    testStraightAndDuplicates();
    testInvalidAndShortPaths();
    testCapturePoseCompensationAndYawWrap();
    testCurveAndMpcDirection();
    testSpeedPolicyAndLongitudinalUnits();
    std::cout << "control_core_test: all checks passed\n";
    return 0;
}
