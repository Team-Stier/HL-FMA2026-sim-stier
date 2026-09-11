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
    const auto short_path = builder.prepare(path, {}, 2.0, 0.1, 10);
    assert(short_path.valid && short_path.stop_only && !short_path.hold_requested);
    path.points_at_capture = {{0.0, 0.0}};
    const auto hold = builder.prepare(path, {}, 0.0, 0.05, 20);
    assert(hold.valid && hold.stop_only && hold.hold_requested);
    path.points_at_capture = {{1.0, 0.0}};
    assert(!builder.prepare(path, {}, 0.0, 0.05, 20).valid);
    path.points_at_capture = {{0.0, 0.0},
        {std::numeric_limits<double>::quiet_NaN(), 0.0}, {20.0, 0.0}};
    assert(!builder.prepare(path, {}, 2.0, 0.1, 10).valid);
    path.points_at_capture = {{0.0, 50.0}, {20.0, 50.0}};
    assert(!builder.prepare(path, {}, 2.0, 0.1, 10).valid);
    path.points_at_capture = {{-2.0, 0.0}, {-1.0, 0.0}};
    assert(!builder.prepare(path, {}, 2.0, 0.1, 10).valid);
}

void testTemporalCurvatureSampling() {
    control::PathSnapshot path;
    for (int i = 0; i <= 200; ++i) {
        const double x = i * 0.1;
        const double bend = std::max(0.0, x - 2.0);
        path.points_at_capture.push_back({x, 0.01 * bend * bend * bend});
    }
    control::ReferenceBuilder slow_builder({}), fast_builder({}), stopped_builder({});
    const auto slow = slow_builder.prepare(path, {}, 2.0, 0.05, 20);
    const auto fast = fast_builder.prepare(path, {}, 6.0, 0.05, 20);
    const auto stopped = stopped_builder.prepare(path, {}, 0.0, 0.05, 20);
    assert(slow.valid && fast.valid && stopped.valid);
    for (std::size_t i = 0; i < 7; ++i) {
        assert(std::abs(slow.curvature.at(3 * i) - fast.curvature.at(i)) < 1e-12);
    }
    for (std::size_t i = 0; i < 10; ++i) assert(std::abs(slow.curvature.at(i)) < 1e-12);
    for (double curvature : stopped.curvature) assert(std::abs(curvature) < 1e-12);
    assert(fast.curvature.at(12) > 0.01);
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

    const auto beyond_preview = control::limitTargetSpeed({}, 15.0, 30.0, 0.0, 20.0, 15.0);
    assert(beyond_preview.valid && beyond_preview.short_path_limited);
    assert(beyond_preview.target_speed_mps <= std::sqrt(2.0 * 2.0 * 18.0));
    assert(control::limitTargetSpeed({}, 15.0, 30.0, 0.0, 2.0, 1.0).target_speed_mps == 0.0);

    control::LongitudinalController longitudinal({});
    const double acceleration = longitudinal.update(10.0, 5.0, 0.05);
    assert(acceleration > 0.0 && acceleration <= 2.0);
    longitudinal.reset();
    const double braking = longitudinal.update(0.0, 5.0, 0.05);
    assert(braking <= 0.0 && braking >= -3.0);
    double sustained = braking;
    for (int i = 0; i < 30; ++i) sustained = longitudinal.update(0.0, 5.0, 0.05);
    assert(sustained == -3.0);

    // Overspeed and fault branches can replace the PI result with full braking.
    // Releasing that override, including after reset, starts from the emitted value.
    for (int i = 0; i < 40; ++i) longitudinal.update(10.0, 5.0, 0.05);
    longitudinal.setAppliedAcceleration(-3.0);
    assert(std::abs(longitudinal.update(10.0, 5.0, 0.05) - (-2.85)) < 1e-9);
    longitudinal.setAppliedAcceleration(-3.0);
    longitudinal.reset();
    double previous = -3.0;
    for (int i = 0; i < 30; ++i) {
        const double released = longitudinal.update(10.0, 5.0, 0.05);
        assert(released - previous <= 0.150000001);
        longitudinal.setAppliedAcceleration(released);
        previous = released;
    }
    assert(previous > 0.0);
}

}  // namespace

int main() {
    testStraightAndDuplicates();
    testInvalidAndShortPaths();
    testTemporalCurvatureSampling();
    testCapturePoseCompensationAndYawWrap();
    testCurveAndMpcDirection();
    testSpeedPolicyAndLongitudinalUnits();
    std::cout << "control_core_test: all checks passed\n";
    return 0;
}
