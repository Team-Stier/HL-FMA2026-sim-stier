#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace control {

struct Point2 {
    double x{0.0};
    double y{0.0};
};

struct Pose2 {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct PathSnapshot {
    std::vector<Point2> points_at_capture;
    Pose2 capture_pose_map;
    double stamp_seconds{0.0};
};

struct ReferenceConfig {
    double duplicate_epsilon_m{0.02};
    double curvature_window_m{0.5};
    double heading_rejection_rad{1.5707963267948966};
    double continuity_weight{0.25};
    double maximum_projection_distance_m{2.0};
    double wheelbase_m{2.944};
    std::size_t minimum_steps{3};
};

struct PreparedReference {
    bool valid{false};
    bool stop_only{false};
    bool hold_requested{false};
    std::string error;
    std::vector<double> curvature;
    double lateral_error_m{0.0};
    double heading_error_rad{0.0};
    double remaining_length_m{0.0};
};

class ReferenceBuilder {
public:
    explicit ReferenceBuilder(ReferenceConfig config);
    PreparedReference prepare(const PathSnapshot &path, const Pose2 &current_pose_map,
                              double speed_mps, double dt_seconds,
                              std::size_t requested_steps);
    void reset() noexcept;

private:
    ReferenceConfig config_;
    bool has_previous_projection_{false};
    Point2 previous_projection_map_;
    double previous_stamp_seconds_{0.0};
};

struct MpcConfig {
    double wheelbase_m{2.944};
    double dt_seconds{0.05};
    std::size_t horizon_steps{20};
    double lateral_error_weight{8.0};
    double heading_error_weight{5.0};
    double steering_weight{0.25};
    double steering_rate_weight{2.0};
    double terminal_multiplier{2.0};
    double maximum_steering_rad{0.48};
    double maximum_steering_rate_radps{0.4};
    std::size_t maximum_iterations{400};
    double gradient_step{0.001};
    double convergence_tolerance{1e-4};
    double maximum_solve_time_ms{20.0};
    double constraint_tolerance{1e-8};
};

struct MpcResult {
    bool success{false};
    std::string reason;
    double steering_rad{0.0};
    double solve_time_ms{0.0};
    double maximum_constraint_violation{0.0};
    std::size_t iterations{0};
    std::vector<double> solution;
};

class LateralMpc {
public:
    explicit LateralMpc(MpcConfig config);
    MpcResult solve(double lateral_error_m, double heading_error_rad,
                    double speed_mps, const std::vector<double> &curvature,
                    double previous_steering_rad,
                    const std::vector<double> *warm_start) const;

private:
    MpcConfig config_;
    std::string configuration_error_;
};

struct SpeedPolicyConfig {
    double limit_margin_mps{0.5};
    double comfortable_deceleration_mps2{2.0};
    double path_end_buffer_m{2.0};
    double maximum_lateral_acceleration_mps2{2.0};
};

struct SpeedPolicyResult {
    bool valid{false};
    std::string error;
    double target_speed_mps{0.0};
    bool short_path_limited{false};
};

SpeedPolicyResult limitTargetSpeed(const SpeedPolicyConfig &config,
                                   double configured_cruise_speed_mps,
                                   double speed_limit_mps,
                                   double maximum_abs_curvature,
                                   double remaining_path_m,
                                   double requested_preview_m);

struct LongitudinalConfig {
    double kp{1.0};
    double ki{0.1};
    double kd{0.0};
    double integral_limit{3.0};
    double maximum_acceleration_mps2{2.0};
    double maximum_deceleration_mps2{3.0};
    double acceleration_rate_mps3{3.0};
};

class LongitudinalController {
public:
    explicit LongitudinalController(LongitudinalConfig config);
    double update(double target_speed_mps, double measured_speed_mps,
                  double dt_seconds);
    void setAppliedAcceleration(double acceleration_mps2) noexcept;
    // Clear feedback history while preserving the last applied acceleration.
    void reset() noexcept;

private:
    LongitudinalConfig config_;
    double integral_{0.0};
    double previous_speed_{0.0};
    double previous_acceleration_{0.0};
    bool has_previous_speed_{false};
};

double normalizeAngle(double angle) noexcept;

}  // namespace control
