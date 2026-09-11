#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <interfaces/msg/control_command.hpp>
#include <interfaces/msg/ego_status.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include "control/controller_core.hpp"

namespace control {
namespace {

double stampSeconds(const builtin_interfaces::msg::Time &stamp) {
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
}

double steadySeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

double approach(double value, double target, double maximum_change) {
    if (value < target) {
        return std::min(target, value + maximum_change);
    }
    return std::max(target, value - maximum_change);
}

struct TimedPose {
    double stamp_seconds{0.0};
    Pose2 pose;
    double z_m{};
};

class MpcControlNode : public rclcpp::Node {
public:
    MpcControlNode() : Node("mpc_control") {
        loadParameters();
        reference_builder_ = std::make_unique<ReferenceBuilder>(reference_config_);
        lateral_mpc_ = std::make_unique<LateralMpc>(mpc_config_);
        longitudinal_ = std::make_unique<LongitudinalController>(longitudinal_config_);
        output_enabled_ = enable_output_ && calibration_verified_;
        if (enable_output_ && !calibration_verified_) {
            throw std::invalid_argument(
                "enable_output requires calibration_verified=true");
        }
        const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort().durability_volatile();
        ego_subscription_ = create_subscription<interfaces::msg::EgoStatus>(
            ego_topic_, qos,
            std::bind(&MpcControlNode::onEgo, this, std::placeholders::_1));
        path_subscription_ = create_subscription<nav_msgs::msg::Path>(
            path_topic_, qos,
            std::bind(&MpcControlNode::onPath, this, std::placeholders::_1));
        speed_subscription_ = create_subscription<std_msgs::msg::Float32>(
            speed_limit_topic_, qos,
            std::bind(&MpcControlNode::onSpeedLimit, this,
                      std::placeholders::_1));
        candidate_publisher_ = create_publisher<interfaces::msg::ControlCommand>(
            candidate_topic_, qos);
        command_publisher_ = create_publisher<interfaces::msg::ControlCommand>(
            command_topic_, qos);
        status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, qos);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(mpc_config_.dt_seconds),
            std::bind(&MpcControlNode::onTimer, this));
        if (!output_enabled_) {
            RCLCPP_WARN(get_logger(),
                        "Control output disabled; publishing /control/candidate only");
        }
    }

private:
    void loadParameters() {
        ego_topic_ = declare_parameter("ego_topic", std::string("/ego_status"));
        path_topic_ = declare_parameter("path_topic", std::string("/local_path"));
        speed_limit_topic_ = declare_parameter(
            "speed_limit_topic", std::string("/speed_limit"));
        candidate_topic_ = declare_parameter(
            "candidate_topic", std::string("/control/candidate"));
        command_topic_ = declare_parameter("command_topic", std::string("/ctrl_cmd"));
        status_topic_ = declare_parameter(
            "status_topic", std::string("/control/status"));
        enable_output_ = declare_parameter("enable_output", false);
        calibration_verified_ = declare_parameter("calibration_verified", false);
        mpc_config_.dt_seconds = declare_parameter("control_period_s", 0.05);
        ego_timeout_s_ = declare_parameter("ego_timeout_s", 0.25);
        path_timeout_s_ = declare_parameter("path_timeout_s", 0.30);
        speed_limit_timeout_s_ = declare_parameter("speed_limit_timeout_s", 0.25);
        capture_pose_tolerance_s_ = declare_parameter(
            "capture_pose_tolerance_s", 0.10);
        respawn_jump_m_ = declare_parameter("respawn_jump_m", 5.0);
        maximum_height_difference_m_ = declare_parameter("maximum_height_difference_m", 2.0);
        reference_config_.maximum_projection_distance_m = declare_parameter(
            "maximum_projection_distance_m", 2.0);
        cruise_speed_mps_ = declare_parameter("configured_cruise_speed_mps", 8.0);
        speed_policy_config_.limit_margin_mps = declare_parameter(
            "speed_limit_margin_mps", 0.5);
        speed_policy_config_.comfortable_deceleration_mps2 = declare_parameter(
            "comfortable_deceleration_mps2", 2.0);
        speed_policy_config_.path_end_buffer_m = declare_parameter(
            "path_end_buffer_m", 2.0);
        speed_policy_config_.maximum_lateral_acceleration_mps2 =
            declare_parameter("maximum_lateral_acceleration_mps2", 2.0);
        longitudinal_config_.maximum_acceleration_mps2 = declare_parameter(
            "maximum_acceleration_mps2", 2.0);
        longitudinal_config_.maximum_deceleration_mps2 = declare_parameter(
            "maximum_deceleration_mps2", 3.0);
        longitudinal_config_.acceleration_rate_mps3 = declare_parameter(
            "acceleration_rate_mps3", 3.0);
        overspeed_tolerance_mps_ = declare_parameter(
            "overspeed_tolerance_mps", 0.2);
        overspeed_release_tolerance_mps_ = declare_parameter(
            "overspeed_release_tolerance_mps", 0.1);
        mpc_config_.wheelbase_m = declare_parameter("wheelbase_m", 2.944);
        mpc_config_.maximum_steering_rad = declare_parameter(
            "maximum_steering_rad", 0.48);
        mpc_config_.maximum_steering_rate_radps = declare_parameter(
            "maximum_steering_rate_radps", 0.4);
        mpc_config_.horizon_steps = static_cast<std::size_t>(
            declare_parameter("horizon_steps", 20));
        mpc_config_.lateral_error_weight = declare_parameter(
            "lateral_error_weight", 8.0);
        mpc_config_.heading_error_weight = declare_parameter(
            "heading_error_weight", 5.0);
        mpc_config_.steering_weight = declare_parameter("steering_weight", 0.25);
        mpc_config_.steering_rate_weight = declare_parameter(
            "steering_rate_weight", 2.0);
        mpc_config_.terminal_multiplier = declare_parameter(
            "terminal_multiplier", 2.0);
        mpc_config_.maximum_iterations = static_cast<std::size_t>(
            declare_parameter("solver_maximum_iterations", 400));
        mpc_config_.gradient_step = declare_parameter(
            "solver_gradient_step", 0.001);
        mpc_config_.convergence_tolerance = declare_parameter(
            "solver_convergence_tolerance", 0.0001);
        mpc_config_.maximum_solve_time_ms = declare_parameter(
            "solver_maximum_time_ms", 20.0);
        longitudinal_config_.kp = declare_parameter("longitudinal_kp", 1.0);
        longitudinal_config_.ki = declare_parameter("longitudinal_ki", 0.1);
        longitudinal_config_.kd = declare_parameter("longitudinal_kd", 0.0);
        longitudinal_config_.integral_limit = declare_parameter(
            "longitudinal_integral_limit", 3.0);
        if (!std::isfinite(mpc_config_.dt_seconds) || !(mpc_config_.dt_seconds > 0.0) ||
            !std::isfinite(static_cast<float>(mpc_config_.maximum_steering_rad)) ||
            !(mpc_config_.maximum_steering_rad > 0.0) ||
            !std::isfinite(mpc_config_.maximum_steering_rate_radps) ||
            !(mpc_config_.maximum_steering_rate_radps > 0.0) ||
            !std::isfinite(static_cast<float>(longitudinal_config_.maximum_acceleration_mps2)) ||
            !std::isfinite(static_cast<float>(longitudinal_config_.maximum_deceleration_mps2)) ||
            !std::isfinite(maximum_height_difference_m_) ||
            !(maximum_height_difference_m_ > 0.0) ||
            !(ego_timeout_s_ > 0.0) || !(path_timeout_s_ > 0.0) ||
            !(speed_limit_timeout_s_ > 0.0) ||
            !(capture_pose_tolerance_s_ >= 0.0) || !(respawn_jump_m_ > 0.0) ||
            !(overspeed_tolerance_mps_ >= overspeed_release_tolerance_mps_) ||
            !(overspeed_release_tolerance_mps_ >= 0.0)) {
            throw std::invalid_argument("invalid timeout, reset, or overspeed parameter");
        }
    }

    bool findCapturePose(double stamp, TimedPose *pose) const {
        if (ego_history_.empty() || !(stamp > 0.0)) {
            return false;
        }
        const TimedPose *best = &ego_history_.front();
        double error = std::abs(best->stamp_seconds - stamp);
        for (const TimedPose &sample : ego_history_) {
            const double candidate = std::abs(sample.stamp_seconds - stamp);
            if (candidate < error) {
                best = &sample;
                error = candidate;
            }
        }
        if (error > capture_pose_tolerance_s_) {
            return false;
        }
        *pose = *best;
        return true;
    }

    void onEgo(const interfaces::msg::EgoStatus::SharedPtr message) {
        const double stamp = stampSeconds(message->header.stamp);
        const Pose2 pose{message->x, message->y, message->heading};
        const bool finite = std::isfinite(pose.x) && std::isfinite(pose.y) &&
                            std::isfinite(pose.yaw) && std::isfinite(message->z) &&
                            std::isfinite(message->speed) && message->speed >= 0.0;
        if (stamp > 0.0 && stamp <= ego_stamp_seconds_) return;
        if (message->header.frame_id != "map" || !finite || !(stamp > 0.0) ||
            stamp > now().seconds()) {
            have_ego_ = false;
            return;
        }
        // Replayed/duplicate samples never refresh receipt freshness or undo a reset.
        const double source_gap = stamp - ego_stamp_seconds_;
        if (ego_stamp_seconds_ > 0.0 &&
            (source_gap > ego_timeout_s_ ||
             steadySeconds() - ego_received_steady_seconds_ > ego_timeout_s_ ||
             std::hypot(pose.x - ego_pose_.x, pose.y - ego_pose_.y) > respawn_jump_m_ ||
             std::abs(message->z - ego_z_m_) > maximum_height_difference_m_ ||
             (source_gap <= ego_timeout_s_ &&
              std::abs(normalizeAngle(pose.yaw - ego_pose_.yaw)) >
                  reference_config_.heading_rejection_rad * 0.5))) {
            resetState("ego discontinuity");
            reset_cutoff_stamp_seconds_ = stamp;
        }
        ego_pose_ = pose;
        ego_z_m_ = message->z;
        speed_mps_ = message->speed;
        ego_stamp_seconds_ = stamp;
        ego_received_steady_seconds_ = steadySeconds();
        have_ego_ = true;
        ego_history_.push_back({stamp, pose, message->z});
        while (ego_history_.size() > 100) {
            ego_history_.pop_front();
        }
    }

    void onPath(const nav_msgs::msg::Path::SharedPtr message) {
        const double stamp = stampSeconds(message->header.stamp);
        if (stamp > 0.0 && (stamp < reset_cutoff_stamp_seconds_ ||
                           stamp < last_path_message_stamp_seconds_)) return;
        if (message->header.frame_id != "base_link" || !(stamp > 0.0) ||
            stamp > now().seconds()) {
            have_path_ = false;
            return;
        }
        if (stamp < reset_cutoff_stamp_seconds_ || stamp < last_path_message_stamp_seconds_ ||
            (stamp == last_path_message_stamp_seconds_ && have_path_ && !message->poses.empty())) {
            return;
        }
        last_path_message_stamp_seconds_ = stamp;
        // Empty or rejected new paths immediately revoke the previous path.
        have_path_ = false;
        if (message->poses.empty()) return;
        TimedPose capture;
        if (!findCapturePose(stamp, &capture)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Path rejected: no Ego pose at path capture stamp");
            return;
        }
        PathSnapshot candidate;
        candidate.stamp_seconds = stamp;
        candidate.capture_pose_map = capture.pose;
        candidate.points_at_capture.reserve(message->poses.size());
        for (const auto &pose : message->poses) {
            const auto &position = pose.pose.position;
            const auto &q = pose.pose.orientation;
            const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            if ((!pose.header.frame_id.empty() && pose.header.frame_id != "base_link") ||
                (!(pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) &&
                 std::abs(stampSeconds(pose.header.stamp) - stamp) > 1e-6) ||
                !std::isfinite(position.x) || !std::isfinite(position.y) ||
                !std::isfinite(position.z) || !std::isfinite(norm) ||
                std::abs(norm - 1.0) > 1e-3) {
                return;
            }
            candidate.points_at_capture.push_back({position.x, position.y});
        }
        if (message->poses.size() == 1) {
            const auto &pose = message->poses.front().pose;
            if (std::hypot(pose.position.x, pose.position.y) >
                    reference_config_.duplicate_epsilon_m ||
                std::abs(pose.orientation.x) + std::abs(pose.orientation.y) +
                    std::abs(pose.orientation.z) > 1e-6) return;
        }
        path_ = std::move(candidate);
        path_capture_z_m_ = capture.z_m;
        path_stamp_seconds_ = stamp;
        path_received_steady_seconds_ = steadySeconds();
        have_path_ = true;
        require_new_path_ = false;
    }

    void onSpeedLimit(const std_msgs::msg::Float32::SharedPtr message) {
        if (!std::isfinite(message->data) || message->data < 0.0F) {
            have_speed_limit_ = false;
            return;
        }
        speed_limit_mps_ = message->data;
        speed_limit_received_steady_seconds_ = steadySeconds();
        have_speed_limit_ = true;
    }

    void resetState(const std::string &reason) {
        warm_start_.clear();
        ego_history_.clear();
        have_speed_limit_ = false;
        longitudinal_->reset();
        reference_builder_->reset();
        have_path_ = false;
        require_new_path_ = true;
        overspeed_override_ = false;
        reset_detected_ = true;
        RCLCPP_WARN(get_logger(), "Controller reset: %s", reason.c_str());
    }

    void publishCommand(double steering, double acceleration,
                        const std::string &state, bool solver_success,
                        double solve_time_ms = 0.0) {
        const double steady_now = steadySeconds();
        const double dt = last_command_steady_seconds_ > 0.0
            ? std::clamp(steady_now - last_command_steady_seconds_,
                         0.0, mpc_config_.dt_seconds)
            : mpc_config_.dt_seconds;
        const bool finite = std::isfinite(steering) && std::isfinite(acceleration);
        if (!finite) {
            steering = 0.0;
            acceleration = -longitudinal_config_.maximum_deceleration_mps2;
            warm_start_.clear();
            longitudinal_->reset();
            stopping_ = true;
        }
        steering = approach(last_published_steering_rad_,
            std::clamp(steering, -mpc_config_.maximum_steering_rad,
                       mpc_config_.maximum_steering_rad),
            mpc_config_.maximum_steering_rate_radps * dt);
        acceleration = std::clamp(acceleration,
            -longitudinal_config_.maximum_deceleration_mps2,
            longitudinal_config_.maximum_acceleration_mps2);
        interfaces::msg::ControlCommand command;
        command.header.stamp = now();
        command.header.frame_id = "base_link";
        command.steering = static_cast<float>(steering);
        command.target_accel = static_cast<float>(acceleration);
        command.turn_signal = 0;
        candidate_publisher_->publish(command);
        if (output_enabled_) command_publisher_->publish(command);
        longitudinal_->setAppliedAcceleration(command.target_accel);
        last_published_steering_rad_ = command.steering;
        previous_steering_rad_ = command.steering;
        last_command_steady_seconds_ = steady_now;
        std_msgs::msg::String status;
        std::ostringstream stream;
        stream << "state=" << (finite ? state : "STOP_NONFINITE_COMMAND")
               << " solver_success=" << (finite && solver_success)
               << " solve_ms=" << solve_time_ms
               << " output_enabled=" << output_enabled_
               << " reset_detected=" << reset_detected_;
        status.data = stream.str();
        status_publisher_->publish(status);
        reset_detected_ = false;
    }

    void publishStop(const std::string &reason, bool immediate = true) {
        warm_start_.clear();
        if (!stopping_) longitudinal_->reset();
        stopping_ = true;
        const double acceleration = immediate || speed_mps_ < 0.1
            ? -longitudinal_config_.maximum_deceleration_mps2
            : std::min(0.0, longitudinal_->update(0.0, speed_mps_, mpc_config_.dt_seconds));
        publishCommand(0.0, acceleration, reason, false);
    }

    void onTimer() {
        const double ros_now = now().seconds();
        const double steady_now = steadySeconds();
        if (!have_ego_ || !(ros_now - ego_stamp_seconds_ >= 0.0) ||
            ros_now - ego_stamp_seconds_ > ego_timeout_s_ ||
            steady_now - ego_received_steady_seconds_ > ego_timeout_s_) {
            publishStop("STOP_STALE_EGO");
            return;
        }
        if (!have_path_ || require_new_path_ ||
            !(ros_now - path_stamp_seconds_ >= 0.0) ||
            ros_now - path_stamp_seconds_ > path_timeout_s_ ||
            steady_now - path_received_steady_seconds_ > path_timeout_s_) {
            publishStop("STOP_NO_LOCAL_PATH");
            return;
        }
        if (!have_speed_limit_ ||
            steady_now - speed_limit_received_steady_seconds_ >
                speed_limit_timeout_s_) {
            publishStop("STOP_STALE_SPEED_LIMIT");
            return;
        }

        if (speed_limit_mps_ == 0.0) {
            publishStop(speed_mps_ < 0.1 ? "HOLD_SPEED_LIMIT" : "STOP_SPEED_LIMIT");
            return;
        }
        if (std::abs(ego_z_m_ - path_capture_z_m_) > maximum_height_difference_m_) {
            have_path_ = false;
            publishStop("STOP_PATH_HEIGHT_MISMATCH");
            return;
        }
        const PreparedReference reference = reference_builder_->prepare(
            path_, ego_pose_, speed_mps_, mpc_config_.dt_seconds,
            mpc_config_.horizon_steps);
        if (!reference.valid) {
            publishStop("STOP_INVALID_PATH:" + reference.error);
            return;
        }
        if (reference.stop_only) {
            const double stopping_distance = speed_mps_ * speed_mps_ /
                (2.0 * longitudinal_config_.maximum_deceleration_mps2);
            const bool immediate = reference.hold_requested || stopping_distance >
                std::max(0.0, reference.remaining_length_m - speed_policy_config_.path_end_buffer_m);
            publishStop(reference.hold_requested
                ? (speed_mps_ < 0.1 ? "HOLD" : "STOP_REQUESTED") : "STOP_SHORT_PATH", immediate);
            return;
        }
        const bool warm_start_valid = !warm_start_.empty() &&
                                      path_stamp_seconds_ >= last_solved_stamp_seconds_;
        const MpcResult lateral = lateral_mpc_->solve(
            reference.lateral_error_m, reference.heading_error_rad, speed_mps_,
            reference.curvature, previous_steering_rad_,
            warm_start_valid ? &warm_start_ : nullptr);
        if (!lateral.success) {
            publishStop("STOP_MPC_FAILURE:" + lateral.reason);
            return;
        }

        double maximum_curvature = 0.0;
        for (double value : reference.curvature) {
            maximum_curvature = std::max(maximum_curvature, std::abs(value));
        }
        const double requested_preview = std::max(1.0, speed_mps_) *
            mpc_config_.dt_seconds * static_cast<double>(mpc_config_.horizon_steps);
        const SpeedPolicyResult speed = limitTargetSpeed(
            speed_policy_config_, cruise_speed_mps_, speed_limit_mps_,
            maximum_curvature, reference.remaining_length_m, requested_preview);
        if (!speed.valid) {
            publishStop("STOP_SPEED_POLICY_FAILURE:" + speed.error);
            return;
        }

        if (!overspeed_override_ &&
            speed_mps_ > speed_limit_mps_ + overspeed_tolerance_mps_) {
            overspeed_override_ = true;
        } else if (overspeed_override_ &&
                   speed_mps_ <= speed_limit_mps_ +
                                     overspeed_release_tolerance_mps_) {
            overspeed_override_ = false;
        }
        if (speed.target_speed_mps == 0.0 && speed_mps_ < 0.1) {
            publishStop("HOLD_PATH_END");
            return;
        }
        stopping_ = false;
        double acceleration = longitudinal_->update(
            speed.target_speed_mps, speed_mps_, mpc_config_.dt_seconds);
        if (overspeed_override_) {
            acceleration = -longitudinal_config_.maximum_deceleration_mps2;
        }

        warm_start_ = lateral.solution;
        if (warm_start_.size() > 1) {
            std::rotate(warm_start_.begin(), warm_start_.begin() + 1,
                        warm_start_.end());
            warm_start_.back() = warm_start_[warm_start_.size() - 2];
        }
        last_solved_stamp_seconds_ = path_stamp_seconds_;
        publishCommand(lateral.steering_rad, acceleration,
                       speed.short_path_limited ? "ACTIVE_SHORT_PATH" : "ACTIVE",
                       true, lateral.solve_time_ms);
    }

    std::string ego_topic_;
    std::string path_topic_;
    std::string speed_limit_topic_;
    std::string candidate_topic_;
    std::string command_topic_;
    std::string status_topic_;
    bool enable_output_{false};
    bool calibration_verified_{false};
    bool output_enabled_{false};
    double ego_timeout_s_{0.25};
    double path_timeout_s_{0.30};
    double speed_limit_timeout_s_{0.25};
    double capture_pose_tolerance_s_{0.10};
    double respawn_jump_m_{5.0};
    double maximum_height_difference_m_{2.0};
    double cruise_speed_mps_{8.0};
    double overspeed_tolerance_mps_{0.2};
    double overspeed_release_tolerance_mps_{0.1};
    ReferenceConfig reference_config_;
    MpcConfig mpc_config_;
    SpeedPolicyConfig speed_policy_config_;
    LongitudinalConfig longitudinal_config_;
    std::unique_ptr<ReferenceBuilder> reference_builder_;
    std::unique_ptr<LateralMpc> lateral_mpc_;
    std::unique_ptr<LongitudinalController> longitudinal_;
    rclcpp::Subscription<interfaces::msg::EgoStatus>::SharedPtr ego_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription_;
    rclcpp::Publisher<interfaces::msg::ControlCommand>::SharedPtr candidate_publisher_;
    rclcpp::Publisher<interfaces::msg::ControlCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<TimedPose> ego_history_;
    PathSnapshot path_;
    Pose2 ego_pose_;
    std::vector<double> warm_start_;
    double ego_z_m_{0.0};
    double path_capture_z_m_{0.0};
    double reset_cutoff_stamp_seconds_{0.0};
    double last_path_message_stamp_seconds_{0.0};
    double ego_stamp_seconds_{0.0};
    double path_stamp_seconds_{0.0};
    double last_solved_stamp_seconds_{0.0};
    double ego_received_steady_seconds_{0.0};
    double path_received_steady_seconds_{0.0};
    double speed_limit_received_steady_seconds_{0.0};
    double speed_mps_{0.0};
    double speed_limit_mps_{0.0};
    double previous_steering_rad_{0.0};
    double last_published_steering_rad_{0.0};
    double last_command_steady_seconds_{0.0};
    bool stopping_{false};
    bool have_ego_{false};
    bool have_path_{false};
    bool have_speed_limit_{false};
    bool require_new_path_{false};
    bool overspeed_override_{false};
    bool reset_detected_{false};
};

}  // namespace
}  // namespace control

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<control::MpcControlNode>());
    } catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("mpc_control"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
