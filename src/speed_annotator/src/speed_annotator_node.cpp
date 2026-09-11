#include "hdmap/hdmap.hpp"
#include "interfaces/msg/dynamic_status.hpp"
#include "interfaces/msg/ego_pose.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>

class SpeedAnnotator final : public rclcpp::Node {
public:
    SpeedAnnotator() : Node("speed_annotator") {
        const auto path = declare_parameter<std::string>("map_path", "");
        map_ = hdmap::hdmap_init(path.empty() ? std::getenv("HDMAP_PATH") : path);
        input_timeout_s_ = declare_parameter("input_timeout_s", 0.50);
        if (!std::isfinite(input_timeout_s_) || !(input_timeout_s_ > 0.0)) {
            throw std::invalid_argument("input_timeout_s must be finite and positive");
        }
        const auto qos = rclcpp::QoS(1).best_effort();
        publisher_ = create_publisher<std_msgs::msg::Float32>("/speed_limit", qos);
        ego_subscription_ = create_subscription<interfaces::msg::EgoPose>("/ego_pose", qos,
            [this](const interfaces::msg::EgoPose& message) {
                const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
                if (stamp <= last_ego_stamp_) return;
                if (rclcpp::Time(message.header.stamp) > now()) {
                    ego_history_.clear();
                    publishCap(0.F);
                    return;
                }
                if (message.header.frame_id != "map" || stamp <= 0 ||
                    !std::isfinite(message.x) || !std::isfinite(message.y) ||
                    !std::isfinite(message.z) || !std::isfinite(message.heading)) {
                    ego_history_.clear();
                    last_ego_stamp_ = stamp;
                    reset_cutoff_stamp_ = stamp;
                    publishCap(0.F);
                    return;
                }
                if (!ego_history_.empty()) {
                    const auto& previous = ego_history_.back();
                    const double source_gap = (stamp - last_ego_stamp_) * 1e-9;
                    const double receipt_gap = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - last_ego_received_).count();
                    if (source_gap > input_timeout_s_ || receipt_gap > input_timeout_s_ ||
                        std::hypot(message.x - previous.x, message.y - previous.y) > 5.0 ||
                        std::abs(message.z - previous.z) > 2.0 ||
                        std::abs(std::remainder(message.heading - previous.heading,
                                              2.0 * std::acos(-1.0))) > std::acos(-1.0) / 4.0) {
                        ego_history_.clear();
                        reset_cutoff_stamp_ = stamp;
                        publishCap(0.F);
                    }
                }
                last_ego_stamp_ = stamp;
                last_ego_received_ = std::chrono::steady_clock::now();
                ego_history_.push_back(message);
                while (ego_history_.size() > 100) ego_history_.pop_front();
                publish();
            });
        dynamic_subscription_ = create_subscription<interfaces::msg::DynamicStatus>("/dynamic_status", qos,
            [this](interfaces::msg::DynamicStatus::ConstSharedPtr message) {
                const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
                if (stamp <= 0 || rclcpp::Time(message->header.stamp) > now()) {
                    publishCap(0.F);
                    return;
                }
                if (dynamic_ && stamp <= rclcpp::Time(dynamic_->header.stamp).nanoseconds()) return;
                dynamic_ = std::move(message);
                dynamic_received_ = std::chrono::steady_clock::now();
                publish();
            });
    }

private:
    void publishCap(float cap) {
        std_msgs::msg::Float32 message;
        message.data = cap;
        publisher_->publish(message);
    }

    void publish() {
        if (!dynamic_) return;
        const auto stamp = rclcpp::Time(dynamic_->header.stamp).nanoseconds();
        // Float32 cannot carry source time: never freshen it by replaying an old cap.
        if (stamp <= last_published_stamp_ || stamp < reset_cutoff_stamp_) return;
        const double source_age = now().seconds() - rclcpp::Time(dynamic_->header.stamp).seconds();
        const double receipt_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - dynamic_received_).count();
        if (dynamic_->header.frame_id != "map" || stamp <= 0 || source_age < 0.0 ||
            source_age > input_timeout_s_ || receipt_age > input_timeout_s_ ||
            dynamic_->speed_cap_mps.size() != map_->cells().size()) {
            last_published_stamp_ = stamp;
            publishCap(0.F);
            return;
        }
        // Prefer an exact stamp, then the nearest Ego within 100 ms.
        const auto exact = std::find_if(ego_history_.rbegin(), ego_history_.rend(),
            [stamp](const auto& candidate) {
                return rclcpp::Time(candidate.header.stamp).nanoseconds() == stamp;
            });
        const interfaces::msg::EgoPose* ego = exact == ego_history_.rend() ? nullptr : &(*exact);
        if (!ego) {
            constexpr std::int64_t kSyncToleranceNs = 100000000;
            std::int64_t best_error = kSyncToleranceNs + 1;
            for (const auto& candidate : ego_history_) {
                const auto candidate_stamp = rclcpp::Time(candidate.header.stamp).nanoseconds();
                const auto error = candidate_stamp > stamp ? candidate_stamp - stamp : stamp - candidate_stamp;
                if (error > kSyncToleranceNs) continue;
                if (!ego || error < best_error ||
                    (error == best_error && candidate_stamp <= stamp &&
                     rclcpp::Time(ego->header.stamp).nanoseconds() > stamp)) {
                    ego = &candidate;
                    best_error = error;
                }
            }
        }
        if (!ego) return;
        last_published_stamp_ = stamp;
        const double c = std::cos(ego->heading);
        const double s = std::sin(ego->heading);
        lanelet::BasicPolygon2d footprint;
        for (const auto [x, y] : std::array<std::array<double, 2>, 4>{{
                 {3.808, .943}, {3.808, -.943}, {-1.040, -.943}, {-1.040, .943}}}) {
            footprint.emplace_back(ego->x + c * x - s * y, ego->y + s * x + c * y);
        }
        float cap = std::numeric_limits<float>::infinity();
        for (const auto cell : map_->cellTree().queryOverlaps(
                 footprint, ego->z, ego->z + 1.507)) {
            if (cell >= dynamic_->speed_cap_mps.size() ||
                !std::isfinite(dynamic_->speed_cap_mps[cell]) ||
                dynamic_->speed_cap_mps[cell] < 0.F) {
                publishCap(0.F);
                return;
            }
            cap = std::min(cap, dynamic_->speed_cap_mps[cell]);
        }
        publishCap(std::isfinite(cap) ? cap : 0.F);
    }

    std::unique_ptr<hdmap::HdMap> map_;
    std::deque<interfaces::msg::EgoPose> ego_history_;
    interfaces::msg::DynamicStatus::ConstSharedPtr dynamic_;
    std::chrono::steady_clock::time_point dynamic_received_;
    std::chrono::steady_clock::time_point last_ego_received_;
    std::int64_t last_ego_stamp_{};
    std::int64_t last_published_stamp_{};
    std::int64_t reset_cutoff_stamp_{};
    double input_timeout_s_{};
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::EgoPose>::SharedPtr ego_subscription_;
    rclcpp::Subscription<interfaces::msg::DynamicStatus>::SharedPtr dynamic_subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedAnnotator>());
    rclcpp::shutdown();
}
