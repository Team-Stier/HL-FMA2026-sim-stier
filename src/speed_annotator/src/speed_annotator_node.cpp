#include "hdmap/hdmap.hpp"
#include "interfaces/msg/dynamic_status.hpp"
#include "interfaces/msg/ego_pose.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>

class SpeedAnnotator final : public rclcpp::Node {
public:
    SpeedAnnotator() : Node("speed_annotator") {
        const auto path = declare_parameter<std::string>("map_path", "");
        map_ = hdmap::hdmap_init(path.empty() ? std::getenv("HDMAP_PATH") : path);
        const auto qos = rclcpp::QoS(1).best_effort();
        publisher_ = create_publisher<std_msgs::msg::Float32>("/speed_limit", qos);
        ego_subscription_ = create_subscription<interfaces::msg::EgoPose>("/ego_pose", qos,
            [this](const interfaces::msg::EgoPose& message) { ego_ = message; publish(); });
        dynamic_subscription_ = create_subscription<interfaces::msg::DynamicStatus>("/dynamic_status", qos,
            [this](const interfaces::msg::DynamicStatus& message) { dynamic_ = message; publish(); });
    }

private:
    void publish() {
        if (!ego_ || !dynamic_) return;

        const double c = std::cos(ego_->heading);
        const double s = std::sin(ego_->heading);
        lanelet::BasicPolygon2d footprint;
        for (const auto [x, y] : std::array<std::array<double, 2>, 4>{{
                 {3.808, .943}, {3.808, -.943}, {-1.040, -.943}, {-1.040, .943}}}) {
            footprint.emplace_back(ego_->x + c * x - s * y, ego_->y + s * x + c * y);
        }

        float cap = std::numeric_limits<float>::infinity();
        for (const auto cell : map_->cellTree().queryOverlaps(
                 footprint, ego_->z, ego_->z + 1.507)) {
            cap = std::min(cap, dynamic_->speed_cap_mps[cell]);
        }
        std_msgs::msg::Float32 message;
        message.data = std::isfinite(cap) ? cap : 0.F;
        publisher_->publish(message);
    }

    std::unique_ptr<hdmap::HdMap> map_;
    std::optional<interfaces::msg::EgoPose> ego_;
    std::optional<interfaces::msg::DynamicStatus> dynamic_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<interfaces::msg::EgoPose>::SharedPtr ego_subscription_;
    rclcpp::Subscription<interfaces::msg::DynamicStatus>::SharedPtr dynamic_subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedAnnotator>());
    rclcpp::shutdown();
}
