#include "hdmap_dynamic_tracker/motion_predictor.hpp"
#include "hdmap_dynamic_tracker/tracker_core.hpp"

#include "hdmap/hdmap.hpp"
#include "interfaces/msg/dynamic_status.hpp"
#include "interfaces/msg/ego_pose.hpp"
#include "interfaces/msg/ego_status.hpp"
#include "interfaces/msg/objects.hpp"
#include "interfaces/msg/traffic_light.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace hdmap_dynamic_tracker {
namespace {

constexpr std::size_t kOccupancyBinCount = 13;
constexpr double kPredictionIntervalS = 0.5;
constexpr std::size_t kSnapshotHistory = 4;
constexpr double kUnlimitedZ = std::numeric_limits<double>::max();

template<typename Message>
std::int64_t stampKey(const Message& message) {
    return static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(message.header.stamp.nanosec);
}

template<typename Message>
double stampSeconds(const Message& message) {
    return static_cast<double>(message.header.stamp.sec) +
        static_cast<double>(message.header.stamp.nanosec) * 1.0e-9;
}

bool finite(float value) {
    return std::isfinite(static_cast<double>(value));
}

template<typename Map>
void pruneSnapshotMap(Map& values) {
    while (values.size() > kSnapshotHistory) {
        values.erase(values.begin());
    }
}

lanelet::BasicPolygon2d toLaneletPolygon(const std::vector<Point2d>& points) {
    lanelet::BasicPolygon2d polygon;
    polygon.reserve(points.size());
    for (const auto& point : points) {
        polygon.emplace_back(point.x, point.y);
    }
    return polygon;
}

struct Snapshot {
    std::int64_t stamp = 0;
    std::uint64_t revision = 0;
    interfaces::msg::EgoPose::SharedPtr ego;
    interfaces::msg::Objects::SharedPtr objects;
    interfaces::msg::TrafficLight::SharedPtr traffic_light;
};

struct SignalCellCap {
    hdmap::CellId cell_id = 0;
    float cap_mps = 0.0F;
};

struct SignalConstraint {
    std::int32_t controller_id = 0;
    std::vector<int> permitted_states;
    std::vector<SignalCellCap> restricted_caps;
};

}  // namespace

class HdMapDynamicTrackerNode final : public rclcpp::Node {
public:
    HdMapDynamicTrackerNode()
        : Node("hdmap_dynamic_tracker"),
          ego_speed_estimator_(
              declare_parameter<double>("ego.maximum_dt_s", 0.25),
              declare_parameter<double>("ego.maximum_speed_kph", 300.0) / 3.6) {
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        target_rate_hz_ = declare_parameter<double>("target_rate_hz", 20.0);
        unknown_probability_ = declare_parameter<double>("occupancy.unknown_probability", -1.0);
        known_free_radius_m_ = declare_parameter<double>("occupancy.known_free_radius_m", 0.0);
        free_space_assumption_verified_ =
            declare_parameter<bool>("occupancy.free_space_assumption_verified", false);
        allow_free_when_object_list_full_ =
            declare_parameter<bool>("occupancy.allow_free_when_object_list_full", false);
        uncertainty_sigma_multiplier_ =
            declare_parameter<double>("occupancy.uncertainty_sigma_multiplier", 2.0);
        restrict_unobserved_signals_ =
            declare_parameter<bool>("signals.restrict_unobserved", true);
        signal_calibration_verified_ =
            declare_parameter<bool>("signals.calibration_verified", false);
        rear_axle_to_front_m_ = declare_parameter<double>("vehicle.rear_axle_to_front_m", 3.808);

        PredictorConfig predictor_config;
        predictor_config.process_acceleration_sigma_mps2 =
            declare_parameter<double>("prediction.process_acceleration_sigma_mps2", 2.0);
        predictor_config.position_measurement_sigma_m =
            declare_parameter<double>("prediction.position_measurement_sigma_m", 0.5);
        predictor_config.initial_velocity_sigma_mps =
            declare_parameter<double>("prediction.initial_velocity_sigma_mps", 5.0);
        predictor_config.track_retention_s =
            declare_parameter<double>("prediction.track_retention_s", 0.5);
        predictor_config.minimum_frame_dt_s =
            declare_parameter<double>("prediction.minimum_frame_dt_s", 1.0e-4);
        predictor_config.maximum_frame_dt_s =
            get_parameter("ego.maximum_dt_s").as_double();
        predictor_config.maximum_position_innovation_m =
            declare_parameter<double>("prediction.maximum_position_innovation_m", 15.0);
        const auto minimum_velocity_observations = declare_parameter<std::int64_t>(
            "prediction.minimum_velocity_observations", 4);
        if (minimum_velocity_observations < 2 ||
            static_cast<std::uint64_t>(minimum_velocity_observations) >
                std::numeric_limits<std::size_t>::max()) {
            throw std::invalid_argument(
                "prediction.minimum_velocity_observations must be at least 2");
        }
        predictor_config.minimum_velocity_observations =
            static_cast<std::size_t>(minimum_velocity_observations);
        predictor_config.minimum_velocity_observation_span_s = declare_parameter<double>(
            "prediction.minimum_velocity_observation_span_s", 0.15);
        predictor_config.minimum_velocity_displacement_m = declare_parameter<double>(
            "prediction.minimum_velocity_displacement_m", 1.0);
        predictor_config.maximum_velocity_innovation_mps = declare_parameter<double>(
            "prediction.maximum_velocity_innovation_mps", 3.0);
        predictor_ = makeEkfPredictor(predictor_config);

        ramp_template_.design_deceleration_mps2 =
            declare_parameter<double>("signals.design_deceleration_mps2", 2.0);
        ramp_template_.braking_distance_factor =
            declare_parameter<double>("signals.braking_distance_factor", 1.0);
        ramp_template_.latency_budget_s =
            declare_parameter<double>("signals.latency_budget_s", 0.25);
        ramp_template_.stop_margin_m =
            declare_parameter<double>("signals.stop_margin_m", 1.0);

        validateParameters();
        const auto map_path = resolveMapPath(declare_parameter<std::string>("map_path", ""));
        static_map_ = hdmap::hdmap_init(map_path);
        initializeStaticMapState();

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        input_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        worker_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = input_callback_group_;
        ego_status_publisher_ = create_publisher<interfaces::msg::EgoStatus>("/ego_status", qos);
        dynamic_status_publisher_ = create_publisher<interfaces::msg::DynamicStatus>("/dynamic_status", qos);
        ego_subscription_ = create_subscription<interfaces::msg::EgoPose>(
            "/ego_pose", qos, [this](interfaces::msg::EgoPose::SharedPtr message) {
                receiveEgo(std::move(message));
            }, subscription_options);
        objects_subscription_ = create_subscription<interfaces::msg::Objects>(
            "/objects", qos, [this](interfaces::msg::Objects::SharedPtr message) {
                receiveObjects(std::move(message));
            }, subscription_options);
        traffic_light_subscription_ = create_subscription<interfaces::msg::TrafficLight>(
            "/traffic_light", qos, [this](interfaces::msg::TrafficLight::SharedPtr message) {
                receiveTrafficLight(std::move(message));
            }, subscription_options);

        const auto timer_period = std::chrono::duration<double>(1.0 / target_rate_hz_);
        worker_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            [this]() { processLatest(); }, worker_callback_group_);

        RCLCPP_INFO(get_logger(),
            "HDMap Dynamic Tracker ready: %zu cells, %zu signal constraints, %.1f Hz",
            static_map_->cells().size(), signal_constraints_.size(), target_rate_hz_);
        RCLCPP_INFO(get_logger(),
            "DynamicStatus layout: %zu speed caps and %zu occupancy bins (%.2f MiB raw arrays)",
            static_speed_caps_.size(), static_speed_caps_.size() * kOccupancyBinCount,
            static_cast<double>((static_speed_caps_.size() +
                static_speed_caps_.size() * kOccupancyBinCount) * sizeof(float)) /
                (1024.0 * 1024.0));
        if (!signal_calibration_verified_) {
            RCLCPP_WARN(get_logger(),
                "Signal braking parameters are marked unverified; keep allow_motion=false until calibrated");
        }
        if (known_free_radius_m_ <= 0.0) {
            RCLCPP_WARN(get_logger(),
                "No region is asserted free: unoccupied cells remain unknown (-1); occupied cells are still published");
        }
    }

private:
    void validateParameters() const {
        if (frame_id_.empty()) {
            throw std::invalid_argument("frame_id must not be empty");
        }
        if (!std::isfinite(target_rate_hz_) || target_rate_hz_ <= 0.0) {
            throw std::invalid_argument("target_rate_hz must be finite and positive");
        }
        if (unknown_probability_ != -1.0) {
            throw std::invalid_argument("DynamicStatus contract requires unknown_probability=-1");
        }
        if (!std::isfinite(known_free_radius_m_) || known_free_radius_m_ < 0.0 ||
            known_free_radius_m_ > 80.0 ||
            !std::isfinite(uncertainty_sigma_multiplier_) || uncertainty_sigma_multiplier_ <= 0.0 ||
            !std::isfinite(rear_axle_to_front_m_) || rear_axle_to_front_m_ < 0.0) {
            throw std::invalid_argument("Invalid occupancy or vehicle parameter");
        }
        if ((known_free_radius_m_ > 0.0 || allow_free_when_object_list_full_) &&
            !free_space_assumption_verified_) {
            throw std::invalid_argument(
                "Free-space assertions require occupancy.free_space_assumption_verified=true");
        }
        if (!validStopRamp(StopRampParameters{1.0,
                ramp_template_.design_deceleration_mps2,
                ramp_template_.braking_distance_factor,
                ramp_template_.latency_budget_s,
                ramp_template_.stop_margin_m})) {
            throw std::invalid_argument("Invalid signal stop-ramp parameters");
        }
    }

    std::string resolveMapPath(const std::string& configured_path) const {
        if (!configured_path.empty()) {
            return configured_path;
        }
        const char* environment_path = std::getenv("HDMAP_PATH");
        if (environment_path == nullptr || std::string(environment_path).empty()) {
            throw std::invalid_argument("map_path is empty and HDMAP_PATH is not set");
        }
        return environment_path;
    }

    void initializeStaticMapState() {
        const auto& cells = static_map_->cells();
        static_speed_caps_.assign(cells.size(), 0.0F);
        cell_centers_.resize(cells.size());
        std::unordered_map<hdmap::LaneletId, float> lane_caps;
        std::size_t missing_speed_limits = 0;
        for (const auto& cell : cells) {
            const auto lane_id = cell.parent().lanelet_id;
            auto cap_iterator = lane_caps.find(lane_id);
            if (cap_iterator == lane_caps.end()) {
                const auto& lane = static_map_->laneletMap().laneletLayer.get(lane_id);
                float cap = 0.0F;
                if (lane.hasAttribute("speed_limit")) {
                    const auto parsed = parseSpeedLimitMps(
                        lane.attributeOr<std::string>("speed_limit", ""));
                    if (parsed && std::isfinite(*parsed) && *parsed >= 0.0 &&
                        *parsed <= std::numeric_limits<float>::max()) {
                        cap = static_cast<float>(*parsed);
                    } else {
                        ++missing_speed_limits;
                    }
                } else {
                    ++missing_speed_limits;
                }
                cap_iterator = lane_caps.emplace(lane_id, cap).first;
            }
            static_speed_caps_.at(cell.id()) = cap_iterator->second;

            const auto polygon = cell.polygon2d();
            Point2d center;
            for (const auto& point : polygon) {
                center.x += point.x();
                center.y += point.y();
            }
            if (!polygon.empty()) {
                center.x /= static_cast<double>(polygon.size());
                center.y /= static_cast<double>(polygon.size());
            }
            cell_centers_.at(cell.id()) = center;
        }
        if (missing_speed_limits > 0) {
            RCLCPP_WARN(get_logger(),
                "%zu lanelets have missing/invalid explicit speed_limit and are capped at 0 m/s",
                missing_speed_limits);
        }
        buildSignalConstraints();
    }

    double cellLength(const hdmap::Cell& cell) const {
        const auto parsed = parsePositiveMetres(
            cell.polygon3d().attributeOr<std::string>("centerline_length_m", ""));
        if (parsed) {
            return *parsed;
        }
        ++cell_length_fallback_count_;
        const auto* previous = cell.previous();
        if (previous != nullptr) {
            const auto& here = cell_centers_.at(cell.id());
            const auto& before = cell_centers_.at(previous->id());
            const double distance = std::hypot(here.x - before.x, here.y - before.y);
            if (std::isfinite(distance) && distance > 0.0) {
                return distance;
            }
        }
        return 1.0;
    }

    void buildSignalConstraints() {
        std::size_t missing_stoplines = 0;
        std::size_t broken_previous_chains = 0;
        std::size_t unscoped_signal_rules = 0;
        std::unordered_map<lanelet::Id, std::unordered_set<hdmap::LaneletId>> rule_lanelets;
        for (const auto& lane : static_map_->laneletMap().laneletLayer) {
            for (const auto& rule : lane.regulatoryElements()) {
                rule_lanelets[rule->id()].insert(lane.id());
            }
        }
        for (const auto& registry_entry : static_map_->signalRegistry()) {
            registry_controller_ids_.insert(registry_entry.first);
            for (const auto& signal : registry_entry.second) {
                SignalConstraint constraint;
                constraint.controller_id = registry_entry.first;
                constraint.permitted_states = parseIntegerCsv(
                    signal->attributeOr<std::string>("permitted_states", ""));
                const auto stop_line = signal->stopLine();
                if (!stop_line) {
                    ++missing_stoplines;
                    unresolved_controller_ids_.insert(registry_entry.first);
                    continue;
                }
                const auto stop_cells = static_map_->stoplineCells().find(stop_line->id());
                if (stop_cells == static_map_->stoplineCells().end() || stop_cells->second.empty()) {
                    ++missing_stoplines;
                    unresolved_controller_ids_.insert(registry_entry.first);
                    continue;
                }

                std::unordered_map<hdmap::CellId, float> minimum_caps;
                const auto scoped_lanelets = rule_lanelets.find(signal->id());
                const bool has_lane_scope = scoped_lanelets != rule_lanelets.end() &&
                    !scoped_lanelets->second.empty();
                if (!has_lane_scope) {
                    ++unscoped_signal_rules;
                }
                bool incomplete_previous_chain = false;
                for (const auto stop_cell_id : stop_cells->second) {
                    if (has_lane_scope && scoped_lanelets->second.count(
                        static_map_->cells().at(stop_cell_id).parent().lanelet_id) == 0U) {
                        continue;
                    }
                    StopRampParameters ramp = ramp_template_;
                    ramp.entry_speed_mps = static_speed_caps_.at(stop_cell_id);
                    const double traversal_limit = stopRampStartDistance(ramp) + rear_axle_to_front_m_;
                    const hdmap::Cell* cell = &static_map_->cells().at(stop_cell_id);
                    double center_distance = 0.0;
                    std::unordered_set<hdmap::CellId> visited;
                    while (cell != nullptr && center_distance <= traversal_limit &&
                        visited.insert(cell->id()).second) {
                        const double rear_axle_distance =
                            std::max(0.0, center_distance - rear_axle_to_front_m_);
                        const float cap = static_cast<float>(stopRampCap(rear_axle_distance, ramp));
                        const auto [iterator, inserted] = minimum_caps.emplace(cell->id(), cap);
                        if (!inserted) {
                            iterator->second = std::min(iterator->second, cap);
                        }
                        center_distance += cellLength(*cell);
                        cell = cell->previous();
                    }
                    if (center_distance <= traversal_limit) {
                        ++broken_previous_chains;
                        incomplete_previous_chain = true;
                    }
                }
                constraint.restricted_caps.reserve(minimum_caps.size());
                for (const auto& cap : minimum_caps) {
                    constraint.restricted_caps.push_back({cap.first, cap.second});
                }
                if (constraint.restricted_caps.empty()) {
                    ++missing_stoplines;
                    unresolved_controller_ids_.insert(registry_entry.first);
                    continue;
                }
                if (incomplete_previous_chain) {
                    unresolved_controller_ids_.insert(registry_entry.first);
                }
                signal_constraints_.push_back(std::move(constraint));
            }
        }
        if (missing_stoplines > 0) {
            RCLCPP_WARN(get_logger(),
                "%zu signal rules have no resolved stopline cells; they remain unresolved, never inferred green",
                missing_stoplines);
        }
        if (broken_previous_chains > 0) {
            RCLCPP_WARN(get_logger(),
                "%zu stopline traversals ended before the requested ramp distance; merge predecessors require RoutingGraph",
                broken_previous_chains);
        }
        if (cell_length_fallback_count_ > 0) {
            RCLCPP_WARN(get_logger(),
                "%zu cells lacked valid centerline_length_m; geometry distance (or 1 m last resort) was used",
                cell_length_fallback_count_);
        }
        if (unscoped_signal_rules > 0) {
            RCLCPP_WARN(get_logger(),
                "%zu signal rules were not attached to lanelets; their full stopline cell set was used",
                unscoped_signal_rules);
        }
    }

    bool validEgo(const interfaces::msg::EgoPose& message) const {
        return stampKey(message) > 0 && message.header.frame_id == frame_id_ &&
            finite(message.x) && finite(message.y) && finite(message.z) &&
            finite(message.heading) && finite(message.pitch) && finite(message.roll);
    }

    bool validObjects(const interfaces::msg::Objects& message) const {
        if (stampKey(message) <= 0 || message.header.frame_id != frame_id_ ||
            message.length < 0 || message.length > 30) {
            return false;
        }
        std::unordered_set<std::uint32_t> ids;
        for (std::int32_t index = 0; index < message.length; ++index) {
            const auto i = static_cast<std::size_t>(index);
            if (!ids.insert(message.id[i]).second ||
                !finite(message.x[i]) || !finite(message.y[i]) || !finite(message.z[i]) ||
                !finite(message.heading[i]) || !finite(message.speed[i]) ||
                !finite(message.size_x[i]) || !finite(message.size_y[i]) || !finite(message.size_z[i]) ||
                message.speed[i] < 0.0F || message.size_x[i] <= 0.0F ||
                message.size_y[i] <= 0.0F || message.size_z[i] <= 0.0F) {
                return false;
            }
        }
        return true;
    }

    bool validTrafficLight(const interfaces::msg::TrafficLight& message) const {
        return stampKey(message) > 0 && message.header.frame_id.empty() && message.state <= 6;
    }

    void receiveEgo(interfaces::msg::EgoPose::SharedPtr message) {
        if (!validEgo(*message)) {
            RCLCPP_WARN(get_logger(), "Rejected invalid /ego_pose sample");
            return;
        }
        const auto key = stampKey(*message);
        if (key == last_ego_status_stamp_) {
            return;
        }
        last_ego_status_stamp_ = key;
        const auto speed = ego_speed_estimator_.update(
            stampSeconds(*message), message->x, message->y);
        if (speed.history_reset) {
            std::lock_guard<std::mutex> lock(tracker_state_mutex_);
            predictor_->reset();
        }

        interfaces::msg::EgoStatus status;
        status.header = message->header;
        status.x = message->x;
        status.y = message->y;
        status.z = message->z;
        status.heading = message->heading;
        status.pitch = message->pitch;
        status.roll = message->roll;
        status.speed = static_cast<float>(speed.speed_mps);
        ego_status_publisher_->publish(std::move(status));

        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        if (key < last_taken_stamp_) {
            ego_messages_.clear();
            object_messages_.clear();
            traffic_light_messages_.clear();
            last_taken_stamp_ = 0;
        } else if (key == last_taken_stamp_) {
            return;
        }
        ego_messages_[key] = std::move(message);
        pruneSnapshotMap(ego_messages_);
        updateNewestCompleteStampLocked();
    }

    void receiveObjects(interfaces::msg::Objects::SharedPtr message) {
        if (!validObjects(*message)) {
            RCLCPP_WARN(get_logger(), "Rejected invalid /objects sample");
            return;
        }
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        object_messages_[stampKey(*message)] = std::move(message);
        pruneSnapshotMap(object_messages_);
        updateNewestCompleteStampLocked();
    }

    void receiveTrafficLight(interfaces::msg::TrafficLight::SharedPtr message) {
        if (!validTrafficLight(*message)) {
            RCLCPP_WARN(get_logger(), "Rejected invalid /traffic_light sample");
            return;
        }
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        traffic_light_messages_[stampKey(*message)] = std::move(message);
        pruneSnapshotMap(traffic_light_messages_);
        updateNewestCompleteStampLocked();
    }

    std::int64_t newestCompleteStampLocked() const {
        for (auto iterator = ego_messages_.rbegin(); iterator != ego_messages_.rend(); ++iterator) {
            if (object_messages_.count(iterator->first) != 0U &&
                traffic_light_messages_.count(iterator->first) != 0U) {
                return iterator->first;
            }
        }
        return 0;
    }

    void updateNewestCompleteStampLocked() {
        const auto complete_stamp = newestCompleteStampLocked();
        if (complete_stamp != 0 && complete_stamp != announced_complete_stamp_) {
            announced_complete_stamp_ = complete_stamp;
            newest_complete_revision_.fetch_add(1, std::memory_order_release);
        } else if (complete_stamp == 0) {
            announced_complete_stamp_ = 0;
        }
    }

    std::optional<Snapshot> takeLatestCompleteSnapshot() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        const auto key = newestCompleteStampLocked();
        if (key <= last_taken_stamp_) {
            return std::nullopt;
        }
        Snapshot snapshot{key, newest_complete_revision_.load(std::memory_order_acquire),
            ego_messages_.at(key), object_messages_.at(key),
            traffic_light_messages_.at(key)};
        last_taken_stamp_ = key;
        auto erase_through = [key](auto& values) {
            values.erase(values.begin(), values.upper_bound(key));
        };
        erase_through(ego_messages_);
        erase_through(object_messages_);
        erase_through(traffic_light_messages_);
        updateNewestCompleteStampLocked();
        return snapshot;
    }

    std::vector<ObjectObservation> makeObservations(const interfaces::msg::Objects& message) const {
        std::vector<ObjectObservation> observations;
        observations.reserve(static_cast<std::size_t>(message.length));
        for (std::int32_t index = 0; index < message.length; ++index) {
            const auto i = static_cast<std::size_t>(index);
            observations.push_back({message.id[i], message.x[i], message.y[i], message.z[i],
                normalizeAngle(message.heading[i]), message.speed[i], message.size_x[i], message.size_y[i],
                message.size_z[i]});
        }
        return observations;
    }

    void markKnownFreeCurrent(
        std::vector<float>& occupancy,
        const interfaces::msg::EgoPose& ego,
        const interfaces::msg::Objects& objects) const {
        if (known_free_radius_m_ <= 0.0 ||
            (!allow_free_when_object_list_full_ && objects.length == 30)) {
            return;
        }
        const lanelet::BoundingBox2d query_box(
            lanelet::BasicPoint2d(ego.x - known_free_radius_m_, ego.y - known_free_radius_m_),
            lanelet::BasicPoint2d(ego.x + known_free_radius_m_, ego.y + known_free_radius_m_));
        const double radius_squared = known_free_radius_m_ * known_free_radius_m_;
        for (const auto cell_id : static_map_->cellTree().search(query_box)) {
            const auto& center = cell_centers_.at(cell_id);
            const double dx = center.x - ego.x;
            const double dy = center.y - ego.y;
            if (dx * dx + dy * dy <= radius_squared) {
                occupancy.at(static_cast<std::size_t>(cell_id) * kOccupancyBinCount) = 0.0F;
            }
        }
    }

    void markFootprint(
        std::vector<float>& occupancy,
        std::size_t bin,
        const std::vector<Point2d>& footprint,
        double min_z,
        double max_z) const {
        const auto polygon = toLaneletPolygon(footprint);
        for (const auto cell_id : static_map_->cellTree().queryOverlaps(polygon, min_z, max_z)) {
            const auto index = static_cast<std::size_t>(cell_id) * kOccupancyBinCount + bin;
            occupancy.at(index) = 1.0F;
        }
    }

    void updateOccupancy(
        std::vector<float>& occupancy,
        double stamp_s,
        const interfaces::msg::EgoPose& ego,
        const interfaces::msg::Objects& objects) {
        markKnownFreeCurrent(occupancy, ego, objects);
        const auto observations = makeObservations(objects);
        predictor_->updateFrame(stamp_s, observations);

        std::unordered_set<std::uint32_t> observed_ids;
        observed_ids.reserve(observations.size());
        for (const auto& observation : observations) {
            observed_ids.insert(observation.id);
            ObjectPrediction exact;
            exact.id = observation.id;
            exact.x = observation.x;
            exact.y = observation.y;
            exact.min_z = observation.min_z;
            exact.heading = observation.heading;
            exact.length = observation.length;
            exact.width = observation.width;
            exact.height = observation.height;
            markFootprint(occupancy, 0, orientedBox(exact),
                exact.min_z, exact.min_z + exact.height);
        }

        const auto current = predictor_->predict(stamp_s, 0.0);
        for (const auto& object : current) {
            if (observed_ids.count(object.id) != 0U) {
                continue;
            }
            const double inflation = predictionUncertaintyInflation(
                object.position_sigma_m, object.direction_uncertainty_m,
                uncertainty_sigma_multiplier_) +
                yawIndependentRotationInflation(object.length, object.width);
            markFootprint(occupancy, 0, orientedBox(object, inflation),
                -kUnlimitedZ, kUnlimitedZ);
        }
        for (std::size_t bin = 1; bin < kOccupancyBinCount; ++bin) {
            const double start_time = static_cast<double>(bin - 1) * kPredictionIntervalS;
            const double end_time = static_cast<double>(bin) * kPredictionIntervalS;
            auto start = predictor_->predict(stamp_s, start_time);
            const auto end = predictor_->predict(stamp_s, end_time);
            if (bin == 1) {
                std::unordered_map<std::uint32_t, const ObjectObservation*> observed_by_id;
                observed_by_id.reserve(observations.size());
                for (const auto& observation : observations) {
                    observed_by_id.emplace(observation.id, &observation);
                }
                for (auto& prediction : start) {
                    const auto observation = observed_by_id.find(prediction.id);
                    if (observation == observed_by_id.end()) {
                        continue;
                    }
                    prediction.x = observation->second->x;
                    prediction.y = observation->second->y;
                    prediction.min_z = observation->second->min_z;
                    prediction.heading = observation->second->heading;
                    prediction.length = observation->second->length;
                    prediction.width = observation->second->width;
                    prediction.height = observation->second->height;
                }
            }
            std::unordered_map<std::uint32_t, ObjectPrediction> end_by_id;
            end_by_id.reserve(end.size());
            for (const auto& prediction : end) {
                end_by_id.emplace(prediction.id, prediction);
            }
            for (const auto& start_prediction : start) {
                const auto iterator = end_by_id.find(start_prediction.id);
                if (iterator == end_by_id.end()) {
                    continue;
                }
                const auto& end_prediction = iterator->second;
                const double maximum_length =
                    std::max(start_prediction.length, end_prediction.length);
                const double maximum_width =
                    std::max(start_prediction.width, end_prediction.width);
                const double inflation = predictionUncertaintyInflation(
                    std::max(start_prediction.position_sigma_m, end_prediction.position_sigma_m),
                    std::max(start_prediction.direction_uncertainty_m,
                        end_prediction.direction_uncertainty_m),
                    uncertainty_sigma_multiplier_) +
                    yawIndependentRotationInflation(maximum_length, maximum_width);
                markFootprint(occupancy, bin,
                    sweptFootprint(start_prediction, end_prediction, inflation),
                    -kUnlimitedZ, kUnlimitedZ);
            }
        }
    }

    bool permitted(const SignalConstraint& constraint, const interfaces::msg::TrafficLight& state) const {
        if (state.id != constraint.controller_id || state.state < 3 || state.state > 5) {
            return false;
        }
        return std::binary_search(
            constraint.permitted_states.begin(), constraint.permitted_states.end(),
            static_cast<int>(state.state));
    }

    void updateSignalCaps(
        std::vector<float>& speed_caps,
        const interfaces::msg::TrafficLight& traffic_light) const {
        if (traffic_light.id != 0 &&
            (registry_controller_ids_.count(traffic_light.id) == 0U ||
                unresolved_controller_ids_.count(traffic_light.id) != 0U)) {
            std::fill(speed_caps.begin(), speed_caps.end(), 0.0F);
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                "Observed controller %d has no complete stopline mapping; applying global 0 m/s cap",
                traffic_light.id);
            return;
        }
        for (const auto& constraint : signal_constraints_) {
            const bool observed_controller = traffic_light.id == constraint.controller_id;
            const bool should_restrict = observed_controller ? !permitted(constraint, traffic_light) :
                restrict_unobserved_signals_;
            if (!should_restrict) {
                continue;
            }
            for (const auto& restriction : constraint.restricted_caps) {
                auto& cap = speed_caps.at(restriction.cell_id);
                cap = std::min(cap, restriction.cap_mps);
            }
        }
        bool permitted_rule_seen = false;
        bool shared_unknown_still_restricts = false;
        for (const auto& constraint : signal_constraints_) {
            if (!permitted(constraint, traffic_light)) {
                continue;
            }
            permitted_rule_seen = true;
            for (const auto& restriction : constraint.restricted_caps) {
                if (speed_caps.at(restriction.cell_id) <
                    static_speed_caps_.at(restriction.cell_id)) {
                    shared_unknown_still_restricts = true;
                    break;
                }
            }
        }
        if (permitted_rule_seen && shared_unknown_still_restricts) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Controller %d is permitted but a shared unobserved signal constraint still caps its approach",
                traffic_light.id);
        }
    }

    interfaces::msg::DynamicStatus buildDynamicStatus(const Snapshot& snapshot) {
        interfaces::msg::DynamicStatus status;
        status.header = snapshot.ego->header;
        status.speed_cap_mps = static_speed_caps_;
        status.occupancy_probability.assign(
            static_speed_caps_.size() * kOccupancyBinCount,
            static_cast<float>(unknown_probability_));
        updateSignalCaps(status.speed_cap_mps, *snapshot.traffic_light);
        std::lock_guard<std::mutex> lock(tracker_state_mutex_);
        updateOccupancy(status.occupancy_probability, stampSeconds(*snapshot.ego),
            *snapshot.ego, *snapshot.objects);
        return status;
    }

    void processLatest() {
        const auto snapshot = takeLatestCompleteSnapshot();
        if (!snapshot) {
            return;
        }
        const auto started = std::chrono::steady_clock::now();
        auto status = buildDynamicStatus(*snapshot);
        if (newest_complete_revision_.load(std::memory_order_acquire) > snapshot->revision) {
            ++discarded_stale_results_;
            return;
        }
        dynamic_status_publisher_->publish(std::move(status));
        ++published_snapshots_;
        const auto elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        if (elapsed > 1000.0 / target_rate_hz_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Tracker frame took %.1f ms (budget %.1f ms); published=%zu stale_discarded=%zu",
                elapsed, 1000.0 / target_rate_hz_, published_snapshots_, discarded_stale_results_);
        }
    }

    std::string frame_id_;
    double target_rate_hz_ = 20.0;
    double unknown_probability_ = -1.0;
    double known_free_radius_m_ = 0.0;
    bool free_space_assumption_verified_ = false;
    bool allow_free_when_object_list_full_ = false;
    double uncertainty_sigma_multiplier_ = 2.0;
    bool restrict_unobserved_signals_ = true;
    bool signal_calibration_verified_ = false;
    double rear_axle_to_front_m_ = 3.808;
    StopRampParameters ramp_template_;

    std::unique_ptr<hdmap::HdMap> static_map_;
    std::vector<float> static_speed_caps_;
    std::vector<Point2d> cell_centers_;
    std::vector<SignalConstraint> signal_constraints_;
    std::unordered_set<std::int32_t> registry_controller_ids_;
    std::unordered_set<std::int32_t> unresolved_controller_ids_;
    mutable std::size_t cell_length_fallback_count_ = 0;

    EgoSpeedEstimator ego_speed_estimator_;
    std::unique_ptr<MotionPredictor> predictor_;
    std::mutex tracker_state_mutex_;

    std::mutex snapshot_mutex_;
    std::map<std::int64_t, interfaces::msg::EgoPose::SharedPtr> ego_messages_;
    std::map<std::int64_t, interfaces::msg::Objects::SharedPtr> object_messages_;
    std::map<std::int64_t, interfaces::msg::TrafficLight::SharedPtr> traffic_light_messages_;
    std::int64_t last_taken_stamp_ = 0;
    std::int64_t last_ego_status_stamp_ = 0;
    std::int64_t announced_complete_stamp_ = 0;
    std::atomic<std::uint64_t> newest_complete_revision_{0};

    std::size_t published_snapshots_ = 0;
    std::size_t discarded_stale_results_ = 0;

    rclcpp::Publisher<interfaces::msg::EgoStatus>::SharedPtr ego_status_publisher_;
    rclcpp::Publisher<interfaces::msg::DynamicStatus>::SharedPtr dynamic_status_publisher_;
    rclcpp::CallbackGroup::SharedPtr input_callback_group_;
    rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
    rclcpp::Subscription<interfaces::msg::EgoPose>::SharedPtr ego_subscription_;
    rclcpp::Subscription<interfaces::msg::Objects>::SharedPtr objects_subscription_;
    rclcpp::Subscription<interfaces::msg::TrafficLight>::SharedPtr traffic_light_subscription_;
    rclcpp::TimerBase::SharedPtr worker_timer_;
};

}  // namespace hdmap_dynamic_tracker

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<hdmap_dynamic_tracker::HdMapDynamicTrackerNode>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "HDMap Dynamic Tracker failed: %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
