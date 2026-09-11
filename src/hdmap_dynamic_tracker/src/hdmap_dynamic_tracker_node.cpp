#include "hdmap_dynamic_tracker/motion_predictor.hpp"
#include "hdmap_dynamic_tracker/signal_state_manager.hpp"
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
    interfaces::msg::EgoPose::SharedPtr ego;
    double ego_speed_mps = 0.0;
    interfaces::msg::Objects::SharedPtr objects;
    interfaces::msg::TrafficLight::SharedPtr traffic_light;
};

struct EgoSample {
    interfaces::msg::EgoPose::SharedPtr message;
    double speed_mps = 0.0;
};

struct SignalCellCap {
    hdmap::CellId cell_id = 0;
    double distance_from_stop_edge_m = 0.0;
    double cell_length_m = 0.0;
};

struct SignalConstraint {
    std::int32_t controller_id = 0;
    std::vector<int> permitted_states;
    std::vector<SignalCellCap> restricted_caps;
};

struct ActiveSignalApproach {
    std::size_t constraint_index = 0;
    double front_axle_to_stopline_m = 0.0;
    double static_speed_cap_mps = 0.0;
};

}  // namespace

class HdMapDynamicTrackerNode final : public rclcpp::Node {
public:
    HdMapDynamicTrackerNode()
        : Node("hdmap_dynamic_tracker"),
          ego_speed_estimator_() {
        frame_id_ = declare_parameter<std::string>("frame_id", "map");
        target_rate_hz_ = declare_parameter<double>("target_rate_hz", 20.0);
        EgoMotionLimits motion_limits;
        motion_limits.maximum_speed_mps = declare_parameter<double>("ego_motion.maximum_speed_mps", 60.0);
        motion_limits.maximum_vertical_speed_mps =
            declare_parameter<double>("ego_motion.maximum_vertical_speed_mps", 10.0);
        motion_limits.maximum_yaw_rate_radps =
            declare_parameter<double>("ego_motion.maximum_yaw_rate_radps", 3.0);
        motion_limits.minimum_frame_dt_s =
            declare_parameter<double>("ego_motion.minimum_frame_dt_s", 1.0e-4);
        motion_limits.maximum_frame_dt_s =
            declare_parameter<double>("ego_motion.maximum_frame_dt_s", 1.0);
        ego_speed_estimator_ = EgoSpeedEstimator(motion_limits);
        unknown_probability_ = declare_parameter<double>("occupancy.unknown_probability", -1.0);
        known_free_radius_m_ = declare_parameter<double>("occupancy.known_free_radius_m", 0.0);
        uncertainty_sigma_multiplier_ =
            declare_parameter<double>("occupancy.uncertainty_sigma_multiplier", 2.0);
        const auto braking_speeds = declare_parameter<std::vector<double>>(
            "signals.braking_calibration.speed_mps", std::vector<double>{});
        const auto braking_distances = declare_parameter<std::vector<double>>(
            "signals.braking_calibration.distance_m", std::vector<double>{});
        if (braking_speeds.size() != braking_distances.size()) {
            throw std::invalid_argument(
                "Signal braking calibration speed and distance arrays must have equal length");
        }
        braking_distance_table_.reserve(braking_speeds.size());
        for (std::size_t index = 0; index < braking_speeds.size(); ++index) {
            braking_distance_table_.push_back({braking_speeds[index], braking_distances[index]});
        }
        rear_axle_to_front_m_ = declare_parameter<double>("vehicle.rear_axle_to_front_m", 3.808);
        wheelbase_m_ = declare_parameter<double>("vehicle.wheelbase_m", 2.944);

        PredictorConfig predictor_config;
        predictor_config.process_acceleration_sigma_mps2 =
            declare_parameter<double>("prediction.process_acceleration_sigma_mps2", 2.0);
        predictor_config.position_measurement_sigma_m =
            declare_parameter<double>("prediction.position_measurement_sigma_m", 0.5);
        predictor_config.initial_velocity_sigma_mps =
            declare_parameter<double>("prediction.initial_velocity_sigma_mps", 5.0);
        predictor_config.minimum_frame_dt_s =
            declare_parameter<double>("prediction.minimum_frame_dt_s", 1.0e-4);
        predictor_ = makeEkfPredictor(predictor_config);

        ramp_template_.design_deceleration_mps2 =
            declare_parameter<double>("signals.design_deceleration_mps2", 3.0);
        ramp_template_.braking_distance_factor =
            declare_parameter<double>("signals.braking_distance_factor", 1.0);
        ramp_template_.latency_budget_s =
            declare_parameter<double>("signals.latency_budget_s", 0.25);
        ramp_template_.stop_margin_m =
            declare_parameter<double>("signals.stop_margin_m", 1.0);

        SignalStateManagerConfig state_manager_config;
        state_manager_config.yellow_decision_deceleration_mps2 = declare_parameter<double>(
            "signals.yellow_decision_deceleration_mps2", 3.0);
        state_manager_config.braking_distance_factor = ramp_template_.braking_distance_factor;
        state_manager_config.latency_budget_s = ramp_template_.latency_budget_s;
        state_manager_config.stop_margin_m = ramp_template_.stop_margin_m;
        state_manager_config.hold_speed_mps =
            declare_parameter<double>("signals.hold_speed_mps", 0.2);
        state_manager_config.hold_distance_m =
            declare_parameter<double>("signals.hold_distance_m", 1.0);
        signal_state_manager_ = std::make_unique<SignalStateManager>(
            state_manager_config, braking_distance_table_);

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
            !std::isfinite(rear_axle_to_front_m_) || rear_axle_to_front_m_ < 0.0 ||
            !std::isfinite(wheelbase_m_) || wheelbase_m_ <= 0.0) {
            throw std::invalid_argument("Invalid occupancy or vehicle parameter");
        }
        StopRampParameters validation_ramp = ramp_template_;
        validation_ramp.entry_speed_mps = 1.0;
        if (!validStopRamp(validation_ramp) ||
            !std::isfinite(stopRampStartDistance(validation_ramp) + rear_axle_to_front_m_)) {
            throw std::invalid_argument("Invalid signal stop-ramp parameters");
        }
        if (!braking_distance_table_.empty() &&
            !validBrakingDistanceTable(braking_distance_table_)) {
            throw std::invalid_argument(
                "Signal braking calibration speeds must increase and distances must be positive and nondecreasing");
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
        for (const auto& cell : cells) {
            static_speed_caps_.at(cell.id()) = static_cast<float>(parseSpeedLimitMps(
                cell.polygon3d().attributeOr<std::string>("speed_limit", "")).value());

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
        buildSignalConstraints();
    }

    double cellLength(const hdmap::Cell& cell) const {
        return parsePositiveMetres(
            cell.polygon3d().attributeOr<std::string>("centerline_length_m", "")).value();
    }

    void buildSignalConstraints() {
        double maximum_ramp_distance = 0.0;
        for (const float static_cap : static_speed_caps_) {
            StopRampParameters ramp = ramp_template_;
            ramp.entry_speed_mps = static_cap;
            if (!validStopRamp(ramp, braking_distance_table_)) {
                throw std::invalid_argument("Signal braking calibration must cover every static speed cap");
            }
            maximum_ramp_distance = std::max(maximum_ramp_distance,
                stopRampStartDistance(ramp, braking_distance_table_));
        }
        std::unordered_map<lanelet::Id, std::unordered_set<hdmap::LaneletId>> rule_lanelets;
        for (const auto& lane : static_map_->laneletMap().laneletLayer) {
            for (const auto& rule : lane.regulatoryElements()) {
                rule_lanelets[rule->id()].insert(lane.id());
            }
        }
        for (const auto& registry_entry : static_map_->signalRegistry()) {
            for (const auto& signal : registry_entry.second) {
                SignalConstraint constraint;
                constraint.controller_id = registry_entry.first;
                constraint.permitted_states = parseIntegerCsv(
                    signal->attributeOr<std::string>("permitted_states", ""));
                const auto stop_line = signal->stopLine();
                if (!stop_line) {
                    continue;
                }
                const auto stop_cells = static_map_->stoplineCells().find(stop_line->id());
                if (stop_cells == static_map_->stoplineCells().end() || stop_cells->second.empty()) {
                    continue;
                }

                std::unordered_map<hdmap::CellId, SignalCellCap> minimum_caps;
                const auto scoped_lanelets = rule_lanelets.find(signal->id());
                const bool has_lane_scope = scoped_lanelets != rule_lanelets.end() &&
                    !scoped_lanelets->second.empty();
                for (const auto stop_cell_id : stop_cells->second) {
                    if (has_lane_scope && scoped_lanelets->second.count(
                        static_map_->cells().at(stop_cell_id).parent().lanelet_id) == 0U) {
                        continue;
                    }
                    const double traversal_limit = maximum_ramp_distance + rear_axle_to_front_m_;
                    const hdmap::Cell* cell = &static_map_->cells().at(stop_cell_id);
                    double distance_from_stop_edge = 0.0;
                    std::unordered_set<hdmap::CellId> visited;
                    while (cell != nullptr && distance_from_stop_edge <= traversal_limit &&
                        visited.insert(cell->id()).second) {
                        const double cell_length = cellLength(*cell);
                        const SignalCellCap candidate{
                            cell->id(), distance_from_stop_edge, cell_length};
                        const auto [iterator, inserted] = minimum_caps.emplace(cell->id(), candidate);
                        if (!inserted) {
                            if (distance_from_stop_edge <
                                iterator->second.distance_from_stop_edge_m) {
                                iterator->second.distance_from_stop_edge_m = distance_from_stop_edge;
                                iterator->second.cell_length_m = cell_length;
                            }
                        }
                        distance_from_stop_edge += cell_length;
                        cell = cell->previous();
                    }
                }
                constraint.restricted_caps.reserve(minimum_caps.size());
                for (const auto& cap : minimum_caps) {
                    constraint.restricted_caps.push_back(cap.second);
                }
                if (constraint.restricted_caps.empty()) {
                    continue;
                }
                signal_constraints_.push_back(std::move(constraint));
            }
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
        // SimBridge stamps are monotone receive times; delayed samples must not
        // roll the estimator or reset cutoff back to an older frame.
        if (key <= last_ego_status_stamp_) {
            return;
        }
        last_ego_status_stamp_ = key;
        const auto speed = ego_speed_estimator_.update(
            stampSeconds(*message), message->x, message->y, message->z, message->heading);
        std::unique_lock<std::mutex> state_lock(tracker_state_mutex_, std::defer_lock);
        if (speed.history_reset) {
            // Keep the reset and its new Ego publication ordered with worker
            // build/publish. The worker releases snapshot_mutex_ before this lock.
            state_lock.lock();
            std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
            reset_cutoff_stamp_ = key;
            auto erase_before = [key](auto& values) {
                values.erase(values.begin(), values.lower_bound(key));
            };
            erase_before(ego_messages_);
            erase_before(object_messages_);
            erase_before(traffic_light_messages_);
            predictor_->reset();
            signal_state_manager_->reset();
            RCLCPP_WARN(get_logger(), "Ego discontinuity at stamp %lld: reset speed and tracker history",
                static_cast<long long>(key));
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
        if (key <= last_taken_stamp_) {
            return;
        }
        ego_messages_[key] = EgoSample{std::move(message), speed.speed_mps};
        pruneSnapshotMap(ego_messages_);
    }

    void receiveObjects(interfaces::msg::Objects::SharedPtr message) {
        if (!validObjects(*message)) {
            RCLCPP_WARN(get_logger(), "Rejected invalid /objects sample");
            return;
        }
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        const auto key = stampKey(*message);
        if (key < reset_cutoff_stamp_ || key <= last_taken_stamp_) return;
        object_messages_[key] = std::move(message);
        pruneSnapshotMap(object_messages_);
    }

    void receiveTrafficLight(interfaces::msg::TrafficLight::SharedPtr message) {
        if (!validTrafficLight(*message)) {
            RCLCPP_WARN(get_logger(), "Rejected invalid /traffic_light sample");
            return;
        }
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        const auto key = stampKey(*message);
        if (key < reset_cutoff_stamp_ || key <= last_taken_stamp_) return;
        traffic_light_messages_[key] = std::move(message);
        pruneSnapshotMap(traffic_light_messages_);
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

    std::optional<Snapshot> takeLatestCompleteSnapshot() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        const auto key = newestCompleteStampLocked();
        if (key <= last_taken_stamp_) {
            return std::nullopt;
        }
        Snapshot snapshot{key,
            ego_messages_.at(key).message, ego_messages_.at(key).speed_mps,
            object_messages_.at(key),
            traffic_light_messages_.at(key)};
        last_taken_stamp_ = key;
        auto erase_through = [key](auto& values) {
            values.erase(values.begin(), values.upper_bound(key));
        };
        erase_through(ego_messages_);
        erase_through(object_messages_);
        erase_through(traffic_light_messages_);
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
        const interfaces::msg::EgoPose& ego) const {
        if (known_free_radius_m_ <= 0.0) {
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
        markKnownFreeCurrent(occupancy, ego);
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

    std::optional<ActiveSignalApproach> activeSignalApproach(
        const interfaces::msg::EgoPose& ego,
        const interfaces::msg::TrafficLight& traffic_light) const {
        const double cosine = std::cos(ego.heading);
        const double sine = std::sin(ego.heading);
        const Point2d front_axle{
            ego.x + wheelbase_m_ * cosine,
            ego.y + wheelbase_m_ * sine};
        constexpr double kPointHalfExtentM = 0.01;
        const auto front_axle_cells = static_map_->cellTree().queryOverlaps(
            toLaneletPolygon({
                {front_axle.x - kPointHalfExtentM, front_axle.y - kPointHalfExtentM},
                {front_axle.x + kPointHalfExtentM, front_axle.y - kPointHalfExtentM},
                {front_axle.x + kPointHalfExtentM, front_axle.y + kPointHalfExtentM},
                {front_axle.x - kPointHalfExtentM, front_axle.y + kPointHalfExtentM}}),
            -kUnlimitedZ, kUnlimitedZ);
        std::unordered_set<hdmap::CellId> current_cells(
            front_axle_cells.begin(), front_axle_cells.end());

        std::optional<ActiveSignalApproach> active;
        for (std::size_t constraint_index = 0;
            constraint_index < signal_constraints_.size(); ++constraint_index) {
            const auto& constraint = signal_constraints_[constraint_index];
            if (constraint.controller_id != traffic_light.id) {
                continue;
            }
            for (const auto& restriction : constraint.restricted_caps) {
                if (current_cells.count(restriction.cell_id) == 0U) {
                    continue;
                }
                const auto& center = cell_centers_.at(restriction.cell_id);
                const double half_cell_length = restriction.cell_length_m * 0.5;
                const double within_cell = std::clamp(
                    (center.x - front_axle.x) * cosine +
                        (center.y - front_axle.y) * sine,
                    -half_cell_length, half_cell_length);
                const double distance = std::max(0.0,
                    restriction.distance_from_stop_edge_m + half_cell_length + within_cell);
                if (!active || distance < active->front_axle_to_stopline_m) {
                    active = ActiveSignalApproach{
                        constraint_index,
                        distance,
                        static_speed_caps_.at(restriction.cell_id)};
                }
            }
        }
        return active;
    }

    void updateSignalCaps(
        std::vector<float>& speed_caps,
        const Snapshot& snapshot) {
        const auto active = activeSignalApproach(*snapshot.ego, *snapshot.traffic_light);
        SignalStateInput input;
        input.controller_id = snapshot.traffic_light->id;
        input.signal_state = snapshot.traffic_light->state;
        input.ego_speed_mps = snapshot.ego_speed_mps;
        input.on_approach = active.has_value();
        if (active) {
            input.approach_id = active->constraint_index;
            input.front_axle_to_stopline_m = active->front_axle_to_stopline_m;
            input.static_speed_cap_mps = active->static_speed_cap_mps;
            input.permitted = permitted(
                signal_constraints_.at(active->constraint_index), *snapshot.traffic_light);
        }

        const auto decision = signal_state_manager_->update(input);
        if (decision.transitioned) {
            RCLCPP_INFO(get_logger(),
                "Signal state: controller=%d raw=%u state=%s speed=%.2f m/s "
                "front_axle_distance=%.2f m stop_required=%.2f m target_cap=%.2f m/s",
                input.controller_id, static_cast<unsigned int>(input.signal_state),
                signalApproachStateName(decision.state), input.ego_speed_mps,
                input.front_axle_to_stopline_m,
                decision.required_stopping_distance_m,
                decision.target_speed_cap_mps);
        }
        if (!active || !decision.apply_stop_cap) {
            return;
        }
        for (const auto& restriction :
            signal_constraints_.at(active->constraint_index).restricted_caps) {
            StopRampParameters ramp = ramp_template_;
            ramp.entry_speed_mps = std::min(
                decision.decision_speed_mps,
                static_cast<double>(static_speed_caps_.at(restriction.cell_id)));
            const double front_bumper_clearance = std::max(
                0.0, restriction.distance_from_stop_edge_m - rear_axle_to_front_m_);
            auto& cap = speed_caps.at(restriction.cell_id);
            cap = std::min(cap,
                static_cast<float>(stopRampCap(front_bumper_clearance, ramp, braking_distance_table_)));
        }
    }

    interfaces::msg::DynamicStatus buildDynamicStatus(const Snapshot& snapshot) {
        interfaces::msg::DynamicStatus status;
        status.header = snapshot.ego->header;
        status.speed_cap_mps = static_speed_caps_;
        status.occupancy_probability.assign(
            static_speed_caps_.size() * kOccupancyBinCount,
            static_cast<float>(unknown_probability_));
        // processLatest owns tracker_state_mutex_ through build and publication.
        updateSignalCaps(status.speed_cap_mps, snapshot);
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
        {
            std::lock_guard<std::mutex> lock(tracker_state_mutex_);
            // A worker may have taken this snapshot immediately before receiveEgo
            // reset the predictor. Never rebuild or publish that pre-reset frame.
            if (snapshot->stamp < reset_cutoff_stamp_) return;
            auto status = buildDynamicStatus(*snapshot);
            dynamic_status_publisher_->publish(std::move(status));
        }
        ++published_snapshots_;
        const auto elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
        if (elapsed > 1000.0 / target_rate_hz_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Tracker frame took %.1f ms (budget %.1f ms); published=%zu",
                elapsed, 1000.0 / target_rate_hz_, published_snapshots_);
        }
    }

    std::string frame_id_;
    double target_rate_hz_ = 20.0;
    double unknown_probability_ = -1.0;
    double known_free_radius_m_ = 0.0;
    double uncertainty_sigma_multiplier_ = 2.0;
    double rear_axle_to_front_m_ = 3.808;
    double wheelbase_m_ = 2.944;
    StopRampParameters ramp_template_;
    std::vector<BrakingDistanceSample> braking_distance_table_;

    std::unique_ptr<hdmap::HdMap> static_map_;
    std::vector<float> static_speed_caps_;
    std::vector<Point2d> cell_centers_;
    std::vector<SignalConstraint> signal_constraints_;

    EgoSpeedEstimator ego_speed_estimator_;
    std::unique_ptr<SignalStateManager> signal_state_manager_;
    std::unique_ptr<MotionPredictor> predictor_;
    std::mutex tracker_state_mutex_;

    std::mutex snapshot_mutex_;
    std::map<std::int64_t, EgoSample> ego_messages_;
    std::map<std::int64_t, interfaces::msg::Objects::SharedPtr> object_messages_;
    std::map<std::int64_t, interfaces::msg::TrafficLight::SharedPtr> traffic_light_messages_;
    std::int64_t last_taken_stamp_ = 0;
    std::int64_t last_ego_status_stamp_ = 0;
    // Written under both mutexes; readers hold the state or snapshot mutex.
    std::int64_t reset_cutoff_stamp_ = 0;

    std::size_t published_snapshots_ = 0;

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
