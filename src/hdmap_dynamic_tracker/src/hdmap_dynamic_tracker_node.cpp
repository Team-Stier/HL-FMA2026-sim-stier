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
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <limits>
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
constexpr double kUnlimitedZ = std::numeric_limits<double>::max();

template<typename Message>
double stampSeconds(const Message& message) {
    return static_cast<double>(message.header.stamp.sec) +
        static_cast<double>(message.header.stamp.nanosec) * 1.0e-9;
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
    interfaces::msg::EgoPose::SharedPtr ego;
    interfaces::msg::Objects::SharedPtr objects;
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
        restrict_unobserved_signals_ =
            declare_parameter<bool>("signals.restrict_unobserved", true);
        signal_calibration_verified_ =
            declare_parameter<bool>("signals.calibration_verified", false);
        maximum_approach_speed_mps_ =
            declare_parameter<double>("signals.maximum_approach_speed_mps", 8.2);
        const auto braking_speeds = declare_parameter<std::vector<double>>(
            "signals.braking_calibration.speed_mps", std::vector<double>{});
        const auto braking_distances = declare_parameter<std::vector<double>>(
            "signals.braking_calibration.distance_m", std::vector<double>{});
        const auto braking_sample_count =
            std::min(braking_speeds.size(), braking_distances.size());
        braking_distance_table_.reserve(braking_sample_count);
        for (std::size_t index = 0; index < braking_sample_count; ++index) {
            braking_distance_table_.push_back({braking_speeds[index], braking_distances[index]});
        }
        rear_axle_to_front_m_ = declare_parameter<double>("vehicle.rear_axle_to_front_m", 3.808);

        PredictorConfig predictor_config;
        predictor_config.process_acceleration_sigma_mps2 =
            declare_parameter<double>("prediction.process_acceleration_sigma_mps2", 2.0);
        predictor_config.process_jerk_sigma_mps3 =
            declare_parameter<double>("prediction.process_jerk_sigma_mps3", 2.0);
        predictor_config.process_turn_acceleration_sigma_radps2 = declare_parameter<double>(
            "prediction.process_turn_acceleration_sigma_radps2", 0.6);
        predictor_config.position_measurement_sigma_m =
            declare_parameter<double>("prediction.position_measurement_sigma_m", 0.5);
        predictor_config.speed_measurement_sigma_mps =
            declare_parameter<double>("prediction.speed_measurement_sigma_mps", 1.0);
        predictor_config.course_measurement_sigma_rad =
            declare_parameter<double>("prediction.course_measurement_sigma_rad", 0.20);
        predictor_config.initial_velocity_sigma_mps =
            declare_parameter<double>("prediction.initial_velocity_sigma_mps", 5.0);
        predictor_config.initial_acceleration_sigma_mps2 = declare_parameter<double>(
            "prediction.initial_acceleration_sigma_mps2", 2.0);
        predictor_config.initial_turn_rate_sigma_radps = declare_parameter<double>(
            "prediction.initial_turn_rate_sigma_radps", 0.5);
        predictor_config.track_retention_s =
            declare_parameter<double>("prediction.track_retention_s", 0.5);
        predictor_config.minimum_frame_dt_s =
            declare_parameter<double>("prediction.minimum_frame_dt_s", 1.0e-4);
        predictor_config.maximum_frame_dt_s =
            get_parameter("ego.maximum_dt_s").as_double();
        predictor_config.maximum_prediction_step_s =
            declare_parameter<double>("prediction.maximum_prediction_step_s", 0.05);
        const auto sweep_substeps_per_bin = declare_parameter<std::int64_t>(
            "prediction.sweep_substeps_per_bin", 5);
        sweep_substeps_per_bin_ = static_cast<std::size_t>(
            std::max<std::int64_t>(1, sweep_substeps_per_bin));
        predictor_ = makeEkfPredictor(predictor_config);

        ramp_template_.design_deceleration_mps2 =
            declare_parameter<double>("signals.design_deceleration_mps2", 2.0);
        ramp_template_.braking_distance_factor =
            declare_parameter<double>("signals.braking_distance_factor", 1.0);
        ramp_template_.latency_budget_s =
            declare_parameter<double>("signals.latency_budget_s", 0.25);
        ramp_template_.stop_margin_m =
            declare_parameter<double>("signals.stop_margin_m", 1.0);

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
            !std::isfinite(rear_axle_to_front_m_) || rear_axle_to_front_m_ < 0.0 ||
            !std::isfinite(maximum_approach_speed_mps_) || maximum_approach_speed_mps_ <= 0.0) {
            throw std::invalid_argument("Invalid occupancy or vehicle parameter");
        }
        if ((known_free_radius_m_ > 0.0 || allow_free_when_object_list_full_) &&
            !free_space_assumption_verified_) {
            throw std::invalid_argument(
                "Free-space assertions require occupancy.free_space_assumption_verified=true");
        }
        StopRampParameters validation_ramp = ramp_template_;
        validation_ramp.entry_speed_mps = maximum_approach_speed_mps_;
        if (!validStopRamp(validation_ramp) ||
            !std::isfinite(stopRampStartDistance(validation_ramp) + rear_axle_to_front_m_)) {
            throw std::invalid_argument("Invalid signal stop-ramp parameters");
        }
        if (!braking_distance_table_.empty() &&
            !validBrakingDistanceTable(braking_distance_table_)) {
            throw std::invalid_argument(
                "Signal braking calibration speeds must increase and distances must be positive and nondecreasing");
        }
        if (signal_calibration_verified_ && braking_distance_table_.empty()) {
            throw std::invalid_argument(
                "Verified signal braking calibration requires a nonempty braking-distance table");
        }
        if (signal_calibration_verified_ && !braking_distance_table_.empty() &&
            braking_distance_table_.back().speed_mps < maximum_approach_speed_mps_) {
            throw std::invalid_argument(
                "Verified braking-distance table must cover maximum_approach_speed_mps");
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
        std::size_t braking_table_fallbacks = 0;
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
                    ramp.entry_speed_mps = maximum_approach_speed_mps_;
                    if (!braking_distance_table_.empty()) {
                        ramp.calibrated_braking_distance_m = conservativeBrakingDistance(
                            ramp.entry_speed_mps, braking_distance_table_);
                        if (!ramp.calibrated_braking_distance_m) {
                            if (signal_calibration_verified_) {
                                throw std::invalid_argument(
                                    "Verified braking-distance table does not cover a signal approach speed");
                            }
                            ++braking_table_fallbacks;
                        }
                    }
                    if (!validStopRamp(ramp)) {
                        throw std::invalid_argument(
                            "Signal braking calibration produced an invalid stop ramp");
                    }
                    const double traversal_limit = stopRampStartDistance(ramp) + rear_axle_to_front_m_;
                    const hdmap::Cell* cell = &static_map_->cells().at(stop_cell_id);
                    double distance_from_stop_edge = 0.0;
                    std::unordered_set<hdmap::CellId> visited;
                    while (cell != nullptr && distance_from_stop_edge <= traversal_limit &&
                        visited.insert(cell->id()).second) {
                        const double front_bumper_clearance =
                            std::max(0.0, distance_from_stop_edge - rear_axle_to_front_m_);
                        const float cap = static_cast<float>(
                            stopRampCap(front_bumper_clearance, ramp));
                        const auto [iterator, inserted] = minimum_caps.emplace(cell->id(), cap);
                        if (!inserted) {
                            iterator->second = std::min(iterator->second, cap);
                        }
                        distance_from_stop_edge += cellLength(*cell);
                        cell = cell->previous();
                    }
                    if (distance_from_stop_edge <= traversal_limit) {
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
        if (braking_table_fallbacks > 0) {
            RCLCPP_WARN(get_logger(),
                "%zu stopline ramps exceed the unverified braking table; theoretical design-deceleration fallback was used",
                braking_table_fallbacks);
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

    void receiveEgo(interfaces::msg::EgoPose::SharedPtr message) {
        const auto speed = ego_speed_estimator_.update(
            stampSeconds(*message), message->x, message->y);

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
        latest_ego_ = std::move(message);
        markSnapshotUpdatedLocked();
    }

    void receiveObjects(interfaces::msg::Objects::SharedPtr message) {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        latest_objects_ = std::move(message);
        markSnapshotUpdatedLocked();
    }

    void receiveTrafficLight(interfaces::msg::TrafficLight::SharedPtr) {}

    void markSnapshotUpdatedLocked() {
        if (latest_ego_ && latest_objects_) {
            ++newest_complete_revision_;
        }
    }

    std::optional<Snapshot> takeLatestCompleteSnapshot() {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        if (!latest_ego_ || !latest_objects_) {
            return std::nullopt;
        }
        if (newest_complete_revision_ == last_taken_revision_) {
            return std::nullopt;
        }
        last_taken_revision_ = newest_complete_revision_;
        return Snapshot{latest_ego_, latest_objects_};
    }

    std::vector<ObjectObservation> makeObservations(const interfaces::msg::Objects& message) const {
        std::vector<ObjectObservation> observations;
        const auto length = std::clamp(message.length, 0, 30);
        observations.reserve(static_cast<std::size_t>(length));
        for (std::int32_t index = 0; index < length; ++index) {
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

        const std::size_t future_sample_count =
            (kOccupancyBinCount - 1U) * sweep_substeps_per_bin_ + 1U;
        const double future_sample_interval_s =
            kPredictionIntervalS / static_cast<double>(sweep_substeps_per_bin_);
        std::vector<double> future_times_s;
        future_times_s.reserve(future_sample_count);
        for (std::size_t index = 0; index < future_sample_count; ++index) {
            future_times_s.push_back(
                static_cast<double>(index) * future_sample_interval_s);
        }
        const auto prediction_sequence =
            predictor_->predictSequence(stamp_s, future_times_s);
        const auto& current = prediction_sequence.front();
        for (const auto& object : current) {
            if (observed_ids.count(object.id) != 0U) {
                continue;
            }
            markFootprint(occupancy, 0, orientedBox(object),
                -kUnlimitedZ, kUnlimitedZ);
        }
        std::unordered_map<std::uint32_t, const ObjectObservation*> observed_by_id;
        observed_by_id.reserve(observations.size());
        for (const auto& observation : observations) {
            observed_by_id.emplace(observation.id, &observation);
        }
        for (std::size_t bin = 1; bin < kOccupancyBinCount; ++bin) {
            std::unordered_map<std::uint32_t, std::vector<ObjectPrediction>> samples_by_id;
            samples_by_id.reserve(current.size());
            for (std::size_t step = 0; step <= sweep_substeps_per_bin_; ++step) {
                const std::size_t sample_index =
                    (bin - 1U) * sweep_substeps_per_bin_ + step;
                for (auto prediction : prediction_sequence.at(sample_index)) {
                    if (bin == 1 && step == 0U) {
                        const auto observation = observed_by_id.find(prediction.id);
                        if (observation != observed_by_id.end()) {
                            prediction.x = observation->second->x;
                            prediction.y = observation->second->y;
                            prediction.min_z = observation->second->min_z;
                            prediction.heading = observation->second->heading;
                            prediction.length = observation->second->length;
                            prediction.width = observation->second->width;
                            prediction.height = observation->second->height;
                        }
                    }
                    samples_by_id[prediction.id].push_back(prediction);
                }
            }
            for (auto& entry : samples_by_id) {
                auto& samples = entry.second;
                if (samples.empty()) {
                    continue;
                }
                markFootprint(occupancy, bin,
                    sweptFootprint(samples),
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
        dynamic_status_publisher_->publish(std::move(status));
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
    bool free_space_assumption_verified_ = false;
    bool allow_free_when_object_list_full_ = false;
    std::size_t sweep_substeps_per_bin_ = 5U;
    bool restrict_unobserved_signals_ = true;
    bool signal_calibration_verified_ = false;
    double maximum_approach_speed_mps_ = 8.2;
    double rear_axle_to_front_m_ = 3.808;
    StopRampParameters ramp_template_;
    std::vector<BrakingDistanceSample> braking_distance_table_;

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
    interfaces::msg::EgoPose::SharedPtr latest_ego_;
    interfaces::msg::Objects::SharedPtr latest_objects_;
    std::uint64_t last_taken_revision_ = 0;
    std::uint64_t newest_complete_revision_ = 0;

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
