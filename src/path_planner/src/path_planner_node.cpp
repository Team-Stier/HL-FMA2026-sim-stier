#include "hdmap/hdmap.hpp"
#include "interfaces/msg/dynamic_status.hpp"
#include "interfaces/msg/ego_status.hpp"
#include "interfaces/msg/search_tree.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace path_planner {

using EgoStatus = interfaces::msg::EgoStatus;
using DynamicStatus = interfaces::msg::DynamicStatus;
struct Primitive {
    std::vector<double> x_m;
    std::vector<double> y_m;
    std::vector<double> yaw_rad;
    std::vector<double> speed_mps;
    std::vector<double> eta_s;
    double lateral_cost{};
    double cost{};
};

struct PlanningSnapshot {
    EgoStatus ego;
    DynamicStatus dynamic;
    lanelet::Id current_lane{};
    bool intersection{};
    std::vector<lanelet::Id> global_path;
    std::vector<lanelet::Id> goal_lanes;
};

class PlanningRegistry {
public:
    void onEgo(const EgoStatus& message) {
        std::lock_guard lock(mutex_);
        snapshot_.ego = message;
        has_ego_ = true;
    }

    void onDynamic(const DynamicStatus& message) {
        std::lock_guard lock(mutex_);
        snapshot_.dynamic = message;
        has_dynamic_ = true;
    }

    bool ready() const {
        std::lock_guard lock(mutex_);
        return has_ego_ && has_dynamic_;
    }

    PlanningSnapshot snapshot() const {
        std::lock_guard lock(mutex_);
        return snapshot_;
    }

    void updateRoute(lanelet::Id current_lane, bool intersection,
        std::vector<lanelet::Id> global_path, std::vector<lanelet::Id> goal_lanes) {
        std::lock_guard lock(mutex_);
        snapshot_.current_lane = current_lane;
        snapshot_.intersection = intersection;
        snapshot_.global_path = std::move(global_path);
        snapshot_.goal_lanes = std::move(goal_lanes);
    }

private:
    mutable std::mutex mutex_;
    PlanningSnapshot snapshot_;
    bool has_ego_{};
    bool has_dynamic_{};
};

struct PlannerConfig {
    double route_update_hz;
    double max_path_length;
    double lateral_cost_weight;
    double time_cost_weight;
    double goal_cost_weight;
    double progress_cost_weight;
    double alignment_cost_weight;
    double checkpoint_radius_m;
    double heading_tolerance_rad;
    double alignment_duration_s;
    std::vector<double> frenet_horizon_candidates_s;
    double xy_resolution_m;
    double time_resolution_s;
    double planning_accel_limit_mps2;
    double planning_decel_limit_mps2;
    double rear_axle_to_front_m;
    double rear_axle_to_rear_m;
    double left_extent_m;
    double right_extent_m;
    double vehicle_height_m;
};

static lanelet::BasicPolygon2d footprint(double x, double y, double yaw,
    const PlannerConfig& config) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    lanelet::BasicPolygon2d polygon;
    for (const auto& corner : std::vector<std::pair<double, double>>{
             {config.rear_axle_to_front_m, config.left_extent_m},
             {config.rear_axle_to_front_m, -config.right_extent_m},
             {-config.rear_axle_to_rear_m, -config.right_extent_m},
             {-config.rear_axle_to_rear_m, config.left_extent_m}}) {
        polygon.emplace_back(x + c * corner.first - s * corner.second,
            y + s * corner.first + c * corner.second);
    }
    boost::geometry::correct(polygon);
    return polygon;
}

class IntersectionMonitor {
public:
    IntersectionMonitor(const lanelet::LaneletMap& map, const PlannerConfig& config)
        : map_(map), config_(config) {}

    std::pair<lanelet::ConstLanelet, bool> inspect(const EgoStatus& ego) const {
        const auto polygon = footprint(ego.x, ego.y, ego.heading, config_);
        const auto box = lanelet::geometry::boundingBox2d(polygon);
        double best_area = -1.;
        lanelet::ConstLanelet current;
        for (const auto& lane : map_.laneletLayer.search(box)) {
            const auto lane_polygon = lane.polygon2d().basicPolygon();
            if (!boost::geometry::overlaps(lane_polygon, polygon) &&
                !boost::geometry::within(polygon.front(), lane_polygon)) {
                continue;
            }
            std::vector<lanelet::BasicPolygon2d> intersections;
            boost::geometry::intersection(lane_polygon, polygon, intersections);
            double area = 0.;
            for (const auto& intersection : intersections) {
                area += std::abs(boost::geometry::area(intersection));
            }
            if (area > best_area) {
                best_area = area;
                current = lane;
            }
        }
        return {current, current.attribute("intersection").value() == "yes"};
    }

private:
    const lanelet::LaneletMap& map_;
    const PlannerConfig& config_;
};

struct Checkpoint { double x; double y; lanelet::ConstLanelet lane; };

class RouteUpdater {
public:
    using Publisher = rclcpp::Publisher<std_msgs::msg::Int64MultiArray>;

    RouteUpdater(const lanelet::LaneletMap& map, const lanelet::routing::RoutingGraph& graph,
        const PlannerConfig& config, const std::string& checkpoint_file,
        Publisher::SharedPtr publisher)
        : graph_(graph), config_(config), monitor_(map, config), publisher_(std::move(publisher)) {
        std::ifstream input(checkpoint_file);
        std::string line;
        std::getline(input, line);
        while (std::getline(input, line)) {
            std::istringstream row(line);
            std::string seq, x, y;
            std::getline(row, seq, ',');
            std::getline(row, x, ',');
            std::getline(row, y, ',');
            const double px = std::stod(x);
            const double py = std::stod(y);
            checkpoints_.push_back({px, py,
                lanelet::geometry::findNearest(map.laneletLayer, {px, py}, 1).front().second});
        }
    }

    void tick(const PlanningSnapshot& input) {
        const auto [current, intersection] = monitor_.inspect(input.ego);
        if (checkpoint_index_ + 1 < checkpoints_.size() &&
            std::hypot(input.ego.x - checkpoints_[checkpoint_index_].x,
                input.ego.y - checkpoints_[checkpoint_index_].y) <= config_.checkpoint_radius_m) {
            ++checkpoint_index_;
        }
        lanelet::ConstLanelets via;
        for (std::size_t index = checkpoint_index_; index + 1 < checkpoints_.size(); ++index) {
            via.push_back(checkpoints_[index].lane);
        }
        const auto path = graph_.shortestPathVia(current, via, checkpoints_.back().lane).value();
        std::vector<lanelet::Id> ids;
        ids.reserve(path.size());
        for (const auto& lane : path) {
            ids.push_back(lane.id());
        }
        const auto goals = updateGoals(input, current, intersection, ids);
        registry_->updateRoute(current.id(), intersection, ids, goals);
        std_msgs::msg::Int64MultiArray message;
        message.data = ids;
        publisher_->publish(message);
    }

    void setRegistry(PlanningRegistry& registry) { registry_ = &registry; }

private:
    std::vector<lanelet::Id> updateGoals(const PlanningSnapshot& input,
        const lanelet::ConstLanelet& current, bool intersection, const std::vector<lanelet::Id>& route) {
        if (!initialized_) {
            previous_current_ = route[0];
            previous_next_ = route[1];
            initialized_ = true;
        }
        const bool advanced = previous_next_ == route[0];
        const bool route_changed = previous_current_ != route[0] && !advanced;
        const bool next_changed = previous_current_ == route[0] && previous_next_ != route[1];
        const auto centerline = current.centerline2d();
        double best = std::numeric_limits<double>::infinity();
        double lane_heading = 0.;
        for (std::size_t index = 1; index < centerline.size(); ++index) {
            const auto a = centerline[index - 1].basicPoint();
            const auto b = centerline[index].basicPoint();
            const double distance = boost::geometry::distance(
                lanelet::BasicPoint2d(input.ego.x, input.ego.y), lanelet::BasicLineString2d{a, b});
            if (distance < best) {
                best = distance;
                lane_heading = std::atan2(b.y() - a.y(), b.x() - a.x());
            }
        }
        const double delta = std::atan2(std::sin(input.ego.heading - lane_heading),
            std::cos(input.ego.heading - lane_heading));
        if (intersection || std::abs(delta) > config_.heading_tolerance_rad || route_changed || next_changed) {
            allow_lane_change_ = false;
            alignment_time_ = 0.;
        } else if (!allow_lane_change_ && !advanced) {
            alignment_time_ += 1. / config_.route_update_hz;
            if (alignment_time_ > config_.alignment_duration_s) {
                allow_lane_change_ = true;
            }
        }
        previous_current_ = route[0];
        previous_next_ = route[1];
        if (allow_lane_change_) {
            return {route[1]};
        }
        const auto following = graph_.following(current, false);
        const auto selected = std::find_if(following.begin(), following.end(),
            [&](const auto& lane) { return lane.id() == route[1]; });
        return {route[0], (selected == following.end() ? following.front() : *selected).id()};
    }

    const lanelet::routing::RoutingGraph& graph_;
    const PlannerConfig& config_;
    IntersectionMonitor monitor_;
    PlanningRegistry* registry_{};
    std::vector<Checkpoint> checkpoints_;
    std::size_t checkpoint_index_{};
    lanelet::Id previous_current_{};
    lanelet::Id previous_next_{};
    bool allow_lane_change_{};
    bool initialized_{};
    double alignment_time_{3.};
    Publisher::SharedPtr publisher_;
};

struct Reference {
    lanelet::BasicPoints2d points;
    std::vector<double> station;

    double length() const { return station.empty() ? 0. : station.back(); }
};

struct Projection {
    double s{};
    double d{};
    double yaw{};
};

static void appendCenterline(Reference& reference, const lanelet::ConstLanelet& lane) {
    for (const auto& point : lane.centerline2d()) {
        const auto basic = point.basicPoint();
        if (!reference.points.empty() && (basic - reference.points.back()).norm() < 1e-6) continue;
        reference.points.push_back(basic);
    }
    reference.station.assign(reference.points.size(), 0.);
    for (std::size_t index = 1; index < reference.points.size(); ++index) {
        reference.station[index] = reference.station[index - 1] +
            (reference.points[index] - reference.points[index - 1]).norm();
    }
}

static Projection project(const Reference& reference, const lanelet::BasicPoint2d& point) {
    Projection result;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1; index < reference.points.size(); ++index) {
        const auto delta = reference.points[index] - reference.points[index - 1];
        const double length_squared = delta.squaredNorm();
        if (length_squared <= 0.) continue;
        const double ratio = std::clamp(
            (point - reference.points[index - 1]).dot(delta) / length_squared, 0., 1.);
        const auto nearest = reference.points[index - 1] + ratio * delta;
        const double squared = (point - nearest).squaredNorm();
        if (squared >= best) continue;
        best = squared;
        const double length = std::sqrt(length_squared);
        result.s = reference.station[index - 1] + ratio * length;
        result.d = (delta.x() * (point.y() - nearest.y()) -
            delta.y() * (point.x() - nearest.x())) / length;
        result.yaw = std::atan2(delta.y(), delta.x());
    }
    return result;
}

struct ReferenceSample {
    lanelet::BasicPoint2d point;
    double yaw{};
};

static ReferenceSample sample(const Reference& reference, double station) {
    station = std::clamp(station, 0., reference.length());
    auto upper = std::upper_bound(reference.station.begin(), reference.station.end(), station);
    std::size_t index = static_cast<std::size_t>(std::distance(reference.station.begin(), upper));
    index = std::clamp<std::size_t>(index, 1, reference.points.size() - 1);
    const double span = reference.station[index] - reference.station[index - 1];
    const double ratio = span > 0. ? (station - reference.station[index - 1]) / span : 0.;
    const auto delta = reference.points[index] - reference.points[index - 1];
    return {reference.points[index - 1] + ratio * delta,
        std::atan2(delta.y(), delta.x())};
}

class ReferenceBuilder {
public:
    ReferenceBuilder(const lanelet::LaneletMap& map,
        const lanelet::routing::RoutingGraph& graph) : map_(map), graph_(graph) {}

    std::vector<Reference> build(lanelet::Id current_id,
        const std::vector<lanelet::Id>& global_path) const {
        const auto current = map_.laneletLayer.get(current_id);
        Reference current_reference;
        appendCenterline(current_reference, current);
        const auto following = graph_.following(current, false);
        if (!following.empty()) {
            auto selected = following.begin();
            if (global_path.size() > 1) {
                const auto match = std::find_if(following.begin(), following.end(), [&](const auto& lane) {
                    return lane.id() == global_path[1];
                });
                if (match != following.end()) selected = match;
            }
            appendCenterline(current_reference, *selected);
        }
        std::vector<Reference> references;
        if (current_reference.points.size() > 1) references.push_back(std::move(current_reference));
        for (const auto& adjacent : {graph_.left(current), graph_.right(current)}) {
            if (!adjacent) continue;
            Reference reference;
            appendCenterline(reference, *adjacent);
            if (reference.points.size() > 1) references.push_back(std::move(reference));
        }
        return references;
    }

private:
    const lanelet::LaneletMap& map_;
    const lanelet::routing::RoutingGraph& graph_;
};

struct Quartic {
    double a0{}, a1{}, a2{}, a3{}, a4{};
    double position(double t) const { return a0 + t * (a1 + t * (a2 + t * (a3 + t * a4))); }
};

static Quartic quartic(double position, double speed, double acceleration,
    double terminal_speed, double terminal_acceleration, double duration) {
    Quartic result{position, speed, acceleration / 2., 0., 0.};
    const double b0 = terminal_speed - result.a1 - 2. * result.a2 * duration;
    const double b1 = terminal_acceleration - 2. * result.a2;
    const double determinant = 12. * std::pow(duration, 4);
    result.a3 = (12. * duration * duration * b0 - 4. * std::pow(duration, 3) * b1) /
        determinant;
    result.a4 = (-6. * duration * b0 + 3. * duration * duration * b1) / determinant;
    return result;
}

struct Quintic {
    double a0{}, a1{}, a2{}, a3{}, a4{}, a5{};
    double position(double t) const {
        return a0 + t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
    }
    double acceleration(double t) const {
        return 2. * a2 + t * (6. * a3 + t * (12. * a4 + t * 20. * a5));
    }
    double jerk(double t) const { return 6. * a3 + t * (24. * a4 + t * 60. * a5); }
};

static Quintic quintic(double position, double speed, double acceleration,
    double terminal_position, double terminal_speed, double terminal_acceleration,
    double duration) {
    const double delta = terminal_position - position;
    return {position, speed, acceleration / 2.,
        (20. * delta - (8. * terminal_speed + 12. * speed) * duration -
            (3. * acceleration - terminal_acceleration) * duration * duration) /
            (2. * std::pow(duration, 3)),
        (-30. * delta + (14. * terminal_speed + 16. * speed) * duration +
            (3. * acceleration - 2. * terminal_acceleration) * duration * duration) /
            (2. * std::pow(duration, 4)),
        (12. * delta - (6. * terminal_speed + 6. * speed) * duration -
            (acceleration - terminal_acceleration) * duration * duration) /
            (2. * std::pow(duration, 5))};
}

static double pathLength(const Primitive& primitive) {
    double result = 0.;
    for (std::size_t index = 1; index < primitive.x_m.size(); ++index) {
        result += std::hypot(primitive.x_m[index] - primitive.x_m[index - 1],
            primitive.y_m[index] - primitive.y_m[index - 1]);
    }
    return result;
}

static void appendWaypoint(Primitive& primitive, double x, double y, double resolution) {
    if (primitive.x_m.empty()) {
        primitive.x_m.push_back(x);
        primitive.y_m.push_back(y);
        return;
    }
    const double dx = x - primitive.x_m.back();
    const double dy = y - primitive.y_m.back();
    const auto steps = std::max<std::size_t>(1,
        static_cast<std::size_t>(std::ceil(std::hypot(dx, dy) / resolution)));
    const double start_x = primitive.x_m.back();
    const double start_y = primitive.y_m.back();
    for (std::size_t step = 1; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        primitive.x_m.push_back(start_x + ratio * dx);
        primitive.y_m.push_back(start_y + ratio * dy);
    }
}

class VelocityPlanner {
public:
    VelocityPlanner(const hdmap::HdMap& map, const PlannerConfig& config)
        : map_(map), config_(config) {}

    bool plan(Primitive& primitive, const PlanningSnapshot& snapshot, bool stop_at_end = false) const {
        const auto count = primitive.x_m.size();
        if (count < 2) return false;
        std::vector<double> distance(count, 0.);
        primitive.speed_mps.assign(count, std::numeric_limits<double>::infinity());
        primitive.eta_s.assign(count, 0.);
        for (std::size_t index = 0; index < count; ++index) {
            if (index) distance[index] = std::hypot(
                primitive.x_m[index] - primitive.x_m[index - 1],
                primitive.y_m[index] - primitive.y_m[index - 1]);
            const auto cells = map_.cellTree().queryOverlaps(
                footprint(primitive.x_m[index], primitive.y_m[index], primitive.yaw_rad[index], config_),
                snapshot.ego.z, snapshot.ego.z + config_.vehicle_height_m);
            for (const auto cell : cells) {
                if (cell < snapshot.dynamic.speed_cap_mps.size()) {
                    primitive.speed_mps[index] = std::min(primitive.speed_mps[index],
                        static_cast<double>(snapshot.dynamic.speed_cap_mps[cell]));
                }
            }
        }
        primitive.speed_mps.front() = snapshot.ego.speed;
        for (std::size_t index = 1; index < count; ++index) {
            const double reachable = std::sqrt(std::max(0.,
                primitive.speed_mps[index - 1] * primitive.speed_mps[index - 1] +
                2. * config_.planning_accel_limit_mps2 * distance[index]));
            primitive.speed_mps[index] = std::min(primitive.speed_mps[index], reachable);
        }
        if (stop_at_end) primitive.speed_mps.back() = 0.;
        for (std::size_t index = count - 1; index > 0; --index) {
            const double reachable = std::sqrt(std::max(0.,
                primitive.speed_mps[index] * primitive.speed_mps[index] +
                2. * config_.planning_decel_limit_mps2 * distance[index]));
            primitive.speed_mps[index - 1] = std::min(primitive.speed_mps[index - 1], reachable);
        }
        if (primitive.speed_mps.front() + 1e-6 < snapshot.ego.speed) return false;
        primitive.speed_mps.front() = snapshot.ego.speed;
        for (std::size_t index = 1; index < count; ++index) {
            const double average = .5 * (primitive.speed_mps[index - 1] + primitive.speed_mps[index]);
            if (!std::isfinite(average) || average <= 0.) return false;
            primitive.eta_s[index] = primitive.eta_s[index - 1] + distance[index] / average;
        }
        return true;
    }

private:
    const hdmap::HdMap& map_;
    const PlannerConfig& config_;
};

class CollisionValidator {
public:
    CollisionValidator(const hdmap::HdMap& map, const PlannerConfig& config)
        : map_(map), config_(config) {}

    std::size_t firstCollision(const Primitive& primitive,
        const PlanningSnapshot& snapshot) const {
        const double snapshot_time = rclcpp::Time(snapshot.ego.header.stamp).seconds();
        const double dynamic_time = rclcpp::Time(snapshot.dynamic.header.stamp).seconds();
        for (std::size_t index = 0; index < primitive.x_m.size(); ++index) {
            const auto cells = map_.cellTree().queryOverlaps(
                footprint(primitive.x_m[index], primitive.y_m[index], primitive.yaw_rad[index], config_),
                snapshot.ego.z, snapshot.ego.z + config_.vehicle_height_m);
            const auto bin = static_cast<std::size_t>(std::clamp(
                std::ceil((snapshot_time + primitive.eta_s[index] - dynamic_time) / .5), 0., 12.));
            for (const auto cell : cells) {
                const auto offset = cell * 13 + bin;
                if (offset < snapshot.dynamic.occupancy_probability.size() &&
                    snapshot.dynamic.occupancy_probability[offset] > 0.F) return index;
            }
        }
        return primitive.x_m.size();
    }

private:
    const hdmap::HdMap& map_;
    const PlannerConfig& config_;
};

struct GoalPoint {
    lanelet::BasicPoint2d point;
    double yaw{};
};

static std::vector<GoalPoint> goalPoints(const lanelet::LaneletMap& map,
    const std::vector<lanelet::Id>& goals) {
    std::vector<GoalPoint> result;
    std::set<lanelet::Id> unique(goals.begin(), goals.end());
    for (const auto id : unique) {
        const auto centerline = map.laneletLayer.get(id).centerline2d();
        if (centerline.size() < 2) continue;
        for (std::size_t index = 0; index < centerline.size(); ++index) {
            const auto delta = index + 1 < centerline.size()
                ? centerline[index + 1].basicPoint() - centerline[index].basicPoint()
                : centerline[index].basicPoint() - centerline[index - 1].basicPoint();
            result.push_back({centerline[index].basicPoint(), std::atan2(delta.y(), delta.x())});
        }
    }
    return result;
}

static double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

class CostEvaluator {
public:
    explicit CostEvaluator(const PlannerConfig& config) : config_(config) {}

    double evaluate(Primitive& primitive, const std::vector<GoalPoint>& goals) const {
        double goal_distance = std::numeric_limits<double>::infinity();
        double goal_yaw = primitive.yaw_rad.back();
        const lanelet::BasicPoint2d end{primitive.x_m.back(), primitive.y_m.back()};
        for (const auto& goal : goals) {
            const double squared = (end - goal.point).squaredNorm();
            if (squared < goal_distance) {
                goal_distance = squared;
                goal_yaw = goal.yaw;
            }
        }
        const double length = pathLength(primitive);
        const double progress = config_.max_path_length - std::min(config_.max_path_length, length);
        primitive.cost = config_.lateral_cost_weight * primitive.lateral_cost +
            config_.time_cost_weight * primitive.eta_s.back() +
            config_.goal_cost_weight * goal_distance +
            config_.progress_cost_weight * progress * progress +
            config_.alignment_cost_weight * std::pow(wrap(primitive.yaw_rad.back() - goal_yaw), 2);
        return primitive.cost;
    }

private:
    const PlannerConfig& config_;
};

static std::pair<double, double> toBaseLink(const EgoStatus& ego, double x, double y) {
    const double dx = x - ego.x;
    const double dy = y - ego.y;
    const double c = std::cos(ego.heading);
    const double s = std::sin(ego.heading);
    return {c * dx + s * dy, -s * dx + c * dy};
}

class PathBuilder {
public:
    nav_msgs::msg::Path build(const Primitive& primitive, const EgoStatus& ego) const {
        nav_msgs::msg::Path path;
        path.header = ego.header;
        path.header.frame_id = "base_link";
        for (std::size_t index = 0; index < primitive.x_m.size(); ++index) {
            const auto [x, y] = toBaseLink(ego, primitive.x_m[index], primitive.y_m[index]);
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            const double yaw = wrap(primitive.yaw_rad[index] - ego.heading);
            pose.pose.orientation.z = std::sin(yaw / 2.);
            pose.pose.orientation.w = std::cos(yaw / 2.);
            path.poses.push_back(std::move(pose));
        }
        return path;
    }
};

class SearchTreeBuilder {
public:
    interfaces::msg::SearchTree build(const std::vector<Primitive>& candidates,
        std::size_t selected, const EgoStatus& ego) const {
        interfaces::msg::SearchTree tree;
        tree.header = ego.header;
        tree.header.frame_id = "base_link";
        for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
            std::int32_t parent = -1;
            for (std::size_t index = 0; index < candidates[candidate_index].x_m.size(); ++index) {
                const auto [x, y] = toBaseLink(ego, candidates[candidate_index].x_m[index],
                    candidates[candidate_index].y_m[index]);
                tree.x.push_back(static_cast<float>(x));
                tree.y.push_back(static_cast<float>(y));
                tree.yaw.push_back(static_cast<float>(wrap(
                    candidates[candidate_index].yaw_rad[index] - ego.heading)));
                tree.parent_index.push_back(parent);
                parent = static_cast<std::int32_t>(tree.x.size() - 1);
            }
            if (candidate_index == selected) tree.final_node_index = parent;
        }
        return tree;
    }
};

class FrenetPlanner {
public:
    using PathPublisher = rclcpp::Publisher<nav_msgs::msg::Path>;
    using TreePublisher = rclcpp::Publisher<interfaces::msg::SearchTree>;

    FrenetPlanner(const hdmap::HdMap& map, const lanelet::routing::RoutingGraph& graph,
        const PlannerConfig& config, PathPublisher::SharedPtr path_publisher,
        TreePublisher::SharedPtr tree_publisher)
        : map_(map), config_(config), references_(map.laneletMap(), graph),
          velocity_(map, config), validator_(map, config), costs_(config),
          path_publisher_(std::move(path_publisher)), tree_publisher_(std::move(tree_publisher)) {}

    void tick(const PlanningSnapshot& snapshot) {
        if (snapshot.current_lane == 0 || snapshot.goal_lanes.empty()) return;
        auto candidates = generate(snapshot);
        std::vector<Primitive> valid;
        for (auto& candidate : candidates) {
            if (!velocity_.plan(candidate, snapshot)) continue;
            const auto collision = validator_.firstCollision(candidate, snapshot);
            if (collision < candidate.x_m.size()) {
                if (collision < 2) continue;
                resize(candidate, collision);
                if (!velocity_.plan(candidate, snapshot, true) ||
                    validator_.firstCollision(candidate, snapshot) < candidate.x_m.size()) continue;
            }
            valid.push_back(std::move(candidate));
        }
        if (valid.empty()) return;
        const auto goals = goalPoints(map_.laneletMap(), snapshot.goal_lanes);
        if (goals.empty()) return;
        std::size_t selected = 0;
        for (std::size_t index = 0; index < valid.size(); ++index) {
            costs_.evaluate(valid[index], goals);
            if (valid[index].cost < valid[selected].cost) selected = index;
        }
        path_publisher_->publish(path_builder_.build(valid[selected], snapshot.ego));
        tree_publisher_->publish(tree_builder_.build(valid, selected, snapshot.ego));
    }

private:
    std::vector<Primitive> generate(const PlanningSnapshot& snapshot) const {
        std::vector<Primitive> result;
        for (const auto& reference : references_.build(snapshot.current_lane, snapshot.global_path)) {
            const auto initial = project(reference, {snapshot.ego.x, snapshot.ego.y});
            const double heading_error = wrap(snapshot.ego.heading - initial.yaw);
            const double longitudinal_speed = std::max(0., snapshot.ego.speed * std::cos(heading_error));
            const double lateral_speed = snapshot.ego.speed * std::sin(heading_error);
            for (const double duration : config_.frenet_horizon_candidates_s) {
                const double minimum_speed = std::max(0., longitudinal_speed -
                    config_.planning_decel_limit_mps2 * duration);
                const double maximum_speed = longitudinal_speed +
                    config_.planning_accel_limit_mps2 * duration;
                const std::vector<double> terminal_speeds{
                    minimum_speed, std::clamp(longitudinal_speed, minimum_speed, maximum_speed), maximum_speed};
                for (std::size_t speed_index = 0; speed_index < terminal_speeds.size(); ++speed_index) {
                    if (speed_index && std::abs(terminal_speeds[speed_index] -
                        terminal_speeds[speed_index - 1]) < 1e-6) continue;
                    const auto longitudinal = quartic(initial.s, longitudinal_speed, 0.,
                        terminal_speeds[speed_index], 0., duration);
                    const auto lateral = quintic(initial.d, lateral_speed, 0., 0., 0., 0., duration);
                    Primitive primitive;
                    double previous_t = 0.;
                    for (double nominal_time = 0.;;) {
                        const double station = longitudinal.position(nominal_time);
                        if (station < 0.) break;
                        const bool reference_end = station >= reference.length();
                        const auto center = sample(reference, std::min(station, reference.length()));
                        const double offset = lateral.position(nominal_time);
                        appendWaypoint(primitive,
                            center.point.x() - std::sin(center.yaw) * offset,
                            center.point.y() + std::cos(center.yaw) * offset,
                            config_.xy_resolution_m);
                        primitive.lateral_cost += (std::pow(lateral.acceleration(nominal_time), 2) +
                            std::pow(lateral.jerk(nominal_time), 2)) * (nominal_time - previous_t);
                        previous_t = nominal_time;
                        if (reference_end || pathLength(primitive) >= config_.max_path_length ||
                            nominal_time >= duration) break;
                        nominal_time = std::min(duration,
                            nominal_time + config_.time_resolution_s);
                    }
                    if (primitive.x_m.size() < 2) continue;
                    primitive.yaw_rad.resize(primitive.x_m.size());
                    for (std::size_t index = 1; index < primitive.x_m.size(); ++index) {
                        primitive.yaw_rad[index - 1] = std::atan2(
                            primitive.y_m[index] - primitive.y_m[index - 1],
                            primitive.x_m[index] - primitive.x_m[index - 1]);
                    }
                    primitive.yaw_rad.back() = primitive.yaw_rad[primitive.yaw_rad.size() - 2];
                    result.push_back(std::move(primitive));
                }
            }
        }
        return result;
    }

    static void resize(Primitive& primitive, std::size_t count) {
        primitive.x_m.resize(count);
        primitive.y_m.resize(count);
        primitive.yaw_rad.resize(count);
        primitive.speed_mps.resize(count);
        primitive.eta_s.resize(count);
    }

    const hdmap::HdMap& map_;
    const PlannerConfig& config_;
    ReferenceBuilder references_;
    VelocityPlanner velocity_;
    CollisionValidator validator_;
    CostEvaluator costs_;
    PathBuilder path_builder_;
    SearchTreeBuilder tree_builder_;
    PathPublisher::SharedPtr path_publisher_;
    TreePublisher::SharedPtr tree_publisher_;
};

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner") {
        constexpr double pi = 3.14159265358979323846;
        const double route_hz = declare_parameter<double>("route_update_hz", 10.);
        const double planner_hz = declare_parameter<double>("frenet_planner_hz", 20.);
        config_ = {
            route_hz,
            declare_parameter<double>("max_path_length", 50.),
            declare_parameter<double>("lateral_cost_weight", 1.),
            declare_parameter<double>("time_cost_weight", 1.),
            declare_parameter<double>("goal_cost_weight", 10.),
            declare_parameter<double>("progress_cost_weight", 1.),
            declare_parameter<double>("alignment_cost_weight", 10.),
            declare_parameter<double>("checkpoint_radius_m", 2.),
            declare_parameter<double>("heading_tolerance_deg", 10.) * pi / 180.,
            declare_parameter<double>("alignment_duration_s", 3.),
            declare_parameter<std::vector<double>>("frenet_horizon_candidates_s", {2., 3., 4.}),
            declare_parameter<double>("xy_resolution_m", .5),
            declare_parameter<double>("time_resolution_s", .5),
            declare_parameter<double>("planning_accel_limit_mps2", 5.45),
            declare_parameter<double>("planning_decel_limit_mps2", 10.37),
            declare_parameter<double>("rear_axle_to_front_m", 3.808),
            declare_parameter<double>("rear_axle_to_rear_m", 1.040),
            declare_parameter<double>("left_extent_m", .943),
            declare_parameter<double>("right_extent_m", .943),
            declare_parameter<double>("vehicle_height_m", 1.507)};
        for (const double weight : {config_.lateral_cost_weight, config_.time_cost_weight,
                 config_.goal_cost_weight, config_.progress_cost_weight,
                 config_.alignment_cost_weight}) {
            if (!std::isfinite(weight) || weight < 0.) {
                throw std::invalid_argument("Planner cost weights must be finite and nonnegative");
            }
        }
        if (config_.frenet_horizon_candidates_s.empty() || config_.xy_resolution_m <= 0. ||
            config_.time_resolution_s <= 0. || config_.max_path_length <= 0. ||
            config_.planning_accel_limit_mps2 <= 0. || config_.planning_decel_limit_mps2 <= 0.) {
            throw std::invalid_argument("Frenet horizons and planner resolutions must be positive");
        }
        for (const double horizon : config_.frenet_horizon_candidates_s) {
            if (!std::isfinite(horizon) || horizon <= 0.) {
                throw std::invalid_argument("Frenet horizons must be finite and positive");
            }
        }
        map_ = hdmap::hdmap_init(declare_parameter<std::string>("map_path", ""));
        rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
            lanelet::Locations::Germany, lanelet::Participants::Vehicle);
        graph_ = lanelet::routing::RoutingGraph::build(map_->laneletMap(), *rules_);
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        ego_subscription_ = create_subscription<EgoStatus>("/ego_status", qos,
            [this](const EgoStatus& message) { registry_.onEgo(message); });
        dynamic_subscription_ = create_subscription<DynamicStatus>("/dynamic_status", qos,
            [this](const DynamicStatus& message) { registry_.onDynamic(message); });
        global_publisher_ = create_publisher<std_msgs::msg::Int64MultiArray>("/global_path", qos);
        local_publisher_ = create_publisher<nav_msgs::msg::Path>("/local_path", qos);
        tree_publisher_ = create_publisher<interfaces::msg::SearchTree>("/search_tree", qos);
        route_updater_ = std::make_unique<RouteUpdater>(map_->laneletMap(), *graph_, config_,
            declare_parameter<std::string>("checkpoint_file", ""), global_publisher_);
        route_updater_->setRegistry(registry_);
        planner_ = std::make_unique<FrenetPlanner>(
            *map_, *graph_, config_, local_publisher_, tree_publisher_);

        route_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1. / route_hz)), [this] {
                if (registry_.ready()) route_updater_->tick(registry_.snapshot());
            });
        planner_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1. / planner_hz)), [this] {
                if (registry_.ready()) planner_->tick(registry_.snapshot());
            });
    }

private:
    PlannerConfig config_{};
    PlanningRegistry registry_;
    std::unique_ptr<hdmap::HdMap> map_;
    lanelet::traffic_rules::TrafficRulesPtr rules_;
    lanelet::routing::RoutingGraphPtr graph_;
    std::unique_ptr<RouteUpdater> route_updater_;
    std::unique_ptr<FrenetPlanner> planner_;
    rclcpp::Subscription<EgoStatus>::SharedPtr ego_subscription_;
    rclcpp::Subscription<DynamicStatus>::SharedPtr dynamic_subscription_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr global_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_publisher_;
    rclcpp::Publisher<interfaces::msg::SearchTree>::SharedPtr tree_publisher_;
    rclcpp::TimerBase::SharedPtr route_timer_;
    rclcpp::TimerBase::SharedPtr planner_timer_;
};

}  // namespace path_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planner::PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
