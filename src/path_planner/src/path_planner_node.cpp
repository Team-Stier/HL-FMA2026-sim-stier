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
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace path_planner {

using EgoStatus = interfaces::msg::EgoStatus;
using DynamicStatus = interfaces::msg::DynamicStatus;
using PrimitivePtr = std::shared_ptr<struct Primitive>;

struct Primitive {
    std::vector<double> x_m;
    std::vector<double> y_m;
    std::vector<double> yaw_rad;
    std::vector<double> speed_mps;
    std::vector<double> eta_s;
    double g{};
    double h{};
    std::shared_ptr<const Primitive> parent;
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
    std::size_t max_node_count;
    double g_weight;
    double h_weight;
    double primitive_length_m;
    double checkpoint_radius_m;
    double heading_tolerance_rad;
    double alignment_duration_s;
    std::vector<double> steering_candidates_rad;
    double xy_resolution_m;
    double yaw_resolution_rad;
    double time_resolution_s;
    double speed_resolution_mps;
    double acceleration_mps2;
    double deceleration_mps2;
    double wheelbase_m;
    double rear_axle_to_front_m;
    double rear_axle_to_rear_m;
    double left_extent_m;
    double right_extent_m;
    double vehicle_height_m;
};

static lanelet::BasicPoint2d mapPoint(const EgoStatus& ego, double x, double y) {
    const double c = std::cos(ego.heading);
    const double s = std::sin(ego.heading);
    return {ego.x + c * x - s * y, ego.y + s * x + c * y};
}

static lanelet::BasicPolygon2d footprint(const EgoStatus& ego, double x, double y, double yaw,
    const PlannerConfig& config) {
    const double heading = ego.heading + yaw;
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    const auto center = mapPoint(ego, x, y);
    lanelet::BasicPolygon2d polygon;
    for (const auto& corner : std::vector<std::pair<double, double>>{
             {config.rear_axle_to_front_m, config.left_extent_m},
             {config.rear_axle_to_front_m, -config.right_extent_m},
             {-config.rear_axle_to_rear_m, -config.right_extent_m},
             {-config.rear_axle_to_rear_m, config.left_extent_m}}) {
        polygon.emplace_back(center.x() + c * corner.first - s * corner.second,
            center.y() + s * corner.first + c * corner.second);
    }
    boost::geometry::correct(polygon);
    return polygon;
}

class IntersectionMonitor {
public:
    IntersectionMonitor(const lanelet::LaneletMap& map, const PlannerConfig& config)
        : map_(map), config_(config) {}

    std::pair<lanelet::ConstLanelet, bool> inspect(const EgoStatus& ego) const {
        const auto polygon = footprint(ego, 0., 0., 0., config_);
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

class CollisionValidator {
public:
    CollisionValidator(const hdmap::HdMap& map, const PlannerConfig& config)
        : map_(map), config_(config) {}

    bool validate(Primitive& primitive, const PlanningSnapshot& snapshot) const {
        primitive.speed_mps.clear();
        primitive.eta_s.clear();
        double speed = primitive.parent->speed_mps.back();
        double eta = primitive.parent->eta_s.back();
        double previous_x = primitive.parent->x_m.back();
        double previous_y = primitive.parent->y_m.back();
        for (std::size_t index = 0; index < primitive.x_m.size(); ++index) {
            const auto polygon = footprint(snapshot.ego, primitive.x_m[index], primitive.y_m[index],
                primitive.yaw_rad[index], config_);
            const auto cells = map_.cellTree().queryOverlaps(polygon, snapshot.ego.z,
                snapshot.ego.z + config_.vehicle_height_m);
            double cap = std::numeric_limits<double>::infinity();
            for (const auto cell : cells) {
                cap = std::min(cap, static_cast<double>(snapshot.dynamic.speed_cap_mps[cell]));
            }
            const double distance = std::hypot(primitive.x_m[index] - previous_x,
                primitive.y_m[index] - previous_y);
            speed = speed > cap
                ? std::max(cap, std::sqrt(speed * speed - 2. * config_.deceleration_mps2 * distance))
                : std::min(cap, std::sqrt(speed * speed + 2. * config_.acceleration_mps2 * distance));
            eta += 2. * distance / (primitive.speed_mps.empty()
                ? primitive.parent->speed_mps.back() + speed : primitive.speed_mps.back() + speed);
            primitive.speed_mps.push_back(speed);
            primitive.eta_s.push_back(eta);
            const double snapshot_time = rclcpp::Time(snapshot.ego.header.stamp).seconds();
            const double dynamic_time = rclcpp::Time(snapshot.dynamic.header.stamp).seconds();
            const auto bin = static_cast<std::size_t>(std::min(12.,
                std::ceil((snapshot_time + eta - dynamic_time) / .5)));
            for (const auto cell : cells) {
                if (snapshot.dynamic.occupancy_probability[cell * 13 + bin] > 0.F) {
                    return false;
                }
            }
            previous_x = primitive.x_m[index];
            previous_y = primitive.y_m[index];
        }
        return true;
    }

private:
    const hdmap::HdMap& map_;
    const PlannerConfig& config_;
};

class PathBuilder {
public:
    nav_msgs::msg::Path build(const PrimitivePtr& final, const std_msgs::msg::Header& source) const {
        std::vector<PrimitivePtr> chain;
        for (auto node = final; node; node = std::const_pointer_cast<Primitive>(node->parent)) {
            chain.push_back(node);
        }
        std::reverse(chain.begin(), chain.end());
        nav_msgs::msg::Path path;
        path.header = source;
        path.header.frame_id = "base_link";
        for (const auto& primitive : chain) {
            for (std::size_t index = 0; index < primitive->x_m.size(); ++index) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = primitive->x_m[index];
                pose.pose.position.y = primitive->y_m[index];
                pose.pose.orientation.z = std::sin(primitive->yaw_rad[index] / 2.);
                pose.pose.orientation.w = std::cos(primitive->yaw_rad[index] / 2.);
                path.poses.push_back(std::move(pose));
            }
        }
        return path;
    }
};

class SearchTreeBuilder {
public:
    interfaces::msg::SearchTree build(const std::vector<PrimitivePtr>& primitives,
        const PrimitivePtr& final, const std_msgs::msg::Header& source) const {
        interfaces::msg::SearchTree tree;
        tree.header = source;
        tree.header.frame_id = "base_link";
        std::unordered_map<const Primitive*, std::int32_t> indices;
        for (std::size_t index = 0; index < primitives.size(); ++index) {
            indices.emplace(primitives[index].get(), static_cast<std::int32_t>(index));
        }
        for (const auto& primitive : primitives) {
            tree.x.push_back(static_cast<float>(primitive->x_m.back()));
            tree.y.push_back(static_cast<float>(primitive->y_m.back()));
            tree.yaw.push_back(static_cast<float>(primitive->yaw_rad.back()));
            tree.parent_index.push_back(primitive->parent ? indices.at(primitive->parent.get()) : -1);
        }
        tree.final_node_index = indices.at(final.get());
        return tree;
    }
};

struct QueueCompare {
    double g_weight;
    double h_weight;

    bool operator()(const PrimitivePtr& left, const PrimitivePtr& right) const {
        return g_weight * left->g + h_weight * left->h >
            g_weight * right->g + h_weight * right->h;
    }
};

class HybridAStarPlanner {
public:
    using PathPublisher = rclcpp::Publisher<nav_msgs::msg::Path>;
    using TreePublisher = rclcpp::Publisher<interfaces::msg::SearchTree>;

    HybridAStarPlanner(const hdmap::HdMap& map, const PlannerConfig& config,
        PathPublisher::SharedPtr path_publisher, TreePublisher::SharedPtr tree_publisher)
        : map_(map), config_(config), validator_(map, config),
          open_(QueueCompare{config.g_weight, config.h_weight}),
          store_(QueueCompare{config.g_weight, config.h_weight}), path_publisher_(std::move(path_publisher)), tree_publisher_(std::move(tree_publisher)) {}

    void tick(const PlanningSnapshot& snapshot) {
        primitives_.clear();
        open_ = decltype(open_)(QueueCompare{config_.g_weight, config_.h_weight});
        store_ = decltype(store_)(QueueCompare{config_.g_weight, config_.h_weight});
        auto root = std::make_shared<Primitive>();
        root->x_m = {0.}; root->y_m = {0.}; root->yaw_rad = {0.};
        root->speed_mps = {snapshot.ego.speed}; root->eta_s = {0.};
        root->h = heuristic(*root, snapshot);
        primitives_.push_back(root);
        open_.push(root);
        visited_.clear();
        visited_.insert(key(*root));
        for (std::size_t expanded = 0; expanded < config_.max_node_count && !open_.empty(); ++expanded) {
            const auto parent = open_.top();
            open_.pop();
            for (auto& primitive : generate(parent)) {
                if (!validator_.validate(*primitive, snapshot)) {
                    continue;
                }
                primitive->h = heuristic(*primitive, snapshot);
                if (!visited_.insert(key(*primitive)).second) {
                    continue;
                }
                primitives_.push_back(primitive);
                if (primitive->g < config_.max_path_length) {
                    open_.push(primitive);
                } else {
                    store_.push(primitive);
                }
            }
        }
        const auto final = store_.empty() ? open_.top() : store_.top();
        path_publisher_->publish(path_builder_.build(final, snapshot.ego.header));
        tree_publisher_->publish(tree_builder_.build(primitives_, final, snapshot.ego.header));
    }

private:
    std::vector<PrimitivePtr> generate(const PrimitivePtr& parent) const {
        std::vector<PrimitivePtr> generated;
        for (const double steering : config_.steering_candidates_rad) {
            auto primitive = std::make_shared<Primitive>();
            primitive->parent = parent;
            double x = parent->x_m.back();
            double y = parent->y_m.back();
            double yaw = parent->yaw_rad.back();
            for (double distance = config_.xy_resolution_m; distance <= config_.primitive_length_m;
                 distance += config_.xy_resolution_m) {
                x += config_.xy_resolution_m * std::cos(yaw);
                y += config_.xy_resolution_m * std::sin(yaw);
                yaw += config_.xy_resolution_m * std::tan(steering) / config_.wheelbase_m;
                primitive->x_m.push_back(x);
                primitive->y_m.push_back(y);
                primitive->yaw_rad.push_back(yaw);
            }
            primitive->g = parent->g + config_.primitive_length_m;
            generated.push_back(std::move(primitive));
        }
        return generated;
    }

    std::array<std::int64_t, 5> key(const Primitive& primitive) const {
        return {
            std::llround(primitive.x_m.back() / config_.xy_resolution_m),
            std::llround(primitive.y_m.back() / config_.xy_resolution_m),
            std::llround(primitive.yaw_rad.back() / config_.yaw_resolution_rad),
            std::llround(primitive.eta_s.back() / config_.time_resolution_s),
            std::llround(primitive.speed_mps.back() / config_.speed_resolution_mps)};
    }

    double heuristic(const Primitive& primitive, const PlanningSnapshot& snapshot) const {
        const auto point = mapPoint(snapshot.ego, primitive.x_m.back(), primitive.y_m.back());
        double distance = std::numeric_limits<double>::infinity();
        for (const auto lane_id : snapshot.goal_lanes) {
            distance = std::min(distance, lanelet::geometry::distance2d(
                map_.laneletMap().laneletLayer.get(lane_id), point));
        }
        return distance;
    }

    const hdmap::HdMap& map_;
    const PlannerConfig& config_;
    CollisionValidator validator_;
    PathBuilder path_builder_;
    SearchTreeBuilder tree_builder_;
    std::vector<PrimitivePtr> primitives_;
    std::set<std::array<std::int64_t, 5>> visited_;
    std::priority_queue<PrimitivePtr, std::vector<PrimitivePtr>, QueueCompare> open_;
    std::priority_queue<PrimitivePtr, std::vector<PrimitivePtr>, QueueCompare> store_;
    PathPublisher::SharedPtr path_publisher_;
    TreePublisher::SharedPtr tree_publisher_;
};

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner") {
        constexpr double pi = 3.14159265358979323846;
        const auto degrees = declare_parameter<std::vector<double>>(
            "steering_candidates_deg", {-18., -9., 0., 9., 18.});
        const double route_hz = declare_parameter<double>("route_update_hz", 10.);
        const double planner_hz = declare_parameter<double>("hybrid_astar_hz", 20.);
        config_ = {
            route_hz,
            declare_parameter<double>("max_path_length", 20.),
            static_cast<std::size_t>(declare_parameter<int>("max_node_count", 10000)),
            declare_parameter<double>("g_weight", 1.),
            declare_parameter<double>("h_weight", 1.),
            declare_parameter<double>("primitive_length_m", 1.),
            declare_parameter<double>("checkpoint_radius_m", 2.),
            declare_parameter<double>("heading_tolerance_deg", 10.) * pi / 180.,
            declare_parameter<double>("alignment_duration_s", 3.), {},
            declare_parameter<double>("xy_resolution_m", .5),
            declare_parameter<double>("yaw_resolution_deg", 5.) * pi / 180.,
            declare_parameter<double>("time_resolution_s", .25),
            declare_parameter<double>("speed_resolution_mps", .5),
            declare_parameter<double>("acceleration_mps2", 2.),
            declare_parameter<double>("deceleration_mps2", 2.),
            declare_parameter<double>("wheelbase_m", 2.944),
            declare_parameter<double>("rear_axle_to_front_m", 3.808),
            declare_parameter<double>("rear_axle_to_rear_m", 1.040),
            declare_parameter<double>("left_extent_m", .943),
            declare_parameter<double>("right_extent_m", .943),
            declare_parameter<double>("vehicle_height_m", 1.507)};
        for (const auto degree : degrees) config_.steering_candidates_rad.push_back(degree * pi / 180.);
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
        planner_ = std::make_unique<HybridAStarPlanner>(
            *map_, config_, local_publisher_, tree_publisher_);

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
    std::unique_ptr<HybridAStarPlanner> planner_;
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
}
