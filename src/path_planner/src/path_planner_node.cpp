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
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <deque>
#include <optional>
#include <functional>
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

constexpr std::int64_t kSyncToleranceNs = 100000000; // 100 ms
constexpr double kMaxSnapshotAgeS = 1.0;
struct Primitive {
    std::vector<double> x_m;
    std::vector<double> y_m;
    std::vector<double> yaw_rad;
    std::vector<double> speed_mps;
    std::vector<double> eta_s;
    std::vector<double> centerline_offset_m;
    std::vector<double> z_m;
    std::vector<double> curvature_per_m;
    std::vector<double> reference_s;
    std::vector<std::vector<hdmap::CellId>> footprint_cells;
    std::size_t reference_index{};
    bool stopping{};
    double lateral_cost{};
    double cost{};
};

struct PlanningSnapshot {
    EgoStatus ego;
    DynamicStatus::ConstSharedPtr dynamic;
    lanelet::Id current_lane{};
    bool intersection{};
    std::vector<lanelet::Id> global_path;
    std::vector<lanelet::Id> goal_lanes;
    std::vector<lanelet::Id> route_history;
    std::optional<lanelet::BasicPoint2d> terminal_goal;
};

static std::int64_t stamp(const EgoStatus& ego) {
    return rclcpp::Time(ego.header.stamp).nanoseconds();
}

class PlanningRegistry {
public:
    // Callbacks and planning run in the node's single-threaded executor.
    bool onEgo(const EgoStatus& message) {
        if (!std::isfinite(message.x) || !std::isfinite(message.y) ||
            !std::isfinite(message.z) || !std::isfinite(message.heading) ||
            !std::isfinite(message.speed) || message.speed < 0.) return false;
        if (has_ego_ && stamp(message) <= stamp(latest_ego_)) return false;
        bool reset = false;
        if (has_ego_) {
            const double dt = (stamp(message) - stamp(latest_ego_)) * 1e-9;
            const double distance = std::hypot(message.x - latest_ego_.x, message.y - latest_ego_.y);
            const double heading = std::abs(std::remainder(message.heading - latest_ego_.heading, 2. * std::acos(-1.)));
            reset = dt > 1. || distance > 2. + 1.5 * latest_ego_.speed * dt ||
                std::abs(message.z - latest_ego_.z) > 1.5 + .3 * latest_ego_.speed * dt ||
                (heading > .8 && dt < .2);
        }
        latest_ego_ = message;
        has_ego_ = true;
        if (reset) {
            cutoff_ = stamp(message);
            egos_.clear();
            pending_.reset();
            snapshot_ = PlanningSnapshot();
            ready_ = false;
        }
        egos_.push_back(message);
        while (egos_.size() > 64) egos_.pop_front();
        match();
        return reset;
    }

    void onDynamic(DynamicStatus::ConstSharedPtr message) {
        const auto time = rclcpp::Time(message->header.stamp).nanoseconds();
        if (time < cutoff_ || (ready_ && time <= stamp(snapshot_.ego))) return;
        if (!pending_ || time >= rclcpp::Time(pending_->header.stamp).nanoseconds()) {
            pending_ = std::move(message);
        }
        match();
    }
    bool hasEgo() const { return has_ego_; }
    bool ready() const { return ready_; }
    EgoStatus latestEgo() const { return latest_ego_; }
    PlanningSnapshot snapshot() const { return snapshot_; }
    PlanningSnapshot egoOnly() const { auto value = snapshot_; value.ego = latest_ego_; return value; }
    void updateRoute(lanelet::Id current, bool intersection, std::vector<lanelet::Id> route,
        std::vector<lanelet::Id> goals, std::vector<lanelet::Id> history = {},
        std::optional<lanelet::BasicPoint2d> terminal = {}) {
        snapshot_.current_lane = current;
        snapshot_.intersection = intersection;
        snapshot_.global_path = std::move(route);
        snapshot_.goal_lanes = std::move(goals);
        snapshot_.route_history = std::move(history);
        snapshot_.terminal_goal = terminal;
    }
private:
    void match() {
        if (!pending_) return;
        const auto time = rclcpp::Time(pending_->header.stamp).nanoseconds();
        const EgoStatus* best = nullptr;
        std::int64_t best_error = kSyncToleranceNs + 1;
        for (const auto& ego : egos_) {
            const auto ego_stamp = stamp(ego);
            const auto error = ego_stamp > time ? ego_stamp - time : time - ego_stamp;
            if (error > kSyncToleranceNs) continue;
            if (!best || error < best_error ||
                (error == best_error && ego_stamp <= time && stamp(*best) > time)) {
                best = &ego;
                best_error = error;
            }
        }
        if (!best) return;
        snapshot_.ego = *best;
        snapshot_.dynamic = std::move(pending_);
        ready_ = true;
    }
    PlanningSnapshot snapshot_;
    EgoStatus latest_ego_;
    std::deque<EgoStatus> egos_;
    DynamicStatus::ConstSharedPtr pending_;
    std::int64_t cutoff_{};
    bool has_ego_{};
    bool ready_{};
};

struct PlannerConfig {
    double route_update_hz;
    double max_path_length;
    double lateral_cost_weight;
    double time_cost_weight;
    double goal_cost_weight;
    double progress_cost_weight;
    double alignment_cost_weight;
    double centerline_deviation_cost_weight;
    double checkpoint_radius_m;
    double heading_tolerance_rad;
    double alignment_duration_s;
    std::vector<double> frenet_horizon_candidates_s;
    double xy_resolution_m;
    double time_resolution_s;
    double lane_overlap_check_delay_s;
    double planning_accel_limit_mps2;
    double planning_decel_limit_mps2;
    double wheelbase_m;
    double maximum_curvature_per_m;
    double maximum_steering_rad;
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

    std::pair<lanelet::ConstLanelet, bool> inspect(const EgoStatus& ego,
        const std::vector<lanelet::Id>& route = {}) const {
        const auto polygon = footprint(ego.x, ego.y, ego.heading, config_);
        const auto box = lanelet::geometry::boundingBox2d(polygon);
        double best_score = -std::numeric_limits<double>::infinity();
        lanelet::ConstLanelet current;
        for (const auto& lane : map_.laneletLayer.search(box)) {
            double nearest = std::numeric_limits<double>::infinity(), yaw = 0., z = 0.;
            const auto center = lane.centerline3d();
            for (std::size_t i = 1; i < center.size(); ++i) {
                const auto a = center[i-1].basicPoint(), b = center[i].basicPoint();
                const lanelet::BasicPoint2d d{b.x()-a.x(), b.y()-a.y()};
                if (d.squaredNorm() < 1e-12) continue;
                const lanelet::BasicPoint2d e{ego.x-a.x(), ego.y-a.y()};
                const double t = std::clamp(e.dot(d)/d.squaredNorm(), 0., 1.);
                const double distance = (e-t*d).norm();
                if (distance < nearest) { nearest=distance; yaw=std::atan2(d.y(),d.x()); z=a.z()+t*(b.z()-a.z()); }
            }
            const double angle = std::abs(std::remainder(ego.heading-yaw, 2.*std::acos(-1.)));
            if (std::abs(ego.z-z) > 1.5 || angle > 1.2) continue;
            auto lane_polygon = lane.polygon2d().basicPolygon();
            boost::geometry::correct(lane_polygon);
            std::vector<lanelet::BasicPolygon2d> intersections;
            boost::geometry::intersection(lane_polygon, polygon, intersections);
            double area=0.;
            for (const auto& part : intersections) area += std::abs(boost::geometry::area(part));
            if (area < 1e-6) continue;
            const bool on_route = std::find(route.begin(), route.end(), lane.id()) != route.end();
            const double score = area - 3.*angle - nearest + (on_route ? 2. : 0.);
            if (score > best_score) { best_score=score; current=lane; }
        }
        return {current, current.id() != 0 && current.attributeOr<std::string>("intersection", "") == "yes"};
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

    void reset() {
        initialized_ = false;
        allow_lane_change_ = false;
        alignment_time_ = 0.;
        last_tick_time_ = last_search_time_ = -1.;
    }

    void tick(const PlanningSnapshot& input) {
        const auto [current, intersection] = monitor_.inspect(input.ego, input.global_path);
        const double now = rclcpp::Time(input.ego.header.stamp).seconds();
        tick_dt_ = last_tick_time_ < 0. ? 0. : std::max(0., now-last_tick_time_);
        last_tick_time_ = now;
        auto ids = input.global_path;
        auto history = input.route_history;
        bool checkpoint_changed = false;
        if (checkpoint_index_ + 1 < checkpoints_.size() &&
            std::hypot(input.ego.x-checkpoints_[checkpoint_index_].x,
                input.ego.y-checkpoints_[checkpoint_index_].y) <= config_.checkpoint_radius_m) {
            ++checkpoint_index_;
            checkpoint_changed = true;
        }
        if (current.id() == 0 || checkpoints_.empty()) {
            registry_->updateRoute(0, false, {}, {});
            if (publisher_) publisher_->publish(std_msgs::msg::Int64MultiArray{});
            return;
        }
        const auto found = std::find(ids.begin(), ids.end(), current.id());
        const bool disconnected = found == ids.end();
        if (!disconnected) {
            history.insert(history.end(), ids.begin(), found);
            ids.erase(ids.begin(), found);
        }
        if (history.size() > 64) history.erase(history.begin(), history.end()-64);
        const bool search_due = last_search_time_ < 0. || now-last_search_time_ >= 1./config_.route_update_hz;
        if (disconnected || checkpoint_changed || (search_due && !intersection)) {
            lanelet::ConstLanelets via;
            for (std::size_t i=checkpoint_index_; i+1<checkpoints_.size(); ++i) via.push_back(checkpoints_[i].lane);
            const auto path = graph_.shortestPathVia(current, via, checkpoints_.back().lane);
            ids.clear();
            if (path) for (const auto& lane : *path) ids.push_back(lane.id());
            if (disconnected) history.clear();
            last_search_time_=now;
        }
        const auto goals = ids.size()>1 ? updateGoals(input,current,intersection,ids) : ids;
        const auto& terminal = checkpoints_.back();
        registry_->updateRoute(current.id(),intersection,ids,goals,history,
            lanelet::BasicPoint2d{terminal.x,terminal.y});
        if (publisher_) { std_msgs::msg::Int64MultiArray message; message.data=ids; publisher_->publish(message); }
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
        const auto previous_position = std::find(input.global_path.begin(), input.global_path.end(), previous_current_);
        const auto new_position = std::find(input.global_path.begin(), input.global_path.end(), route[0]);
        const bool advanced = previous_position != input.global_path.end() &&
            new_position != input.global_path.end() && new_position > previous_position;
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
            alignment_time_ += tick_dt_;
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
        if (following.empty()) return {route[0], route[1]};
        const auto selected = std::find_if(following.begin(), following.end(),
            [&](const auto& lane) { return lane.id() == route[1]; });
        return selected == following.end() ? std::vector<lanelet::Id>{route[0]} :
            std::vector<lanelet::Id>{route[0], selected->id()};
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
    double alignment_time_{};
    double last_tick_time_{-1.}, last_search_time_{-1.}, tick_dt_{};
    Publisher::SharedPtr publisher_;
};

struct Reference {
    lanelet::BasicPoints2d points;
    std::vector<double> station, heights;
    std::vector<lanelet::Id> lane_ids;
    std::set<lanelet::Id> allowed_lanes;
    lanelet::BasicPoint2d corridor_origin{0.,0.};
    std::map<lanelet::Id,lanelet::BasicPolygon2d> corridor;
    using Spline = boost::math::interpolators::cardinal_cubic_b_spline<double>;
    std::array<Spline, 3> spline;
    std::vector<double> arc, parameter;
    double ego_s{}, target_s{};
    std::optional<double> terminal_s;
    bool smooth{};
    bool lane_change{};
    double length() const { return smooth ? arc.back() : (station.empty() ? 0. : station.back()); }
};

struct Projection {
    double s{}, d{}, yaw{}, residual{};
};
struct ReferenceSample {
    lanelet::BasicPoint2d point;
    double yaw{}, z{}, curvature{};
};

static void appendCenterline(Reference& reference, const lanelet::ConstLanelet& lane) {
    reference.lane_ids.push_back(lane.id());
    reference.allowed_lanes.insert(lane.id());
    for (const auto& point : lane.centerline3d()) {
        const lanelet::BasicPoint2d basic{point.x(), point.y()};
        if (!reference.points.empty() && (basic-reference.points.back()).norm()<1e-6) continue;
        reference.station.push_back(reference.points.empty() ? 0. :
            reference.station.back()+(basic-reference.points.back()).norm());
        reference.points.push_back(basic);
        reference.heights.push_back(point.z());
    }
}

static double interpolate(const std::vector<double>& x, const std::vector<double>& y, double value) {
    value=std::clamp(value,x.front(),x.back());
    auto upper=std::upper_bound(x.begin(),x.end(),value);
    const auto i=std::clamp<std::size_t>(std::distance(x.begin(),upper),1,x.size()-1);
    return y[i-1]+(y[i]-y[i-1])*(value-x[i-1])/(x[i]-x[i-1]);
}

static ReferenceSample sample(const Reference& reference, double station) {
    station=std::clamp(station,0.,reference.length());
    if (reference.smooth) {
        const double u=interpolate(reference.arc,reference.parameter,station);
        const double dx=reference.spline[0].prime(u), dy=reference.spline[1].prime(u);
        const double norm=std::hypot(dx,dy);
        return {{reference.spline[0](u),reference.spline[1](u)},std::atan2(dy,dx),
            reference.spline[2](u), norm>1e-8 ?
            (dx*reference.spline[1].double_prime(u)-dy*reference.spline[0].double_prime(u))/std::pow(norm,3) :
            std::numeric_limits<double>::infinity()};
    }
    auto upper=std::upper_bound(reference.station.begin(),reference.station.end(),station);
    const auto i=std::clamp<std::size_t>(std::distance(reference.station.begin(),upper),1,reference.points.size()-1);
    const double ratio=(station-reference.station[i-1])/(reference.station[i]-reference.station[i-1]);
    const auto delta=reference.points[i]-reference.points[i-1];
    return {reference.points[i-1]+ratio*delta,std::atan2(delta.y(),delta.x()),
        reference.heights.empty() ? 0. : reference.heights[i-1]+ratio*(reference.heights[i]-reference.heights[i-1]),0.};
}

static Projection project(const Reference& reference, const lanelet::BasicPoint2d& point) {
    double best=std::numeric_limits<double>::infinity(), best_s=0.;
    const auto& stations=reference.smooth ? reference.arc : reference.station;
    for (std::size_t i=1;i<stations.size();++i) {
        const auto a=sample(reference,stations[i-1]).point, b=sample(reference,stations[i]).point;
        const auto delta=b-a;
        if (delta.squaredNorm()<1e-12) continue;
        const double ratio=std::clamp((point-a).dot(delta)/delta.squaredNorm(),0.,1.);
        const double error=(point-a-ratio*delta).squaredNorm();
        if (error<best) { best=error; best_s=stations[i-1]+ratio*(stations[i]-stations[i-1]); }
    }
    if (reference.smooth) {
        double lo=std::max(0.,best_s-.2),hi=std::min(reference.length(),best_s+.2);
        for(int i=0;i<28;++i) {
            const double a=(2.*lo+hi)/3., b=(lo+2.*hi)/3.;
            if ((sample(reference,a).point-point).squaredNorm() < (sample(reference,b).point-point).squaredNorm()) hi=b;
            else lo=a;
        }
        best_s=(lo+hi)*.5;
    }
    const auto center=sample(reference,best_s);
    const auto error=point-center.point;
    return {best_s,-std::sin(center.yaw)*error.x()+std::cos(center.yaw)*error.y(),center.yaw,
        std::cos(center.yaw)*error.x()+std::sin(center.yaw)*error.y()};
}

static bool smoothReference(Reference& reference, double begin, double end) {
    if (reference.points.size()<2 || end-begin<.1) return false;
    const auto raw=reference;
    const auto intervals=std::max<std::size_t>(5,std::ceil((end-begin)/.5));
    const double step=(end-begin)/intervals;
    std::array<std::vector<double>,3> coordinates;
    for(std::size_t i=0;i<=intervals;++i) {
        const auto q=sample(raw,begin+i*step);
        coordinates[0].push_back(q.point.x()); coordinates[1].push_back(q.point.y()); coordinates[2].push_back(q.z);
    }
    for(std::size_t axis=0;axis<3;++axis) {
        const auto& values=coordinates[axis];
        reference.spline[axis]=Reference::Spline(values.data(),values.size(),0.,step,
            (values[1]-values[0])/step,(values.back()-values[values.size()-2])/step);
    }
    const auto n=std::max<std::size_t>(2,std::ceil((end-begin)/.1));
    reference.arc={0.};reference.parameter={0.};
    lanelet::BasicPoint2d previous{coordinates[0][0],coordinates[1][0]};
    for(std::size_t i=1;i<=n;++i) {
        const double u=(end-begin)*i/n;
        const lanelet::BasicPoint2d q{reference.spline[0](u),reference.spline[1](u)};
        // The spline may round centerline corners, but must stay near the source corridor.
        const auto raw_q=sample(raw,begin+u).point;
        if ((q-raw_q).norm()>.25 || (q-previous).norm()<1e-8) return false;
        reference.parameter.push_back(u);
        reference.arc.push_back(reference.arc.back()+(q-previous).norm());
        previous=q;
    }
    reference.smooth=true;
    return true;
}

struct ReferenceSet { std::vector<Reference> references; };

class ReferenceBuilder {
public:
    ReferenceBuilder(const lanelet::LaneletMap& map,const lanelet::routing::RoutingGraph& graph)
        :map_(map),graph_(graph) {}
    ReferenceSet build(const PlanningSnapshot& input,double forward_m,double rear_m) const {
        ReferenceSet result;
        if(input.global_path.empty() || input.global_path.front()!=input.current_lane) return result;
        const auto current=map_.laneletLayer.get(input.current_lane);
        std::vector<lanelet::Id> starts{input.current_lane};
        for(const auto& adjacent:{graph_.left(current),graph_.right(current)}) {
            if(adjacent && std::find(input.global_path.begin(),input.global_path.end(),adjacent->id())!=input.global_path.end() &&
                std::find(input.goal_lanes.begin(),input.goal_lanes.end(),adjacent->id())!=input.goal_lanes.end())
                starts.push_back(adjacent->id());
        }
        for(const auto start:starts) {
            Reference reference;
            const auto route_start=std::find(input.global_path.begin(),input.global_path.end(),start);
            std::vector<lanelet::Id> sequence;
            if(start==input.current_lane) sequence=input.route_history;
            double retained=0.;std::size_t keep=sequence.size();
            while(keep>0 && retained<rear_m) retained+=lanelet::geometry::length2d(map_.laneletLayer.get(sequence[--keep]));
            sequence.erase(sequence.begin(),sequence.begin()+keep);
            // Expand only a unique, graph-connected predecessor chain, with a cycle guard.
            std::set<lanelet::Id> visited(sequence.begin(),sequence.end());
            visited.insert(start);
            double rear_length=0.;
            for(const auto id:sequence) rear_length+=lanelet::geometry::length2d(map_.laneletLayer.get(id));
            auto previous_lane=map_.laneletLayer.get(sequence.empty() ? start : sequence.front());
            while(rear_length<rear_m) {
                const auto previous=graph_.previous(previous_lane,false);
                if(previous.size()!=1 || !visited.insert(previous.front().id()).second) break;
                sequence.insert(sequence.begin(),previous.front().id());
                rear_length+=lanelet::geometry::length2d(previous.front());
                previous_lane=previous.front();
            }
            sequence.insert(sequence.end(),route_start,input.global_path.end());
            bool started=false;
            for(const auto id:sequence) {
                const auto lane=map_.laneletLayer.get(id);
                if(!reference.lane_ids.empty()) {
                    const auto next=graph_.following(map_.laneletLayer.get(reference.lane_ids.back()),false);
                    if(std::none_of(next.begin(),next.end(),[&](const auto& value){return value.id()==id;})) {
                        if(!started) { reference={}; }
                        else break; // A lateral edge is handled by the separate adjacent reference.
                    }
                }
                appendCenterline(reference,lane);
                if(id==start) started=true;
                if(started && reference.points.size()>1) {
                    const auto projection=project(reference,{input.ego.x,input.ego.y});
                    if(reference.length()-projection.s>=forward_m) break;
                }
            }
            if(reference.points.size()<2) continue;
            const auto initial=project(reference,{input.ego.x,input.ego.y});
            const double begin=std::max(0.,initial.s-rear_m),end=std::min(reference.length(),initial.s+forward_m);
            if(!smoothReference(reference,begin,end)) continue;
            const auto smooth_initial=project(reference,{input.ego.x,input.ego.y});
            if(std::abs(smooth_initial.residual)>.02 ||
                std::abs(sample(reference,smooth_initial.s).z-input.ego.z)>1.5 ||
                std::abs(std::remainder(input.ego.heading-smooth_initial.yaw,2.*std::acos(-1.)))>1.2) continue;
            reference.lane_change=start!=input.current_lane;
            reference.ego_s=smooth_initial.s;
            reference.target_s=reference.length();
            reference.allowed_lanes.insert(input.current_lane); // The lane-change footprint starts here.
            reference.corridor_origin=sample(reference,reference.ego_s).point;
            for(const auto id:reference.allowed_lanes) {
                auto polygon=map_.laneletLayer.get(id).polygon2d().basicPolygon();
                for(auto& point:polygon) point-=reference.corridor_origin;
                boost::geometry::correct(polygon);
                reference.corridor.emplace(id,std::move(polygon));
            }
            if(input.terminal_goal && reference.lane_ids.back()==input.global_path.back()) {
                const auto terminal=project(reference,*input.terminal_goal);
                if(std::abs(terminal.residual)<.05 && std::abs(terminal.d)<2.) reference.terminal_s=terminal.s;
            }
            result.references.push_back(std::move(reference));
        }
        if(result.references.size()>1) std::rotate(result.references.begin(),result.references.begin()+1,result.references.end());
        return result;
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

// Lane keeping recovers over the current-speed horizon; lane changes retain their full geometric distance.
struct LateralProfile {
    double distance;
    Quintic curve;
    LateralProfile(double offset,double slope,double speed,double duration,double path_distance,bool lane_change=false)
        :distance(lane_change ? path_distance : std::min(path_distance,std::max(6.,speed*duration))),
         curve(quintic(offset,slope,0.,0.,0.,0.,distance)) {}
    double position(double s) const { return curve.position(std::min(s,distance)); }
    double acceleration(double s) const { return s<distance ? curve.acceleration(s) : 0.; }
};

static double pathLength(const Primitive& primitive) {
    double result = 0.;
    for (std::size_t index = 1; index < primitive.x_m.size(); ++index) {
        result += std::hypot(primitive.x_m[index] - primitive.x_m[index - 1],
            primitive.y_m[index] - primitive.y_m[index - 1]);
    }
    return result;
}

static void resize(Primitive& p,std::size_t count) {
    p.x_m.resize(count);p.y_m.resize(count);p.yaw_rad.resize(count);
    p.z_m.resize(count);p.curvature_per_m.resize(count);p.reference_s.resize(count);
    p.footprint_cells.resize(count);p.centerline_offset_m.resize(count);
    p.speed_mps.resize(count);p.eta_s.resize(count);
}
static double signedCurvature(double ax,double ay,double bx,double by,double cx,double cy) {
    const double denominator=std::hypot(bx-ax,by-ay)*std::hypot(cx-bx,cy-by)*std::hypot(cx-ax,cy-ay);
    return denominator>1e-12 ? 2.*((bx-ax)*(cy-ay)-(by-ay)*(cx-ax))/denominator : 0.;
}
static bool kinematicallyFeasible(const Primitive& p,double initial_heading,double wheelbase,
    double maximum_curvature,double maximum_steering) {
    if(p.x_m.size()<2) return false;
    const double first_yaw=p.yaw_rad.empty() ? std::atan2(p.y_m[1]-p.y_m[0],p.x_m[1]-p.x_m[0]) : p.yaw_rad.front();
    if(std::abs(std::remainder(first_yaw-initial_heading,2.*std::acos(-1.)))>.03) return false;
    for(std::size_t i=0;i<p.x_m.size();++i) {
        double curvature=0.;
        if(!p.curvature_per_m.empty()) curvature=p.curvature_per_m[i];
        else if(i>0 && i+1<p.x_m.size()) curvature=signedCurvature(p.x_m[i-1],p.y_m[i-1],p.x_m[i],p.y_m[i],p.x_m[i+1],p.y_m[i+1]);
        if(!std::isfinite(curvature) || std::abs(curvature)>maximum_curvature ||
            std::abs(std::atan(wheelbase*curvature))>maximum_steering) return false;
        if(i>0 && std::hypot(p.x_m[i]-p.x_m[i-1],p.y_m[i]-p.y_m[i-1])<1e-7) return false;
        if(i>0 && i+1<p.x_m.size() &&
            (p.x_m[i]-p.x_m[i-1])*(p.x_m[i+1]-p.x_m[i])+
            (p.y_m[i]-p.y_m[i-1])*(p.y_m[i+1]-p.y_m[i])<=0.) return false;
    }
    return true;
}
static double centerlineDeviationCost(const Primitive& p) {
    double sum=0.;for(const auto offset:p.centerline_offset_m) sum+=offset*offset;
    return p.centerline_offset_m.empty() ? 0. : sum/p.centerline_offset_m.size();
}
struct RoadSample { std::vector<hdmap::CellId> cells; bool covered{}; };
static bool coveredBy(const lanelet::BasicPolygon2d& shape,const std::vector<const lanelet::BasicPolygon2d*>& corridor) {
    for(const auto* polygon:corridor) if(boost::geometry::covered_by(shape,*polygon)) return true;
    std::vector<lanelet::BasicPolygon2d> remainder{shape};
    for(const auto* polygon:corridor) {
        std::vector<lanelet::BasicPolygon2d> next;
        for(const auto& part:remainder) {
            std::vector<lanelet::BasicPolygon2d> difference;
            boost::geometry::difference(part,*polygon,difference);
            next.insert(next.end(),difference.begin(),difference.end());
        }
        remainder=std::move(next);
        if(remainder.empty()) return true;
    }
    double area=0.;for(const auto& part:remainder) area+=std::abs(boost::geometry::area(part));
    return area<=1e-4;
}
static RoadSample roadSample(const hdmap::HdMap& map,const PlannerConfig& config,
    const Reference& reference,double x,double y,double yaw,double z,
    double station=std::numeric_limits<double>::quiet_NaN()) {
    const auto shape=footprint(x,y,yaw,config);
    if(!std::isfinite(station)) station=project(reference,{x,y}).s;
    const double front_z=sample(reference,station+config.rear_axle_to_front_m).z;
    const double rear_z=sample(reference,station-config.rear_axle_to_rear_m).z;
    const double low=std::min({z,front_z,rear_z}),high=std::max({z,front_z,rear_z});
    const auto hits=map.cellTree().queryOverlaps(shape,low,high+config.vehicle_height_m);
    RoadSample result;std::set<lanelet::Id> parents;
    std::vector<const lanelet::BasicPolygon2d*> corridor;
    for(const auto id:hits) {
        if(id>=map.cells().size()) return result;
        const auto& cell=map.cells()[id];const auto box=cell.boundingBox3d();
        if(box.max().z()<low-.5 || box.min().z()>high+.5) continue;
        result.cells.push_back(id);const auto parent=cell.parent().lanelet_id;
        if(reference.allowed_lanes.count(parent) && parents.insert(parent).second) {
            const auto polygon=reference.corridor.find(parent);
            if(polygon==reference.corridor.end()) return result;
            corridor.push_back(&polygon->second);
        }
    }
    auto translated=shape;
    for(auto& point:translated) point-=reference.corridor_origin;
    result.covered=!result.cells.empty() && coveredBy(translated,corridor);
    return result;
}

class VelocityPlanner {
public:
    VelocityPlanner(const hdmap::HdMap& map, const PlannerConfig& config)
        : map_(map), config_(config) {}

    bool plan(Primitive& primitive, const PlanningSnapshot& snapshot, const Reference& reference, bool stop_at_end = false) const {
        auto count = primitive.x_m.size();
        if (count < 2) return false;
        std::vector<double> distance(count, 0.);
        primitive.speed_mps.assign(count, std::numeric_limits<double>::infinity());
        primitive.eta_s.assign(count, 0.);
        primitive.footprint_cells.resize(count);
        for (std::size_t index = 0; index < count; ++index) {
            if (index) distance[index] = std::hypot(
                primitive.x_m[index] - primitive.x_m[index - 1],
                primitive.y_m[index] - primitive.y_m[index - 1]);
            const auto road=roadSample(map_,config_,reference,primitive.x_m[index],primitive.y_m[index],
                primitive.yaw_rad[index],primitive.z_m[index],primitive.reference_s[index]);
            if(!road.covered) return false;
            primitive.footprint_cells[index]=road.cells;
            const auto& cells=road.cells;
            for (const auto cell : cells) {
                if (cell < snapshot.dynamic->speed_cap_mps.size()) {
                    primitive.speed_mps[index] = std::min(primitive.speed_mps[index],
                        static_cast<double>(snapshot.dynamic->speed_cap_mps[cell]));
                }
            }
        }
        std::size_t stop_index = count;
        for (std::size_t index = 0; index < count; ++index) {
            if (!(primitive.speed_mps[index] > 0.)) {
                stop_index = index;
                break;
            }
        }
        if (stop_index == 0) return false;
        if (stop_index < count) {
            if (stop_index < 2) return false;
            resize(primitive, stop_index);
            distance.resize(stop_index);
            count = stop_index;
            stop_at_end = true;
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

struct CollisionResult {
    std::size_t index{};
    bool forbidden_lane{};
};

class CollisionValidator {
public:
    CollisionValidator(const hdmap::HdMap& map, const PlannerConfig& config)
        : map_(map), config_(config) {}

    CollisionResult firstCollision(const Primitive& primitive,
        const PlanningSnapshot& snapshot, const Reference& /*reference*/) const {
        const double snapshot_time = rclcpp::Time(snapshot.ego.header.stamp).seconds();
        const double dynamic_time = rclcpp::Time(snapshot.dynamic->header.stamp).seconds();
        for (std::size_t index = 0; index < primitive.x_m.size(); ++index) {
            // Geometry is unchanged after the preceding velocity pass; reuse its complete footprint query.
            if(index>=primitive.footprint_cells.size() || primitive.footprint_cells[index].empty()) return {index,true};
            const auto& cells=primitive.footprint_cells[index];
            const auto bin = static_cast<std::size_t>(std::clamp(
                std::ceil((snapshot_time + primitive.eta_s[index] - dynamic_time) / .5), 0., 12.));
            for (const auto cell : cells) {
                if (cell >= map_.cells().size()) return {index, true};
                const auto offset = cell * 13 + bin;
                if (offset < snapshot.dynamic->occupancy_probability.size() &&
                    snapshot.dynamic->occupancy_probability[offset] > 0.F) return {index, false};
            }
        }
        return {primitive.x_m.size(), false};
    }

private:
    const hdmap::HdMap& map_;
    const PlannerConfig& config_;
};

static double wrap(double angle) { return std::atan2(std::sin(angle),std::cos(angle)); }

class CostEvaluator {
public:
    explicit CostEvaluator(const PlannerConfig& config):config_(config) {}
    double evaluate(Primitive& p,const Reference& reference,double target_s) const {
        p.cost=geometryCost(p,reference,target_s,p.x_m.size()-1,centerlineDeviationCost(p))+
            config_.time_cost_weight*p.eta_s.back();
        return p.cost;
    }
    double lowerBound(const Primitive& p,const Reference& reference,double target_s) const {
        // Other references use a different station frame; zero is a safe, deliberately loose bound.
        if(p.reference_index!=0 || p.reference_s.size()!=p.x_m.size()) return 0.;
        double minimum=std::numeric_limits<double>::infinity(),offset_sum=0.;
        for(std::size_t i=0;i<p.x_m.size();++i) {
            offset_sum+=p.centerline_offset_m[i]*p.centerline_offset_m[i];
            if(i>0) minimum=std::min(minimum,geometryCost(p,reference,target_s,i,offset_sum/(i+1)));
        }
        // A collision may shorten the candidate to any checked prefix. ETA is nonnegative.
        return minimum;
    }
private:
    double geometryCost(const Primitive& p,const Reference& reference,double target_s,
        std::size_t i,double offset_cost) const {
        Projection end;
        if(p.reference_index==0 && p.reference_s.size()==p.x_m.size()) {
            end={p.reference_s[i],p.centerline_offset_m[i],sample(reference,p.reference_s[i]).yaw,0.};
        } else end=project(reference,{p.x_m[i],p.y_m[i]});
        if(p.stopping) target_s=std::min(target_s,end.s);
        const double deficit=std::max(0.,target_s-end.s);
        // Lateral alignment retains the target-lane preference without attracting endpoints backward.
        const double goal=end.d*end.d+(reference.terminal_s ? std::pow(end.s-*reference.terminal_s,2) : 0.);
        return config_.lateral_cost_weight*p.lateral_cost+config_.goal_cost_weight*goal+
            config_.progress_cost_weight*deficit*deficit+
            config_.alignment_cost_weight*std::pow(wrap(p.yaw_rad[i]-end.yaw),2)+
            config_.centerline_deviation_cost_weight*offset_cost;
    }
    const PlannerConfig& config_;
};

static std::pair<double, double> toBaseLink(const EgoStatus& ego, double x, double y,
                                            double wheelbase_m) {
    const double dx = x - ego.x - wheelbase_m * std::cos(ego.heading);
    const double dy = y - ego.y - wheelbase_m * std::sin(ego.heading);
    const double c = std::cos(ego.heading);
    const double s = std::sin(ego.heading);
    return {c * dx + s * dy, -s * dx + c * dy};
}

class PathBuilder {
public:
    explicit PathBuilder(double wheelbase_m) : wheelbase_m_(wheelbase_m) {}

    nav_msgs::msg::Path build(const Primitive& primitive, const EgoStatus& ego) const {
        nav_msgs::msg::Path path;
        path.header = ego.header;
        path.header.frame_id = "base_link";
        for (std::size_t index = 0; index < primitive.x_m.size(); ++index) {
            const auto [x, y] = toBaseLink(ego, primitive.x_m[index], primitive.y_m[index],
                                           wheelbase_m_);
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

private:
    double wheelbase_m_;
};

class SearchTreeBuilder {
public:
    explicit SearchTreeBuilder(double wheelbase_m) : wheelbase_m_(wheelbase_m) {}

    interfaces::msg::SearchTree build(const std::vector<Primitive>& candidates,
        std::size_t selected, const EgoStatus& ego) const {
        interfaces::msg::SearchTree tree;
        tree.header = ego.header;
        tree.header.frame_id = "base_link";
        tree.final_node_index = -1;
        for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
            std::int32_t parent = -1;
            for (std::size_t index = 0; index < candidates[candidate_index].x_m.size(); ++index) {
                const auto [x, y] = toBaseLink(ego, candidates[candidate_index].x_m[index],
                    candidates[candidate_index].y_m[index], wheelbase_m_);
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

private:
    double wheelbase_m_;
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
          path_builder_(config.wheelbase_m), tree_builder_(config.wheelbase_m),
          path_publisher_(std::move(path_publisher)), tree_publisher_(std::move(tree_publisher)) {}

    void invalidate(const EgoStatus& ego,const char* reason) {
        if(path_publisher_) path_publisher_->publish(path_builder_.build(Primitive{},ego));
        if(tree_publisher_) tree_publisher_->publish(tree_builder_.build({},0,ego));
        logFailure("Local path invalid", reason);
    }
    void skip(const char* reason) {
        logFailure("Local path skipped", reason);
    }
    void publishHold(const EgoStatus& ego) {
        Primitive hold;
        hold.x_m = {ego.x + config_.wheelbase_m * std::cos(ego.heading)};
        hold.y_m = {ego.y + config_.wheelbase_m * std::sin(ego.heading)};
        hold.yaw_rad = {ego.heading};
        if(path_publisher_) path_publisher_->publish(path_builder_.build(hold,ego));
        if(tree_publisher_) tree_publisher_->publish(tree_builder_.build({},0,ego));
        last_failure_.clear();
    }
    void tick(const PlanningSnapshot& snapshot) {
        if(snapshot.current_lane==0 || !snapshot.dynamic) { invalidate(snapshot.ego,"missing route/input");return; }
        const double horizon=*std::max_element(config_.frenet_horizon_candidates_s.begin(),config_.frenet_horizon_candidates_s.end());
        const double budget=std::min(config_.max_path_length,snapshot.ego.speed*horizon+
            .5*config_.planning_accel_limit_mps2*horizon*horizon);
        const auto reference_set=references_.build(snapshot,budget+config_.rear_axle_to_front_m+2.,config_.rear_axle_to_rear_m+5.);
        if(reference_set.references.empty()) { skip("disconnected/reference projection");return; }
        auto candidates=generate(snapshot,reference_set.references);
        const auto& progress=reference_set.references.front();
        const double target=std::min({progress.ego_s+budget,progress.length()-config_.rear_axle_to_front_m,
            progress.terminal_s.value_or(std::numeric_limits<double>::infinity())});
        std::vector<std::pair<double,std::size_t>> ordered;
        for(std::size_t i=0;i<candidates.size();++i) ordered.emplace_back(costs_.lowerBound(candidates[i],progress,target),i);
        std::sort(ordered.begin(),ordered.end());
        std::vector<std::pair<std::size_t,Primitive>> checked;
        double best_cost=std::numeric_limits<double>::infinity();
        std::size_t velocity_rejected=0,collision_rejected=0;
        for(const auto& [bound,original_index]:ordered) {
            if(bound>best_cost) break;
            auto& candidate=candidates[original_index];
            const auto& reference=reference_set.references[candidate.reference_index];
            if(!velocity_.plan(candidate,snapshot,reference)) { ++velocity_rejected;continue; }
            const auto collision=validator_.firstCollision(candidate,snapshot,reference);
            if(collision.forbidden_lane) { ++collision_rejected;continue; }
            if(collision.index<candidate.x_m.size()) {
                if(collision.index<2) { ++collision_rejected;continue; }
                resize(candidate,collision.index);
                if(!velocity_.plan(candidate,snapshot,reference,true)) { ++velocity_rejected;continue; }
                const auto recheck=validator_.firstCollision(candidate,snapshot,reference);
                if(recheck.forbidden_lane || recheck.index<candidate.x_m.size()) { ++collision_rejected;continue; }
            }
            const auto n=candidate.x_m.size();
            if(candidate.y_m.size()!=n || candidate.yaw_rad.size()!=n || candidate.z_m.size()!=n ||
                candidate.curvature_per_m.size()!=n || candidate.reference_s.size()!=n ||
                candidate.footprint_cells.size()!=n || candidate.centerline_offset_m.size()!=n ||
                candidate.speed_mps.size()!=n || candidate.eta_s.size()!=n) continue;
            best_cost=std::min(best_cost,costs_.evaluate(candidate,progress,target));
            checked.emplace_back(original_index,std::move(candidate));
        }
        if(checked.empty()) {
            RCLCPP_DEBUG(rclcpp::get_logger("path_planner"),"Candidates geometric=%zu velocity=%zu collision=%zu",candidates.size(),velocity_rejected,collision_rejected);
            if(snapshot.ego.speed < .1) { publishHold(snapshot.ego);return; }
            skip("no feasible candidate");return;
        }
        std::sort(checked.begin(),checked.end(),[](const auto& a,const auto& b){return a.first<b.first;});
        std::vector<Primitive> valid;
        for(auto& entry:checked) valid.push_back(std::move(entry.second));
        std::size_t selected=0;
        for(std::size_t i=0;i<valid.size();++i) {
            costs_.evaluate(valid[i],progress,target);
            if(valid[i].cost<valid[selected].cost) selected=i;
        }
        if(path_publisher_) path_publisher_->publish(path_builder_.build(valid[selected],snapshot.ego));
        if(tree_publisher_) tree_publisher_->publish(tree_builder_.build(valid,selected,snapshot.ego));
        last_failure_.clear();
    }

private:
    std::vector<Primitive> generate(const PlanningSnapshot& input,
        const std::vector<Reference>& references) const {
        std::vector<Primitive> result;
        const double resolution=std::min(.25,config_.xy_resolution_m);
        for(std::size_t ri=0;ri<references.size();++ri) {
            const auto& reference=references[ri];
            const auto initial=project(reference,{input.ego.x,input.ego.y});
            const auto start=sample(reference,initial.s);
            const double heading_error=wrap(input.ego.heading-initial.yaw);
            if(std::abs(initial.residual)>.02 || std::abs(heading_error)>1.2) continue;
            const double speed=std::max(0.,input.ego.speed*std::cos(heading_error));
            for(const auto duration:config_.frenet_horizon_candidates_s) {
                std::vector<double> terminal{std::max(0.,speed-config_.planning_decel_limit_mps2*duration),
                    speed,speed+config_.planning_accel_limit_mps2*duration};
                terminal.erase(std::unique(terminal.begin(),terminal.end()),terminal.end());
                for(const auto target:terminal) {
                    const auto longitudinal=quartic(initial.s,speed,0.,target,0.,duration);
                    double end=std::min(longitudinal.position(duration),reference.length()-config_.rear_axle_to_front_m);
                    if(reference.terminal_s) end=std::min(end,*reference.terminal_s);
                    if(end-initial.s<.05) continue;
                    // Spatial lateral boundaries also define the correct initial tangent at zero speed.
                    const LateralProfile lateral(initial.d,(1.-start.curvature*initial.d)*std::tan(heading_error),
                        speed,duration,end-initial.s,reference.lane_change);
                    const auto evaluate=[&](double station) {
                        const auto center=sample(reference,station);
                        const double offset=lateral.position(station-initial.s);
                        return lanelet::BasicPoint2d{center.point.x()-std::sin(center.yaw)*offset,
                            center.point.y()+std::cos(center.yaw)*offset};
                    };
                    std::vector<double> stations{initial.s};
                    std::function<bool(double,double,int)> subdivide=[&](double a,double b,int depth) {
                        const auto pa=evaluate(a),pb=evaluate(b),pm=evaluate((a+b)*.5);
                        if((pb-pa).norm()>resolution || (pm-(pa+pb)*.5).norm()>.001) {
                            if(depth>=20) return false;
                            return subdivide(a,(a+b)*.5,depth+1) && subdivide((a+b)*.5,b,depth+1);
                        }
                        stations.push_back(b);return true;
                    };
                    if(!subdivide(initial.s,end,0)) continue;
                    Primitive p;p.reference_index=ri;
                    double length=0.;bool valid=true;
                    for(auto station:stations) {
                        auto point=evaluate(station);
                        if(!p.x_m.empty()) {
                            const double distance=std::hypot(point.x()-p.x_m.back(),point.y()-p.y_m.back());
                            if(distance<1e-8) continue;
                            if(length+distance>config_.max_path_length) {
                                double lo=stations.front(),hi=station;
                                // The last appended point is on this same curve; locate the exact length cutoff.
                                const auto prior=project(reference,{p.x_m.back(),p.y_m.back()});
                                lo=std::max(initial.s,prior.s);
                                for(int k=0;k<35;++k) {
                                    const double mid=(lo+hi)*.5;const auto q=evaluate(mid);
                                    if(length+std::hypot(q.x()-p.x_m.back(),q.y()-p.y_m.back())>config_.max_path_length) hi=mid;
                                    else lo=mid;
                                }
                                station=lo;point=evaluate(station);
                            }
                            length+=std::hypot(point.x()-p.x_m.back(),point.y()-p.y_m.back());
                        }
                        const double h=std::min(.02,(end-initial.s)/100.);
                        const auto before=evaluate(station-h),after=evaluate(station+h);
                        const auto half_before=evaluate(station-h*.5),half_after=evaluate(station+h*.5);
                        const double curvature=signedCurvature(before.x(),before.y(),point.x(),point.y(),after.x(),after.y());
                        const double fine=signedCurvature(half_before.x(),half_before.y(),point.x(),point.y(),half_after.x(),half_after.y());
                        if(!std::isfinite(curvature) || std::abs(curvature-fine)>.01 ||
                            std::abs(curvature)>config_.maximum_curvature_per_m ||
                            std::abs(fine)>config_.maximum_curvature_per_m) { valid=false;break; }
                        p.x_m.push_back(point.x());p.y_m.push_back(point.y());
                        p.yaw_rad.push_back(std::atan2(after.y()-before.y(),after.x()-before.x()));
                        p.z_m.push_back(sample(reference,station).z);
                        p.curvature_per_m.push_back(fine);
                        p.reference_s.push_back(station);
                        p.centerline_offset_m.push_back(lateral.position(station-initial.s));
                        p.lateral_cost+=std::pow(lateral.acceleration(station-initial.s),2)*resolution;
                        if(length>=config_.max_path_length-1e-7) break;
                    }
                    if(!valid || p.x_m.size()<2 || pathLength(p)>config_.max_path_length+1e-6 ||
                        std::hypot(p.x_m.front()-input.ego.x,p.y_m.front()-input.ego.y)>.02) continue;
                    p.stopping=reference.terminal_s && project(reference,{p.x_m.back(),p.y_m.back()}).s>=*reference.terminal_s-.05;
                    if(!kinematicallyFeasible(p,input.ego.heading,config_.wheelbase_m,
                        config_.maximum_curvature_per_m,config_.maximum_steering_rad)) continue;
                    result.push_back(std::move(p));
                }
            }
        }
        return result;
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
    std::string last_failure_;

    void logFailure(const char* prefix, const char* reason) {
        if(last_failure_!=reason) {
            RCLCPP_WARN(rclcpp::get_logger("path_planner"),"%s: %s",prefix,reason);
            last_failure_=reason;
        }
    }
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
            declare_parameter<double>("centerline_deviation_cost_weight", 1.),
            declare_parameter<double>("checkpoint_radius_m", 2.),
            declare_parameter<double>("heading_tolerance_deg", 10.) * pi / 180.,
            declare_parameter<double>("alignment_duration_s", 3.),
            declare_parameter<std::vector<double>>("frenet_horizon_candidates_s", {2., 3., 4.}),
            declare_parameter<double>("xy_resolution_m", .5),
            declare_parameter<double>("time_resolution_s", .5),
            declare_parameter<double>("lane_overlap_check_delay_s", 1.),
            declare_parameter<double>("planning_accel_limit_mps2", 5.45),
            declare_parameter<double>("planning_decel_limit_mps2", 10.37),
            declare_parameter<double>("wheelbase_m", 2.944),
            declare_parameter<double>("maximum_curvature_per_m", 1. / 5.9),
            declare_parameter<double>("maximum_steering_deg", 26.565) * pi / 180.,
            declare_parameter<double>("rear_axle_to_front_m", 3.808),
            declare_parameter<double>("rear_axle_to_rear_m", 1.040),
            declare_parameter<double>("left_extent_m", .943),
            declare_parameter<double>("right_extent_m", .943),
            declare_parameter<double>("vehicle_height_m", 1.507)};
        for (const double weight : {config_.lateral_cost_weight, config_.time_cost_weight,
                 config_.goal_cost_weight, config_.progress_cost_weight,
                 config_.alignment_cost_weight, config_.centerline_deviation_cost_weight}) {
            if (!std::isfinite(weight) || weight < 0.) {
                throw std::invalid_argument("Planner cost weights must be finite and nonnegative");
            }
        }
        if (config_.frenet_horizon_candidates_s.empty() || config_.xy_resolution_m <= 0. ||
            config_.time_resolution_s <= 0. || config_.max_path_length <= 0. ||
            !std::isfinite(config_.lane_overlap_check_delay_s) ||
            config_.lane_overlap_check_delay_s < 0. ||
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
            [this](const EgoStatus& message) {
                if(registry_.onEgo(message) && planner_) {
                    planner_->invalidate(message,"pose discontinuity");
                    route_updater_->reset();
                    route_updater_->tick(registry_.egoOnly());
                    last_planned_stamp_=0;
                }
            });
        dynamic_subscription_ = create_subscription<DynamicStatus>("/dynamic_status", qos,
            [this](DynamicStatus::ConstSharedPtr message) { registry_.onDynamic(std::move(message)); });
        global_publisher_ = create_publisher<std_msgs::msg::Int64MultiArray>("/global_path", qos);
        local_publisher_ = create_publisher<nav_msgs::msg::Path>("/local_path", qos);
        tree_publisher_ = create_publisher<interfaces::msg::SearchTree>("/search_tree", qos);
        route_updater_ = std::make_unique<RouteUpdater>(map_->laneletMap(), *graph_, config_,
            declare_parameter<std::string>("checkpoint_file", ""), global_publisher_);
        route_updater_->setRegistry(registry_);
        planner_ = std::make_unique<FrenetPlanner>(
            *map_, *graph_, config_, local_publisher_, tree_publisher_);

        planner_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1. / planner_hz)), [this] {
                if(!registry_.hasEgo()) return;
                const auto input=registry_.snapshot();
                const double age=registry_.ready() ? now().seconds()-rclcpp::Time(input.ego.header.stamp).seconds() : 1.;
                if(!registry_.ready() || age<0. || age>kMaxSnapshotAgeS) {
                    planner_->skip("missing/stale synchronized input");return;
                }
                if(stamp(input.ego)<=last_planned_stamp_) return;
                route_updater_->tick(input);
                planner_->tick(registry_.snapshot());
                last_planned_stamp_=stamp(input.ego);
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
    std::int64_t last_planned_stamp_{};
    rclcpp::TimerBase::SharedPtr planner_timer_;
};

}  // namespace path_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planner::PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
