#define main path_planner_node_main
#include "../src/path_planner_node.cpp"
#undef main

#include <cassert>

int main() {
    using namespace path_planner;
    lanelet::Id id = 1;
    const auto boundary = [&](double y, const char* subtype) {
        lanelet::LineString3d line(id++, {lanelet::Point3d(id++, 0., y, 0.),
                                        lanelet::Point3d(id++, 100., y, 0.)});
        line.attributes()["type"] = "line_thin";
        line.attributes()["subtype"] = subtype;
        return line;
    };
    const auto a = boundary(0., "solid");
    const auto b = boundary(3.5, "dashed");
    const auto c = boundary(7., "dashed");
    const auto d = boundary(10.5, "solid");
    const auto e = boundary(14., "solid");
    lanelet::LaneletMap map;
    const auto lane = [&](const auto& left, const auto& right, double y) {
        lanelet::Lanelet result(id++, left, right);
        result.attributes()["subtype"] = "road";
        result.attributes()["location"] = "urban";
        result.attributes()["one_way"] = "yes";
        result.setCenterline(lanelet::LineString3d(id++, {
            lanelet::Point3d(id++, 0., y, 0.), lanelet::Point3d(id++, 100., y, 0.)}));
        map.add(result);
        return result;
    };
    const auto first = lane(b, a, 1.75);
    const auto second = lane(c, b, 5.25);
    const auto third = lane(d, c, 8.75);
    const auto blocked = lane(e, d, 12.25);
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    const auto graph = lanelet::routing::RoutingGraph::build(map, *rules);
    assert(graph->left(first) && graph->left(first)->id() == second.id());
    assert(!graph->left(third));
    assert(graph->adjacentLeft(third)->id() == blocked.id());

    const auto centers = centerlinePoints(map, *graph, {first.id(), first.id()});
    assert(centers.first.size() == 2 && centers.second.size() == 4);
    assert(nearestPointSquared({0., 5.25}, centers.first) == 3.5 * 3.5);
    assert(nearestPointSquared({0., 5.25}, centers.second) == 0.);
    assert(nearestPointSquared({0., 8.75}, centers.second) > 0.); // No recursive neighbor.
    assert(nearestPointSquared({50., 1.75}, centers.first) == 2500.); // Points, not segments.
    assert(nearestPointSquared({0., 3.4}, centers.second) <
           nearestPointSquared({0., 3.5}, centers.second));
    assert(nearestPointSquared({0., 3.6}, centers.second) <
           nearestPointSquared({0., 3.5}, centers.second)); // Either side falls toward its center.
    const auto middle = centerlinePoints(map, *graph, {second.id()});
    assert(middle.second.size() == 6); // Both immediate neighbors.
    const auto solid = centerlinePoints(map, *graph, {third.id()});
    assert(nearestPointSquared({0., 12.25}, solid.second) > 0.);
    assert(centerlinePoints(map, *graph, {}).first.empty());

    auto off_center = std::make_shared<Primitive>();
    auto on_center = std::make_shared<Primitive>();
    off_center->g = on_center->g = 10.;
    off_center->h_goal = on_center->h_goal = 2.;
    off_center->h_snap = 1.;
    std::priority_queue<PrimitivePtr, std::vector<PrimitivePtr>, QueueCompare> candidates(
        QueueCompare{1., 10., 1.});
    candidates.push(off_center);
    candidates.push(on_center);
    assert(candidates.top() == on_center);
    assert(!(QueueCompare{1., 10., 0.})(off_center, on_center));
    on_center->h_goal = 3.;
    assert((QueueCompare{1., 10., 1.})(on_center, off_center));
    EgoStatus ego;
    ego.x = 10.; ego.y = 20.; ego.heading = std::acos(-1.) / 2.;
    assert((mapPoint(ego, 2., 0.) - lanelet::BasicPoint2d(10., 22.)).norm() < 1e-6);
}
