#include "hdmap/hdmap.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "check_map hdmap.bin checkpoints.csv\n";
        return 1;
    }
    const auto start = std::chrono::steady_clock::now();
    const auto map = hdmap::hdmap_init(argv[1]);
    const auto loaded = std::chrono::steady_clock::now();
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    const auto graph = lanelet::routing::RoutingGraph::build(map->laneletMap(), *rules);
    const auto errors = graph->checkValidity(false);
    std::size_t invalid_cells = 0;
    std::map<hdmap::LaneletId, std::pair<const hdmap::Cell*, const hdmap::Cell*>> ends;
    for (const auto& cell : map->cells()) {
        auto& end = ends[cell.parent().lanelet_id];
        if (!end.first || cell.parent().index_in_lanelet < end.first->parent().index_in_lanelet) {
            end.first = &cell;
        }
        if (!end.second || cell.parent().index_in_lanelet > end.second->parent().index_in_lanelet) {
            end.second = &cell;
        }
        if (cell.polygon3d().size() < 3 || cell.id() >= map->cells().size()) {
            ++invalid_cells;
        }
        if (cell.previous() && &map->cells().at(cell.previous()->id()) != cell.previous()) {
            ++invalid_cells;
        }
        const auto candidates = map->cellTree().search(cell.boundingBox3d());
        if (std::find(candidates.begin(), candidates.end(), cell.id()) == candidates.end()) {
            ++invalid_cells;
        }
    }
    std::size_t invalid_previous_links = 0;
    for (const auto& lane : map->laneletMap().laneletLayer) {
        const auto upstream = graph->previous(lane, false);
        const auto* expected = upstream.size() == 1 ? ends.at(upstream.front().id()).second : nullptr;
        if (ends.at(lane.id()).first->previous() != expected) {
            ++invalid_previous_links;
        }
    }
    double minimum_speed = std::numeric_limits<double>::infinity();
    double maximum_speed = 0.;
    for (const auto& lane : map->laneletMap().laneletLayer) {
        const auto speed = rules->speedLimit(lane).speedLimit.value();
        minimum_speed = std::min(minimum_speed, speed);
        maximum_speed = std::max(maximum_speed, speed);
    }
    std::ifstream checkpoints(argv[2]);
    std::string line;
    std::getline(checkpoints, line);
    lanelet::ConstLanelets via;
    while (std::getline(checkpoints, line)) {
        std::istringstream row(line);
        std::string sequence, coordinate_x, coordinate_y;
        std::getline(row, sequence, ',');
        std::getline(row, coordinate_x, ',');
        std::getline(row, coordinate_y, ',');
        const auto nearest = lanelet::geometry::findNearest(map->laneletMap().laneletLayer,
            lanelet::BasicPoint2d(std::stod(coordinate_x), std::stod(coordinate_y)), 1);
        if (nearest.empty()) {
            return 2;
        }
        via.push_back(nearest.front().second);
    }
    if (via.size() < 2) {
        return 3;
    }
    const auto route_start = std::chrono::steady_clock::now();
    lanelet::Optional<lanelet::routing::LaneletPath> path;
    for (int iteration = 0; iteration < 100; ++iteration) {
        path = graph->shortestPathVia(via.front(), lanelet::ConstLanelets(via.begin() + 1, via.end() - 1), via.back());
    }
    const auto finish = std::chrono::steady_clock::now();
    std::cout << "{\"load_ms\":" << std::chrono::duration<double, std::milli>(loaded - start).count()
        << ",\"lanelets\":" << map->laneletMap().laneletLayer.size()
        << ",\"cells\":" << map->cells().size()
        << ",\"controllers\":" << map->signalRegistry().size()
        << ",\"minimum_speed_mps\":" << minimum_speed << ",\"maximum_speed_mps\":" << maximum_speed
        << ",\"graph_errors\":" << errors.size() << ",\"invalid_cells\":" << invalid_cells
        << ",\"invalid_previous_links\":" << invalid_previous_links
        << ",\"checkpoint_route_found\":" << (path ? "true" : "false")
        << ",\"path_lanelets\":" << (path ? path->size() : 0)
        << ",\"shortest_path_via_mean_ms\":" << std::chrono::duration<double, std::milli>(finish - route_start).count() / 100.
        << "}\n";
    for (const auto& error : errors) {
        std::cerr << error << '\n';
    }
    return invalid_cells || invalid_previous_links || !errors.empty() || !path ? 4 : 0;
}
