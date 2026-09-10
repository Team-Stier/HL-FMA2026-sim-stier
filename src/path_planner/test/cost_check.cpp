#define main path_planner_node_main
#include "../src/path_planner_node.cpp"
#undef main

#include <cassert>

int main() {
    using namespace path_planner;
    constexpr double tolerance = 1e-6;

    const auto longitudinal = quartic(3., 4., 0., 8., 0., 2.);
    assert(std::abs(longitudinal.position(0.) - 3.) < tolerance);
    assert(std::abs(longitudinal.position(2.) - 15.) < tolerance);

    const auto lateral = quintic(3.5, 0., 0., 0., 0., 0., 3.);
    assert(std::abs(lateral.position(0.) - 3.5) < tolerance);
    assert(std::abs(lateral.position(3.)) < tolerance);
    assert(std::abs(lateral.acceleration(0.)) < tolerance);
    assert(std::abs(lateral.acceleration(3.)) < tolerance);

    Reference reference;
    reference.points = {{0., 0.}, {10., 0.}, {20., 0.}};
    reference.station = {0., 10., 20.};
    const auto projection = project(reference, {4., 3.});
    assert(std::abs(projection.s - 4.) < tolerance);
    assert(std::abs(projection.d - 3.) < tolerance);
    const auto endpoint = sample(reference, 25.);
    assert((endpoint.point - lanelet::BasicPoint2d(20., 0.)).norm() < tolerance);

    EgoStatus ego;
    ego.x = 10.;
    ego.y = 20.;
    ego.heading = std::acos(-1.) / 2.;
    const auto [x, y] = toBaseLink(ego, 10., 22.);
    assert(std::abs(x - 2.) < tolerance);
    assert(std::abs(y) < tolerance);
}
