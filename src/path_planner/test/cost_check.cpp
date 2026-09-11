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

    // Lateral convergence must stay completed while the longer longitudinal path continues.
    const LateralProfile recovery(-.04,-.005,6.,3.5,50.);
    assert(std::abs(recovery.position(0.)+.04)<tolerance);
    assert(std::abs((recovery.position(1e-4)-recovery.position(-1e-4))/2e-4+.005)<tolerance);
    assert(std::abs(recovery.position(21.))<tolerance);
    assert(std::abs(recovery.position(50.))<tolerance);
    assert(recovery.acceleration(50.)==0.);
    const LateralProfile stationary_recovery(.01,0.,0.,4.,50.);
    assert(std::abs(stationary_recovery.position(10.))<tolerance);

    Reference reference;
    reference.points = {{0., 0.}, {10., 0.}, {20., 0.}};
    reference.station = {0., 10., 20.};
    const auto projection = project(reference, {4., 3.});
    assert(std::abs(projection.s - 4.) < tolerance);
    assert(std::abs(projection.d - 3.) < tolerance);
    const auto endpoint = sample(reference, 25.);
    assert((endpoint.point - lanelet::BasicPoint2d(20., 0.)).norm() < tolerance);

    Primitive centered;
    centered.centerline_offset_m = {0., 0., 0.};
    Primitive offset;
    offset.centerline_offset_m = {2., 2., 2.};
    assert(centerlineDeviationCost(centered) < centerlineDeviationCost(offset));

    Primitive straight;
    straight.x_m = {0., 1., 2.};
    straight.y_m = {0., 0., 0.};
    assert(kinematicallyFeasible(straight, 0., 2.95, 1. / 5.9, 26.565 * std::acos(-1.) / 180.));
    Primitive corner;
    corner.x_m = {0., 1., 1.};
    corner.y_m = {0., 0., 1.};
    assert(!kinematicallyFeasible(corner, 0., 2.95, 1. / 5.9, 26.565 * std::acos(-1.) / 180.));

    Primitive reverse;
    reverse.x_m = {0., 1., 0.};
    reverse.y_m = {0., 0., 0.};
    assert(!kinematicallyFeasible(
        reverse, 0., 2.95, 1. / 5.9, 26.565 * std::acos(-1.) / 180.));

    // A 10 cm lanelet must not limit the horizon or remove predecessor coverage.
    lanelet::LaneletMap tiny_map;
    std::vector<lanelet::Point3d> left_edge,right_edge;
    const std::vector<double> boundaries{0.,10.,10.1,80.};
    lanelet::Id next_id=1000;
    for(const double x:boundaries) {
        left_edge.emplace_back(next_id++,x,1.8,.1*x);
        right_edge.emplace_back(next_id++,x,-1.8,.1*x);
    }
    std::vector<lanelet::Id> tiny_ids;
    for(std::size_t i=1;i<boundaries.size();++i) {
        lanelet::LineString3d left_line(next_id++,{left_edge[i-1],left_edge[i]});
        lanelet::LineString3d right_line(next_id++,{right_edge[i-1],right_edge[i]});
        lanelet::Lanelet lane(next_id++,left_line,right_line);
        lane.attributes()["subtype"]="road";lane.attributes()["location"]="urban";lane.attributes()["one_way"]="yes";
        tiny_ids.push_back(lane.id());tiny_map.add(lane);
    }
    const auto rules=lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,lanelet::Participants::Vehicle);
    const auto graph=lanelet::routing::RoutingGraph::build(tiny_map,*rules);
    PlanningSnapshot tiny_input;tiny_input.ego.x=8.;tiny_input.ego.z=.8;
    tiny_input.current_lane=tiny_ids.front();tiny_input.global_path=tiny_ids;
    ReferenceBuilder references(tiny_map,*graph);
    auto tiny_refs=references.build(tiny_input,55.,6.);
    assert(tiny_refs.references.size()==1);
    assert(tiny_refs.references.front().length()-tiny_refs.references.front().ego_s>54.9);
    assert(tiny_refs.references.front().allowed_lanes.count(tiny_ids.back()));
    tiny_input.current_lane=tiny_ids.back();tiny_input.global_path={tiny_ids.back()};
    tiny_input.ego.x=9.8;tiny_input.ego.z=.98;
    tiny_refs=references.build(tiny_input,55.,6.); // No history: walk unique predecessors through the tiny lane.
    assert(tiny_refs.references.size()==1);
    assert(tiny_refs.references.front().allowed_lanes.count(tiny_ids.front()));
    const auto start_projection=project(tiny_refs.references.front(),{9.8,0.});
    assert(std::abs(start_projection.residual)<1e-4);
    assert(start_projection.s>5.9);

    // The alignment gate controls a real lateral edge; once allowed, that route is the progress frame.
    lanelet::LaneletMap lateral_map;
    lanelet::LineString3d middle(2000,{lanelet::Point3d(2001,0.,1.8,0.),lanelet::Point3d(2002,80.,1.8,0.)});
    middle.attributes()["type"]="line_thin";middle.attributes()["subtype"]="dashed";
    lanelet::LineString3d outer_right(2003,{lanelet::Point3d(2004,0.,-1.8,0.),lanelet::Point3d(2005,80.,-1.8,0.)});
    lanelet::LineString3d outer_left(2006,{lanelet::Point3d(2007,0.,5.4,0.),lanelet::Point3d(2008,80.,5.4,0.)});
    lanelet::Lanelet right_lane(2009,middle,outer_right),left_lane(2010,outer_left,middle);
    for(auto* lane:{&right_lane,&left_lane}) {
        lane->attributes()["subtype"]="road";lane->attributes()["location"]="urban";lane->attributes()["one_way"]="yes";
        lateral_map.add(*lane);
    }
    const auto lateral_graph=lanelet::routing::RoutingGraph::build(lateral_map,*rules);
    assert(lateral_graph->left(right_lane));
    ReferenceBuilder lateral_builder(lateral_map,*lateral_graph);
    PlanningSnapshot lateral_input;lateral_input.ego.x=8.;lateral_input.current_lane=right_lane.id();
    lateral_input.global_path={right_lane.id(),left_lane.id()};lateral_input.goal_lanes={right_lane.id()};
    auto lateral_refs=lateral_builder.build(lateral_input,55.,6.);
    assert(lateral_refs.references.size()==1);
    lateral_input.goal_lanes={left_lane.id()};lateral_refs=lateral_builder.build(lateral_input,55.,6.);
    assert(lateral_refs.references.size()==2);
    assert(lateral_refs.references.front().lane_ids.back()==left_lane.id());
    assert(lateral_refs.references.front().allowed_lanes.count(right_lane.id()));
    assert(lateral_refs.references.front().lane_change);
    const LateralProfile stopped_lane_change(-3.6,0.,0.,4.,50.,lateral_refs.references.front().lane_change);
    assert(stopped_lane_change.position(6.)<-3.);
    assert(std::abs(stopped_lane_change.position(50.))<tolerance);
    for(double x=1.;x<49.;x+=1.) {
        const double k=signedCurvature(x-.1,stopped_lane_change.position(x-.1),x,stopped_lane_change.position(x),
            x+.1,stopped_lane_change.position(x+.1));
        assert(std::abs(k)<1./5.9);
    }


    // Exact stamps must progress even when dynamics arrive before ego or lag by one frame.
    PlanningRegistry registry;
    const auto make_ego=[](int second,int nanosecond,double x) {
        EgoStatus e;e.header.stamp.sec=second;e.header.stamp.nanosec=nanosecond;
        e.x=x;e.speed=2.;return e;
    };
    const auto make_dynamic=[](const EgoStatus& e) {
        auto d=std::make_shared<DynamicStatus>();d->header=e.header;return d;
    };
    const auto e0=make_ego(10,0,0.),e1=make_ego(10,40000000,.08),e2=make_ego(10,80000000,.16);
    registry.onDynamic(make_dynamic(e0));assert(!registry.ready());
    assert(!registry.onEgo(e0));assert(registry.ready());assert(stamp(registry.snapshot().ego)==stamp(e0));
    registry.onEgo(e1);registry.onEgo(e2);registry.onDynamic(make_dynamic(e1));
    assert(stamp(registry.snapshot().ego)==stamp(e1));
    const auto respawn=make_ego(10,120000000,100.);
    assert(registry.onEgo(respawn));assert(!registry.ready());
    registry.onDynamic(make_dynamic(e2));assert(!registry.ready());
    registry.onDynamic(make_dynamic(respawn));assert(registry.ready());
    assert(stamp(registry.snapshot().ego)==stamp(respawn));
    PlanningRegistry nearby;
    const auto near_ego=make_ego(11,0,1.);
    nearby.onEgo(near_ego);
    auto near_dynamic=make_dynamic(near_ego);
    near_dynamic->header.stamp.nanosec=40000000; // 40 ms later than ego
    nearby.onDynamic(near_dynamic);
    assert(nearby.ready());
    assert(stamp(nearby.snapshot().ego)==stamp(near_ego));
    SearchTreeBuilder trees(2.944);
    const auto empty_tree=trees.build({},0,near_ego);
    assert(empty_tree.final_node_index==-1);
    assert(empty_tree.x.empty());
    PathBuilder paths(2.944);
    Primitive hold;hold.x_m={10.};hold.y_m={20.+2.944};hold.yaw_rad={std::acos(-1.)/2.};
    EgoStatus hold_ego;hold_ego.x=10.;hold_ego.y=20.;hold_ego.heading=std::acos(-1.)/2.;
    const auto hold_path=paths.build(hold,hold_ego);
    assert(hold_path.poses.size()==1);
    assert(std::hypot(hold_path.poses.front().pose.position.x,hold_path.poses.front().pose.position.y)<1e-6);

    Reference continuous;
    for(int i=0;i<=40;++i) {
        const double angle=i*.025;
        continuous.points.emplace_back(20.*std::sin(angle),20.*(1.-std::cos(angle)));
        continuous.heights.push_back(40.+i*.05);
        continuous.station.push_back(i==0 ? 0. : continuous.station.back()+
            (continuous.points.back()-continuous.points[continuous.points.size()-2]).norm());
    }
    assert(smoothReference(continuous,0.,continuous.length()));
    const auto center=sample(continuous,8.);
    const auto projected=project(continuous,center.point);
    assert(std::abs(projected.s-8.)<1e-4);
    assert(std::abs(center.z-40.8)<.02);
    const auto left=sample(continuous,8.-1e-5),right=sample(continuous,8.+1e-5);
    assert(std::abs(left.yaw-right.yaw)<1e-4);
    for(const double h:{.2,.1,.05}) {
        const auto a=sample(continuous,8.-h),b=sample(continuous,8.),c=sample(continuous,8.+h);
        const double k=signedCurvature(a.point.x(),a.point.y(),b.point.x(),b.point.y(),c.point.x(),c.point.y());
        assert(std::abs(k-.05)<.006);
    }
    // The same endpoint earns the same route progress regardless of unnecessary path length.
    PlannerConfig costs{};costs.progress_cost_weight=1.;costs.goal_cost_weight=1.;
    Reference common;common.points={{0.,0.},{80.,0.}};common.station={0.,80.};
    Primitive short_path;short_path.x_m={0.,14.};short_path.y_m={0.,0.};short_path.yaw_rad={0.,0.};short_path.eta_s={0.,1.};
    Primitive long_path=short_path;long_path.x_m.back()=50.;
    CostEvaluator evaluator(costs);
    assert(evaluator.evaluate(long_path,common,50.)<evaluator.evaluate(short_path,common,50.));
    Primitive bulge=short_path;bulge.x_m={0.,7.,14.};bulge.y_m={0.,5.,0.};bulge.yaw_rad={0.,0.,0.};bulge.eta_s={0.,.5,1.};
    assert(evaluator.evaluate(bulge,common,50.)==evaluator.evaluate(short_path,common,50.));
    Primitive target_lane_path=long_path;target_lane_path.y_m.back()=3.6;
    const auto& lane_change_progress=lateral_refs.references.front();
    assert(evaluator.evaluate(target_lane_path,lane_change_progress,lane_change_progress.ego_s+30.)<
        evaluator.evaluate(long_path,lane_change_progress,lane_change_progress.ego_s+30.));

    // The pruning bound covers every collision-trimmable prefix, not just the original endpoint.
    Primitive prefixes;prefixes.x_m={0.,1.,2.,3.};prefixes.y_m={0.,.2,.1,0.};prefixes.yaw_rad={0.,.1,-.1,0.};
    prefixes.reference_s={0.,1.,2.,3.};prefixes.centerline_offset_m={0.,.2,.1,0.};
    prefixes.eta_s={0.,1.,2.,3.};prefixes.lateral_cost=2.;
    const double bound=evaluator.lowerBound(prefixes,common,10.);
    for(std::size_t n=2;n<=4;++n) { auto prefix=prefixes;resize(prefix,n);assert(bound<=evaluator.evaluate(prefix,common,10.)+1e-12); }
    prefixes.reference_index=1;assert(evaluator.lowerBound(prefixes,common,10.)==0.);

    const auto rectangle=[](double x0,double y0,double x1,double y1) {
        lanelet::BasicPolygon2d p{{x0,y0},{x1,y0},{x1,y1},{x0,y1}};
        boost::geometry::correct(p);return p;
    };
    const auto covered=[](const lanelet::BasicPolygon2d& shape,const std::vector<lanelet::BasicPolygon2d>& polygons) {
        std::vector<const lanelet::BasicPolygon2d*> pointers;for(const auto& polygon:polygons) pointers.push_back(&polygon);
        return coveredBy(shape,pointers);
    };
    const auto body=rectangle(-2.,-1.,2.,1.);
    assert(covered(body,{rectangle(-3.,-2.,0.,2.),rectangle(0.,-2.,3.,2.)}));
    assert(!covered(body,{rectangle(-3.,-2.,1.,2.)}));
    // All four corners are covered, but an interior island still makes the footprint invalid.
    assert(!covered(body,{rectangle(-3.,-2.,-.2,2.),rectangle(.2,-2.,3.,2.),
        rectangle(-.2,-2.,.2,-.2),rectangle(-.2,.2,.2,2.)}));

    EgoStatus ego;
    ego.x = 10.;
    ego.y = 20.;
    ego.heading = std::acos(-1.) / 2.;
    const auto [x, y] = toBaseLink(ego, 10., 22., 2.944);
    assert(std::abs(x - (2. - 2.944)) < tolerance);
    assert(std::abs(y) < tolerance);
}
