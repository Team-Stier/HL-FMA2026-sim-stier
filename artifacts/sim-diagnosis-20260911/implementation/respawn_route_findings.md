# Active-node respawn at 1789080612: routing result

The cleared global route is correct for this start/goal pair. Current source `IntersectionMonitor::inspect` matches the captured pose `(732.833496,-250.512314,46.900002,-2.472699)` to **lane528793, intersection=true**. Its map centerline z is46.9; heading and footprint match. There is no height or opposite-direction lane mismatch here.

The active final checkpoint `(183.1307878308055,142.04053259506597)` maps to **lane7355**, polygon distance0. Native Lanelet RoutingGraph `shortestPath(528793,7355)` returns **NONE**; same-direction adjacent lane528611 also has no route to7355. Their direct successors are12627 and12504 respectively. This is directed reachability failure for the supplied checkpoint, not evidence that the respawn reset retained an old global path. The exact structural/map boundary responsible for this disconnected directed route has not been isolated in this bounded check.

Before the reset the recorded route was `[2534,3307,4119,4843,5027,5211,5650,6137,7355]`, which the native graph reproduces. The CSV's initial checkpoint is8862, already passed: a route via8862 from2534 is NONE while the recorded route goes directly to7355, consistent with checkpoint_index having advanced. `RouteUpdater::reset` preserves checkpoint_index. The active destination therefore remains7355 after respawn.

The new Ego arrived at1789080612.090659; tracker/controller/planner discontinuity logs were1789080612.091174/.091501/.091779. Global route cleared by1789080612.110099. At1789080612.347939 the local planner reported disconnected/reference projection. With ids empty, `RouteUpdater::tick` treats the current lane as disconnected and invokes shortestPathVia again; the intersection condition does not suppress this search. Empty-route persistence is therefore consistent with repeated unsuccessful routing, not an intersection freeze.

For the next **active-node** respawn verification, keep the current checkpoint CSV and use this reachable pose:

```json
{"start_lane":2534,"pose":{"x":34.9641914367676,"y":45.885799407959,"z":35.5499992370605,"yaw":0.315992802381516},"target_lane":7355,"target":{"x":183.1307878308055,"y":142.04053259506597},"expected_route":[2534,3307,4119,4843,5027,5211,5650,6137,7355]}
```

The native current matcher returns2534 for this pose. This suggestion assumes the active planner's existing checkpoint index is preserved; a newly started planner using a CSV whose first checkpoint is8862 would need to reach8862 first and would not be the same test.

Evidence: `respawn_route_probe.cpp` includes current repository planner source without modifying it, and links the existing cost_check build dependencies. `respawn_route_probe.txt` records matches, target nearest lanes, direct/via shortest paths and aligned poses. No repository files or running nodes were changed.
