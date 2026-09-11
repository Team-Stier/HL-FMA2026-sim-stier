# Local path diagnosis (read-only, 2026-09-11)

Repository source/tests/configuration were not modified. Temporary reproductions and analysis live in `/tmp/local_path_diagnosis/`. The initial analysis snapshot contains 2,265 selected paths with exact ego timestamp matches from `/tmp/sim_diagnosis_20260911/events.jsonl`. Times below are Unix wall-clock seconds, also shown in Asia/Seoul where helpful. Map coordinates are metres.

## 1. Endpoints really stay attached to lanelet ends

**Confirmed live and in the original generator.**

- `1789070769.1349847–1789070772.525767` (05:06:09.135–05:06:12.526 KST): lanelet **96205** endpoint near **(998.975233, 263.204409)** remains within **0.024916 m** of the native lanelet end for **3.391 s / 69 published paths**. Ego travels **35.258 m** while selected path length shrinks **51.561 → 16.304 m**.
- `1789070711.3196476–1789070712.5623634` (05:05:11.320–05:05:12.562 KST): lanelet **27227** endpoint near **(795.361717, -87.315994)** remains within **0.082144 m** for **1.243 s / 26 paths**. Ego travels **6.385 m** and local length shrinks **7.659 → 1.262 m**.
- `1789070731.3155248–1789070732.4604821`: lanelet **28078**, **1.145 s**, ego **7.864 m**, path **8.700 → 1.191 m**.

Cause: `src/path_planner/src/path_planner_node.cpp:394` builds a reference from the current lane and **one** longitudinal successor; adjacent references at line **420** contain only the adjacent lane. At lines **814–829**, the generator clamps to the reference endpoint and immediately ends the candidate. It does not extend references to the configured 50 m lookahead.

The active checkpoint route contains tiny lanes. Exact-source/map reproduction reports current-plus-successor lengths of **7.5 m for 401650**, **8.0 m for 401647**, **4.44072 m for 410360**, etc.; some adjacent references are **0.5–1.0 m**. At the configured checkpoint spawn (1310.818187,464.331725), multiple horizons and speeds terminate at **(1309.84,485.719)** with the same **21.409 m** remaining reference length.

Evidence: `/tmp/local_path_diagnosis/capture_summary.json`, `/tmp/local_path_diagnosis/result.txt`.

## 2. Lane transition can publish a path starting ahead of the vehicle

**Confirmed live; start point reproduced to 2.5e-8 m.**

At **1789070759.1455138** (05:05:59.146 KST), ego is **(885.206482,172.318573)**, speed **6.642 m/s**, current route begins **33348,99175,96120,96205**. Local path begins **(886.023413,173.065200)**, **1.106719 m ahead** of ego rather than at the rear axle.

Native lane 33348 starts at **(885.820075,173.287685)**. Ego's longitudinal projection is **-1.106719 m** before that reference start; lateral offset is **-0.301407 m**. Clamping projection to `s=0` and rebuilding position from the lateral offset gives **(886.023413104,173.065199614)**, matching the recorded first point within **2.455e-8 m**.

Cause: `project()` clamps its segment ratio to `[0,1]` at **line 342**, retaining only signed normal offset at **350**. `generate()` uses this clipped station and lateral offset at **794–820**. When current-lane selection advances while the rear axle remains behind the next lane start, the longitudinal residual disappears. `kinematicallyFeasible()` starts at the candidate's first segment, so it does not verify the missing ego-to-first-waypoint segment.

The measured start discontinuity is established; any resulting control failure requires correlation with the controller diagnostics.

## 3. Small convex lateral excursions are present even on nearly straight reference geometry

**Confirmed geometric observation; no claim that every such excursion is unnecessary.**

At **1789070763.1898332** (05:06:03.190 KST), ego **(913.757568,196.719223)**, heading **0.704772 rad**, speed **8.824726 m/s**, actual current lane **99175** (the frozen global route still starts at 33348) has a nearly straight forward reference. Signed offset starts **-0.026505 m**, rises to **+0.247063 m**, then returns to **0 m** over a **39.053 m** path. This is approximately **25 cm of lateral overshoot** relative to the native reference, not merely road curvature. Replaying the original `IntersectionMonitor` and `FrenetPlanner::generate()` with this captured ego state reproduces all **81 published points** within **8.43e-7 m**. The initial route-based sparse analysis measured a lower 24.3 cm peak; the exact-source full-point reproduction supersedes it.

An exact-source isolated reproduction on a perfectly straight reference, centered ego, **10 m/s and 5° heading error**, produces feasible candidate humps of **0.322–0.678 m**, followed by a return to zero lateral offset. This follows directly from **lines 793–807**: `lateral_speed = ego.speed * sin(heading_error)` and a quintic terminal condition `d(T)=d'(T)=d''(T)=0`. The initial lateral velocity must be preserved, so a hump itself is not proof of an incorrect polynomial. Whether the excursion is excessive depends on lateral-motion tuning, reference choice and candidate cost. Geometry/cost code does not impose monotonic lateral convergence.

Evidence: `/tmp/local_path_diagnosis/bulge_summary.json`, `/tmp/local_path_diagnosis/result.txt`. The final exact-source replay uses the actual footprint-selected current lane, accounting for the frozen route in the intersection. Replay results are in `/tmp/local_path_diagnosis/match_result.txt`. A compact exact-data SVG is `/tmp/sim_diagnosis_20260911/path_evidence.svg`.

## 4. Slopes silently remove speed and collision coverage farther along valid road

**Confirmed live, separated from XY road departure.**

For **114 sampled paths / 2,212 sampled vehicle footprints** (every 20th path, every 4th point), **300 footprints** have **zero** cells at the planner's fixed ego-Z query interval but **all 300** overlap native road cells when the same footprint is queried across all elevations. **No sampled footprint** has zero XY road overlap. This does not prove full footprint containment, only that XY overlaps exist.

- Uphill, **1789070680.9769566** (05:04:40.977): ego **(576.231201,-132.358582,z42.376221)**. At **19.198 m** ahead, footprint **(593.540077,-124.069256)** overlaps lane **34407** cells at **z44.384968–44.964519**. Fixed ego-Z query returns nothing.
- Downhill, **1789070774.1778646** (05:06:14.178): ego **(998.683411,262.928650,z52.126160)**. At **17.439 m** ahead, footprint **(1012.640981,274.037357)** overlaps lane **96736** cells at **z51.427283–51.615210**. Fixed ego-Z query again returns nothing.

Cause: both `VelocityPlanner::plan()` (**lines 553–555**) and `CollisionValidator::firstCollision()` (**lines 607–609**) pass `snapshot.ego.z` through `snapshot.ego.z + vehicle_height_m` at **every future XY point**. `src/hdmap/include/hdmap/cell_tree.hpp:106` adds ±0.5 m tolerance, which is still exceeded on these grades. No-overlap leaves local speed cap at infinity and skips occupancy/lane rejection. These are definite blind regions in planning; the sample does not establish an actual obstacle collision or missed signal caused by them.

Evidence: `/tmp/local_path_diagnosis/coverage_summary.json`.

## 5. A footprint completely outside the road also passes these checks

**Confirmed isolated original-code reproduction.**

A synthetic two-point straight path **(-100000,-100000)→(-99990,-100000)**, ego speed zero and no overlapping cells, passes `VelocityPlanner::plan()` and `CollisionValidator::firstCollision()`: `velocity_valid=1`, end speed **10.4403 m/s**, `collision_index=2` for **2 points**, `forbidden_lane=0`.

Cause: the collision validator at **lines 607–631** rejects overlapping forbidden cells, but it never requires any road coverage or checks the fraction of vehicle footprint outside drivable polygons. A fully outside footprint gives an empty loop; partially outside footprints can also pass if remaining hits belong to allowed lanes. The internal speed planner similarly treats missing cells as no cap. This establishes a protection gap. It does **not** establish that the live simulator's curb excursion was caused by this exact branch.

The isolated harness creates no ROS node and sends no messages. It includes a `/tmp` copy of the planner with only `private:` visibility changed to `public:` for access; algorithm statements are unchanged. Original snapshot and compiled harness: `/tmp/local_path_diagnosis/planner_snapshot.cpp`, `diagnose.cpp`, `diagnose`.

## 6. The 50 m limit is exceeded by one full temporal sample

**Confirmed live and isolated.**

Of the initial **2,265** selected paths, **711** exceed **50.001 m** and the maximum is **64.243124 m**, with active `max_path_length=50.0`, `time_resolution_s=0.5`. An isolated straight-road reproduction gives **54.4302 m** despite the same limit.

Cause: `appendWaypoint()` at **line 817** appends an entire nominal-time jump, spatially subdivided by **lines 516–527**, and only then **line 825** checks total length. This is an overshoot of the intended lookahead rather than a guarantee of unsafe behavior.

## Additional observed static behavior

- Planner returns without publishing an empty/invalid path when prerequisites, geometry, velocity or collision validation leave no valid candidate: **lines 756,777,779**. Consumers must age out the previous path; no explicit diagnostic reason is emitted from those branches.
- Planned speed/ETA are used internally for collision scoring but `PathBuilder::build()` at **699–713** publishes only geometry. Controller speed behavior therefore must be checked in its own speed-limit/control path; internal stop-at-end velocity does not itself command braking.
- No repo graph refresh was necessary: repository code remained unchanged.

## 7. Body containment during the observed control failures

Vehicle rectangle uses the planner dimensions: front extent 3.808 m, rear extent 1.040 m, left/right extent 0.943 m; total footprint area **9.143328 m²**. A temporary C++ Boost Geometry program intersects each recorded rectangle with all nearby `TrafficRules::canPass(Vehicle)` lane polygons and computes their union, avoiding overlap double counting. This is an XY native-map check; it does not identify physical sidewalk surfaces absent from the map.

- **First-run INVALID_PATH, t1789070803–1789070818:** all **66** sampled poses (approximately every 0.2 s) have **100%** vehicle-body coverage in the native drivable-road union, within floating-point tolerance. At final **(1120.405884,438.596130)**, heading **1.845389 rad**, speed zero, coverage is the union of lanes **398540 and 398690**. At an earlier invalid-path pose **(1120.911743,436.801117)**, coverage includes **451363,452050,398540,451993,398690**. All four corners and rear axle are on drivable polygons. This recorded incident establishes path/lane deviation and subsequent stop, **not a measured sidewalk exit**.
- **Natural red-stop pose supplied by parent:** **(601.33795,-119.50121,z45.2575)**, heading **0.76112 rad**, speed zero. Entire body remains inside the road union of **34407 + 34191**; all four corners and rear axle are covered. This is a lane boundary crossing, **not road departure** according to the native map.
- **After deliberate relocation at t1789071055.996540:** among **175** sampled poses through t1789071095, **7** show more than 1% uncovered body area. The maximum is **16.176% (1.479 m²)** at **t1789071063.479660**, ego **(711.852722,-267.927399)**, heading **-1.781354 rad**, speed **5.093193 m/s**, overlapping lanes **12872 and 12627**. All four corners and the rear axle are still inside drivable polygons; the uncovered portion is an interior gap between the lane polygons. This is not a complete road exit or front-corner departure. Whether that interior map gap corresponds to physical sidewalk/median or a map seam requires source geometry comparison. By t1789071094.962090, the stopped body at **(711.088501,-272.082825)** is again fully covered by lanes **12872 + 12995**.

An independent 80 × 40 grid of native point-in-lane checks found **518/3,200 = 16.1875%** uncovered area at the maximum relocation pose, agreeing with the polygon-union result. Evidence: `/tmp/sim_diagnosis_20260911/containment_summary.json`, `containment_results.jsonl`, and temporary `/tmp/local_path_diagnosis/containment.cpp`.
