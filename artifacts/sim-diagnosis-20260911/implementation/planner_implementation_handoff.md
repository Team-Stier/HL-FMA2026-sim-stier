# Planner implementation handoff (revision 8)

Source SHA256: 4c5703ec5cd43eb64725e8f1aaad21119324d90306d8af66236039ab0abd31cc

Changed only src/path_planner: CMakeLists.txt, package.xml, src/path_planner_node.cpp, test/cost_check.cpp.
Existing nav_msgs/Path + SearchTree topics, source stamps, base_link frame and position.z semantics remain unchanged. No new interface messages or project-package coupling. Boost dependency declaration uses an already installed library.

Implemented:
- Exact-stamp ego ring/dynamic shared pointer matching, newest complete pair, reset cutoff and stale-pair rejection; one planning timer; current/route progression from the same input; direction/height/route-aware lane association; transient reset preserves checkpoint index.
- Distance-based graph-connected references, unique predecessor chain/history, per-reference allowed lanes, adjacent alignment gate, common-route progress/lateral cost and final checkpoint projection.
- Existing Boost cardinal cubic B-spline x/y/z with arc-length lookup; bounded deviation from raw centerline; projection onto the same continuous reference; direct adaptive spatial sampling and h/h2 curvature checks; exact path-length cap.
- Complete footprint coverage by same-layer allowed lane polygons, including interior gaps; relative coordinates, per-reference corridor cache and per-candidate station/cell cache. Empty cell geometry fails closed.
- Conservative prefix-minimum geometry-cost pruning; every selected candidate still receives unchanged velocity calculations and full geometry/occupancy validation. Other-reference lower bound is zero; tie order is original candidate order. SearchTree contains checked valid candidates.
- Lane-keeping lateral recovery ends at min(path_distance,max(6m,v_long*duration)); the remaining longitudinal path continues on the reference. Explicit adjacent lane changes retain the full convergence distance to avoid a stopped-lane-change curvature regression.
- Empty Path and empty SearchTree immediately revoke stale/invalid/no-candidate outputs; no node restarts performed by this agent.

Blocked by automatic approval review, not implemented:
- First zero-cap stop-prefix trimming, finite stopping ETA/HOLD, explicit emergency mode.
- Review confused an older numbered item 5 with the final plan's excluded message item; parent supplied the actual final plan turn ID but re-review rejected it. Parent must obtain explicit scope clarification before this part.
- Verified byte-for-byte: original VelocityPlanner forward/backward speed propagation, stop_at_end handling and ETA loop are unchanged.

Validation:
- Revision 8 colcon build/install succeeded: /tmp/sim_fix_20260911/planner-build8.txt (42.3 s).
- ctest --test-dir build/path_planner --output-on-failure: cost_check passed.
- cost_check covers real Lanelet2 10 cm successor + distance horizon, rear predecessor chain, legal dashed-boundary adjacent gate/common progress, stopped 3.6 m lane change, spline projection/height/curvature, exact stamp arrival ordering/reset cutoff, complete footprint including interior island, path-length-independent progress, prefix-safe pruning, and lateral hold-after-convergence.
- git diff --check passed; no source files outside the assigned package changed by this agent.
- Independent revision-8 pruning/exhaustive comparison passed: 31/31 outputs identical, 4,607 XYZ/quaternion poses exactly equal, 11 empty paths in both; checked valid candidates 264 to 40. Single-pass timing: median 20.40 ms / max 54.74 ms pruned, versus 48.58 / 112.21 ms exhaustive. This input set uses goal=current and does not validate terminal checkpoint or lane-change decisions. Evidence: /tmp/control_planner_pruning_check/summary.json, per_case.json, variant.diff.
- Same-curve live recovery did not resolve road-relative drift. First empty Path at 1789080713.374068: all 15 generated candidates reject the current footprint at point 0. The preceding selected 47.6905 m path initially moves farther outward. No additional source tuning applied. Detailed input provenance and limitations: /tmp/recovery_failure/findings.md.

Early north-curve diagnostic: /tmp/north_geometry/results_earlier.txt. At stamp 1789079837329978300 (d=-0.04084 m), original 50 m path moves to d=-0.08770 m at 15 m ahead; shorter lateral recovery keeps 50 m/15 valid candidates but reaches d=-0.01125 m at 15 m and approximately zero by 20 m. Already-short/off-road snapshots are not repaired by this change.
