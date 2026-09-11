# Final planner pruning equivalence check

Result: **31/31 saved inputs matched exactly** between pruning and exhaustive evaluation of the same frozen final version 8 source.

- Source SHA-256: `4c5703ec5cd43eb64725e8f1aaad21119324d90306d8af66236039ab0abd31cc`.
- Variant difference: only `if(bound>best_cost) break;` is removed in exhaustive; verified in `variant.diff`.
- Path positions XYZ and orientations XYZW use identical replay instrumentation with 17 significant digits. All **4,607 poses** matched exactly, as did path lengths and reference metadata. Maximum XY and length difference: **0 m**.
- Both variants produced empty paths for **11 inputs**; the other **20** selected identical nonempty paths.
- Evaluated valid candidate total: **40 pruned / 264 exhaustive**. SearchTree candidate count deliberately differs and is excluded from winner equality.
- Single-pass runtime median: **20.40 ms pruned / 48.58 ms exhaustive**; maximum **54.74 / 112.21 ms**. These timings are incidental measurements, not an isolated performance benchmark.
- Original source, configuration, replay index, and all referenced EgoStatus/DynamicStatus CDR hashes remained unchanged.
- Build and execution artifacts are confined to this directory. Runs used isolated ROS domains **211** and **212** with `/replay/path` and `/replay/tree` publishers only.

The frozen lower-bound calculation considers every possible accepted prefix and excludes the nonnegative ETA cost. Both variants use the same final cost function, configuration, candidate ordering, lateral recovery logic, and map. This isolates pruning from previous cost/geometry changes.

Scope: these 31 recorded inputs exercise normal routes and invalid/short-path circumstances. The reused harness reconstructs route and history from stored lane IDs and sets `goal_lanes` to the current route head; it does not reconstruct a terminal checkpoint coordinate or lane-change intent. This result is a regression check over the recorded cases, not proof for all possible maps and inputs.

Artifacts:

- `summary.json`: aggregate result and source hash.
- `per_case.json`: per-input geometry equality, point count, length, candidate count, runtime.
- `manifest.json`: frozen source/config/input hashes.
- `planner_frozen.cpp`, `planner_frozen.yaml`: exact source and configuration used.
- `pruned/results.jsonl`, `exhaustive/results.jsonl`: full outputs.
- `build_variants.py`, `compare.py`: reproducible build and comparison scripts.
- `variant.diff`: the sole planner-source difference.
