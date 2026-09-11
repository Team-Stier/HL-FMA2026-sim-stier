# Updated north curve: controller follow-up diagnosis

No additional source changes were made. The live run uses the updated control implementation and original Path/Float32 interfaces.

- Initial pose: (797.2449, -86.6392), heading 0.33903; target (856.8892, 99.0126).
- Movement begins at t1789079833.873. Before the first planner failure, every observed state is ACTIVE or ACTIVE_SHORT_PATH; there is no MPC failure, projection rejection or freshness stop.
- At t1789079842.6935 the planner emits an empty Path. The next command at t9842.7208 brakes at -3 m/s². A brief valid Path at t9842.9217 permits one ACTIVE tick, then the next empty Path returns to braking. There is no maximum-steering fallback.
- Final stop: (841.7485, -48.4636), heading 0.93748. Neutral steering during emergency braking is a remaining reason the post-failure vehicle can drift further away from a curved road; it did not cause the initial planner failure.

## Tracking relative to the executed path

The current controller core was replayed over 178 pre-failure commands using the recorded capture pose, current Ego, path, previous emitted steering and carried warm start. All reference preparations and all MPC solves succeeded. Maximum lateral error was 0.02304 m; maximum heading error was 0.9594°. Replay steering differed from the emitted command by at most 0.000436 rad (p95 0.00000163 rad). Observer callback order and wall-time output rate limits explain the small residual; source capture associations are exact.

## Accumulated drift relative to a persistent reference

| Time (epoch suffix) | Route centerline signed displacement |
|---|---:|
| 9838.021 | -0.0572 m |
| 9839.021 | -0.1575 m |
| 9840.021 | -0.3454 m |
| 9841.021 | -0.3824 m |
| 9842.021 | -0.4032 m |

The initial fixed path from t9834.0178 gives almost the same drift: -0.3409 m at t9840 and -0.4032 m at t9842. This is not merely a raw-centerline versus spline discrepancy. The final pre-failure path at t9842.6154 starts 0.4424 m to the right of the route centerline and ends essentially on it, but each replanning step absorbs the current error into a new path start.

The planner agent independently identified that lateral convergence is placed at the candidate endpoint, generally 50 m ahead, and is investigating a shorter independent convergence distance. The observed small local tracking errors together with growing persistent-reference error strongly match that mechanism. No additional controller bug is established by this run, so controller source remains unchanged.

## Exact failure snapshot / artifacts

- `/tmp/sim_fix_20260911/snapshots/1789079842649147125.ego.cdr` and corresponding `.dynamic.cdr.gz`: Ego (835.0701, -57.5466), heading 0.919824, speed 7.64068; route [28004, 28075, 28078, 29001].
- `combined.jsonl`: replay inputs plus controller results for 179 commands including the brief recovery tick.
- `geometry_combined.jsonl`: route centerline and four fixed-path projections.
- `summary.json`: numerical pre-failure tracking and command statistics.
- `replay.cpp`, `replay_input.txt`, `replay_output.jsonl`: temporary reproducible core replay.
