# Recovery run: first no-feasible path

Production planner revision 8 SHA256: `4c5703ec5cd43eb64725e8f1aaad21119324d90306d8af66236039ab0abd31cc`. The live run used revision 7; revision 8 differs only in preserving full lateral-convergence distance for explicit adjacent-lane references, so this same-lane case has the same geometry. No production source or configuration was changed during this probe.

## Exact first failure and provenance

Case `known_north_curve_road173_recovery`: planner started at UTC epoch 1789080700.058946, controller at 1789080703.067252. The first empty Path arrived at **1789080713.374068**, with source stamp **1789080713289339454**. Its recorded matching ego was **(830.27392578125, -63.46121597290039, 51.45564270019531)**, heading **0.8316797614097595 rad**, speed **7.587375164031982 m/s**, capture error **0 ms**. The route used was `[28004, 28075, 28078, 29001]`.

This exact ego was reconstructed from `/tmp/sim_fix_20260911/events.jsonl`. Its DynamicStatus was not captured at that stamp, so the probe used the preceding saved DynamicStatus at **1789080711408945837** (1.880394 s older). This limits any time-dependent velocity/occupancy comparison, but **does not affect the demonstrated static footprint rejection**, which precedes those stages. The next fully matched saved ego/DynamicStatus pair, **1789080713449244494**, independently has the same rejection.

## Native original-geometry replay

The temporary probe exposes existing planner methods in a source copy and checks the unchanged geometry, velocity and collision methods. `scale=1` is the production lateral-distance formula. At the exact first-empty ego, all **15 generated candidates fail `roadSample` at point index 0**, the current footprint. Their signed lateral offset is **-0.383825113 m**; heading error is **-0.026701702 rad**. Each finds **6 map cells**, and initial curvature is approximately **0.01050/m**, well below the **0.16949/m** geometric limit. This is a current full-body containment failure against the allowed same-layer route corridor, not missing height cells, insufficient candidate generation, or curvature rejection. Velocity and dynamic collision checks are never reached. The matched following snapshot also rejects all 15 at index 0 (offset **-0.409106773 m**).

This probe checks the permitted route corridor. It does not independently quantify the union of every driving-lane polygon; the broader road-coverage scan is a separate artifact. The actual vehicle's growing error relative to the fixed road reference is independently confirmed by controller analysis.

## Immediately preceding valid path

At source stamp **1789080713089044616**, ego offset is **-0.349837103 m** and heading error **-0.027546506 rad**. The native probe generates 15 candidates: **11 pass**, while 4 fail full-body coverage at **1.422–1.625 m ahead**. Its selected path is **47.6905444 m**, with offset **-0.401950520 m at 2 m ahead**, **-0.434593628 m at 5 m**, **-0.332770715 m at 10 m**, and **-0.010754614 m at 20 m**. Thus the selected geometry initially continues outward before recovering. The captured live path has 257 poses and 47.6905446 m length (within 0.000001 m of the rounded probe result); this probe did not export a separate pointwise comparison, and used the preceding DynamicStatus as disclosed above.

At the earlier fully matched stamp **1789080711408945837**, all 15 candidates pass and the selected path remains 50 m long, but its offset initially grows from **-0.07903 m** to **-0.17162 m at 5 m** and **-0.17285 m at 10 m**. Separating the lateral and longitudinal horizons did not remove this live failure.

## Remaining limitation

The control replay reports all 169 pre-failure frames with a valid reference and successful MPC; error to the latest generated path stays within **8.15 mm / 0.302 degrees**, while fixed-road offset grows to approximately **-0.384 m**. Separate simulator-response measurements show reduced yaw response at speed. These observations support a remaining interaction between vehicle/model response, initial outward heading, and repeatedly regenerated recovery paths. They do **not** prove a unique causal allocation between model mismatch, controller behavior and planner boundary conditions. No further parameter tuning or safety-check weakening was applied. Once the current footprint is already outside the allowed corridor, changing only future convergence distance cannot make point 0 valid.

The additional `scale=0.5` and `scale=0.25` rows in `results.txt` are isolated diagnostic counterfactuals in `/tmp`: they can reduce future outward excursion at earlier poses, but still fail at point 0 in both failed poses. They have no live validation and are not applied or recommended as a proven fix.

## Reproduction artifacts

- `inputs.txt`: five ego poses, timestamps and exact DynamicStatus input paths.
- `live_events.json`: compact unmodified live Path receipt/capture metadata.
- `results.txt`: full candidate rejection counts, first failed points and selected lateral samples.
- `prepare.py`, `planner.cpp`, `probe.cpp`, `config.inc`, `probe`: temporary instrumentation and executable; production source hash above is the reference.
- Matching saved input sources: `/tmp/sim_fix_20260911/snapshots/1789080708329900653.{ego.cdr,dynamic.cdr.gz,json}`, `1789080711408945837.{ego.cdr,dynamic.cdr.gz,json}`, and `1789080713449244494.{ego.cdr,dynamic.cdr.gz,json}`.
- Independent control geometry analysis: `/tmp/sim_fix_20260911/control_recovery_analysis/geometry_combined.jsonl`.

Deferred first-zero stopping/finite-ETA/HOLD edits remain unimplemented following automatic approval-review rejection. This failure occurs in static geometry before that deferred logic.
