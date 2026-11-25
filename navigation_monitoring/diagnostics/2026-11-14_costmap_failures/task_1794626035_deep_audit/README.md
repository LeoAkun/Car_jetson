# task_1794626035 (11:13) bag audit

This directory contains read-only offline scripts and outputs for the 11:13 failed 110→114 run and the 11:22 same-route success comparison. No runtime configuration was changed.

## Bags and time bases

- Failed dedicated bag: `runs/2026-11-14_11-12-06_manual_publish_monitor/navigation_core_bag`
- Failed Nav2 log: `nav2_2026-11-14_11-14-11.log`
- Success long bag: `runs/2026-11-14_11-02-27_manual_publish_monitor/navigation_core_bag`
- Success Nav2 log: `runs/2026-11-14_11-12-06_manual_publish_monitor/nav2_2026-11-14_11-24-10.log` (under the dedicated run directory)

`lethal_events_1113_1122.csv` is a compact extract from the existing authoritative `lethal_events_analysis.json`.

## Static-map conversion used for cross-check

Map YAML: `multi_map_navigation/src/map/3b2mus/3b2mus_clean.yaml`; `resolution=0.1`, `origin=(-92.9,-532,0)`, `negate=0`. For world `(x,y)`:

```
mx = floor((x + 92.9) / 0.1)
my = floor((y + 532) / 0.1)
pgm_row = (H - 1) - my
pixel = PGM[pgm_row, mx]
occupancy = 100 if (255-pixel)/255 > 0.65; 0 if < 0.25; otherwise -1
```

Thus PGM pixel 0 means static occupied and 254 means static free. This mapping is also implemented in `analyze_first_lethal_raw.py` in the parent diagnostics directory.

## Costmap/TF alignment

`analyze_lethal_events.py` chooses `/tf` transforms by bag-arrival timestamp nearest each planner log event, composes `map→odom` and `odom→base_link`, and chooses a published `/global_costmap/costmap_raw` frame by message-header timestamp nearest the event. It records both `global_frame_offset_s` and `global_bag_offset_s`; raw planner memory is 5 Hz while published global snapshots are about 1 Hz, so a large header offset means the snapshot is a stale cross-check rather than exact planner-time memory.

The first 11:14:58.812 event has global frame offset +0.496 s and bag offset +0.562 s; its exact available snapshot has two `raw=254, static=100` boundary cells. PGM coordinates are `(157.350004,-183.049995)` and `(157.350004,-183.149995)`, both pixel 0. Later synchronized snapshots and the final cluster likewise classify raw 254 boundary cells as static occupancy (e.g. 11:16:39.073: 23 cells; 11:17:06.390: 13 cells), with raw253 cells on static-free pixels representing inflated costs.

## Odom/LIO discontinuity evidence

`check_tf_pose_1113.py`, `correlate_odom_jumps.py`, `static_before_after_jump.py` and their `.out` files are the reproducible jump audit. `map→odom` is constant throughout the failed bag (`x=156.8051605, y=-192.3490295, yaw=1.5683263`; 1795 TF samples). Therefore this is not a map→odom relocalization jump.

The causal sequence is visible by bag arrival:

1. LIO mapping correction arrives at 11:16:38.306530. `/lio_sam/mapping/odometry` (and `_incremental`) has a 1.5080 m step between message headers 11:16:36.356398 and 11:16:37.656163.
2. Four milliseconds later, `/odom`/TF jumps: `/odom` header 11:16:38.650534 `(46.61046,5.26911)` → 11:16:38.660534 `(47.33349,6.19983)`, 1.17856 m in 10 ms; `odom→base_link` TF has the same ~1.187 m step.
3. Before the jump (TF bag arrival 11:16:38.299824), static footprint boundary is all free (`0:30`); after the jump (arrival 11:16:38.310491), the quantized pose has static center occupancy 100 and boundary `{0:7,100:23}`. The first dense planner warning follows at 11:16:39.073431. A second ~1.006 m `/odom` jump occurs at header 11:16:39.650534 and still leaves the footprint in static occupancy.

The success run has ordinary correction steps (maximum observed ~0.414 m/10 ms during its action window) and no 1.18 m/1.01 m pair; it recovers from ten transient lethal warnings and reaches both waypoints.

`historical_cross_epoch_evidence.py` and `.out` add same-system historical corroboration from the 2026-11-10 bag, which did record `/odom_incremental`.  A mapping correction there is followed by a 0.665 m/10 ms incremental step and a 1.232 m/10 ms fused `/odom` step; replaying the source `anchor * inverse(queue.front) * queue.back` formula reproduces the selected large-jump events with about `10⁻¹⁴ m` residual.  The all-sample maximum residual is larger because recorder arrival order is not exact executor callback order.  This supports queue/correction-epoch amplification as the highest-probability implementation mechanism, but it is not direct frame-by-frame evidence for 11:13 and the historical binary provenance is unknown.

At the first dense failure, the local costmap is simultaneously clear: 11:16:39.073 has local-frame offset -0.00044 s, local center cost 0, footprint boundary `{0:59}`, and no nearby local lethal cell. At 11:17:06 the local center and boundary are still all zero. The local costmap consumes the unfiltered merged scan and does not contain the global static map. This independently separates the terminal global static-map collision from a near-body lidar mark.

## Log/source semantics

The Smac text comes from `nav2_smac_planner/src/a_star.cpp`: `Starting point in lethal space` is thrown when `_start->isNodeValid()` fails. For Hybrid-A*, that delegates to the full-footprint collision checker; it is not a relocalization status.

The custom obstacle-layer message `缓冲区内有效点数为 0！雷达数据在此处丢失` is invalid instrumentation. In `nav2_costmap_2d/plugins/obstacle_layer.cpp`, `total_points` is initialized to zero but is never incremented; only the other counters are updated. The warning branch therefore runs regardless of real observations. Recorded front, back, merged, and filtered scan topics remain near 30 Hz.

## Interpretation limits

The bag proves the pose/costmap collision mechanism and disproves the exact proposed front-lidar mask as the trigger: the 132–135° / 0.60–0.83 m / transformed-window condition matched zero points at the first warning and throughout the dense final cluster (only two points at 11:16:03). It records `/lio_sam/mapping/odometry_incremental`, but not the high-rate `/odom_incremental` that TransformFusion actually queues, so it cannot directly identify the exact cross-correction queue pair. It also does not identify which internal LIO feature/ICP correspondence caused the 1.508 m mapping correction; that requires LIO debug/scan-matching diagnostics or replay with point clouds.
