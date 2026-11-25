# 2026-11-03 11:51 standalone docking run

The vehicle was manually returned to P0, approximately 2.5 m from the physical charging face, with its rear facing the dock and autonomous mode enabled. The recorder started before the one-shot docking run. The attempt acquired the dock, reversed from about 2.50 m to 0.69 m, entered `FINAL_APPROACH`, then failed on a temporarily invalid dock observation. It did not reach the 0.45-0.47 m receiver window.

The saved `motion_summary.json` reports:

- 1,693 docking observations: 1,673 valid, 15 warming up, and 5 with no complete face in the gate.
- 1,814 rear scan callbacks and 3,030 local odometry samples.
- The state changed from `APPROACH` to `FINAL_APPROACH` at t=75.11 s. The controller requested stop for `dock_observation_invalid` at t=86.11 s; the standalone action failed at t=87.34 s after stop confirmation.
- The final vehicle message reported zero vehicle and wheel speeds, park status, and autonomous mode. The final ECU command had zero motor, brake enabled, and neutral selected.

`motion_trace.jsonl` contains the recorded topic samples. The saved scan stream is decimated for storage, so it is not a complete per-scan reconstruction of the exact live tracker sequence at the failure instant. `replay.py` and `replay_results.json` provide an offline diagnostic on those retained scans. In the production geometry replay, 295 retained samples were valid and 3 lacked a complete face; lowering the edge-depth split threshold changed the retained-sample result, while wider width and pose gates did not. This is not enough evidence to change the production geometry thresholds, and the live failure should not be attributed to a specific geometric gate from this replay alone.

The shared controller now handles short observation loss by commanding zero while retaining docking control ownership. It permits up to 2.0 s for reacquisition, then requires a valid target and measured linear and angular speeds below their stop thresholds continuously for 0.5 s before resuming. Odometry and the configured travel limit continue to be checked during the hold. Non-perception faults, excessive travel, odometry jumps, or reacquisition timeout remain fail-closed. This revised behavior passed software simulation tests but has not yet been exercised on the vehicle.

The run ended with the vehicle stopped about 0.69 m from the face, outside P0. A new live attempt requires manually restoring P0 first.
