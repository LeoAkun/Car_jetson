# IMU preintegration robustness validation (2026-11-11)

No sensor driver, chassis process, navigation task, rosbag recording, or bag
playback was started for this validation.

## Changes under test

- The FDILink driver maps the device microsecond timestamp into ROS time and
  rejects partial serial frames before parsing them.
- IMU and mapping callbacks enqueue messages quickly. One worker owns the
  GTSAM state and processes the messages in timestamp order.
- `imuMaxGapSec` defaults to 0.2 seconds. A larger positive gap resets the
  preintegration epoch without terminating LIO-SAM. Setting it to `0` disables
  positive-gap resets. Non-monotonic samples are still dropped.
- `/lio_sam/imu_preintegration_ready` becomes true after three successful
  optimization updates. The process manager requires this signal before it
  reports LIO-SAM startup success.

## Validation

- `lio_sam` and `fdilink_ahrs` built and installed successfully.
- Process-manager tests: 22 passed.
- Synthetic ROS test used isolated `ROS_DOMAIN_ID=88`, stationary 100 Hz IMU,
  10 Hz correction odometry, and one injected 0.35 second timestamp jump.
- Observed readiness sequence: `true -> false -> true`.
- The node remained alive and emitted no fatal preintegration failure.

The synthetic log is in `synthetic_gap_test.log`. This test confirms recovery
from a timestamp gap and callback decoupling. It cannot validate the serial
device timestamp values or reproduce vehicle vibration, USB faults, LiDAR load,
or the complete outdoor navigation stack; those remain for the next controlled
real-vehicle test.

## Follow-up after the 10:49 field test

The 10:49 test still used the deleted, pre-update driver image. Its LIO-SAM
preintegration became ready, then received 0.232 s and 0.287 s timestamp gaps.
The process manager counted the same not-ready state from its 0.25 s Nav2
startup loop and failed LIO-SAM in less than one second.

The follow-up fix adds a 5 second recovery window after preintegration has once
been ready. Nav2 remains degraded while LIO-SAM rebuilds, and the fast startup
poll no longer increments the ordinary health-failure counter. A process exit
or a recovery timeout still fails immediately. Resetting preintegration also
clears queued LiDAR corrections and rejects corrections older than the new IMU
epoch.

Validation after this change:

- Process-manager tests: 27 passed.
- `lio_sam`, `multi_map_navigation`, and `fdilink_ahrs` built and installed.
- A second isolated synthetic test injected a 0.35 second timestamp jump. The
  ready sequence was `true at 0.413 s -> false at 2.002 s -> true at 2.413 s`.
- The reset discarded one queued pre-gap LiDAR correction. The process stayed
  alive and emitted no fatal GTSAM error. Results are in
  `synthetic_gap_test_after_queue_fix.result.json` and the companion `.log`.
- The live IMU was restarted once. The original parent launch respawned PID
  `65965` from the installed binary, whose SHA-256 matched the installed file,
  and `/imu` had exactly one publisher.

`restart_imu.sh` now uses the same CP2102 by-id path as the production launch
file and waits for the parent launch's `respawn=True` child before it considers
starting a standalone driver. This prevents two drivers from opening the same
serial port during a manual IMU restart.

Operational note: the first script invocation stopped before changing any
process because its old `/dev/wheeltec_FDI_IMU_GNSS` alias was absent. After the
device path was corrected, the then-current script briefly started a standalone
copy while the parent launch was also respawning; that standalone copy exited
with a serial exception and the parent-owned copy remained healthy. The added
respawn wait is the correction for that race.
