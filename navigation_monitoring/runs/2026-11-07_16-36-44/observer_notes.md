# Read-only observer notes

Observed text logs only. No topics, services, parameters, drivers, mode controls, or running rosbag databases were accessed or changed.

- Run: `2026-11-07_16-36-44`; management log is a symlink to the previous run, so only entries at/after epoch `1794040604` were considered.
- Boot: `ca32f49f-62a3-4e77-aee9-af55c9ac556a` at monitor start and subsequent `/proc` read. No boot change observed.
- `16:36:44.565` (+08:00): navigation manager accepted `task_1794040604`, map `3b2mus`, route `113 → 106 → 107`; began waypoint 1/3 and stack startup.
- `16:36:48.078`: GPS starting point `(104.19, -6.42)`; pose initialization started PSO with 30 particles, radius 10 m. All 30 initial particle scores and all six iteration scores were `inf`.
- `16:36:49.215`: `PSO搜索无有效结果`.
- `16:36:49.785`: monitor recorded `process_fault`: `nav2_init_pose: launch process exited with code 0`; robot state changed to `fault` at `16:36:49.790`, status became idle/0 at `16:36:49.960`.
- Navigation manager stopped waiting for `map -> odom` and reported stack startup failure. Localization startup processes were shut down by the management stack. LIO-SAM and Nav2 never appeared in monitor process snapshots; no `nav2_*.log` exists in this run.
- All recorded vehicle snapshots show speed `0.0 km/h`, steering `0.135°`, drive mode `1`, `is_autodrive=true`, zero `cmd_vel`, no goal statuses. There was no captured mode transition. The `running` task state was brief startup activity and does not establish vehicle motion.
- No waypoint completion was observed; waypoint 113 was not reached in this run.
- `16:37:06.434`: monitor terminal result `fault`, `protective_abort=false`; monitor stopped at `16:37:06.438`.
- Recorder text log shows successful start and topic subscriptions. No recorder error was present when inspected; the database itself was not opened.

The root agent was notified immediately after the fault was seen. This observer did not attempt recovery or resubmission.
