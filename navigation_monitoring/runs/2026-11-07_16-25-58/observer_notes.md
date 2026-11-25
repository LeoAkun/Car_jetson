# Read-only retry observer notes

Observation ended 2026-11-07 16:30:52 +08:00 after this navigation attempt reached a terminal state. Observer only read logs, monitoring JSONL, current_run_dir and /proc/sys/kernel/random/boot_id. No control topic, service, process-manager mutation, or code change was issued by this observer.

- Run: `2026-11-07_16-25-58`; task: `task_1794039959`.
- Requested route from manager: map `3b2mus`, waypoints `113 -> 106 -> 107`.
- Boot ID at all checks: `ca32f49f-62a3-4e77-aee9-af55c9ac556a`. No third reboot observed.
- 16:26:52: Nav2 critical nodes/lifecycle ready; first goal accepted.
- 16:27:50.001: manager confirmed arrival at waypoint 1/3 (113), with SUCCEEDED result.
- 16:27:56.009: manager began waypoint 2/3 (106); its goal accepted 16:28:02.069. No arrival at 106 or 107 was observed.
- 16:27:18.127: controller optimizer failure and follow_path abort.
- 16:28:08/10 and 16:29:11/12: planner rejected path to (115.12, -143.71), reporting starting point in lethal space; follow_path halt result failures also occurred.
- 16:29:43.820 and 16:29:50.034: repeated optimizer failures; 16:29:50.075 backup recovery failed.
- 16:30:10.823: Failed to make progress, followed by follow_path abort. 16:30:26/27: another lethal-space planning failure sequence.
- At 16:30:21 and later, chassis telemetry reported drive_mode=3 and is_autodrive=false while speed was nonzero. This observation alone does not identify the source of motion.
- 16:30:43.311: manager reported `operator_stop_route_deviation_20261107`.
- 16:30:45.519: manager reported navigation task terminated because of the injected stack-fault reason and waiting for a new task.
- 16:30:48.418 snapshot: robot_state=fault, robot_status=0, goal_statuses=[4,5], last observed cmd_vel=[0.2645,0.0] (already about 5 s old, not a fresh command); chassis speed=4.2 km/h, drive_mode=3, is_autodrive=false. Do not interpret this as autonomous route completion.

The Nav2 log repeatedly reported costmap zero valid point buffers despite the monitor reporting scan around 30 Hz. These report different stages of the sensor pipeline and are not equivalent claims.

Nav2 log event counts at observer archive time:
- Optimizer fail to compute path: 3
- Failed to make progress: 1
- Starting point in lethal space: 6
- backup failed: 1
- costmap zero valid points: 227
- control loop rate warning: 43

Evidence: `management_stack.log`, `nav2_2026-11-07_16-26-35.log`, and `monitor.raw.jsonl` in this run directory. Shutdown tracebacks follow the operator-stop termination; the notes do not attribute the earlier driving issues to those later tracebacks.
