# Known Issues

## NAV-001: Navigation stack crashes and remains falsely reported as running

- Status: Fixed in source; pending controlled vehicle validation
- Severity: P0 / safety-critical navigation failure
- First confirmed: 2026-07-22
- Reproduced: 2026-07-23 20:51:39 CST
- Reminder: 2026-07-24 09:00 CST

### Impact

During an active multi-map navigation task, the Nav2 component container aborts
and LIO-SAM IMU preintegration can also crash. The robot stops receiving Nav2
velocity commands, but `process_manager` continues to report the navigation
stack as running because the outer `ros2 launch` processes remain alive. The
active waypoint then hangs indefinitely and the stack does not recover.

### Confirmed failure chain

1. Nav2 aborts with signal 6 in OMPL's Dubins implementation:

   ```text
   DubinsStateSpace::distance
   nav2_smac_planner::Smoother::findBoundaryExpansion
   nav2_smac_planner::Smoother::enforceEndBoundaryConditions
   nav2_smac_planner::Smoother::smooth
   nav2_smac_planner::SmacPlannerHybrid::createPlan
   ```

   Failed assertion:

   ```text
   fabs(p * sin(alpha + t) + ca - cb) < 2 * DUBINS_EPS
   ```

2. `lio_sam_imuPreintegration` exited with signal 11 at nearly the same time.
3. The outer LIO-SAM and Nav2 launch processes stayed alive.
4. `ProcessManagerNode.is_process_running()` checked only `Popen.poll()` on the
   outer launch process, so the failed child nodes were not detected.
5. `/cmd_vel` had no publisher and the third waypoint remained pending.

### Evidence from 2026-07-23

- Nav2 crash report:
  `/var/crash/_opt_ros_humble_lib_rclcpp_components_component_container_isolated.1001.crash`
- Nav2 ROS log:
  `/home/akun/.ros/log/component_container_isolated_69365_1784810521545.log`
- LIO-SAM launch log:
  `/home/akun/.ros/log/2026-07-23-20-41-57-781869-yunle-Default-string-69199/launch.log`
- Navigation manager log:
  `/home/akun/.ros/log/python3_68027_1784810393152.log`
- Source location missing the numerical guard:
  `nav2/src/navigation2/nav2_smac_planner/src/smoother.cpp`
- Process liveness check:
  `multi_map_navigation/src/multi_map_navigation/multi_map_navigation/process_manager.py`

At the time of monitoring, the robot's reported speed remained `0.0` for five
consecutive samples and `/cmd_vel` had no publisher. IMU data was still present
at about 100 Hz, but the IMU driver continued to report `head_type error`
frames. The IMU serial integrity issue should be tracked during the fix because
it may contribute to LIO-SAM instability, although it did not directly cause
the confirmed Nav2 assertion.

### Implemented fix (2026-07-29)

1. SMAC smoother boundary expansion now normalizes and clamps Dubins angles,
   rejects non-finite or degenerate poses, uses per-call OMPL state, and rejects
   costmap interpolation outside the map instead of aborting the Nav2 container.
2. A regression test repeatedly exercises the Dubins 0/2pi angle boundary and
   covers NaN, coincident, and out-of-map candidates.
3. `process_manager` now checks required child executables and ROS nodes in
   addition to the outer `ros2 launch` process. Three consecutive ROS graph
   failures are required to reject brief DDS discovery loss; a launch exit is
   rejected immediately.
4. Confirmed stack failure is published on `/process_manager/fault`.
   `navigation_manager` cancels the active goal, publishes zero velocity,
   closes the managed stack, and reports `fault` without resuming the task.
5. The failed launch process group is also terminated directly by
   `process_manager`, so cleanup does not depend solely on the task manager.

### Remaining validation

- Reproduce the recorded route with path smoothing enabled on the vehicle.
- Kill the Nav2 container and LIO-SAM IMU preintegration node during a controlled
  stationary test and verify detection and cleanup within five seconds.
- Capture a core/backtrace for the independent IMU preintegration signal 11.

### Original required fix

1. Add a finite-angle and numerical-tolerance guard before calling OMPL
   `DubinsStateSpace::distance()` from the SMAC smoother, matching the existing
   protection in analytic expansion code.
2. Add a regression test using the recorded boundary poses that trigger the
   assertion.
3. Monitor required child nodes or ROS lifecycle/action availability instead
   of considering only the outer `ros2 launch` process alive.
4. Restart or fail the navigation task deterministically when Nav2 or required
   LIO-SAM child nodes die.
5. Capture a core/backtrace for the independent IMU preintegration signal 11.
6. Add IMU serial error-rate and topic freshness checks before navigation.

### Acceptance criteria

- The recorded path no longer triggers an OMPL assertion under repeated runs.
- Killing the Nav2 container or IMU preintegration node is detected within
  five seconds.
- The task transitions to an explicit failed/recovery state instead of hanging.
- The manager never reports Nav2 or LIO-SAM healthy when required child nodes
  are absent.
- A stopped or stale `/imu` topic prevents navigation startup.

### Temporary operating rule

Do not continue a task after either the Nav2 container or
`lio_sam_imuPreintegration` exits. Stop the residual navigation stack, verify
the robot speed is zero, then relocalize and start a fresh stack only for
controlled testing.
