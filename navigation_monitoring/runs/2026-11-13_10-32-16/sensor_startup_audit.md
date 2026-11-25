# Sensor and navigation startup audit

## Current state

- No whole-vehicle ROS driver is running. The ROS graph contains only `/rosout` and `/parameter_events`.
- IMU, GNSS, RoboSense, 2D lidar, merged scan, chassis driver and navigation-management processes are absent.
- The RealSense D435i is present on USB. `/dev/ttyS8`, `/dev/ttyUSB0` and `/dev/ttyUSB1` exist.
- Expected stable device aliases `/dev/wheeltec_FDI_IMU_GNSS`, `/dev/laser_front` and `/dev/laser_back` are absent.
- The sensor bridge interface is up at `192.168.1.140`.

## Startup design

`ros2 launch launch_real run.launch.py` starts the whole-vehicle layer. Its launch timers start IMU, chassis, GNSS conversion and the 2D lidar chain at about +1 second, RoboSense at about +2 seconds, and the RealSense depth camera at about +3 seconds.

Publishing a navigation task does not start those drivers. A received task causes `navigation_manager` to start only the task-specific sequence:

`re_localization -> nav2_init_pose -> LIO-SAM -> Nav2`.

The old-host configuration contains a `robot-service.service` design that sleeps 30 seconds after boot and then runs `launch_real`. That system service is not installed on the current host. `/etc/rc.local` also does not launch the robot stack. Therefore the current host does not automatically restore sensor drivers after a reboot.

## Morning timeline

- 10:32: monitoring and rosbag recording began while sensor data was absent.
- 10:36: `task_1794537392` was received, but initial pose remained waiting for GPS.
- 10:45:47: a separate manual `launch_real run.launch.py` invocation started the whole-vehicle stack.
- 10:45:48: IMU, chassis, GNSS conversion, front/rear 2D lidar and merged scan processes started.
- 10:45:49: RoboSense processes started.
- 10:45:50: RealSense started.
- 10:46:05: the multi-map navigation management stack started.
- The sensor layer remained available for `task_1794538015` and `task_1794539326`; both reached Nav2.
- The prior boot ended as a crash around 11:12:35 and the host booted again at 11:13:55. The whole-vehicle drivers were lost.
- 11:17:44: only the multi-map navigation management stack was started.
- 11:18: `task_1794539938` was received, but the absent sensor layer left initial pose waiting for GPS; LIO-SAM and Nav2 were never reached.

## Operator/assistant attribution

The saved run policy states that, for this monitored session, the assistant was responsible only for monitoring and rosbag recording, while the operator was responsible for navigation startup and task publication. The run record marks both `navigation_started_by_assistant` and `task_published_by_assistant` as false.

ROS and Linux launch logs show that the 10:45:47 whole-vehicle launch ran under user `akun`, but they do not identify whether the command came from the operator terminal or an automation terminal. It was a separate launch action and was not caused by task publication.

In earlier sessions before this responsibility split, the assistant did start navigation and publish tasks when explicitly requested by the operator.
