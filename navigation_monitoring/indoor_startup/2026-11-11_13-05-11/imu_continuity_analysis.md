# IMU continuity analysis — 2026-11-11_13-05-11

Read-only analysis of `imu_samples.jsonl` and `imu.log`; no process was started or modified.

## Aggregate

| metric | value |
|---|---:|
| samples | 11494 |
| received span | 114.881 s |
| received rate | 100.043 Hz |
| received interval p50 / max | 10.012 / 19.692 ms |
| header interval p50 / max | 10.000 / 320.000 ms |
| header interval >0.2 s | 1 |
| received interval >0.2 s | 0 |
| header interval <=0 | 0 |
| nonfinite accel / gyro | 0 / 0 |

Header minus received starts at `-0.001502 s` and settles around median `+0.353951 s` (p01 `+0.350711`, p99 `+0.357205`; max `+0.357684`). This is a future-dated header relative to receipt after startup.

## Phase breakdown

| phase | samples | received Hz | recv max | header max | header >0.2 | bad quaternion norm |
|---|---:|---:|---:|---:|---:|---:|
| sensor_baseline | 1933 | 100.225 | 14.164 ms | 320.000 ms | 1 | 2 |
| lio_start | 309 | 100.003 | 19.692 ms | 10.039 ms | 0 | 0 |
| nav2_start | 9252 | 100.006 | 15.164 ms | 10.043 ms | 0 | 0 |

## Concrete samples

- Line 6 (`2026-11-11 13:05:14.213032659+08:00`): receive interval `1.496 ms`, header interval `320.000 ms`; header-minus-receive `+0.350047 s`. This is the only header >0.2 s event.
- Quaternion anomaly line 1 (`2026-11-11 13:05:14.204581881+08:00`): norm `6.86973e+11`; previous/next context is included in `imu_continuity_analysis.json`.
- Quaternion anomaly line 6 (`2026-11-11 13:05:14.213032659+08:00`): norm `1.17951e+32`; previous/next context is included in `imu_continuity_analysis.json`.

## Driver log correlation

`[INFO] [1794373514.202403649] [ahrs_bringup]: Serial Port initialized: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
`[INFO] [1794373514.202690536] [ahrs_bringup]: ahrsBringup::processLoop: start`
`[WARN] [1794373514.210352023] [ahrs_bringup]: check crc16 failed (ahrs), total_crc_errors=1`
`[INFO] [1794373671.516454773] [rclcpp]: signal_handler(SIGINT/SIGTERM)`
`[WARN] [1794373671.522453096] [ahrs_bringup]: Short IMU serial read for frame type: received=0 expected=1 total_short_reads=1`

The CRC warning is temporally near the early header jump in this run; ROS samples alone cannot prove that the CRC event caused the published quaternion or timestamp discontinuity. CRC-invalid frames are rejected in the driver, while this analysis only sees accepted `/imu` messages.

## Interpretation

- Arrival transport is continuous at approximately 100 Hz in both runs; no received gap exceeded 0.2 s and no header timestamp regressed.
- Both runs independently show a one-off device/header jump of 0.32–0.34 s during `sensor_baseline`, followed by a stable approximately 0.35–0.36 s future offset. This is evidence of a device timestamp discontinuity/epoch adjustment before LIO/Nav2, rather than executor starvation.
- The second run has two bad quaternion samples during the same startup window; the first is line 1 and the second is line 6. The first run has none. All acceleration and angular velocity fields in accepted samples are finite.
- These files do not expose raw serial frames, so they cannot independently validate CRC coverage or identify whether the malformed quaternion originated in the AHRS payload, frame association, or driver state.
