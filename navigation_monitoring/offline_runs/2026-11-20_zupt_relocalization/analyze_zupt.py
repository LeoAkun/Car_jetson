#!/usr/bin/env python3
import json
import math
import re
import statistics
import sys
from pathlib import Path

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def percentile(values, quantile):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    low = int(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] * (high - position) + ordered[high] * (position - low)


def vector_norm(vector):
    return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)


run_root = Path(sys.argv[1]).resolve()
bag = run_root / "output_bag"
reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
    rosbag2_py.ConverterOptions("cdr", "cdr"),
)

topics = {
    "/odom": [],
    "/odom_incremental": [],
    "/lio_sam/mapping/odometry": [],
    "/lio_sam/mapping/odometry_incremental": [],
}
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic not in topics:
        continue
    message = deserialize_message(data, Odometry)
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    position = message.pose.pose.position
    topics[topic].append(
        (stamp, position.x, position.y, position.z, vector_norm(message.twist.twist.linear))
    )

summary = {"topics": {}}
for topic, rows in topics.items():
    speeds = [row[4] for row in rows]
    displacement = []
    dt = []
    for previous, current in zip(rows, rows[1:]):
        dt.append(current[0] - previous[0])
        displacement.append(
            math.sqrt(sum((current[index] - previous[index]) ** 2 for index in (1, 2, 3)))
        )
    if rows:
        net = math.sqrt(sum((rows[-1][index] - rows[0][index]) ** 2 for index in (1, 2, 3)))
    else:
        net = None
    summary["topics"][topic] = {
        "samples": len(rows),
        "duration_sec": rows[-1][0] - rows[0][0] if len(rows) > 1 else None,
        "speed_mps_max": max(speeds) if speeds else None,
        "speed_mps_mean": statistics.fmean(speeds) if speeds else None,
        "speed_mps_p95": percentile(speeds, 0.95),
        "speed_mps_p99": percentile(speeds, 0.99),
        "step_m_max": max(displacement) if displacement else None,
        "step_m_p99": percentile(displacement, 0.99),
        "net_displacement_m": net,
        "stamp_gap_over_0_2": sum(value > 0.2 for value in dt),
    }

imu_log = (run_root / "logs/lio_sam_imuPreintegration.log").read_text(
    errors="replace"
)
pattern = re.compile(
    r"LIO_ZERO_VELOCITY_UPDATE speed_kmh=([0-9.+-]+) "
    r"velocity_before_constraint=([0-9.+-]+) velocity_after=([0-9.+-]+) "
    r"accel_bias_norm=([0-9.+-]+)"
)
updates = [tuple(map(float, match.groups())) for match in pattern.finditer(imu_log)]
summary["zero_velocity_updates"] = {
    "log_count": len(updates),
    "speed_kmh": [row[0] for row in updates],
    "velocity_before_mps": [row[1] for row in updates],
    "velocity_after_mps": [row[2] for row in updates],
    "accel_bias_norm": [row[3] for row in updates],
}
summary["log_markers"] = {
    "fatal_count": len(re.findall(r"FATAL|fatal", imu_log)),
    "failure_detection_count": imu_log.count("failureDetection"),
    "correction_requires_relocalization_count": imu_log.count(
        "LIO_CORRECTION_REQUIRES_RELOCALIZATION"
    ),
}
print(json.dumps(summary, indent=2, ensure_ascii=False))
