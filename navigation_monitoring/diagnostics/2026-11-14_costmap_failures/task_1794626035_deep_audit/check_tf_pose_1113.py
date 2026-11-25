#!/usr/bin/env python3
import math
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/akun/workspace/Car_jetson/navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor/navigation_core_bag"


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def adiff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3"),
    rosbag2_py.ConverterOptions("", ""),
)
types = {x.name: x.type for x in reader.get_all_topics_and_types()}
feedback_topic = "/navigate_to_pose/_action/feedback"
selected = ["/tf", feedback_topic, "/odom", "/cmd_vel"]
reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
classes = {t: get_message(types[t]) for t in selected if t in types}

tf_pairs = Counter()
map_odom = []
feedback = []
odom = []
cmd = []

while reader.has_next():
    topic, raw, t_ns = reader.read_next()
    msg = deserialize_message(raw, classes[topic])
    if topic == "/tf":
        for tr in msg.transforms:
            p, c = tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")
            tf_pairs[(p, c)] += 1
            if (p, c) == ("map", "odom"):
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                map_odom.append((stamp, tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation)))
    elif topic == feedback_topic:
        p = msg.feedback.current_pose.pose
        stamp = msg.feedback.current_pose.header.stamp.sec + msg.feedback.current_pose.header.stamp.nanosec * 1e-9
        feedback.append((stamp, p.position.x, p.position.y, yaw(p.orientation)))
    elif topic == "/odom":
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom.append((stamp, p.position.x, p.position.y, yaw(p.orientation)))
    elif topic == "/cmd_vel":
        # stamped or bare Twist, depending on runtime
        v = msg.twist if hasattr(msg, "twist") else msg
        cmd.append((t_ns * 1e-9, v.linear.x, v.linear.y, v.angular.z))

print("tf pairs", tf_pairs.most_common())
print("counts map_odom feedback odom cmd", len(map_odom), len(feedback), len(odom), len(cmd))


def summarize(name, rows):
    if len(rows) < 2:
        print(name, "insufficient")
        return
    steps = []
    for a, b in zip(rows, rows[1:]):
        dt = b[0] - a[0]
        ds = math.hypot(b[1] - a[1], b[2] - a[2])
        dy = abs(adiff(b[3], a[3]))
        if 0 < dt < 2:
            steps.append((ds, ds/dt, dy, dy/dt, dt, a, b))
    print("\n", name, "range", rows[0][0], rows[-1][0])
    print("top displacement steps")
    for x in sorted(steps, reverse=True)[:12]:
        print(" ds=%.4f v=%.3f dyaw=%.3f wrate=%.3f dt=%.3f from=%s to=%s" % (x[0], x[1], x[2], x[3], x[4], x[5], x[6]))
    print("top yaw steps")
    for x in sorted(steps, key=lambda z:z[2], reverse=True)[:8]:
        print(" ds=%.4f v=%.3f dyaw=%.3f wrate=%.3f dt=%.3f from=%s to=%s" % (x[0], x[1], x[2], x[3], x[4], x[5], x[6]))
    for target in [1794626098.812,1794626109.794,1794626143.093,1794626199.073,1794626226.390]:
        near = min(range(len(rows)), key=lambda i: abs(rows[i][0]-target))
        print(" near", target, rows[max(0,near-2):min(len(rows),near+3)])


summarize("feedback", feedback)
summarize("map->odom", map_odom)
summarize("odom", odom)

if cmd:
    for target in [1794626098.812,1794626109.794,1794626143.093,1794626199.073,1794626226.390]:
        rows=[x for x in cmd if abs(x[0]-target)<0.6]
        print("cmd near",target, rows[:20])
