#!/usr/bin/env python3
import json
import math
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

src = sys.argv[1]
dst = sys.argv[2]
duration_sec = float(sys.argv[3])
topics = {'/imu', '/rslidar_Pointcloud2', '/tf_static'}
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
metadata = {x.name: x for x in reader.get_all_topics_and_types() if x.name in topics}
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
for info in metadata.values():
    writer.create_topic(info)
first_ns = None
end_ns = None
counts = {name: 0 for name in topics}
arrival_minus_header = []
header_ns_values = []
while reader.has_next():
    topic, data, stamp_ns = reader.read_next()
    if first_ns is None:
        first_ns = stamp_ns
        end_ns = first_ns + int(duration_sec * 1e9)
    if stamp_ns > end_ns:
        break
    writer.write(topic, data, stamp_ns)
    counts[topic] += 1
    if topic == '/imu':
        msg = deserialize_message(data, Imu)
        header_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        header_ns_values.append(header_ns)
        arrival_minus_header.append((stamp_ns - header_ns) / 1e9)
def percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    pos = (len(s) - 1) * q
    lo = int(math.floor(pos)); hi = int(math.ceil(pos))
    if lo == hi:
        return s[lo]
    return s[lo] * (hi-pos) + s[hi] * (pos-lo)
deltas = [(b-a)/1e9 for a,b in zip(header_ns_values, header_ns_values[1:])]
summary = {
    'source': src,
    'destination': dst,
    'duration_requested_sec': duration_sec,
    'first_bag_timestamp_ns': first_ns,
    'counts': counts,
    'imu_arrival_minus_header_sec': {
        'min': min(arrival_minus_header) if arrival_minus_header else None,
        'median': statistics.median(arrival_minus_header) if arrival_minus_header else None,
        'p99': percentile(arrival_minus_header, .99),
        'max': max(arrival_minus_header) if arrival_minus_header else None,
        'first_20': arrival_minus_header[:20],
    },
    'imu_header_delta_sec': {
        'min': min(deltas) if deltas else None,
        'median': statistics.median(deltas) if deltas else None,
        'p99': percentile(deltas, .99),
        'max': max(deltas) if deltas else None,
        'negative_or_zero': sum(x <= 0 for x in deltas),
        'over_0_2_sec': sum(x > .2 for x in deltas),
    },
}
print(json.dumps(summary, indent=2, ensure_ascii=False))
