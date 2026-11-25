#!/usr/bin/env python3
import collections
import sys
import time
import rosbag2_py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

bag = sys.argv[1]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
reader.set_filter(rosbag2_py.StorageFilter(topics=['/imu','/rslidar_Pointcloud2','/tf_static']))
rclpy.init()
node = Node('isolated_sensor_window_replayer')
imu_pub = node.create_publisher(Imu, '/imu', qos_profile_sensor_data)
cloud_pub = node.create_publisher(PointCloud2, '/rslidar_Pointcloud2', qos_profile_sensor_data)
tf_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
tf_pub = node.create_publisher(TFMessage, '/tf_static', tf_qos)
pubs = {'/imu': (imu_pub, Imu), '/rslidar_Pointcloud2': (cloud_pub, PointCloud2), '/tf_static': (tf_pub, TFMessage)}
counts = collections.Counter()
print('replayer_ready', flush=True)
time.sleep(2.0)
first_stamp = None
start = None
while reader.has_next() and rclpy.ok():
    topic, data, stamp_ns = reader.read_next()
    if first_stamp is None:
        first_stamp = stamp_ns
        start = time.monotonic()
        print(f'first_stamp_ns={first_stamp}', flush=True)
    deadline = start + (stamp_ns - first_stamp) / 1e9
    while rclpy.ok():
        remain = deadline - time.monotonic()
        if remain <= 0:
            break
        time.sleep(min(remain, 0.002))
    pub, cls = pubs[topic]
    pub.publish(deserialize_message(data, cls))
    counts[topic] += 1
    if sum(counts.values()) % 1000 == 0:
        print(dict(counts), flush=True)
print('complete', dict(counts), 'elapsed', time.monotonic()-start if start else 0, flush=True)
time.sleep(1.0)
node.destroy_node()
rclpy.shutdown()
