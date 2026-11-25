import math
import sqlite3
import time
from collections import Counter

import rclpy
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import StaticTransformBroadcaster

from offline_common import HERE, ROOT, Runtime

BAG = ROOT / 'bag/rosbag2_2026_04_22-19_12_20/rosbag2_2026_04_22-19_12_20_0.db3'


def test_bounded_real_sensor_bag_through_lio_sam():
    # Immutable read-only SQLite; only these two topics are ever selected/published.
    with sqlite3.connect(f'file:{BAG}?mode=ro&immutable=1', uri=True) as connection:
        topics = {name: (i, kind) for i, name, kind in connection.execute('select id,name,type from topics')}
        imu_id = topics['/imu'][0]
        cloud_id = topics['/rslidar_Pointcloud2'][0]
        start = connection.execute('select min(timestamp) from messages where topic_id=?', (imu_id,)).fetchone()[0]
        end = start + 20_000_000_000
        rows = connection.execute(
            'select topic_id,timestamp,data from messages where timestamp between ? and ? '
            'and topic_id in (?,?) order by timestamp', (start, end, imu_id, cloud_id)).fetchall()
    assert len(rows) > 1000
    with Runtime('slam') as rt:
        static = StaticTransformBroadcaster(rt.node)
        transforms = []
        for parent, child in [('map', 'odom'), ('base_link', 'rslidar')]:
            transform = TransformStamped()
            transform.header.stamp = rt.node.get_clock().now().to_msg()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.
            if child == 'rslidar':
                # The recorded sensor mount and the current launch static TF.
                transform.transform.translation.x = 0.35
                transform.transform.translation.z = 0.51
            transforms.append(transform)
        static.sendTransform(transforms)
        output = HERE / ('slam-output-bag-' + str(time.time_ns()))
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        count = Counter()
        last_odom = {}

        def receive(topic, msg):
            count[topic] += 1
            if isinstance(msg, Odometry):
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                assert all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                last_odom[topic] = (p.x, p.y, p.z)
            writer.write(topic, serialize_message(msg), rt.node.get_clock().now().nanoseconds)

        output_topics = [
            ('/lio_sam/deskew/cloud_deskewed', PointCloud2),
            ('/lio_sam/feature/cloud_corner', PointCloud2),
            ('/lio_sam/feature/cloud_surface', PointCloud2),
            ('/lio_sam/mapping/odometry', Odometry),
            ('/offline/slam_odom', Odometry),
        ]
        for topic, cls in output_topics:
            kind = 'nav_msgs/msg/Odometry' if cls is Odometry else 'sensor_msgs/msg/PointCloud2'
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
            rt.node.create_subscription(cls, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data)
        imu_pub = rt.node.create_publisher(Imu, '/offline/imu', 100)
        cloud_pub = rt.node.create_publisher(PointCloud2, '/offline/raw_cloud', 10)
        params = ROOT / 'slam/src/LIO-SAM/config/params_rslidar.yaml'
        for suffix in ('imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization'):
            rt.start('lio_sam', 'lio_sam_' + suffix, [
                '--params-file', str(params), '-p', 'pointCloudTopic:=/offline/raw_cloud',
                '-p', 'imuTopic:=/offline/imu', '-p', 'odomTopic:=/offline/slam_odom',
                '-p', 'gpsTopic:=/offline/no_gps', '-p', 'savePCD:=false',
                '-p', 'savePCDDirectory:=/car-host-step9.MgehLL/unused-slam-export/',
                '-p', 'loopClosureEnableFlag:=false', '-p', 'numberOfCores:=2'])
        params_client = rt.node.create_client(GetParameters, '/lio_sam_mapOptimization/get_parameters')
        reply = rt.call(params_client, GetParameters.Request(
            names=['savePCD', 'loopClosureEnableFlag', 'extrinsicTrans']))
        assert len(reply.values) == 3 and all(v.type == 1 and not v.bool_value for v in reply.values[:2])
        assert list(reply.values[2].double_array_value) == [0., 0., -0.11]
        rt.wait(lambda: imu_pub.get_subscription_count() >= 2 and cloud_pub.get_subscription_count() >= 1, 15.)
        rt.pump(1.)
        # Shift message header times by one common constant in memory. Raw point
        # coordinates, ring/time fields, IMU units and relative timing are untouched.
        wall_start = time.monotonic()
        new_start = rt.node.get_clock().now().nanoseconds
        shift = new_start - start
        inputs = Counter()
        for topic_id, timestamp, payload in rows:
            target = wall_start + (timestamp - start) / 1e9
            while time.monotonic() < target:
                rt.healthy()
                rclpy.spin_once(rt.node, timeout_sec=min(.005, max(0., target - time.monotonic())))
            cls, pub = (Imu, imu_pub) if topic_id == imu_id else (PointCloud2, cloud_pub)
            message = deserialize_message(payload, cls)
            old_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            new_ns = old_ns + shift
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(new_ns, 1_000_000_000)
            pub.publish(message)
            inputs[cls.__name__] += 1
            rclpy.spin_once(rt.node, timeout_sec=0.)
        rt.pump(3.)
        print('Raw input messages:', dict(inputs))
        print('Recomputed outputs:', dict(count))
        print('Last finite odometry:', last_odom)
        print('New output-only recording:', output)
        assert inputs['Imu'] > 1000 and inputs['PointCloud2'] > 20
        for topic, _ in output_topics:
            assert count[topic] >= (100 if topic == '/offline/slam_odom' else 10), (topic, count[topic])
        del writer
        # Read-only consistency check on the NEW recording, not on the source bag.
        database = next(output.glob('*.db3'))
        with sqlite3.connect(f'file:{database}?mode=ro&immutable=1', uri=True) as check:
            assert check.execute('pragma quick_check').fetchone()[0] == 'ok'
            assert check.execute('select count(*) from messages').fetchone()[0] == sum(count.values())
