#!/usr/bin/env python3
import argparse
import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from re_localization.srv import ReLocalization


def quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


parser = argparse.ArgumentParser()
parser.add_argument('--session-id', required=True)
parser.add_argument('--map-id', default='map2')
parser.add_argument('--session-start-ns', type=int, required=True)
parser.add_argument('--observation-id', type=int, required=True)
parser.add_argument('--min-cloud-ns', type=int, required=True)
parser.add_argument('--x', type=float, required=True)
parser.add_argument('--y', type=float, required=True)
parser.add_argument('--yaw-deg', type=float, default=0.0)
parser.add_argument('--refresh-cloud', action='store_true')
args = parser.parse_args()

rclpy.init()
node = rclpy.create_node('offline_relocalization_probe')
client = node.create_client(ReLocalization, '/re_localization')
if not client.wait_for_service(timeout_sec=20.0):
    raise SystemExit('re_localization service unavailable')

request = ReLocalization.Request()
request.initial_pose = PoseWithCovarianceStamped()
request.initial_pose.header.frame_id = 'map'
request.initial_pose.pose.pose.position.x = args.x
request.initial_pose.pose.pose.position.y = args.y
q = quaternion(math.radians(args.yaw_deg))
request.initial_pose.pose.pose.orientation.x = q[0]
request.initial_pose.pose.pose.orientation.y = q[1]
request.initial_pose.pose.pose.orientation.z = q[2]
request.initial_pose.pose.pose.orientation.w = q[3]
request.session_id = args.session_id
request.map_id = args.map_id
request.session_start.sec = args.session_start_ns // 1_000_000_000
request.session_start.nanosec = args.session_start_ns % 1_000_000_000
request.observation_id = args.observation_id
request.refresh_cloud = args.refresh_cloud
request.min_cloud_stamp.sec = args.min_cloud_ns // 1_000_000_000
request.min_cloud_stamp.nanosec = args.min_cloud_ns % 1_000_000_000

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
if not future.done():
    raise SystemExit('re_localization call timed out')
response = future.result()
pose = response.pose.pose.pose
print(json.dumps({
    'success': bool(response.success),
    'fitness_score': float(response.fitness_score),
    'cloud_stamp_ns': stamp_to_ns(response.cloud_stamp),
    'observation_id': int(response.observation_id),
    'session_id': response.session_id,
    'map_id': response.map_id,
    'message': response.message,
    'pose': {
        'x': pose.position.x,
        'y': pose.position.y,
        'z': pose.position.z,
        'qx': pose.orientation.x,
        'qy': pose.orientation.y,
        'qz': pose.orientation.z,
        'qw': pose.orientation.w,
    },
}, ensure_ascii=False))
node.destroy_node()
rclpy.shutdown()
