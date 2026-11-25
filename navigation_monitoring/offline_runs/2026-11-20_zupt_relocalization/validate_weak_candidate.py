#!/usr/bin/env python3
import argparse
import json
import math
import time

import rclpy
from nav2_init_pose.nav2_init_pose import PoseInitNode


def stamp_ns(stamp):
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


parser = argparse.ArgumentParser()
parser.add_argument('--x', type=float, required=True)
parser.add_argument('--y', type=float, required=True)
parser.add_argument('--yaw-deg', type=float, required=True)
parser.add_argument('--prior-x', type=float, required=True)
parser.add_argument('--prior-y', type=float, required=True)
args, ros_args = parser.parse_known_args()

rclpy.init(args=ros_args)
node = PoseInitNode()
captured = {}
original_publish = node.publish_localization_result


def capture(success, score, pose=None, reason=''):
    captured.update({
        'success': bool(success),
        'fitness_score': float(score),
        'reason': reason,
        'observation_id': int(node.observation_id),
        'confirmation_count': int(node.confirmation_count),
        'first_fitness_score': float(node.first_fitness_score),
        'first_cloud_stamp_ns': stamp_ns(node.first_localization_cloud_stamp),
        'final_cloud_stamp_ns': stamp_ns(node.localization_cloud_stamp),
    })
    return original_publish(success, score, pose, reason)


node.publish_localization_result = capture
if not node.re_localization_client.wait_for_service(timeout_sec=30.0):
    raise SystemExit('re_localization service unavailable')
node.localization_deadline = time.monotonic() + node.max_localization_duration
score, pose, cloud_stamp = node._evaluate_candidate(
    args.x, args.y, args.yaw_deg)
node.localization_cloud_stamp = cloud_stamp
initial = {
    'fitness_score': score,
    'cloud_stamp_ns': stamp_ns(cloud_stamp),
    'observation_id': int(node.observation_id),
}
accepted = node.validate_and_accept(
    pose,
    score,
    prior_x=args.prior_x,
    prior_y=args.prior_y,
    prior_distance_limit=15.0,
)
print(json.dumps({
    'initial': initial,
    'accepted': bool(accepted),
    'result': captured,
}, ensure_ascii=False))
node.destroy_node()
rclpy.shutdown()

