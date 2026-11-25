#!/usr/bin/env python3
"""Audit newly accepted scan corrections against the production static map."""

from bisect import bisect_left
from collections import Counter
from pathlib import Path
import json
import math
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from scipy.spatial.transform import Rotation, Slerp
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from multi_map_navigation_msgs.msg import LioCorrectionEvent


ROOT = Path('/home/akun/workspace/Car_jetson')
sys.path.insert(0, str(
    ROOT / 'multi_map_navigation/src/multi_map_navigation'))
from multi_map_navigation.static_footprint_gate import (  # noqa: E402
    StaticMapFootprintChecker,
)

ORIGINAL = ROOT / (
    'navigation_monitoring/runs/2026-11-23_11-41-36_manual_publish_monitor/'
    'navigation_core_bag_segment_2')
REPLAY = Path('/tmp/lio-zupt-timestamp.nHPZb0')
MAP_YAML = ROOT / 'multi_map_navigation/src/map/map2/map2_clean.yaml'
WINDOWS = ('1', '3', '5')
NS = 1_000_000_000


def stamp_ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def matrix_from_parts(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()
    result[:3, 3] = [translation.x, translation.y, translation.z]
    return result


def pose_matrix(pose):
    return matrix_from_parts(pose.position, pose.orientation)


def nearest(rows, stamps, target):
    index = bisect_left(stamps, target)
    candidates = []
    if index < len(rows):
        candidates.append(rows[index])
    if index:
        candidates.append(rows[index - 1])
    return min(candidates, key=lambda row: abs(row[0] - target))


def topic_rows(path, topic, message_type):
    result = []
    for shard in sorted(path.glob('*.db3')):
        with sqlite3.connect(
                shard.resolve().as_uri() + '?mode=ro&immutable=1',
                uri=True) as database:
            row = database.execute(
                'SELECT id FROM topics WHERE name=?', (topic,)).fetchone()
            if row is None:
                continue
            for _, data in database.execute(
                    'SELECT timestamp, data FROM messages WHERE topic_id=? '
                    'ORDER BY timestamp', (row[0],)):
                message = deserialize_message(data, message_type)
                result.append((stamp_ns(message.header.stamp), message))
    return sorted(result)


def original_transforms():
    map_odom = []
    odom_base = []
    for shard in sorted(ORIGINAL.glob('*.db3')):
        with sqlite3.connect(
                shard.resolve().as_uri() + '?mode=ro&immutable=1',
                uri=True) as database:
            row = database.execute(
                "SELECT id FROM topics WHERE name='/tf'").fetchone()
            if row is None:
                continue
            for (data,) in database.execute(
                    'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',
                    (row[0],)):
                message = deserialize_message(data, TFMessage)
                for transform in message.transforms:
                    stamp = stamp_ns(transform.header.stamp)
                    matrix = matrix_from_parts(
                        transform.transform.translation,
                        transform.transform.rotation)
                    key = (transform.header.frame_id, transform.child_frame_id)
                    if key == ('map', 'odom'):
                        map_odom.append((stamp, matrix))
                    elif key == ('odom', 'base_link'):
                        odom_base.append((stamp, matrix))
    map_odom.sort()
    odom_base.sort()
    return map_odom, odom_base


def planar_pose(matrix):
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return float(matrix[0, 3]), float(matrix[1, 3]), yaw


def check_matrix(checker, matrix):
    return checker.check_rectangular_footprint(*planar_pose(matrix))


def interpolate(start, finish, count=31):
    rotations = Rotation.from_matrix(np.stack([
        start[:3, :3], finish[:3, :3]]))
    slerp = Slerp([0.0, 1.0], rotations)
    for fraction in np.linspace(0.0, 1.0, count):
        matrix = np.eye(4)
        matrix[:3, :3] = slerp([fraction]).as_matrix()[0]
        matrix[:3, 3] = (
            start[:3, 3] + fraction * (finish[:3, 3] - start[:3, 3]))
        yield matrix


def main():
    checker = StaticMapFootprintChecker(MAP_YAML)
    map_odom, odom_base = original_transforms()
    map_stamps = [row[0] for row in map_odom]
    base_stamps = [row[0] for row in odom_base]
    report = {}

    for window in WINDOWS:
        old_path = REPLAY / f'gate15seq-{window}-rate1.0/output'
        new_path = REPLAY / f'gate30seq-{window}-rate1.0/output'
        old_events = topic_rows(
            old_path, '/lio_sam/correction_event', LioCorrectionEvent)
        new_events = topic_rows(
            new_path, '/lio_sam/correction_event', LioCorrectionEvent)
        mapping = topic_rows(
            new_path, '/lio_sam/mapping/odometry', Odometry)
        mapping_by_stamp = {
            stamp: pose_matrix(message.pose.pose) for stamp, message in mapping}
        new_by_stamp = {stamp: message for stamp, message in new_events}
        changed = []
        for stamp, message in old_events:
            if (message.source == LioCorrectionEvent.SOURCE_SCAN_TO_MAP
                    and not message.accepted
                    and not message.reason.startswith('scan-to-map skipped')
                    and stamp in new_by_stamp
                    and new_by_stamp[stamp].accepted
                    and stamp in mapping_by_stamp):
                changed.append(stamp)

        anchor = None
        for stamp, new_lio_base in mapping_by_stamp.items():
            original_map_odom = nearest(map_odom, map_stamps, stamp)
            original_odom_base = nearest(odom_base, base_stamps, stamp)
            if (abs(original_map_odom[0] - stamp) <= 250_000_000
                    and abs(original_odom_base[0] - stamp) <= 250_000_000):
                original_map_base = (
                    original_map_odom[1] @ original_odom_base[1])
                legal, _ = check_matrix(checker, original_map_base)
                if legal:
                    anchor = (stamp, original_map_base @ np.linalg.inv(new_lio_base))
                    break
        if anchor is None:
            raise RuntimeError(f'window {window}: no legal common map/LIO anchor')
        map_lio = anchor[1]

        outcomes = Counter()
        records = []
        for stamp in changed:
            original_map_odom = nearest(map_odom, map_stamps, stamp)
            original_odom_base = nearest(odom_base, base_stamps, stamp)
            tf_gap_ms = max(
                abs(original_map_odom[0] - stamp),
                abs(original_odom_base[0] - stamp)) / 1e6
            original_map_base = original_map_odom[1] @ original_odom_base[1]
            target_map_base = map_lio @ mapping_by_stamp[stamp]
            original_legal, original_reason = check_matrix(
                checker, original_map_base)
            target_legal, target_reason = check_matrix(checker, target_map_base)
            interpolation_failures = []
            for index, candidate in enumerate(interpolate(
                    original_map_base, target_map_base)):
                legal, reason = check_matrix(checker, candidate)
                if not legal:
                    interpolation_failures.append((index, reason))
            if original_legal and target_legal and not interpolation_failures:
                outcomes['fully_static_legal'] += 1
            else:
                outcomes['static_illegal'] += 1
            event = new_by_stamp[stamp]
            records.append({
                'stamp_ns': stamp,
                'tf_gap_ms': round(tf_gap_ms, 3),
                'original_tf_fresh': tf_gap_ms <= 250.0,
                'translation_residual_m': event.translation_residual,
                'yaw_residual_deg': math.degrees(event.yaw_residual),
                'original_map_pose': [round(value, 6) for value in planar_pose(
                    original_map_base)],
                'target_map_pose': [round(value, 6) for value in planar_pose(
                    target_map_base)],
                'original_legal': original_legal,
                'original_reason': original_reason,
                'target_legal': target_legal,
                'target_reason': target_reason,
                'interpolation_failures': interpolation_failures[:3],
            })
        report[window] = {
            'anchor_stamp_ns': anchor[0],
            'old_rejected_new_accepted': len(changed),
            'outcomes': outcomes,
            'records': records,
        }
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if any(result['outcomes']['static_illegal'] for result in report.values()):
        raise SystemExit('At least one newly accepted correction is static-illegal')


if __name__ == '__main__':
    main()
