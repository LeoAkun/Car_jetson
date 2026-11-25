"""Replay recorded CAN/IMU at recorded odometry ticks without ROS nodes or motion."""

import inspect
import json
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from charging_dock_standalone import feedback_odometry as feedback


def main():
    """Compare the old source selection with the installed fixed integrator."""
    directory = Path(__file__).resolve().parent
    source = inspect.getsource(feedback.FeedbackIntegrator)
    selection = 'use_wheels = wheel_speeds is not None\n'
    if source.count(selection) != 1:
        raise RuntimeError('Expected the fixed wheel-source selection')
    legacy_source = source.replace(
        selection, 'use_wheels = wheel_speeds is not None and speed_kmh == 0.\n')
    namespace = dict(vars(feedback))
    exec(compile(legacy_source, '<legacy-source-selection>', 'exec'), namespace)
    integrators = {
        'legacy_source_selection': namespace['FeedbackIntegrator'](),
        'signed_wheel_source': feedback.FeedbackIntegrator(),
    }
    results = {name: {'estimates': 0, 'first_failure': None}
               for name in integrators}
    vehicle = imu = None
    fault_time = None
    with sqlite3.connect(f'file:{directory / "bag/bag_0.db3"}?mode=ro', uri=True) as db:
        topics = {topic_id: (name, get_message(message_type))
                  for topic_id, name, message_type in db.execute(
                      'SELECT id, name, type FROM topics')
                  if name in ('/vehicle_status', '/imu',
                              '/docking_feedback_odometry/state')}
        for topic_id, timestamp, data in db.execute(
                'SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id'):
            if topic_id not in topics:
                continue
            name, message_class = topics[topic_id]
            message = deserialize_message(data, message_class)
            if name == '/vehicle_status':
                vehicle = message
                continue
            if name == '/imu':
                imu = message
                continue
            if vehicle is None or imu is None:
                continue
            now = timestamp / 1e9
            if fault_time is not None and now > fault_time + 4.:
                break
            sample = {
                'time': now, 'main_speed': vehicle.cur_speed,
                'gear': vehicle.shift_level,
                'left_wheel_speed': vehicle.left_wheel_speed,
                'right_wheel_speed': vehicle.right_wheel_speed,
            }
            for variant, integrator in integrators.items():
                result = results[variant]
                estimate = integrator.update(
                    now, feedback.seconds(vehicle.header.stamp),
                    feedback.seconds(imu.header.stamp), vehicle.cur_speed,
                    vehicle.shift_level, imu.angular_velocity.z,
                    (vehicle.left_wheel_speed, vehicle.right_wheel_speed))
                if integrator.failed and result['first_failure'] is None:
                    result['first_failure'] = dict(sample, reason=integrator.reason)
                    if variant == 'legacy_source_selection':
                        fault_time = now
                if estimate is None:
                    continue
                pose = dict(time=now, stamp=estimate.stamp, x=estimate.pose.x,
                            y=estimate.pose.y, speed=estimate.speed)
                result['estimates'] += 1
                result['last_estimate'] = pose
                if fault_time is not None and now >= fault_time:
                    result.setdefault('first_fault_window_pose', pose)
                    if abs(estimate.speed) < 1e-9:
                        result.setdefault('stopped_pose', pose)
    legacy = results['legacy_source_selection']
    fixed = results['signed_wheel_source']
    assert legacy['first_failure']['reason'] == 'moving_without_direction_feedback'
    assert fixed['first_failure'] is None
    assert fixed['stopped_pose']['time'] > legacy['first_failure']['time']
    assert fixed['last_estimate']['speed'] == 0.
    report = {
        'method': 'Read-only bag; recorded state publication times approximate timer ticks; '
                  'same CAN/IMU acquisition stamps; no ROS nodes or vehicle commands.',
        'scope': 'Through four seconds after the original source-selection fault.',
        'results': results,
    }
    output = directory / 'feedback_replay_results.json'
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
