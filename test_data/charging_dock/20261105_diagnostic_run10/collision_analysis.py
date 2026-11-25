"""Inspect sampled run10 scans offline; no ROS nodes or hardware access.

Run with the charging_dock source directory on PYTHONPATH. The complete bag
ends before the failure, so this cannot reconstruct its original scan frame.
Only motion_trace.jsonl is read. Output is collision_analysis.json beside it.
"""

from bisect import bisect_left
from dataclasses import asdict
import json
import math
from pathlib import Path
from types import SimpleNamespace

from charging_dock.dock_collision import (
    collision, dock_plane_collision, locked_target_indices)
from charging_dock.relative_geometry import integrate, Pose2, wrap


HERE = Path(__file__).resolve().parent
BASE_SENSOR = Pose2(-0.43, -0.28, math.radians(225))
BOUNDS = (0.43, 0.43, 0.43)
MARGIN = 0.02


def stamp(header):
    value = header['stamp']
    return value['sec'] + value['nanosec'] / 1e9


def pose(message):
    q, p = message['orientation'], message['position']
    return Pose2(p['x'], p['y'], math.atan2(
        2 * q['w'] * q['z'], 1 - 2 * q['z'] ** 2))


def main():
    rows = {}
    odometry = []
    for line in (HERE / 'motion_trace.jsonl').open():
        row = json.loads(line)
        topic, message = row['topic'], row['message']
        if topic == '/charging_dock/local_odom':
            odometry.append((stamp(message['header']),
                             pose(message['pose']['pose']), row))
        elif topic in ('/charging_dock/observation', '/laser/scan_back',
                       '/cmd_vel_dock', '/vehicle_status', '/ecu',
                       '/charging_dock/state'):
            rows.setdefault(topic, []).append(row)

    timestamps = [entry[0] for entry in odometry]

    def lookup(time):
        index = bisect_left(timestamps, time)
        if index == 0:
            return odometry[0][1]
        if index == len(timestamps):
            return odometry[-1][1]
        a, b = odometry[index - 1], odometry[index]
        fraction = (time - a[0]) / (b[0] - a[0])
        return Pose2(
            a[1].x + (b[1].x - a[1].x) * fraction,
            a[1].y + (b[1].y - a[1].y) * fraction,
            wrap(a[1].yaw + wrap(b[1].yaw - a[1].yaw) * fraction))

    def latest(topic, time):
        return next(row for row in reversed(rows[topic]) if row['t'] <= time)

    failure = next(row for row in rows['/charging_dock/state']
                   if row['message']['data'] ==
                   'STOPPING reason=swept_footprint_obstacle')
    lock = next(row for row in rows['/charging_dock/state']
                if row['message']['data'].startswith('LOCKED_FINAL_APPROACH'))
    # Approximation only: the node does not publish its exact accepted list of
    # averaged observations. Reconstruct the final stationary collection span.
    transitions = []
    for row in rows['/charging_dock/state']:
        if (not transitions or row['message']['data']
                != transitions[-1]['message']['data']):
            transitions.append(row)
    collection = next(row for row in reversed(transitions)
                      if row['t'] < lock['t'] and row['message']['data'] ==
                      'LOCK_VERIFY reason=collecting_stopped_dock_pose')
    local_poses = []
    for row in rows['/charging_dock/observation']:
        message = row['message']
        if collection['t'] <= row['t'] < lock['t'] and message['valid']:
            local_poses.append(lookup(stamp(message['header'])).compose(
                pose(message['pose'])))
    locked_dock = Pose2(
        sum(p.x for p in local_poses) / len(local_poses),
        sum(p.y for p in local_poses) / len(local_poses),
        math.atan2(sum(math.sin(p.yaw) for p in local_poses),
                   sum(math.cos(p.yaw) for p in local_poses)))
    locked_face = locked_dock.compose(Pose2(-0.057))
    scans = [row for row in rows['/laser/scan_back']
             if failure['t'] - 1 < row['t'] < failure['t'] + 1.3]
    results = []
    for row in scans:
        message = row['message']
        scan = SimpleNamespace(**message)
        local_base = lookup(stamp(message['header']))
        local_sensor = local_base.compose(BASE_SENSOR)
        exempt = locked_target_indices(scan, local_sensor, locked_face,
                                       0.25, 0.05)
        command = latest('/cmd_vel_dock', row['t'])
        velocity = command['message']['linear']['x']
        angular = command['message']['angular']['z']
        # The sampled frames used here have negligible measured translation.
        # This uses the command's stopping arc, not a claim to the exact tick.
        horizon = max(0.05, 0.20 + abs(velocity) / (2 * 0.10))
        hits = []
        for index, value in enumerate(scan.ranges):
            if (index in exempt or not math.isfinite(value)
                    or not scan.range_min < value <= scan.range_max):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            sensor_point = Pose2(value * math.cos(angle),
                                 value * math.sin(angle))
            point = BASE_SENSOR.compose(sensor_point)
            for step in range(int(math.ceil(horizon / 0.05)) + 1):
                when = step * 0.05
                future_point = point.relative_to(integrate(
                    Pose2(), velocity, angular, when))
                if (-BOUNDS[1] - MARGIN <= future_point.x
                        <= BOUNDS[0] + MARGIN
                        and abs(future_point.y) <= BOUNDS[2] + MARGIN):
                    hits.append({
                        'index': index, 'range_m': value,
                        'sensor_angle_deg': math.degrees(angle),
                        'point_base_at_scan': asdict(point),
                        'point_locked_backplane_approx': asdict(
                            local_sensor.compose(sensor_point).relative_to(
                                locked_face)),
                        'first_sweep_hit_seconds': when,
                        'inside_unexpanded_footprint': (
                            -BOUNDS[1] <= point.x <= BOUNDS[0]
                            and abs(point.y) <= BOUNDS[2]),
                    })
                    break
        track = {}
        for index in (1205, 1208):
            value = scan.ranges[index]
            if math.isfinite(value):
                angle = scan.angle_min + index * scan.angle_increment
                point = BASE_SENSOR.compose(Pose2(
                    value * math.cos(angle), value * math.sin(angle)))
                track[str(index)] = {'range_m': value,
                                     'point_base_at_scan': asdict(point)}
        results.append({
            't': row['t'], 'scan_ros_stamp': stamp(message['header']),
            'relative_to_failure_s': row['t'] - failure['t'],
            'odom_interpolated_at_scan': asdict(local_base),
            'locked_dock_base_approx': asdict(locked_dock.relative_to(local_base)),
            'latest_command_t': command['t'],
            'velocity_command_m_s': velocity, 'omega_command_rad_s': angular,
            'sweep_horizon_s': horizon, 'target_exempt_count': len(exempt),
            'scan_collision': collision(scan, BASE_SENSOR, exempt, velocity,
                                        angular, BOUNDS, MARGIN, horizon),
            'dock_plane_collision_approx': dock_plane_collision(
                locked_dock.relative_to(local_base), velocity, angular,
                BOUNDS, MARGIN, horizon),
            'hit_count': len(hits), 'hits': hits, 'beam_track': track,
        })

    after = next(result for result in results if result['t'] > failure['t'])
    before = next(result for result in reversed(results)
                  if result['t'] < failure['t'])
    report = {
        'source': 'motion_trace.jsonl sampled scans only',
        'limitations': [
            'The original failure scan at t302.715 is absent.',
            'The nearest later sampled scan is after braking began; its points '
            'cannot be asserted to be the original trigger.',
            'Transforms use interpolated local odometry and launch static TF; '
            'the exact controller callback schedule is not reproduced.',
            'Locked pose uses an approximate observation average; this does not '
            'affect points far in front of the locked backplane.',
            'No camera/image identifies the transient object. A person, another '
            'object, movable self structure or measurement artifact is unproven.',
        ],
        'failure_t': failure['t'], 'lock_t': lock['t'],
        'static_base_sensor_from_launch': asdict(BASE_SENSOR),
        'bounds_front_rear_halfwidth_m': BOUNDS, 'margin_m': MARGIN,
        'self_point_filter': 'No automatic inside-footprint mask in collision().',
        'target_filter': {'locked_backplane_depth_tolerance_m': 0.015,
                          'total_width_m': 0.25 + 0.05},
        'approx_lock_observation_count': len(local_poses),
        'approx_locked_dock_local': asdict(locked_dock),
        'feedback_at_failure': latest('/vehicle_status', failure['t']),
        'ecu_at_failure': latest('/ecu', failure['t']),
        'nearest_sample_before_t': before['t'],
        'nearest_sample_before_collision': before['scan_collision'],
        'nearest_sample_after_t': after['t'],
        'nearest_sample_after_collision': after['scan_collision'],
        'interpretation': (
            'The immediate post-failure sampled frame contains a new short-range '
            'cluster on the vehicle right/front side, inside the expanded '
            'footprint already at zero sweep time. It is far from the rear dock '
            'and is not eligible for the locked backplane exemption. Nearby '
            'earlier samples show distant returns at these beam indices; later '
            'samples return to distant background while odometry is nearly '
            'stationary. This supports a transient near return, rather than a '
            'permanent fixed self return, but does not prove object identity '
            'or the original triggering beam.'),
        'sampled_scan_results': results,
    }
    output = HERE / 'collision_analysis.json'
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: report[key] for key in (
        'failure_t', 'nearest_sample_before_t', 'nearest_sample_before_collision',
        'nearest_sample_after_t', 'nearest_sample_after_collision')}))
    print('Saved', output)


if __name__ == '__main__':
    main()
