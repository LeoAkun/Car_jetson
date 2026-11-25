"""Offline geometry checks for decimated scans; never connects to ROS."""

import bisect
from collections import Counter
from dataclasses import fields, replace
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / 'real_ws/src/charging_dock'))
from charging_dock.relative_geometry import Pose2, wrap
from charging_dock.relative_lidar import FaceConfig, RelativeFaceTracker


def stamp(message):
    value = message['header']['stamp']
    return value['sec'] + value['nanosec'] / 1e9


def pose(value):
    return Pose2(value['position']['x'], value['position']['y'],
                 2 * math.atan2(value['orientation']['z'], value['orientation']['w']))


def main():
    records = [json.loads(line) for line in (HERE / 'motion_trace.jsonl').open()]
    odometry = [(stamp(row['message']), pose(row['message']['pose']['pose']))
                for row in records if row['topic'] == '/charging_dock/local_odom']
    odometry.sort(key=lambda item: item[0])
    times = [item[0] for item in odometry]

    def base_at(t):
        index = bisect.bisect_left(times, t)
        if index == 0:
            return odometry[0][1]
        if index == len(times):
            return odometry[-1][1]
        t0, p0 = odometry[index - 1]
        t1, p1 = odometry[index]
        alpha = (t - t0) / (t1 - t0)
        return Pose2(p0.x + alpha * (p1.x - p0.x),
                     p0.y + alpha * (p1.y - p0.y),
                     p0.yaw + alpha * wrap(p1.yaw - p0.yaw))

    sending = next(row['t'] for row in records
                   if row['topic'] == '/standalone_docking/state'
                   and row['message']['data'].startswith('SENDING'))
    first = next(row for row in records if row['topic'] == '/charging_dock/local_odom'
                 and row['t'] >= sending)
    origin = base_at(stamp(first['message']))
    params = yaml.safe_load((ROOT / 'real_ws/src/charging_dock/config/relative_docking.yaml')
                            .read_text())['relative_docking_controller']['ros__parameters']
    defaults = FaceConfig()
    config = FaceConfig(**{field.name: params.get('face_' + field.name,
                                                getattr(defaults, field.name))
                           for field in fields(FaceConfig)})
    # Recorded scans are decimated to 5Hz. Disable only the temporal warmup for
    # these geometry comparisons; do not interpret these as live readiness.
    config = replace(config, stable_frames=1, timeout=.5)
    variants = {
        'production_geometry': config,
        'wider_width_diagnostic_only': replace(config, width_tolerance=.10),
        'lower_depth_step_diagnostic_only': replace(config, edge_depth=.01),
        'larger_depth_step_diagnostic_only': replace(config, edge_depth=.04),
        'wider_pose_gate_diagnostic_only': replace(config, position_gate=1.,
                                                  depth_gate=1., angle_gate=.8),
    }
    scans = [row for row in records if row['topic'] == '/laser/scan_back'
             and row['t'] >= sending]
    result = {}
    for name, values in variants.items():
        tracker = RelativeFaceTracker(values)
        tracker.reset(origin.compose(Pose2(-2.557)))
        reasons, near_failure = Counter(), []
        for row in scans:
            message = row['message']
            scan = NS(**{key: message[key] for key in
                         ('ranges', 'angle_min', 'angle_increment', 'range_min', 'range_max')})
            scan.header = NS(stamp=NS(**message['header']['stamp']))
            observation = tracker.process(scan, stamp(message) + .01,
                                          base_at(stamp(message)),
                                          Pose2(-.43, -.28, math.radians(225)))
            reasons[observation.reason] += 1
            if 85.5 <= row['t'] <= 86.8:
                near_failure.append({'t': row['t'], 'scan_stamp': stamp(message),
                                     'reason': observation.reason,
                                     'x': observation.pose.x if observation.pose else None,
                                     'width': observation.width,
                                     'residual': observation.residual})
        result[name] = {'reasons': dict(reasons), 'near_failure': near_failure}
    (HERE / 'replay_results.json').write_text(json.dumps(result, indent=2))
    for name, values in result.items():
        print(name, values)


if __name__ == '__main__':
    main()
