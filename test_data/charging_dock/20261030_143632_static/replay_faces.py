"""Replay parked scans; diagnostic configurations never issue vehicle commands."""

from collections import Counter
from dataclasses import asdict, fields
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / 'real_ws/src/charging_dock'))
from charging_dock.relative_geometry import Pose2
from charging_dock.relative_lidar import FaceConfig, RelativeFaceTracker


def replay(filename, reverse=False):
    data = json.loads((HERE / filename).read_text())
    scans = data['scans']
    first = scans[0]
    sensor = Pose2(**data.get('sensor', first.get('sensor')))
    origin = Pose2(**first.get('base', {'x': 0., 'y': 0., 'yaw': 0.}))
    config_path = HERE.parents[2] / 'real_ws/src/charging_dock/config/relative_docking.yaml'
    params = yaml.safe_load(config_path.read_text())['relative_docking_controller'][
        'ros__parameters']
    default = FaceConfig()
    configurations = {
        'current_production': FaceConfig(**{
            f.name: params.get('face_' + f.name, getattr(default, f.name))
            for f in fields(FaceConfig)}),
        'previous_strict_plane': FaceConfig(width=.25),
        # Hypotheses for diagnosis only; a fit does not calibrate the contact plane.
        'diagnostic_compound': FaceConfig(width=.25, width_tolerance=.05,
                                         edge_depth=.025, residual=.03),
        'diagnostic_central_patch': FaceConfig(width=.11),
    }
    result = {}
    for name, config in configurations.items():
        tracker = RelativeFaceTracker(config)
        tracker.reset(origin.compose(Pose2(-2.557)))
        reasons, poses = Counter(), []
        for s in scans:
            stamp = s['stamp']
            sec = int(stamp)
            ranges = s['ranges'][::-1] if reverse else s['ranges']
            scan = NS(header=NS(stamp=NS(sec=sec, nanosec=round((stamp-sec)*1e9))),
                      ranges=ranges, angle_min=s['angle_min'],
                      angle_increment=s['angle_increment'], range_min=s['range_min'],
                      range_max=s['range_max'])
            base = Pose2(**s.get('base', {'x': 0., 'y': 0., 'yaw': 0.}))
            observation = tracker.process(scan, stamp+.01, base, sensor)
            reasons[observation.reason] += 1
            if observation.pose is not None:
                p = observation.pose
                poses.append([p.x, p.y, p.yaw, observation.width,
                              observation.residual, observation.quality])
        result[name] = {'config': asdict(config), 'reasons': dict(reasons)}
        if poses:
            result[name]['median_x_y_yaw_width_rms_quality'] = np.median(
                poses, axis=0).tolist()
    return result


if __name__ == '__main__':
    results = {
        'original': replay('scan_with_tf.json'),
        'original_reversed_offline': replay('scan_with_tf.json', reverse=True),
        'corrected_driver_live': replay('scan_inverted_true_live.json'),
    }
    (HERE / 'replay_results_accepted_coarse.json').write_text(json.dumps(results, indent=2))
    for dataset, configurations in results.items():
        print(dataset)
        for name, values in configurations.items():
            print(' ', name, values['reasons'])
