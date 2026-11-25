#!/usr/bin/env python3
"""Run the native real-critic harness without initializing ROS or a DDS context."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml


def main():
    directory = Path(__file__).resolve().parent
    project = directory.parents[3]
    config_path = project / 'nav2/src/nav2/config/nav2_real_params.yaml'
    previous_path = directory / 'before/nav2/src/nav2/config/nav2_real_params.yaml'
    converter_path = (project / 'real_ws/src/driver/car_control/cmdvel_2_ecu/'
                      'config/config.yaml')
    current = yaml.safe_load(config_path.read_text())
    previous = yaml.safe_load(previous_path.read_text())
    settings = current['controller_server']['ros__parameters']['FollowPath']
    old_settings = previous['controller_server']['ros__parameters']['FollowPath']
    goal = settings['GoalCritic']
    path = settings['PathFollowCritic']
    converter = yaml.safe_load(converter_path.read_text())['/cmdvel_to_ecu']['ros__parameters']
    assert goal['enabled'] and path['enabled']
    assert not settings['PreferForwardCritic']['enabled']
    arguments = [old_settings['GoalCritic']['threshold_to_consider'],
                 goal['threshold_to_consider'], path['threshold_to_consider'],
                 goal['cost_weight'], path['cost_weight'], goal['cost_power'],
                 path['cost_power'], path['offset_from_furthest'],
                 settings['time_steps'], settings['model_dt'], converter['LOW_SPEED_THRESH']]
    binary = directory / 'build/critic_harness'
    process = subprocess.run([str(binary), *map(str, arguments)], text=True,
                             capture_output=True, check=False)
    if process.stderr:
        print(process.stderr, file=sys.stderr)
    result = json.loads(process.stdout)
    source = project / 'nav2/src/navigation2/nav2_mppi_controller'
    evidence = [config_path, previous_path, converter_path, directory / 'critic_harness.cpp',
                directory / 'CMakeLists.txt', binary,
                source / 'src/critics/goal_critic.cpp',
                source / 'src/critics/path_follow_critic.cpp',
                source / 'include/nav2_mppi_controller/tools/utils.hpp']
    result['settings'] = dict(zip(
        ['old_goal_threshold_m', 'new_goal_threshold_m', 'path_threshold_m',
         'goal_weight', 'path_weight', 'goal_power', 'path_power',
         'path_offset', 'time_steps', 'model_dt_s', 'candidate_speed_mps'], arguments))
    result['method'] = ('Compiles and invokes the unmodified local C++ GoalCritic::score '
                        'and PathFollowCritic::score. Protected configuration is injected '
                        'by derived wrappers; no initialize/on_configure calls. '
                        'All-free path validity is supplied, with the real path-furthest '
                        'calculation left enabled.')
    result['limits'] = [
        'No rclcpp::init, context, node, executor, publisher, launch, vehicle command or live parameter change.',
        'Three manually constructed straight trajectories, only two critic scores; not full MPPI optimization or closed-loop simulation.',
        'No obstacle/costmap, steering dynamics, odometry frame or real chassis-response validation.',
        'Exact threshold results reflect the real float arithmetic and strict comparison; +/-0.0001m cases test both sides.',
        'Historical observations cannot establish that this attraction gap uniquely caused the November 2 oscillation.',
    ]
    result['sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                        for path in evidence}
    output = directory / 'critic_validation.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'passed': result['passed'], 'checks': result['checks'],
                      'sweep': result['sweep'], 'settings': result['settings'],
                      'output': str(output)}, ensure_ascii=False, indent=2))
    return process.returncode


if __name__ == '__main__':
    raise SystemExit(main())
