#!/usr/bin/env python3
"""Run before/after native retry tests without ROS initialization or nodes."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    directory = Path(__file__).resolve().parent
    project = directory.parents[3]
    source = project / 'nav2/src/navigation2/nav2_mppi_controller'
    before = directory / 'before/nav2/src/navigation2/nav2_mppi_controller'
    results = {}
    evidence = [directory / 'retry_harness.cpp', directory / 'CMakeLists.txt',
                directory / 'run_validation.py',
                source / 'src/optimizer.cpp', source / 'include/nav2_mppi_controller/optimizer.hpp',
                source / 'src/critic_manager.cpp',
                source / 'test/optimizer_retry_regression_test.cpp', source / 'test/CMakeLists.txt',
                before / 'src/optimizer.cpp', before / 'include/nav2_mppi_controller/optimizer.hpp']
    for version in ('before', 'after'):
        binary = directory / 'build' / ('retry_' + version)
        process = subprocess.run([str(binary)], text=True, capture_output=True, check=False)
        (directory / ('retry_' + version + '.stderr.log')).write_text(process.stderr)
        result = json.loads(process.stdout)
        result['exit_code'] = process.returncode
        (directory / ('retry_' + version + '.json')).write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        results[version] = result
        evidence.append(binary)
    results['regression_reproduced_before'] = (
        results['before']['failed'] > 0 and results['before']['exit_code'] == 1)
    results['fixed_version_passed'] = (
        results['after']['failed'] == 0 and results['after']['exit_code'] == 0)
    results['method'] = (
        'Compile and invoke real Optimizer::prepare, optimize, fallback, and real '
        'CriticManager::evalTrajectoriesScores with synthetic fail-first/always-fail '
        'critics and a counting tail critic. Model rollout and control update are real. '
        'The after executable compiles the permanent regression test directly; '
        'the before executable uses a matching legacy-signature harness snapshot.')
    results['limits'] = [
        'No rclcpp::init, ROS context, node, executor, costmap node, publisher or vehicle command.',
        'Use the prepare/do-optimize/while-fallback structure, ending before evalControl Twist packaging, which reads a Costmap2DROS frame name.',
        'Synthetic fail-first and always-fail critics exercise dispatch and recovery, not actual sensor/obstacle collision causes.',
        'The no-node model is a differential-drive rollout; the retry-state logic is shared with Ackermann but this does not validate vehicle geometry.',
        'Before/after executables compile the recorded/local optimizer sources and link existing dependency libraries; they do not modify installed libraries.',
        'No claim of full navigation simulation, real vehicle acceptance or identification of the first historical collision cell.',
    ]
    results['sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in evidence}
    output = directory / 'retry_validation.json'
    output.write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({
        'before': {key: results['before'][key] for key in ('total', 'passed', 'failed')},
        'after': {key: results['after'][key] for key in ('total', 'passed', 'failed')},
        'regression_reproduced_before': results['regression_reproduced_before'],
        'fixed_version_passed': results['fixed_version_passed'],
        'output': str(output),
    }, ensure_ascii=False, indent=2))
    return 0 if results['regression_reproduced_before'] and results['fixed_version_passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
