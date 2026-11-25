"""Inspect the resolved graph without starting hardware processes."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest
import yaml


def launch_module():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'standalone_docking.launch.py'
    spec = importlib.util.spec_from_file_location('standalone_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_with_defaults(module):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def test_default_graph_contains_required_sensors_and_no_navigation():
    module = launch_module()
    nodes = [n for n in module.create_nodes(context_with_defaults(module)) if isinstance(n, Node)]
    packages = [n.node_package for n in nodes]
    assert packages.count('cmdvel_2_ecu') == 1
    assert packages.count('lakibeam1') == 1
    assert 'chassis_driver' in packages and 'fdilink_ahrs' in packages
    assert 'nav2' not in packages and 'multi_map_navigation' not in packages
    assert len(nodes) == 9


def test_rear_lidar_scan_order_matches_static_p0_diagnostics():
    config = Path(__file__).resolve().parents[1] / 'config' / 'hardware.yaml'
    params = yaml.safe_load(config.read_text())['standalone_rear_lidar']['ros__parameters']
    assert (params['hostip'], params['port']) == ('192.168.1.140', '2369')
    assert params['inverted'] is True


def test_motion_requires_explicit_calibration_and_external_sensors_can_be_reused():
    module = launch_module()
    context = context_with_defaults(module)
    context.launch_configurations['dry_run'] = 'false'
    with pytest.raises(ValueError, match='calibrated'):
        module.create_nodes(context)
    context.launch_configurations['calibrated'] = 'true'
    for name in ('start_chassis', 'start_imu', 'start_rear_lidar', 'publish_rear_tf'):
        context.launch_configurations[name] = 'false'
    nodes = [n for n in module.create_nodes(context) if isinstance(n, Node)]
    assert len(nodes) == 5
