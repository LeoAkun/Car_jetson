"""Rear lidar + IMU + chassis + shared docking, without map navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(name + ' must be true or false')
    return value == 'true'


def create_nodes(context):
    """Resolve geometry once; every node must agree on the local frame tree."""
    config = LaunchConfiguration('dock_params').perform(context)
    hardware = LaunchConfiguration('hardware_params').perform(context)
    with open(config, encoding='utf-8') as stream:
        geometry = yaml.safe_load(stream)['relative_docking_controller']['ros__parameters']
    dry_run = _boolean(context, 'dry_run')
    calibrated = _boolean(context, 'calibrated')
    if not dry_run and not calibrated:
        raise ValueError('Motion requires calibrated:=true after vehicle calibration')
    local, base, scan = 'dock_odom', geometry.get('base_frame', 'base_link'), geometry.get(
        'scan_frame', 'rear_laser')
    frames = {'local_frame': local, 'base_frame': base}
    ecu_config = os.path.join(get_package_share_directory('cmdvel_2_ecu'),
                              'config', 'config.yaml')
    nodes = [
        Node(package='charging_dock', executable='command_mux', name='command_mux',
             parameters=[config, frames, {'measured_handoff': True,
                                          'nav_topic': '/charging_dock_standalone/unused_nav',
                                          'legacy_nav_topic': ''}], output='screen'),
        Node(package='charging_dock', executable='relative_docking_controller',
             name='relative_docking_controller', parameters=[config, frames, {
                 'global_frame': local, 'allow_motion': not dry_run,
                 'calibrated': calibrated}], output='screen'),
        Node(package='cmdvel_2_ecu', executable='cmdvel_2_ecu', name='cmdvel_to_ecu',
             parameters=[ecu_config, {'cmd_vel_topic': '/cmd_vel_selected',
                                      'docking_mode': False, 'docking_max_speed': .10,
                                      'docking_max_steer_deg': 8.,
                                      'docking_ecu_speed_scale': 3.6,
                                      'docking_stop_threshold': .001,
                                      'input_timeout_sec': .20}], output='screen'),
        Node(package='charging_dock_standalone', executable='feedback_odometry',
             name='docking_feedback_odometry', parameters=[hardware, frames], output='screen'),
        Node(package='charging_dock_standalone', executable='standalone_docking',
             name='standalone_docking', parameters=[hardware, frames, {
                 'scan_frame': scan, 'scan_topic': geometry.get('scan_topic', '/laser/scan_back'),
                 'dry_run': dry_run, 'auto_start': _boolean(context, 'auto_start')}],
             output='screen'),
    ]
    if _boolean(context, 'start_rear_lidar'):
        nodes.append(Node(
            package='lakibeam1', executable='lakibeam1_scan_node', name='standalone_rear_lidar',
            parameters=[hardware, {'frame_id': scan,
                                   'output_topic': geometry.get(
                                       'scan_topic', '/laser/scan_back')}],
            output='screen'))
    if _boolean(context, 'publish_rear_tf'):
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='standalone_rear_tf', arguments=[
                '--x', LaunchConfiguration('rear_x'), '--y', LaunchConfiguration('rear_y'),
                '--z', LaunchConfiguration('rear_z'), '--roll', '0.', '--pitch', '0.',
                '--yaw', LaunchConfiguration('rear_yaw'),
                '--frame-id', base, '--child-frame-id', scan], output='screen'))
    if _boolean(context, 'start_imu'):
        nodes.append(Node(package='fdilink_ahrs', executable='ahrs_driver_node',
                          name='standalone_imu', parameters=[hardware], output='screen'))
    if _boolean(context, 'start_chassis'):
        nodes.append(Node(package='chassis_driver', executable='chassis_driver_node',
                          name='standalone_chassis', parameters=[hardware], output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(
            reason='A required standalone docking process exited'))])) for node in nodes]
    return handlers + nodes


def generate_launch_description():
    standalone = get_package_share_directory('charging_dock_standalone')
    charging = get_package_share_directory('charging_dock')
    defaults = {
        'dock_params': os.path.join(charging, 'config', 'relative_docking.yaml'),
        'hardware_params': os.path.join(standalone, 'config', 'hardware.yaml'),
        'dry_run': 'true', 'calibrated': 'false', 'auto_start': 'true',
        'start_rear_lidar': 'true', 'start_imu': 'true', 'start_chassis': 'true',
        'publish_rear_tf': 'true',
        'rear_x': '-0.43', 'rear_y': '-0.28', 'rear_z': '0.15',
        'rear_yaw': '3.9269908169872414',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        OpaqueFunction(function=create_nodes),
    ])
