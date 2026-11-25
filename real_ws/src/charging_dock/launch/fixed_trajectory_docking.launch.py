# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""Launch the fail-closed Stage 5 fixed-trajectory docking stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    charging_share = get_package_share_directory('charging_dock')
    stage5_config = os.path.join(
        charging_share, 'config', 'stage5_docking.yaml')
    ecu_config = os.path.join(
        get_package_share_directory('cmdvel_2_ecu'),
        'config', 'docking_config.yaml')

    dry_run = LaunchConfiguration('dry_run')
    allow_motion = LaunchConfiguration('allow_motion')
    run_distance_limit = LaunchConfiguration('run_distance_limit_m')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Keep the active controller at zero velocity.'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='Second interlock required for non-zero commands.'),
        DeclareLaunchArgument(
            'run_distance_limit_m', default_value='0.10',
            description='Maximum travel permitted for this test run.'),
        Node(
            package='charging_dock',
            executable='command_mux',
            name='command_mux',
            parameters=[stage5_config],
            output='screen',
        ),
        Node(
            package='charging_dock',
            executable='active_docking_controller',
            name='active_docking_controller',
            parameters=[
                stage5_config,
                {
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                    'publish_nonzero_command': ParameterValue(
                        allow_motion, value_type=bool),
                    'run_distance_limit_m': ParameterValue(
                        run_distance_limit, value_type=float),
                },
            ],
            output='screen',
        ),
        Node(
            package='cmdvel_2_ecu',
            executable='cmdvel_2_ecu',
            name='cmdvel_to_ecu',
            parameters=[ecu_config],
            output='screen',
        ),
    ])
