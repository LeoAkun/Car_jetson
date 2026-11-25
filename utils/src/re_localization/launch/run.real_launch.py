import os
import yaml

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory


def resolve_map_path(context):
    """An omitted map override must retain the selected YAML's map path."""
    map_path = launch.substitutions.LaunchConfiguration('map_path').perform(context)
    if not map_path:
        config_path = launch.substitutions.LaunchConfiguration(
            'localization_param_dir').perform(context)
        with open(config_path, encoding='utf-8') as stream:
            map_path = yaml.safe_load(stream)['/**']['ros__parameters']['map_path']
    return [launch.actions.SetLaunchConfiguration('resolved_map_path', map_path)]


def generate_launch_description():

    ld = launch.LaunchDescription()

    default_config = os.path.join(
        get_package_share_directory('re_localization'),
        'param', 'localization_real.yaml')

    # 声明地图路径参数
    map_path_arg = launch.actions.DeclareLaunchArgument(
        'map_path',
        default_value='',
        description='Path to the PCD map file; if empty, use the selected YAML config.'
    )

    config_arg = launch.actions.DeclareLaunchArgument(
        'localization_param_dir', default_value=default_config,
        description='Path to the localization parameter YAML.')

    session_id_arg = launch.actions.DeclareLaunchArgument(
        'localization_session_id', default_value='',
        description='Unique identifier for the localization attempt.')
    map_id_arg = launch.actions.DeclareLaunchArgument(
        'map_id', default_value='',
        description='Map identifier bound to this localization server.')
    session_start_arg = launch.actions.DeclareLaunchArgument(
        'session_start_ns', default_value='0',
        description='ROS timestamp in nanoseconds at which this session began.')
    autostart_arg = launch.actions.DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Configure and activate automatically for standalone launches.')
    autostart = launch.substitutions.LaunchConfiguration('autostart')

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('re_localization'),
            'param',
            'localization_real.yaml'))

    map_path = launch.substitutions.LaunchConfiguration('resolved_map_path')
    localization_session_id = launch.substitutions.LaunchConfiguration(
        'localization_session_id')
    map_id = launch.substitutions.LaunchConfiguration('map_id')
    session_start_ns = launch.substitutions.LaunchConfiguration('session_start_ns')

    pcl_localization = launch_ros.actions.LifecycleNode(
        name='pcl_localization',
        namespace='',
        package='re_localization',
        executable='pcl_localization_node',
        parameters=[
            localization_param_dir,
            {
                'map_path': map_path,
                'localization_session_id': localization_session_id,
                'map_id': map_id,
                'session_start_ns': ParameterValue(
                    session_start_ns, value_type=int),
            }
        ],
        # remappings=[('/cloud','/livox_Pointcloud2')],
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
        condition=IfCondition(autostart),
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(autostart),
    )

    # ld.add_action(init_pose_match_node)
    ld.add_action(map_path_arg)
    ld.add_action(config_arg)
    ld.add_action(session_id_arg)
    ld.add_action(map_id_arg)
    ld.add_action(session_start_arg)
    ld.add_action(autostart_arg)
    ld.add_action(launch.actions.OpaqueFunction(function=resolve_map_path))
    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    ld.add_action(pcl_localization)
    # ld.add_action(lidar_tf)
    ld.add_action(to_inactive)

    return ld
