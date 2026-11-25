"""Start the real Nav2 components with isolated velocity command topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': 'false', 'autostart': 'true'},
            convert_types=True,
        ),
        allow_substs=True,
    )
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav_command = '/indoor_startup/cmd_vel_nav'
    output_command = '/indoor_startup/cmd_vel'
    navigation_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    component_specs = [
        ('nav2_map_server', 'nav2_map_server::MapServer', 'map_server', []),
        (
            'nav2_controller',
            'nav2_controller::ControllerServer',
            'controller_server',
            [('cmd_vel', nav_command)],
        ),
        ('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server', []),
        ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server', []),
        (
            'nav2_behaviors',
            'behavior_server::BehaviorServer',
            'behavior_server',
            [('cmd_vel', nav_command)],
        ),
        ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', []),
        (
            'nav2_waypoint_follower',
            'nav2_waypoint_follower::WaypointFollower',
            'waypoint_follower',
            [],
        ),
        (
            'nav2_velocity_smoother',
            'nav2_velocity_smoother::VelocitySmoother',
            'velocity_smoother',
            [('cmd_vel', nav_command), ('cmd_vel_smoothed', output_command)],
        ),
    ]
    components = [
        ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[configured_params, {'use_sim_time': False}],
            remappings=tf_remappings + command_remappings,
        )
        for package, plugin, name, command_remappings in component_specs
    ]
    for manager_name, node_names in [
        ('lifecycle_manager_localization', ['map_server']),
        ('lifecycle_manager_navigation', navigation_nodes),
    ]:
        components.append(
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name=manager_name,
                parameters=[{
                    'use_sim_time': False,
                    'autostart': True,
                    'node_names': node_names,
                }],
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='Absolute path to the prepared Nav2 startup parameters',
        ),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            output='screen',
            # Costmap nodes are constructed inside controller/planner components;
            # they need the complete YAML in the container's global ROS arguments.
            parameters=[configured_params, {'use_sim_time': False}],
        ),
        LoadComposableNodes(
            target_container='/nav2_container',
            composable_node_descriptions=components,
        ),
    ])
