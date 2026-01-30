import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.actions import TimerAction
from launch.actions import ExecuteProcess # 导入 ExecuteProcess

def generate_launch_description():
    # 获取与拼接默认路径
    fishbot_navigation2_dir = get_package_share_directory(
        'nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 地图路径 - 默认使用学校室外地图
    map_path = "/home/akun/workspace/Car_jetson/utils/src/pcd2pgm/pgm/real/map_room.yaml" # 实验室地图
    # map_path = "/home/akun/workspace/Car_jetson/mapp5.yaml" # 学校室外地图

    # nav2配置路径
    nav2_dir = get_package_share_directory('nav2')
    params_dir = os.path.join(nav2_dir, 'config', 'nav2_real_params.yaml')

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=map_path)
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=params_dir)

    # 三维点云转laserscan
    pointcloud2d_to_laser = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/livox_Pointcloud2'),
                        ('scan', '/scan')],
        parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.1415,  # -M_PI/2
                'angle_max': 3.1415,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'qos_overrides./cloud_in.reliability': 'best_effort',
                'qos_overrides./cloud_in.durability': 'volatile',
                'qos_overrides./scan.reliability': 'best_effort',
                'qos_overrides./scan.durability': 'volatile'
            }],
        name='pointcloud_to_laserscan'
    )

     # 发布体素点云节点
    voxel_pub = launch_ros.actions.Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d_cloud',
        output = 'screen' # 日志输出到屏幕
    )

    return launch.LaunchDescription([
        # pointcloud2d_to_laser,

        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(
            #     [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch_without_amcl.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        voxel_pub,

        
    ])
