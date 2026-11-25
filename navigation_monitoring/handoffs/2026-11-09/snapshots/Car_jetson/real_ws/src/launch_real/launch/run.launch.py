import launch, os, launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression

def generate_launch_description():

    enable_pothole_detection = launch.substitutions.LaunchConfiguration(
        'enable_pothole_detection'
    )
    start_cmdvel_2_ecu_enabled = launch.substitutions.LaunchConfiguration(
        'start_cmdvel_2_ecu'
    )
    relative_docking_enabled = launch.substitutions.LaunchConfiguration(
        'enable_relative_lidar_docking'
    )

    # 启动前向下视 RealSense D435i 深度相机
    start_depth_camera = launch.actions.GroupAction(
        scoped=True,
        forwarding=False,
        actions=[
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('launch_real'),
                        'launch',
                        'realsense.launch.py'
                    )
                )
            )
        ]
    )

    # 坑道检测默认关闭，避免在实车验证前影响现有运行负载。
    start_pothole_detection = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('launch_real'),
            'launch',
            'pothole_detection.launch.py'
        )),
        condition=launch.conditions.IfCondition(enable_pothole_detection)
    )

    # 启动livox
    start_livox = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'launch_ROS2',
            'msg_MID360_launch.py'
        ))
    )

    # 启动livox消息转换
    livox_custom_convert_pointcloud2 = launch_ros.actions.Node(
        package='convert_laser',
        executable='convert',
        output = 'screen' # 日志输出到屏幕
    )

    # 启动速腾32线雷达
    start_rslidar = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rslidar_sdk'),
            'launch',
            'start.py'
        ))
    )

    # 启动imu
    start_imu = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('fdilink_ahrs'),
            'launch',
            'ahrs_driver.launch.py'
        ))
    )

    # 启动底盘控制驱动
    start_car = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('chassis_driver'),
            'launch',
            'chassis_driver.launch.py'
        ))
    )

    # 启动cmd_vel与ecu的转换节点
    config_file = os.path.join(
        get_package_share_directory('cmdvel_2_ecu'),
        'config',
        'config.yaml'
    )
    alarm_config_file = os.path.join(
        get_package_share_directory('alarm_controller'),
        'config',
        'alarm_controller.yaml'
    )
    start_cmdvel_2_ecu = launch_ros.actions.Node(
        parameters=[config_file],
        package='cmdvel_2_ecu',
        executable='cmdvel_2_ecu',
        output='screen',  # 日志输出到屏幕
        condition=launch.conditions.IfCondition(PythonExpression([
            "'", relative_docking_enabled, "'.lower() != 'true' and '",
            start_cmdvel_2_ecu_enabled, "'.lower() == 'true'"]))
    )

    relative_docking_config = launch.substitutions.LaunchConfiguration(
        'relative_docking_params')
    dock_mux = launch_ros.actions.Node(
        package='charging_dock', executable='command_mux', name='command_mux',
        parameters=[relative_docking_config], output='screen',
        condition=launch.conditions.IfCondition(relative_docking_enabled))
    dock_controller = launch_ros.actions.Node(
        package='charging_dock', executable='relative_docking_controller',
        name='relative_docking_controller', parameters=[relative_docking_config],
        output='screen', condition=launch.conditions.IfCondition(relative_docking_enabled))
    dock_converter = launch_ros.actions.Node(
        package='cmdvel_2_ecu', executable='cmdvel_2_ecu', name='cmdvel_to_ecu',
        parameters=[config_file, {'cmd_vel_topic': '/cmd_vel_selected',
                                  'docking_mode': False,
                                  'docking_max_speed': 0.10,
                                  'docking_max_steer_deg': 8.0,
                                  'docking_ecu_speed_scale': 3.6,
                                  'docking_stop_threshold': 0.001,
                                  'input_timeout_sec': 0.20}],
        output='screen', condition=launch.conditions.IfCondition(relative_docking_enabled))

    # 启动前后单线雷达，并使用各自的安装坐标系。
    start_scan_laser = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('lakibeam1'),
            'launch',
            'lakibeam1_scan_dual_lidar.launch.py'
        )),
        launch_arguments={
            'front_frame_id': 'front_laser',
            'rear_frame_id': 'rear_laser'
        }.items()
    )

    # 启动单线雷达融合
    start_scan_merge = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros2_laser_scan_merger'),
            'launch',
            'merge_2_scan.launch.py'
        ))
    )

    # 启动GPS节点
    start_gps = launch.actions.GroupAction(actions=[
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gnss_converter'),
                'launch',
                'gnss_converter.launch.py'
            )),
            launch_arguments={'serial_port': '/dev/ttyS8'}.items()
        )
    ])

    # 启动报警节点
    start_alarm = launch_ros.actions.Node(
        parameters=[alarm_config_file],
        package='alarm_controller',
        executable='alarm_controller',
        name='alarm_controller',
        output = 'screen' # 日志输出到屏幕
    )

    # 启动门锁
    start_lock = launch.actions.GroupAction(actions=[
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lock_controller'),
                'launch',
                'lock_controller.launch.py'
            )),
            launch_arguments={
                'serial_port': (
                    '/dev/serial/by-path/'
                    'pci-0000:00:14.0-usb-0:1.4:1.0-port0'
                )
            }.items()
        )
    ])

    # 依次启动
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=3.0, actions=[start_depth_camera]),
        launch.actions.TimerAction(period=6.0, actions=[start_pothole_detection]),
        launch.actions.TimerAction(period=1.0, actions=[start_imu]),
        # launch.actions.TimerAction(period=2.0, actions=[start_livox]),
        launch.actions.TimerAction(period=2.0, actions=[livox_custom_convert_pointcloud2]),
        launch.actions.TimerAction(period=2.0, actions=[start_rslidar]),
        launch.actions.TimerAction(period=1.0, actions=[start_car]),
        launch.actions.TimerAction(period=1.0, actions=[start_cmdvel_2_ecu]),
        launch.actions.TimerAction(period=1.0, actions=[start_alarm]),
        launch.actions.TimerAction(period=1.0, actions=[start_gps]),
        launch.actions.TimerAction(period=1.0, actions=[start_lock]),
        launch.actions.TimerAction(period=1.0, actions=[start_scan_laser]),
        launch.actions.TimerAction(period=1.0, actions=[start_scan_merge])

    ])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'enable_pothole_detection',
            default_value='false',
            description='Start the optional depth-camera pothole detector'
        ),
        launch.actions.DeclareLaunchArgument(
            'start_cmdvel_2_ecu',
            default_value='true',
            description='Start the normal /cmd_vel to /ecu converter'
        ),
        launch.actions.DeclareLaunchArgument(
            'enable_relative_lidar_docking', default_value='false',
            description='Use the exclusive measured-stop mux and relative lidar docking action'),
        launch.actions.DeclareLaunchArgument(
            'relative_docking_params', default_value=os.path.join(
                get_package_share_directory('charging_dock'), 'config',
                'relative_docking.yaml')),
        dock_mux,
        dock_controller,
        dock_converter,
        action_group,

        # # laser_link到base_link的tf2变换
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '-0.22',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'base_link', '--child-frame-id', 'laser_link'
        #     ]
        # ),

        # livox_frame到base_link的tf2变换
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_mid360_publisher',
            arguments=[
                '--x', '0.35', '--y', '0.0', '--z', '0.51',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'
            ]
        ),

        # rslidar到base_link的tf2变换
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_mid360_publisher',
            arguments=[
                '--x', '0.35', '--y', '0.0', '--z', '0.51',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'rslidar'
            ]
        ),

        # imu_link到livox_frame的tf2变换
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_imu_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '-0.11',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'livox_frame', '--child-frame-id', 'imu_link'
            ]
        ),

        # gnss_link到livox_frame的tf2变换
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_gnss_publisher',
            arguments=[
                '--x', '-0.7', '--y', '0.03', '--z', '0.0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'livox_frame', '--child-frame-id', 'gnss_link'
            ]
        ),
    ])
