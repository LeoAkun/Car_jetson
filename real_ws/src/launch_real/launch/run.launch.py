import launch, os, launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

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
    start_cmdvel_2_ecu = launch_ros.actions.Node(
        parameters=[config_file],
        package='cmdvel_2_ecu',
        executable='cmdvel_2_ecu',
        output = 'screen' # 日志输出到屏幕
    )

    # 启动单线雷达，注意gnss的驱动把frame_id的launch参数设置为了gnss_link，所以会覆盖掉后面启动的launch参数，因此手动设置参数
    start_scan_laser = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('lakibeam1'),
            'launch',
            'lakibeam1_scan_dual_lidar.launch.py'
        )),
        launch_arguments={
            'frame_id': 'laser'
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
    start_gps = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gnss_converter'),
            'launch',
            'gnss_converter.launch.py'
        ))
    )

    # 启动报警节点
    start_alarm = launch_ros.actions.Node(
        parameters=[config_file],
        package='alarm_controller',
        executable='alarm_controller',
        output = 'screen' # 日志输出到屏幕
    )

    # 启动门锁
    start_lock = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('lock_controller'),
            'launch',
            'lock_controller.launch.py'
        ))
    )

    # 依次启动
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=1.0, actions=[start_imu]),
        launch.actions.TimerAction(period=2.0, actions=[start_livox]),
        launch.actions.TimerAction(period=2.0, actions=[livox_custom_convert_pointcloud2]),
        launch.actions.TimerAction(period=1.0, actions=[start_car]),
        launch.actions.TimerAction(period=1.0, actions=[start_cmdvel_2_ecu]),
        # launch.actions.TimerAction(period=1.0, actions=[start_alarm]),
        launch.actions.TimerAction(period=1.0, actions=[start_gps]),
        # launch.actions.TimerAction(period=1.0, actions=[start_lock]),
        launch.actions.TimerAction(period=1.0, actions=[start_scan_laser]),
        launch.actions.TimerAction(period=1.0, actions=[start_scan_merge])
    ])

    return launch.LaunchDescription([
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

    