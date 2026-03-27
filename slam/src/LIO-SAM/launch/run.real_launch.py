import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')

    # 机器人tf变换路径
    # xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro') # 数据集修改，在rviz中加载liosam默认的小车模型
    # xacro_path = os.path.join(get_package_share_directory("robot"), 'urdf/robot/robot.urdf.xacro') # 仿真修改，在rviz中加载仿真的小车模型

    # rviz2配置文件路径
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    # liosam配置文件路径（包含GPS转换配置）
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params_rslidar.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    # print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        # 注释掉静态TF，让LIO-SAM在GPS融合时动态管理map->odom变换
        # Node(
        #      package='tf2_ros',
        #      executable='static_transform_publisher',
        #      arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        #      parameters=[parameter_file],
        #      output='screen'
        #      ),   

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': Command(['xacro', ' ', xacro_path]),
        #     }]
        # ),

        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',           
            name='ekf_gps',
            output='screen',
            respawn=True,  
            remappings=[
                ('odometry/filtered', 'odometry/navsat'),
            ],
            # 如果你有參數檔，通常會這樣帶入
            # parameters=['path/to/ekf_gps.yaml']
        ),

        # GPS转换节点 - 将NavSatFix转换为Odometry
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[parameter_file],
            remappings=[
                ('imu/data', '/imu'), # 节点输入：航向角imu话题
                ('gps/fix', '/sensing/gnss/pose_with_covariance'), # 节点输入：GPS
                ('odometry/filtered', '/odometry/filtered_local'), # （可选）节点输入：可选的局部里程计输入
                ('odometry/gps', '/odometry/gpsz'), # 节点输出：GPS里程计话题
                ('gps/filtered', '/gps/filtered'),  # 节点输出：过滤后的GPS数据
            ]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
