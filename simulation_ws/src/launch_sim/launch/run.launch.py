import launch, os, launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    # 启动仿真机器人
    start_gazebo_simulation = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot'),
            'launch',
            'gazebo_sim.launch.py'
        )),
        launch_arguments={
            'model': '/home/akun/workspace/Car_jetson/URDF_ws/src/robot/urdf/robot/robot.urdf.xacro'  # 这里传递参数
        }.items()
    )

    # 启动时间转换
    convert_time = launch_ros.actions.Node(
        package='convert_time',
        executable='convert_time',
        output = 'screen' # 日志输出到屏幕
    )

    # 启动速度平滑
    cmd_vel_smooth = launch_ros.actions.Node(
        package='cmd_vel_smooth',
        executable='cmd_vel_smooth',
        output = 'screen' # 日志输出到屏幕
    )

    # 启动LIO-SAM
    start_LIO_SAM = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('lio_sam'),
            'launch',
            'run.launch_sim.py'
        ))
    )

    # 依次启动
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=1.0, actions=[start_gazebo_simulation]),
        launch.actions.TimerAction(period=3.0, actions=[convert_time]),
        # launch.actions.TimerAction(period=3.0, actions=[cmd_vel_smooth]),
        # launch.actions.TimerAction(period=8.0, actions=[start_LIO_SAM])
    ])

    return launch.LaunchDescription([
        action_group
    ])

    