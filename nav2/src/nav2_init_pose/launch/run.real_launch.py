from launch_ros.actions import Node
import launch, os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 启动匹配服务端
    start_service = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('re_localization'),
            'launch',
            'run.real_launch.py'
        ))
    )

    # 启动客户端
    start_nav2_init_pose_node = Node(
        package='nav2_init_pose',
        executable='nav2_init_pose',
        output = 'screen' # 日志输出到屏幕
    )

    return launch.LaunchDescription([
        start_service,
        start_nav2_init_pose_node
    ])