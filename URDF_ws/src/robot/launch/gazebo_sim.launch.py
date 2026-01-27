import launch
import launch.launch_description_sources
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from ament_index_python import get_package_share_directory
import os

import launch_ros.parameter_descriptions
def generate_launch_description():
    
    os.environ['GAZEBO_MASTER_URI'] = 'http://localhost:11346'
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(get_package_share_directory("robot"), 'models')

    # 获取机器人的xacro文件路径
    xacro_path_default = os.path.join(get_package_share_directory("robot"), 'urdf/robot/robot.urdf.xacro')

    # rviz2配置文件
    # default_rviz_config_path = os.path.join(get_package_share_directory("robot"), 'rviz_config/robot_model.rviz')
    
    # 获取gazebo布置好的世界路径
    default_gazebo_world_path = os.path.join(get_package_share_directory("robot"), 'world/large.world')
    
    # 模型文件路径参数
    urdf_param = DeclareLaunchArgument(
            name = 'model',
            default_value = str(xacro_path_default),
            description = "加载的模型路径"
        )

    # 打印机器人模型路径
    print_model_path = LogInfo(msg=['模型路径: ', launch.substitutions.LaunchConfiguration('model')])

    # 获取xacro文件的内容，转换为urdf参数值对象，以供传入robot_state_publisher
    cmd_result = launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(cmd_result, value_type=str)

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description':robot_description_value}]
        )

    gazebo_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
           [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py'] 
        ),
        launch_arguments=[('world', default_gazebo_world_path),('verbose','true')] # 开启日志
    )

    urdf2sdf = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description',
                    '-entity','robot'] # 通过话题加载urdf,机器人实体名字为robot
    )  

    return LaunchDescription([   
        gazebo_launch,
        urdf_param,
        robot_state_publisher,
        urdf2sdf,
        print_model_path,

    ])