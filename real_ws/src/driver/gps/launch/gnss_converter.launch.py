from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for GNSS receiver'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gnss_link',
        description='Frame ID for GNSS data'
    )

    # GNSS Converter 节点
    gnss_converter_node = Node(
        package='gnss_converter',
        executable='gnss_converter_node',
        name='gnss_converter_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        frame_id_arg,
        gnss_converter_node,
    ])
