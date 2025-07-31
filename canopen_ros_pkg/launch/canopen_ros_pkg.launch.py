import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    canopen_ros_pkg_path = get_package_share_directory('canopen_ros_pkg')

    motor_config_file_path = LaunchConfiguration('motor_config_file_path')
    channel = LaunchConfiguration('channel')

    canopen_ros_node = Node(
        package='canopen_ros_pkg',
        executable='canopen_ros_node',
        name='canopen_ros_node',
        namespace='canopen',
        parameters=[
            {'motor_config_file_path': motor_config_file_path,
             'channel': channel}
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_config_file_path',
            default_value=os.path.join(canopen_ros_pkg_path, 'config', 'motor_config_all.json'),
            description='Motor config file path'
        ),
        DeclareLaunchArgument(
            'channel',
            default_value='can1',
            description='CAN device'
        ),
        canopen_ros_node
    ])
