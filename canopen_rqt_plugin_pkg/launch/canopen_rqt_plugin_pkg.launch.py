from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    canopen_controller = Node(
        package='canopen_rqt_plugin_pkg',
        executable='run_rqt',
        name='canopen_controller',
        namespace='canopen',
        output='screen'
    )

    return LaunchDescription([
        canopen_controller
    ])