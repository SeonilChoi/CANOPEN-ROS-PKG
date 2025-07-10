import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    canopen_ros_pkg_path = get_package_share_directory('canopen_ros_pkg')
    canopen_rqt_plugin_pkg_path = get_package_share_directory('canopen_rqt_plugin_pkg')

    canopen_ros_pkg_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                canopen_ros_pkg_path, 'launch', 'canopen_ros_pkg.launch.py'
            )
        )
    )

    canopen_rqt_plugin_pkg_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                canopen_rqt_plugin_pkg_path, 'launch', 'canopen_rqt_plugin_pkg.launch.py'
            )
        )
    )

    return LaunchDescription([
        canopen_ros_pkg_launcher,
        canopen_rqt_plugin_pkg_launcher
    ])
