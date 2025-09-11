from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xsens_launch = os.path.join(
        get_package_share_directory('xsens_mti_ros2_driver'),
        'launch',
        'xsens_mti_node.launch.py'
    )

    return LaunchDescription([
        Node(
            package='imu_ahrs',
            executable='imu_ahrs',
            name='imu_ahrs',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(xsens_launch)
        )
    ])