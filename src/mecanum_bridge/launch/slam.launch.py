import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'mecanum_bridge'

    slam_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'slam.yaml'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config
        ]
    )

    return LaunchDescription([
        slam_toolbox_node
    ])
