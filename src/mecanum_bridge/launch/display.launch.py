from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    rviz_config = os.path.join(
    get_package_share_directory('mecanum_bridge'),
    'rviz',
    'mecanum_slam.rviz'
)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
    ])