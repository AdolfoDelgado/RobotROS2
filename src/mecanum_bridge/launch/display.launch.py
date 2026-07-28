import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_file_name = 'CarroOmnidireccional.urdf'
    urdf = os.path.join(
        get_package_share_directory('mecanum_bridge'),
        'urdf',
        urdf_file_name
    )

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
    ])
