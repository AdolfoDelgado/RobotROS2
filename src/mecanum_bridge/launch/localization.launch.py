import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'mecanum_bridge'

    pkg_share = get_package_share_directory(package_name)

    map_file = os.path.expanduser(
        '~/maps/cuarto.yaml'
    )

    amcl_config = os.path.join(
        pkg_share,
        'config',
        'amcl.yaml'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': False}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl'
                ]
            }
        ]
    )

    return LaunchDescription([
        map_server,
        amcl,
        lifecycle_manager
    ])