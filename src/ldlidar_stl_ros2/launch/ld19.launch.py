#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # LD LiDAR node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'Lidar_Link'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},

            # Dirección del escaneo
            {'laser_scan_dir': True},

            # Recorte angular
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )


    # Publica el robot_description del URDF
    urdf_file = os.path.join(
    get_package_share_directory('mecanum_bridge'),
    'urdf',
    'CarroOmnidireccional.urdf'
    )


    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ]
    )


    return LaunchDescription([
        ldlidar_node,
        robot_state_publisher
    ])
