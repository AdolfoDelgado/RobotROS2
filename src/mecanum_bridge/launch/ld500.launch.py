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

    # Robot
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc}
        ],
    )

    # RViz
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # LiDAR
    lidar_node = launch_ros.actions.Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'Lidar_Link'},      # Debe coincidir con tu URDF
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
        ],
    )

    return launch.LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        lidar_node,
    ])
