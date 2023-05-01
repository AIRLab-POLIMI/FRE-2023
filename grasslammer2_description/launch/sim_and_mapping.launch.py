import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro 

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('grasslammer2_description'))
    slam_path = os.path.join(get_package_share_directory('slam_toolbox'))

    start_simulation = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(pkg_path, 'launch', 'simulation.launch.py')
                            ),
                )

    remove_ground_node = Node(package='ground_removal', executable='plane_filter',)

    density_filter = Node(package='map_filter', executable='map_filter',)

    pointcloud_converter = Node(package='pointcloud_to_laserscan',
                                executable='pointcloud_to_laserscan_node',
                                remappings=[('cloud_in', '/filtered_point_cloud')]
                                )

    start_slam = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(slam_path, 'launch', 'online_sync_launch.py')
                            ),
                )

    return LaunchDescription([
        start_simulation,
        remove_ground_node, 
        density_filter,
        pointcloud_converter,
        start_slam,
    ])