import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro 

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('grasslammer2_description'))

    density_filter = Node(package='map_filter', executable='map_filter_approx_ransac',)

    pointcloud_converter = Node(package='map_filter',
                                executable='cloud_to_scan_ransac',
                                )

    return LaunchDescription([
        density_filter,
        pointcloud_converter,
    ])