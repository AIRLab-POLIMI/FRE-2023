import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():


    laser_reader = Node(
        package='grasslammer2_nav_py',
        executable='laser_reader',
        name='laser_reader',
        #output='screen'
        )



    return LaunchDescription([
        laser_reader, 
        
    ])
