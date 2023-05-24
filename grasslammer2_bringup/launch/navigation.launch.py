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

    in_row_navigation = Node(
        package='grasslammer2_nav_py',
        executable='in_row_navigation',
        name='in_row_navigation',
        #output='screen'
        )

    end_of_line_detection = Node(
        package='grasslammer2_nav_py',
        executable='end_of_line_detection',
        name='end_of_line_detection',
        #output='screen'
        )
    
    aut_nav_cmd = Node(
        package='control_int',
        executable='aut_nav',
        name='aut_nav_cmd',
        #output='screen'
        )




    return LaunchDescription([
        laser_reader, 
        in_row_navigation,
        end_of_line_detection,
        aut_nav_cmd, 

        
    ])