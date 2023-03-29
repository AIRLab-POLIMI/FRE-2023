import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        respawn = False,
        arguments=["grasslammer_velocity_controller"],
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        respawn = False,
        arguments=["joint_state_broadcaster"],
    )

    
    return LaunchDescription([
        diff_drive_spawner, 
        joint_broad_spawner,
        
    ])
