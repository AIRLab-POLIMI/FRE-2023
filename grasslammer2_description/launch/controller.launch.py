import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('grasslammer2_description'))
    ekf_file_path = os.path.join(pkg_path, 'config/ekf.yaml')
    
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

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
                    ekf_file_path, 
                    ]
        )

    
    return LaunchDescription([
        diff_drive_spawner, 
        joint_broad_spawner,
        start_robot_localization_cmd,
        
    ])
