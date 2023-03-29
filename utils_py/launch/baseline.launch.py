import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('grasslammer2_description'))

    start_controller = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'controller.launch.py')
                                ),
    )        

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["grasslammer_velocity_controller"],
    )

    start_optimator = Node(package = 'utils_py',
                                    executable = 'optimator',
                                    name = 'optimator',
                                )

   

    return LaunchDescription([
        #start_controller,
        diff_drive_spawner,
        start_optimator
         
    ])