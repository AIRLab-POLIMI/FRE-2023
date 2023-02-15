import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('grasslammer2_description'))
    pkg_path_fre = os.path.join(get_package_share_directory('virtual_maize_field')) # I want to launch the gaz_sim.launch.py that is the launch file for the 2022 FRE environment
    


    start_gazebo = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'gaz_world.launch.py')
                                ),
    )
    start_gazebo_fre = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path_fre, 'launch', 'gaz_sim.launch.py')
                                ),
    )


    start_urdf = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'bot_state_publisher.launch.py')
                                ),
    )

    start_control = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'controller.launch.py')
                                ),
                    )
    

    

    return LaunchDescription([
        start_gazebo,
        #start_gazebo_fre,
        start_urdf,
        start_control,
         
    ])