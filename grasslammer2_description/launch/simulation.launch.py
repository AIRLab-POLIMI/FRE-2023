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
    pkg_path_joy = os.path.join(get_package_share_directory('joystick_ros2'))

    start_gazebo = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'gaz_world.launch.py')
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
    start_ekf = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'ekf_launcher.launch.py')
                                ),
    )

    
    # Control robot using joystick
    teleop_joy = GroupAction(
                    actions=[

                        SetRemap(src='/cmd_vel',dst='/grasslammer_velocity_controller/cmd_vel_unstamped'),

                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(pkg_path_joy + '/launch/joystick.launch.py'),
                        )
                    ]
                )           
    

    

    return LaunchDescription([
        start_gazebo,
        start_urdf,
        start_control,
        #start_ekf,
        #teleop_joy,
         
    ])