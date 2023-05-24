import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  # Set the path to different files and folders. 
  pkg_share = os.path.join(get_package_share_directory('grasslammer2_description'))
  
  robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
  
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

   
  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    respawn = True,
    parameters=[robot_localization_file_path, 
                {'use_sim_time': use_sim_time},
                ])

  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options

  ld.add_action(declare_use_sim_time_cmd)
  

  # Add any actions
  
  ld.add_action(start_robot_localization_cmd)

  return ld