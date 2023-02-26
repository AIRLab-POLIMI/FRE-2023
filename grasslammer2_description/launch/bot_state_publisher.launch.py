import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro 

def generate_launch_description():

    

    #Find the urdf in order to be published by robot state publisher
    pkg_path = os.path.join(get_package_share_directory('grasslammer2_description'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'model.urdf')
    robot_description_config = xacro.process_file(urdf_file)

    spawn_x_val = '-1'
    spawn_y_val = '0'
    spawn_z_val = '.5'
    spawn_yaw_val = '0.0'#'1.57'

    #Create robot state publisher node 
    params = {'robot_description' : robot_description_config.toxml()}
    node_robot_state_publisher = Node(
                package='robot_state_publisher', 
                executable='robot_state_publisher', 
                output='screen',
                parameters=[params]
    )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=[ '-topic', 'robot_description',
                                    '-entity', 'grasslammer',
                                    '-x', spawn_x_val,
                                    '-y', spawn_y_val,
                                    '-z', spawn_z_val,
                                    '-Y', spawn_yaw_val],
                        output='screen')

    map_odom_static_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = [spawn_x_val,
                                    spawn_y_val,
                                    spawn_z_val,
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "map", "odom"])

    odom_base_footprint_tf = Node(package = 'grasslammer2_description',
                                    executable = 'tf_builder',
                                    name = 'tf_builder',
                                )          
    rplidar_node = Node(package='rplidar_ros',
                        executable='rplidar_composition',
                        output='screen',
                        parameters=[{
                            'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                            'frame_id': 'laser_frame',
                            'angle_compensate': True,
                            'scan_mode': 'Standard'
                        }]
                    )



    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity, 
        map_odom_static_tf,
        odom_base_footprint_tf,
        rplidar_node,
    ])