import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    yolo = Node(
        package='yolotor',
        executable='yolo_node',
        name='yolo_node',
        #output='screen'
        )
    led_link = Node(
        package='yolotor',
        executable='led_link_yolo',
        name='led_link_yolo',
        #output='screen'
        )
    obstacle_detector = Node(
        package='obstacle_detector',
        executable='obstacle_detector',
        name='obstacle_detector',
        #output='screen'
        )

    return LaunchDescription([
        yolo,
        led_link,
        obstacle_detector,
    ])