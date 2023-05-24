import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    disparity_viewer = Node(
                    package="image_view",
                    executable="stereo_view",
                    remappings=[
                                ('/stereo/left/image', '/left/image_rect'),
                                ('/stereo/right/image', '/right/image_rect'),
                                ('/stereo/disparity', '/disparity'),
                    ]
    )
    return LaunchDescription([

        disparity_viewer,


    ])