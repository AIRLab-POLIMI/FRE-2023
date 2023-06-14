import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ROSTimer


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('grasslammer2_bringup'))

    sensors = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'sensors.launch.py')
                                ),
    )

    navigation = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'navigation.launch.py')
                                ),
    )
    mapping = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'mapping.launch.py')
                                ),
    )
    turning = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'turning.launch.py')
                                ),
    )    
    ransac_filters = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(pkg_path, 'launch', 'ransac_filters.launch.py')
                                ),
    )

    switcher = Node(
        package='grasslammer2_nav_py',
        executable='switcher',
        name='switcher',
        output='screen'
        )


    return LaunchDescription([
        sensors,
        ransac_filters,
        ROSTimer(period=3.0, actions=[        
            mapping,
            turning,
            navigation,
        ]),        
        ROSTimer(period=6.0, actions=[        
            switcher
        ]),
    ])