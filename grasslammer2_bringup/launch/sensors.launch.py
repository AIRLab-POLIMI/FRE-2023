import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    oak_path = get_package_share_directory('depthai_examples')
    mono_lidar_path = get_package_share_directory('sllidar_ros2')
    velodyne_path = get_package_share_directory('velodyne')
    
    
    t265 = Node(
        package='t265',
        executable='publisher',
    )
    
    tf_base_link_t265 = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["-0.25", "0", "-0.29", "0", "0.087", "0", "t265", "base_footprint"]
    )
    
    mono_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mono_lidar_path, 'launch', 'sllidar_s2_launch.py')
            ),
        )
        
    tf_base_link_laser = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.18", "0", "0.04", "0", "-0.087", "0", "base_link", "laser"]
    )
    
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_path, 'launch', 'velodyne-all-nodes-VLP32e-launch.py')
            ),
        )
        
    tf_base_link_velodyne = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.18", "0", "0.17", "0", "-0.087", "0", "base_link", "velodyne"]
    )
    oak_d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(oak_path, 'launch', 'rgb_stereo_node.launch.py')
            ),
        )
    tf_base_link_oak_d = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.23", "0", "0.02", "0", "-0.087", "0", "base_link", "oak-d-base-frame"]
    )

    tf_base_link_base_footprint = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_footprint", "base_link"]
    )
       
    scan_rotation = Node(
            package = "grasslammer2_bringup",
            executable = "raw_ls_rotation"
        )

    pkg_path = os.path.join(get_package_share_directory('grasslammer2_description'))

    remove_ground_node = Node(package='ground_removal', executable='plane_filter',)

    density_filter = Node(package='map_filter', executable='map_filter',)

    pointcloud_converter = Node(package='map_filter',
                                executable='cloud_to_scan',
                                )
        
        
        
        
    return LaunchDescription([
    
        tf_base_link_t265,
        tf_base_link_laser,
        tf_base_link_velodyne,
        tf_base_link_oak_d,
        tf_base_link_base_footprint,
        
        
        t265,
        mono_lidar_launch,
        velodyne_launch, 
        #oak_d_launch, 
        scan_rotation,
        remove_ground_node,
        density_filter,
        pointcloud_converter,

    ])
