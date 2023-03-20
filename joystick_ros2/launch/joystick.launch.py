import os 

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description():

    joystick_parameters = os.path.join(get_package_share_directory('joystick_ros2'), 'config', 'joystick.yaml')

    joy_node = Node(
                package = 'joy',
                executable = 'joy_node',
                parameters = [joystick_parameters],
    )

    teleop_node = Node(
                package = 'teleop_twist_joy',
                executable = 'teleop_node', 
                name = 'teleop_node',
                parameters = [joystick_parameters]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])