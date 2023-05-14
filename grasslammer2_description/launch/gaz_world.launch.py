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

    # set path to this package 
    path_pkg = os.path.join(get_package_share_directory('grasslammer2_description'))

    #set path to gazebo_ros package
    path_gz = os.path.join(get_package_share_directory('gazebo_ros'))

    #set path to world file 
    world_file = 'cust_fre_nav_3.world' # World options are the ones in /worlds folder 

    




    path_world = os.path.join(path_pkg, 'worlds', world_file) # uncomment if you want to use you're custom world 
    #path_world='' # uncomment if you want to use the empty world
    


    gazebo_models_path = os.path.join(path_pkg, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path



    ###################### Set gazebo variables ####################
    
    ############ SIM TIME 
    #use sim time -> set to true 
    use_sim_time = LaunchConfiguration('use_sim_time') #take the one given
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use sim time',
    ) #declaring launch argument setting sim_time to true


    ############ WORLD 
    world = LaunchConfiguration('world') #trigger the gazebo variable
    world_arg = DeclareLaunchArgument(
            name='world', default_value=path_world, description='Full path to the world model file to load'


    )

    start_gazebo_server = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(path_gz, 'launch', 'gzserver.launch.py')
                            ),
                            launch_arguments={'world': world}.items()
                        )
 
    # Start Gazebo client    
    start_gazebo_client = IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    os.path.join(path_gz, 'launch', 'gzclient.launch.py')
                                ),
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        start_gazebo_server,
        start_gazebo_client,

    ])
 


