# joystick_ros2 :video_game:
Use joypad to teleop your robot in ROS2 


### How to launch
To launch the joystick package, you can place those lines in the `launch` file of your personal package.

```python
def generate_launch_description():
        pkg_path_joy = os.path.join(get_package_share_directory('joystick_ros2'))
		
        # Control robot using joystick
        teleop_joy = GroupAction(
                        actions=[
    
                            SetRemap(src='/cmd_vel',dst='/your_robot_name/cmd_vel_unstamped'),
    
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(pkg_path_joy + '/launch/joystick.launch.py'),
                            )
                        ]
                    )           
        
    
        
    
        return LaunchDescription([
            #other nodes, 
            teleop_joy,
             
        ])`
```
### Remapping
To remap the joystick buttons  you can modify `joystick.yaml`. Numbers after *x* and *yaw* under **axis_linear** and **axis_angular** are the joystick buttons ID (to listen them use  */joy topic*). 
