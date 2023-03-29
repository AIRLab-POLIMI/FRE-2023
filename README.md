# Grasslammer2

This repo contains the grasslammer simulation using ros2. 

## Installation

In order to launch the simulation you first have to clone the repository and to install the required dependencies. 

```bash
git clone https://github.com/t-Zeta/grasslammer2_common.git
```

## Usage
There are currently two branches: `main`, `navigation`. The second already contains some test code for the navigation task. It is recommended to create a branch for every sub-team. 

To launch the simulation you have to `colcon build` your environment and then you can use `ros2 launch grasslammer2_simulation simulation.launch.py`. It is a nested launch file, indeed it includes other launch files. You can change the simulation world inside `gaz_world.launch.py` using the **world_file** variable. 

To teleop the robot you can use either `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/grasslammer_velocity_controller/cmd_vel_unstamped` or uncomment the **#teleop_joy** line (57) in the simulation's launch file.
