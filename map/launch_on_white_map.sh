gnome-terminal -- sh -c "bash -c \" ros2 launch grasslammer2_description simulation.launch.py; exec bash\"" 
sleep 10
gnome-terminal -- sh -c "bash -c \" ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz; exec bash\""
gnome-terminal -- sh -c "bash -c \" ros2 launch grasslammer2_description bringup_launch.py map:=white.yaml; exec bash\"" 
gnome-terminal -- sh -c "bash -c \" ros2 run grasslammer2_nav_py converter_cmd_vel_sim; exec bash\"" 