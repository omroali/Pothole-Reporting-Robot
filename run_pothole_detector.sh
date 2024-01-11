#!/bin/zsh

# Change to the workspace directory
cd ros2_ws

# Build the project using colcon
colcon build

# Source the setup file (assuming it's a zsh setup file)
source /opt/ros/humble/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
source ../../limo_ros2/install/setup.zsh
source install/setup.zsh

# Starting up Gazebo and Rviz
GAZEBO="Gazebo"
RVIZ="Rviz"
gnome-terminal --tab --title="$GAZEBO" -- zsh -c "ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:='../worlds/potholes_simple.world'; zsh -i" &
sleep 1
gnome-terminal --tab --title="$RVIZ" -- zsh -c "ros2 launch limo_navigation limo_navigation.launch.py map:=../maps/simple_map.yaml params_file:=/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/params/nav2_params.yaml use_sim_time:=true; zsh -i" &
sleep 10

# Running launch file to begin the pothile detection
ros2 launch my_robot_bringup pothole_detection_demo.launch.py
