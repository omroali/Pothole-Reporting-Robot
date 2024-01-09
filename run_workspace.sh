#!/bin/zsh

# Change to the workspace directory
cd /home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/ros2_ws

# Build the project using colcon
colcon build

# Source the setup file (assuming it's a zsh setup file)
source /opt/ros/humble/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
source ../limo_ros2/install/setup.zsh
source install/setup.zsh
# Run the commands
#
# Open a new Zsh terminal and run the commands
gnome-terminal --tab --title="Gazebo" -- zsh -c "ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:='/home/krono/dev/RobotProgramming/CMP9767_LIMO/assignment_template/worlds/potholes_simple.world'; zsh -i"
sleep 5 
gnome-terminal --tab --title="Rviz" -- zsh -c "ros2 launch limo_navigation limo_navigation.launch.py map:=/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/saved_maps/simple_map.yaml use_sim_time:=true; zsh -i"
sleep 5

# ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:='/home/krono/dev/RobotProgramming/CMP9767_LIMO/assignment_template/worlds/potholes_simple.world'
# ros2 launch limo_navigation limo_navigation.launch.py map:=/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/saved_maps/simple_map.yaml use_sim_time:=true

