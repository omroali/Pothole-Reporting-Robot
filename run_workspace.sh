#!/bin/zsh

# Change to the workspace directory
cd /home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/ros2_ws

# Build the project using colcon
colcon build

# Source the setup file (assuming it's a zsh setup file)
source /opt/ros/humble/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
source ../../limo_ros2/install/setup.zsh
source install/setup.zsh
# Run the commands
#
# Open a new Zsh terminal and run the commands
GAZEBO="Gazebo"
RVIZ="Rviz"

gnome-terminal --tab --title="$GAZEBO" -- zsh -c "ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:='../worlds/potholes_simple.world'; zsh -i" &
sleep 1
gnome-terminal --tab --title="$RVIZ" -- zsh -c "ros2 launch limo_navigation limo_navigation.launch.py map:=../maps/simple_map.yaml params_file:=/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/params/nav2_params.yaml use_sim_time:=true; zsh -i" &
sleep 10

gnome-terminal --tab --title="Pothole Reporter" -- zsh -c "ros2 launch my_robot_bringup pothole_detection_demo.launch.py; zsh -i"
sleep 10
# ros2 run my_robot_controller follow_waypoint

SEND_WAYPOINTS="Detecting"
gnome-terminal --title="$SEND_WAYPOINTS" -- zsh -c "ros2 run my_robot_controller follow_waypoint; zsh -i"
#
#
# # Wait for the terminal to be killed
# while pgrep -f "$SEND_WAYPOINTS" > /dev/null; do
#     sleep 1
# done
#
# pkill -f "$GAZEBO"
# pkill -f "$RVIZ"
#
#
#
#


# ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:='/home/krono/dev/RobotProgramming/CMP9767_LIMO/assignment_template/worlds/potholes_simple.world'
# ros2 launch limo_navigation limo_navigation.launch.py map:=/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/saved_maps/simple_map.yaml use_sim_time:=true
