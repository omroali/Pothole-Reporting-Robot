#!/bin/bash

# Change to the workspace directory
cd ros2_ws

# Build the project using colcon
colcon build

# Source the setup file (assuming it's a bash setup file)
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ../../limo_ros2/install/setup.bash
source install/setup.bash

# Starting up Gazebo and Rviz
GAZEBO="Gazebo"
RVIZ="Rviz"
gnome-terminal --tab --title="$GAZEBO" -- bash -c "ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:='../worlds/potholes_simple.world'; bash" &
sleep 1
gnome-terminal --tab --title="$RVIZ" -- bash -c "ros2 launch limo_navigation limo_navigation.launch.py map:=../maps/simple_map.yaml params_file:=/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/params/nav2_params.yaml use_sim_time:=true; bash" &
sleep 10

# Running launch file to begin the pothile detection
gnome-terminal --tab --title="Pothole Reporter" -- bash -c "ros2 launch my_robot_bringup pothole_detection_demo.launch.py; bash" &
sleep 10

# Running waypoint detection in another window to view the data separately from the detection
SEND_WAYPOINTS="Detecting"
gnome-terminal --title="$SEND_WAYPOINTS" -- bash -c "ros2 run my_robot_controller follow_waypoint; bash"

