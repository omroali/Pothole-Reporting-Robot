# Summary of Solution
This repository contains my implementation of a pothole detector using the provided simple world map. The solution makes use of OpenCV to evaluate the pothole colours and thus their contours, as well as using the Depth sensing camera to determine to the position of a given pothole relative to the world frame. The robot navigates around the map using a set of waypoints and detects the potholes positions and size. Once the run is complete, a summary report will be produced, which can be found in the `Pothole-Reporting-Robot/ros2_ws/summary_reports` to be evaluated.

# Guideline to run the code
This assumes that `ros2 humble` is already installed onto the system this will run on. This has also only been configured and tested on a native enviroments of ROS2.

### Additional python packages to include
```shell
pip install reportlab
pip install matplotlib
```

### Configuration
From a desired directory you will need to clone the limo_ros2 repository, as in the native instructions in https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup.
In a folder on the same level as the `limo_ros2` installation, clone this `Pothole-Reporting-Robot` repository
```shell
git clone https://github.com/Olseda20/Pothole-Reporting-Robot.git
```
Your file structure should look like this. This is important due to how some of the automation of the pothole detection is being run.
```
.
├── limo_ros2
│   ├── src
│   ├── install
│   ├── build
│   └── ... 
└── Pothole-Reporting-Robot
    ├── README.md
    ├── ros2_ws
    └── ... 
```
Once both directories are available change into the `Pothole-Reporting-Robot` directory.
```shell
cd Pothole-Reporting-Robot
```

Ideally if the rviz2 config file could be loaded into rviz by default, this would be useful in seeing the markers of the pothole in realtime. However this information will be produced in the summary report.

From here all that needs to be run is the the shell script. This will start up
1. Gazbo
2. Rviz2
3. Launch the my_robot_bringup launch file
4. Begin running follow_waypoint to begin recording the pothole position

If zsh is installed on your system, this can be started by making sure it is executable and simply running the script
```shell
chmod +x run_pothole_detector.sh
./run_pothole_detector.sh
```
Otherwise use the bash variant
```shell
chmod +x run_pothole_detector_bash.sh
./run_pothole_detector_bash.sh
```

Now enjoy as the potholes are being detected.   
Once this is complete, navigate to the directory `Pothole-Reporting-Robot/ros2_ws/summary_reports` to see the output of the detection.

A demo run can be seen here: https://www.youtube.com/watch?v=fvODO8_kK-I

---
### To manually deploy the parts 
If you would like to run the different sections independently, navigate to the `Pothole-Reporting-Robot/ros2_ws` directory and run:
For every new terminal, please made sure all of the `source` setups files are being run. 
Note: please use the correct setup for your interpreter
```shell
source /opt/ros/humble/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
colcon build
source ../../limo_ros2/install/setup.zsh
source install/setup.zsh
```

To start Gazebo:
```
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:='../worlds/potholes_simple.world'
```

In a new terminal to start RViz:
```
ros2 launch limo_navigation limo_navigation.launch.py map:=../maps/simple_map.yaml params_file:=/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/params/nav2_params.yaml use_sim_time:=true;
```

In a new terminal to start the node for pothole detection
```
ros2 run my_robot_controller simple_pothole_detector
```

In a new terminal to start the node to publishing the pothole positions as markers in the '/odom' frame
```
ros2 run my_robot_controller pothole_mapper
```

In a new terminal to begin the waypoint following and report generation
```
ros2 run my_robot_controller pothole_reporter
```
