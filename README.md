# Pothole-Reporting-Robot


# Summary of Solution
This repository contains my implementation of a pothole detector in a simpler world map. 
It makes use of OpenCV to evaluate the pothole colours and contours as well as a Depth sensing camera to determine to the position of a given pothole relative to the world frame. 
The robot navigates around the map using a set of waypoints and detects the potholes positions, allowing for slight flexibility in (x,y) coordinates and pothole size. 
Once the run is complete it will produce a summary report which can be found in the `Pothole-Reporting-Robot/ros2_ws/summary_reports` folder.

# Guideline for run
This assumes that 'ros2 humble' is already installed onto the system this will run on.

### additional requirements include
pip install reportlab


### running the code
This has only been configured and tested on a native enviroment of ROS2.

From a desired directory you will need to clone the limo_ros2 repository, as in the native instructions in https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup.
In a folder next to the limo_ros2 installation, clone the Pothole-Reporting-Robot repository
```
git clone https://github.com/Olseda20/Pothole-Reporting-Robot.git
```
Your file structure should have both these folders on the same level. This is important due to how some of the automation of the pothole detection is being run.
```
.
├── limo_ros2
│   ├── src
│   ├── install
│   ├── build
│   └── ... 
└── Pothole-Reporting-Robot
    ├── src
    ├── ros2_ws
    └── ... 
```
Once both directories are available 'cd' into the `Pothole-Reporting-Robot` directory.
```
cd Pothole-Reporting-Robot
```

Ideally if the rviz2 config file could be loaded into rviz by default, this would be useful in seeing the markers of the pothole in realtime. However this information will be produced in the summary report.

From here all that needs to be run is the the shell script. This will start up
- Gazbo
- Rviz2
- Launch the my_robot_bringup launch file
- Begin running follow_waypoint to begin recording the pothole position

If zsh is installed on your system, this can be started by simply running
```
./run_workspace.sh
```

otherwise use the bash variant
```
./run_workspace_bash.sh
```

Now enjoy as the potholes are being detected and once this is complete you can navigate to the 
`Pothole-Reporting-Robot/ros2_ws/summary_reports` directory to see the output of the detection.

