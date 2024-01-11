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


from a desired directory you will need to clone the limo_ros2 repository and the Pothole-Reporting-Robot repository
```
git clone limo_ros2
git clone Pothole-Reporting-Robot
```
this should produce 2 files in parallel with a structure similar to this
```
.
├── limo_ros2
│   ├── build
│   ├── install
│   ├── log
│   └── src
└── Pothole-Reporting-Robot
    ├── build
    ├── install
    ├── log
    ├── ros2_ws
    └── src
```
Once both directories are available change into the Pothole-Reporting-Robot directory.
```
cd Pothole-Reporting-Robot
```

From here all that needs to be run is the the shell script that will start up
- Gazbo
- Rviz2
- Launch the my_robot_bringup launch file
- Begin running follow_waypoint

If zsh is installed on your system, this can be started by simply running
```
./run_workspace.sh
```

otherwise use the bash variant instead
```
./run_workspace_bash.sh
```





### requirements

