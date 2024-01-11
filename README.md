# Pothole-Reporting-Robot
hello
## Docker setup 
following https://www.docker.com/blog/containerized-python-development-part-1/
```
docker build -t prr-project .
```

## Limo Repo
https://github.com/Olseda20/CMP9767_LIMO


## Assignment setup from wiki

```
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes_simple.world
```

the realistic map is called potholes.world
path is found at 
```
/opt/ros/lcas_addons/src/limo_ros2/src/limo_gazebosim/worlds/potholes_simple.world
```



```
limo-setup
limo-gazebo-simple-world

#to move robot
teleop stuff

#to make map
limo-slam-toolbox-default

#to use map
limo-navstack-rviz map:=/path/to/map.yaml use_sim_time:=true

# image capture https://github.com/LCAS/CMP9767_LIMO/wiki/Workshop-9-%E2%80%90-Robot-Vision
maybe do the workshop
```
pip install reportlab
