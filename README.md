# CoppeliaSim Quadcopter interfacing with ROS

A Quadcopter geofencing simulation.

## Dependencies

[ROS Melodic Morena](http://wiki.ros.org/melodic/Installation/Ubuntu) install the desktop version (ros-melodic-desktop)

[Coppelia Sim](https://www.coppeliarobotics.com/downloads)

python shapely 
```
sudo apt install python-shapely
```

## Steps

1. Start roscore.
```
roscore
```

2. Start coppeliasim (through GUI or a new terminal)
```
path_to_coppeliasim/coppeliaSim.sh
```

3. Open the simulation in CoppeliaSim - the latest is in 
`SimulationFiles/Demo Simulations/parkscene.ttt`. The boundaries for the geofence are in the 
`parkscene.geojson` file.

#### Note: The final demo, using the pulchowk campus geojson map is in the sprint_pulchowk_obj.ttt file in the same folder

## To run the breach detection test with manual w a s d keyboard control:

`./manual_control_breach_test.sh`

## To run the path planning test:

`./path_follow_test.sh`

Then press the 'o' key in CoppeliaSim to record the start point (which right now is the point where the drone is at)
Note: the end points can be changed by changing the end_point variable in `catkin_ws/src/path_planner/scripts/main.py`

## To run breach detection in a new geojson file:

Source the setup file:

`source setup.sh`

Start the breach detector and rerouter script in two new terminals as:

`rosrun geofencer breach_detector.py [geojson file for scene]`

`rosrun geofencer control.py`

## To run the path planner in a new geojson map:

Source the setup file:

`source setup.sh`

`rosrun path_planner main.py [geojson file for scene]`


Note: The pulchowk map's offseted coordinates are in the Offsetted_Map.png file

## To run the waypoint follower:

`./waypoint_test.sh`

to change/add waypoints - edit the `waypoints` list in `catkin_ws/src/path_planner/scripts/waypoint.py`
