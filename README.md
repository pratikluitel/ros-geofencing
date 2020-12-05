# CoppeliaSim Quadcopter interfacing with ROS

A Quadcopter geofencing simulation.

## Dependencies

ROS Melodic Morena - [Installation](http://wiki.ros.org/melodic/Installation/Ubuntu) - install ros-melodic-desktop
Coppelia Sim
python shapely - `sudo apt install python-shapely`

## Steps

1. Start roscore.
`roscore`

2. Start coppeliasim (through GUI or a new terminal)
`path_to_coppeliasim/coppeliaSim.sh`

3. Build catkin_ws and source it using setup.sh in a new terminal.
```
source setup.sh
```

4. Open the simulation in CoppeliaSim - the latest is in SimulationFiles/Demo Simulations/parkscene.ttt. The boundaries for the geofence are in the parkscene..geojson file.

5. Start the breach detector and rerouter script in a new terminal.
`rosrun geofencer control.py parkscene.geojson`

6. You can control the Quadricopter by using arrow keys.

