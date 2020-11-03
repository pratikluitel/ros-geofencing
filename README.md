# CoppeliaSim Quadcopter interfacing with ROS

Makes the simulation's Quadricopter a ROSnode which the master node can communicate with.

1. Build catkin_ws and source it using setup.sh
```
source setup.sh
```

2. start roscore
`roscore`

3. start coppeliasim

4. start the breach detector
`rosrun geofencer breach_detector.py`

5. You can control the Quadricopter by publishing a 3D multiarray to the targetCoord and targetRotation topics.

Message format:

`rostopic pub /targetCoord std_msgs/Float32MultiArray "{layout : {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [0,1.6,0]}"`

