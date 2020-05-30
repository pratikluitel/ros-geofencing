CoppeliaSim Quadcopter interfacing with ROS

Makes the simulation's Quadricopter a ROSnode which the master node can communicate with.

Note: MAKE SURE YOU RUN ROSCORE BEFORE OPENING COPPELIASIM

You can control the Quadricopter by publishing a 3D multiarray to the targetCoord and targetRotation topics.

Message format:
rostopic pub /targetCoord[id of quadcopter] std_msgs/Float32MultiArray "{layout : {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [0,1.6,0]}"

Note: the id is assigned randomly (afaik) by the ROS middleware, use autocomplete to find out the id of the lone quadcopter.
