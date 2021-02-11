#!/usr/bin/env python

import sys
import json
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, String

pub = rospy.Publisher('/pointStack', Float32MultiArray, queue_size=100)

stack = []

def callback(data):
    print("Data received, processing")
    gps = data.data
    
    stack.append([gps[0],gps[1],gps[2],0,0,0,1.0,0,15,0.5,0.5])

    info = Float32MultiArray()

    # flip the array vertically because we need to retrieve the last point first in lua
    actualStack = np.flip(np.array(stack),0)
    
    #send a flattened list, because lua cant handle multiple dimensions
    msg = list(actualStack.reshape((len(actualStack)*11,)))

    #add length of stack to the end, we need this info to remove control points in the simulation
    msg.append(len(actualStack))
    info.data = msg

    pub.publish(info)
    
    if len(stack) > 20:
        stack.pop(0)
    pass

def listener():
    rospy.init_node('main', anonymous=True)

    rate = rospy.Rate(100)
    rospy.Subscriber('/GPSdata', Float32MultiArray, callback)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    print("Node running")
    listener()
