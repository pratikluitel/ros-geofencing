#!/usr/bin/env python

import sys
import json
import rospy
import numpy as np
from shapely.geometry import Point, Polygon
from std_msgs.msg import Float32MultiArray, String

pub = rospy.Publisher('/pointStack', Float32MultiArray, queue_size=100)
breachPub = rospy.Publisher('/geofence_status', String, queue_size=100)

#for kicks
zfence = (-1, 5.5)
polys = []

stack = []

#mostly for kicks
breach = False


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

    p = Point(gps[0], gps[1])

    for poly in polys:
        if p.within(poly):
            breach = True
            breachPub.publish("breached")
            return

    if gps[2] < zfence[0] or gps[2] > zfence[1]:
        breach = True
        breachPub.publish("breached")
        return
    
    breach = False
    breachPub.publish(" ")
    pass

def listener():
    rospy.init_node('main', anonymous=True)

    rate = rospy.Rate(100)
    rospy.Subscriber('/GPSdata', Float32MultiArray, callback)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    with open(sys.argv[1]) as f:
        data = json.load(f)

    for feature in data["features"]:
        if feature["geometry"]["type"] == "Polygon":
            polys.append(Polygon(feature["geometry"]["coordinates"]))

    print("Node running")
    listener()
