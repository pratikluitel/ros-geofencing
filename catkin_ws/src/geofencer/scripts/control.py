#!/usr/bin/env python

import sys
import json
import rospy
import numpy as np
from shapely.geometry import Point, Polygon
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher('/pointStack', Float32MultiArray, queue_size=100)
zfence = (-1, 5.5)
polys = []

stack = []

breach = False


def callback(data):
    print("Data received, processing")
    gps = data.data
    
    stack.append([gps[0],gps[1],gps[2],0,0,0,1.0,0,15,0.5,0.5])

    if len(stack) > 10:
        stack.pop(0)

    info = Float32MultiArray()
    info.data = list(np.array(stack).reshape((len(stack)*11,)))    

    pub.publish(info)

    p = Point(gps[0], gps[1])

    for poly in polys:
        if p.within(poly):
            breach = True
            return

    if gps[2] < zfence[0] or gps[2] > zfence[1]:
        breach = True
        return
    
    breach = False

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
