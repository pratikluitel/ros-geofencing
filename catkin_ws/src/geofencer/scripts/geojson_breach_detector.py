#!/usr/bin/env python

import sys
import json
import rospy
from shapely.geometry import Point, Polygon
from std_msgs.msg import String, Float32MultiArray, Empty

pub = rospy.Publisher('/geofence_status', String, queue_size=100)
#flag = 0
zfence = (-1, 5.5)
polys = []

def callback(data):
    #print("Data received, processing")
    gps = data.data
    print(gps)
    p = Point(gps[0], gps[1])

    for poly in polys:
        if p.within(poly):
            print("breached")
            pub.publish("breached")
            return

    if gps[2] < zfence[0] or gps[2] > zfence[1]:
        print("breached")
        pub.publish("breached")
        return
    
    print("not breached")
    pub.publish(" ")
    pass


def listener():
    rospy.init_node('breach_detector', anonymous=True)

    rate = rospy.Rate(100)
    rospy.Subscriber('/GPSdata', Float32MultiArray, callback)

    while not rospy.is_shutdown():
        #message = "Breached" if flag else "Okay"
        #pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    with open(sys.argv[1]) as f:
        data = json.load(f)

    for feature in data["features"]:
        if feature["geometry"]["type"] == "Polygon":
            polys.append(Polygon(feature["geometry"]["coordinates"]))

    print("Node running")
    listener()
