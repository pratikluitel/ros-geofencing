#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray

pub = rospy.Publisher('/geofence_status', String, queue_size=100)
#flag = 0
xfence = (-3.75, 5)
yfence = (-5, 3.75)
zfence = (0, 5.5)



def callback(data):
    #print("Data received, processing")
    gps = data.data
    print(gps)
    if gps[0] < xfence[0] or gps[0] > xfence[1] or gps[1] < yfence[0] or gps[1] > yfence[1] or gps[2] < zfence[0] or gps[2] > zfence[1]:
        #print("breached")
        pub.publish("breached")
    else:
        #print("not breached")
        pub.publish("not breached")


def listener():
    rospy.init_node('breach_detector', anonymous=True)

    rate = rospy.Rate(100)
    rospy.Subscriber('/GPSdata', Float32MultiArray, callback)

    while not rospy.is_shutdown():
        #message = "Breached" if flag else "Okay"
        #pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    print("Node running")
    listener()
