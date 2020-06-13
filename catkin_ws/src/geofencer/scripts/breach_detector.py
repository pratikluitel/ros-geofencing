#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray, Empty

pub = rospy.Publisher('/geofence_status', Float32MultiArray, queue_size=100)
#flag = 0
xfence = (-5, 5)
yfence = (-5, 5)
zfence = (-1, 5.5)



def callback(data):
    #print("Data received, processing")
    gps = data.data
    print(gps)
    if gps[0] < xfence[0] or gps[0] > xfence[1] or gps[1] < yfence[0] or gps[1] > yfence[1] or gps[2] < zfence[0] or gps[2] > zfence[1]:
        print("breached")
        #pub.publish("breached")
        diff = Float32MultiArray()
        xneg = min(0, gps[0]-xfence[0])
        xpos = max(0, gps[0]-xfence[1])
        yneg = min(0, gps[1]-yfence[0])
        ypos = max(0, gps[1]-yfence[1])
        zneg = min(0, gps[2]-zfence[0])
        zpos = max(0, gps[2]-zfence[1])
        diff.data = [ -xneg if not xpos else -xpos, -yneg if not ypos else -ypos, -zneg if not zpos else -zpos ]
        pub.publish(diff)
    else:
        #print("not breached")
        #pub.publish(Empty)
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
    print("Node running")
    listener()
