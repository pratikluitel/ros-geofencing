#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, String

def callback(data):
    gps = data.data
    points.append([gps[0],gps[1],gps[2]])
    global time
    time+=1

def geocallback(data):
    breach = data.data
    breaches.append(breach) 

def listener():
    rospy.init_node('main', anonymous=True)

    rate = rospy.Rate(100)
    rospy.Subscriber('/GPSdata', Float32MultiArray, callback)
    rospy.Subscriber('/geofence_status',String, geocallback)

    while not rospy.is_shutdown():
        rate.sleep()

points = []
breaches = []
col='blue'
time=0
listener()
mask = np.array(breaches) == ' '
fig, axs = plt.subplots(2)
fig.suptitle('x and y coordinates vs time plot')
x = np.array([point[0] for point in points])
y = np.array([point[1] for point in points])
t = np.array(range(time))*0.1 #rate of sampling is 100ms
axs[0].plot(t[mask],x[mask],'blue')
axs[0].plot(t[np.logical_not(mask)],x[np.logical_not(mask)],'red')
axs[0].set(ylabel='x')

axs[1].plot(t[mask],y[mask],'blue')
axs[1].plot(t[np.logical_not(mask)],y[np.logical_not(mask)],'red')
axs[1].set(ylabel='y')
axs[1].set(xlabel='time (sec)')
plt.show()
