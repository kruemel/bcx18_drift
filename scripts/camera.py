#!/usr/bin/env python

import rospy
from sensor_msgs import Image

def callback(data):
    global pub, rate
    pub.publish(data)
    rate.sleep()

pub = rospy.Publisher('show_image', Image)
rospy.Subscriber('image', Image, callback)
rospy.init_node('show_image', anonymous=True)
rate = rospy.Rate(2)


rospy.spin()