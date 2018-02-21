#!/usr/bin/env python

import rospy

from std_msgs.msg import Empty


rospy.init_node('heartbeat_node')

topic_heartbeat = 'heartbeat'
pub = rospy.Publisher(topic_heartbeat, Empty, queue_size=10)

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    pub.publish(Empty())
    rate.sleep()

rospy.loginfo('Time to rest for heartbeat_node, zzzZZzzzZZZ.')