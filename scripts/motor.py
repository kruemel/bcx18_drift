#!/usr/bin/env python

import rospy
import sys
import time

import navio.pwm
import navio.util

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

PWM_STEER_ID = 1
PWM_MOTOR_ID = 2
SERVO_MIN = 1.250 #ms
SERVO_MAX = 1.750 #ms
SERVO_STOP = 1.500

last_heartbeat = 0
stop = False
pwm_steer = None
pwm_motor = None
last_lin = 0
last_ang = 0

def linear_mapping(x, in_min, in_max, out_min, out_max):
    # function for linear mapping between two ranges
    # https://machinelearning1.wordpress.com/2014/07/13/linear-vector-mapping-scaling-matlab/
    a = in_min
    b = in_max
    c = out_min
    d = out_max
    y = ((c + d) + (d - c) * ((2 * x - (a + b)) / (b - a))) / 2
    return y

def heartbeat_callback(msg):
    global last_heartbeat
    last_heartbeat = rospy.Time.now()
    return

def stop_callback(msg):
    global stop
    stop = True
    return

def twist_callback(msg):
    global pwm_steer, pwm_motor, stop, last_lin, last_ang

    if stop:
        return

    lin = linear_mapping(msg.linear.x, -1.0, 1.0, SERVO_MIN, SERVO_MAX)
    ang = linear_mapping(msg.angular.z, -1.0, 1.0, SERVO_MIN, SERVO_MAX)

    rospy.loginfo('twist lin: %f ang: %f', lin, ang)

    if last_ang != ang:
        last_ang = ang
        pwm_steer.set_duty_cycle(ang)

    if last_lin != lin:
        last_lin = lin
        pwm_motor.set_duty_cycle(lin)

    return


navio.util.check_apm()

pwm_steer = navio.pwm.PWM(PWM_STEER_ID)
pwm_steer.initialize()
pwm_steer.set_period(50)
pwm_steer.enable()

pwm_motor = navio.pwm.PWM(PWM_MOTOR_ID)
pwm_motor.initialize()
pwm_motor.set_period(50)
pwm_motor.enable()

rospy.init_node('motor_node')

topic_heartbeat = 'heartbeat'
rospy.loginfo('Subscribing to topic: %s', topic_heartbeat)
rospy.Subscriber(topic_heartbeat, Empty, heartbeat_callback)

topic_stop = 'stop'
rospy.loginfo('Subscribing to topic: %s', topic_stop)
rospy.Subscriber(topic_stop, Empty, stop_callback)

topic_twist = 'cmd_vel'
rospy.loginfo('Subscribing to topic: %s', topic_twist)
rospy.Subscriber(topic_twist, Twist, twist_callback)

last_heartbeat = rospy.Time.now()
max_heartbeat = rospy.Duration(0.1)     # 100 ms

rate = rospy.Rate(60)
while not rospy.is_shutdown():

    stop_time = last_heartbeat + max_heartbeat
    if rospy.Time.now() >= stop_time:
        stop = True

    if stop:
        rospy.loginfo('Stop triggered!')
        pwm_steer.set_duty_cycle(SERVO_STOP)
        pwm_motor.set_duty_cycle(SERVO_STOP)

    rate.sleep()


pwm_steer.disable()
pwm_motor.disable()

pwm_steer.deinitialize()
pwm_motor.deinitialize()

rospy.loginfo('Time to rest for motor_node, zzzZZzzzZZZ.')