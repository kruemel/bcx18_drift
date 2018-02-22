#!/usr/bin/env python

import rospy
import sys
import time

import navio.pwm
import navio.util

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

PWM_STEER_ID = 1
PWM_MOTOR_ID = 0
SERVO_MIN = 1.250 #ms
SERVO_MAX = 1.750 #ms
SERVO_STOP = 1.500

state = 0
last_time = 0
pwm_steer = None
pwm_motor = None

def next_state(duration, new_state):
    global state, last_time

    new_time = last_time + rospy.Duration(duration)
    if rospy.Time.now() >= new_time:
        rospy.loginfo('Transition from state %d to %d.', state, new_state)
        state = new_state
        last_time = rospy.Time.now()
    return

def start_callback(msg):
    global state
    if state == 0:
        next_state(0, 1)
    return

def control_rover(steer, motor):
    global pwm_steer, pwm_motor
    
    pwm_steer.set_duty_cycle(steer)
    pwm_motor.set_duty_cycle(motor)
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

rospy.init_node('drift_node')

topic_start = 'start'
rospy.loginfo('Subscribing to topic: %s', topic_start)
rospy.Subscriber(topic_start, Empty, start_callback)

last_time = rospy.Time.now()

rospy.loginfo('Waiting for the inital start message.s')

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    
    if state == 1:
        control_rover(SERVO_STOP, 1.650)
        next_state(0.250, 2)
        pass
    elif state == 2:
        control_rover(SERVO_STOP, SERVO_MAX)
        next_state(0.250, 3)
        pass
    elif state == 3:
        control_rover(SERVO_MAX, SERVO_MAX)
        next_state(0.150, 4)
        pass
    elif state == 4:
        control_rover(SERVO_STOP, SERVO_STOP)
        break
    else:
        pass

    rate.sleep()

pwm_steer.disable()
pwm_motor.disable()

pwm_steer.deinitialize()
pwm_motor.deinitialize()

rospy.loginfo('Time to rest for heartbeat_node, zzzZZzzzZZZ.')