import sys
import time

import navio.pwm
import navio.util

navio.util.check_apm()

PWM_OUTPUT = 1
SERVO_MIN = 1.250 #ms
SERVO_MAX = 1.750 #ms

with navio.pwm.PWM(PWM_OUTPUT) as pwm:
    pwm.set_period(50)
    pwm.enable()

#    while (True):
#        pwm.set_duty_cycle(SERVO_MIN)
#        time.sleep(1)
#        pwm.set_duty_cycle(SERVO_MAX)
#        time.sleep(1)

    pwm.set_duty_cycle(SERVO_MIN)
    time.sleep(5)
    pwm.set_duty_cycle(1.5)
    pwm.disable()

