#!/bin/sh

sudo chmod a+rw /sys/class/pwm/pwmchip0/pwm1/enable
sudo chmod a+rw /sys/class/pwm/pwmchip0/pwm1/period
sudo chmod a+rw /sys/class/pwm/pwmchip0/pwm1/duty_cycle
sudo chmod a+rw /sys/class/pwm/pwmchip0/pwm2/enable
sudo chmod a+rw /sys/class/pwm/pwmchip0/pwm2/period
sudo chmod a+rw /sys/class/pwm/pwmchip0/pwm2/duty_cycle