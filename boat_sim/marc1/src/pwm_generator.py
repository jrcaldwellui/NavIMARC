#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
#import RPi.GPIO as GPIO
import time

def callbackL(msg):	
	rospy.loginfo("Data L: %d",msg.data)	
	#pwmL.changeDutyCycle(msg.data)

def callbackR(msg):
	rospy.loginfo("Data R: %d",msg.data)	
	#pwmR.changeDutyCycle(msg.data)
'''
def piSetup():
	global pwmL
	global pwmR

	PWM_PIN_L = 18
	PWM_PIN_R = 19


	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PWM_PIN_L, GPIO.OUT)
	GPIO.setup(PWM_PIN_R, GPIO.OUT)

	pwmL = GPIO.PWM(PWM_PIN_L, 50)
	pwmL.start(0)

	pwmR = GPIO.PWM(PWM_PIN_R, 50)
	pwmR.start(0)
'''

def listener():
	#starts node, will kick off other nodes named pwm
	rospy.init_node('pwm')

	rospy.Subscriber('l_duty_cycle',Int8,callbackL)
	rospy.Subscriber('r_duty_cycle',Int8,callbackR)

	#keep the node alive waiting for messages
	rospy.spin()

if __name__ == '__main__':
	listener()
