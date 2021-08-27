#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import subprocess
import time

#Note: If keys are misaligned, press and hold Home button for few seconds and release
def process(data):
	if data.buttons[0] == 1:
		print("Button Y")
	elif data.buttons[1] == 1:
		print("Button B")
	elif data.buttons[2] == 1:
		print("Button A")
	elif data.buttons[3] == 1:
		print("Button X")
	elif data.buttons[4] == 1:
		print("Button LB")
	elif data.buttons[5] == 1:
		print("Button RB")
	elif data.buttons[6] == 1:
		print("Button LT")
	elif data.buttons[7] == 1:
		print("Button RT")
	elif data.buttons[8] == 1:
		print("Back")
	elif data.buttons[9] == 1:
		print("Start")
	elif data.buttons[10] == 1:
		print("LJ Press")
	elif data.buttons[11] == 1:
		print("RJ Press")
	elif data.buttons[12] == 1:
		print("Home")	

if __name__=="__main__":
	rospy.init_node("joy_buttons")
	rospy.Subscriber("/joy", Joy, process, queue_size=1)
	rospy.spin()
