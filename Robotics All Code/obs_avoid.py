#!/usr/bin/env python
import rospy
import random
import time
from sensor_msgs.msg import LaserScan
from numpy import *
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
#determine whether we are setting new setpoints after bumper or during cruise

velx=0.1
angz=0
setpoint=0.3


def bumperCallback(data):	
	global pressed
	data.bumper

	if data.state == data.PRESSED:	
		pressed = True


def laserCallBack(data):
	global velx
	global angz
	global setpoint
	ranges = data.ranges
	left_ranges = ranges[430:639]
	center_ranges = ranges[214:423]
	right_ranges = ranges[0:209]
	left_ranges = [x for x in left_ranges if x == x]
	center_ranges = [x for x in center_ranges if x == x]
	right_ranges = [x for x in right_ranges if x == x]
	print([mean(left_ranges),mean(center_ranges),mean(right_ranges)])
	
	if (mean(center_ranges)<=1.2):
		velx=0.1
		if (mean(left_ranges)>mean(right_ranges)):
			angz=0.35
			velx = 0.1
		else:
			angz=0
			velx=0.1
		if (mean(left_ranges)<mean(right_ranges)):
			angz=-0.35
			velx=0.1
		else:
			angz=0
			velx=0.1


def main():
	global x 
	global z
	global pressed 
	global velx
	global angz
	global setpoint
	velx = 0.15
	angz = 0
	pressed = False
	direc = 1
	vel_cb_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	laser_sub = rospy.Subscriber('/scan',LaserScan,laserCallBack)
	rospy.init_node('laserTest')
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
	rate = rospy.Rate(20)
	vel_cb = Twist()
	'''preverror = 0
	integral = 0
	Kp = 0.1
	Ki = 0.3
	Kd = 0.005	
	setpoint = 0.3
	dt=0.1'''
	while not rospy.is_shutdown():
		
		if setpoint==0:
			break;			
		if pressed == True:	
			break
		else:  
			vel_cb.linear.x=velx
			vel_cb.angular.z=angz	
			vel_cb_pub.publish(vel_cb)
		
		rate.sleep()

if __name__ == "__main__":
	main()
