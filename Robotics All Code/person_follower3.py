#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from kobuki_msgs.msg import BumperEvent
from time import time

xr = -0.1
yr = 0.
pressed = False
target_zone = 3
target_dist = 1
d_min = 0.5
d_max = 1.6
d = 1
tolerance = 0.1
follow = False
kp = 0.1
kz = 1
zone1 = 0
zone2 = 0
zone3 = 0

# Performs hysteresis around x = 0
def hysteresis(x,thresh,sign):
	y = 0
	if sign > 0:
		if x > -1*thresh:
			y = 1
		else:
			y = -1
	if sign < 0:
		if x < thresh:
			y = -1
		else:
			y = 1
	return y

def bumperCallback(data):	
	global pressed
	data.bumper
	if data.state == data.PRESSED:	
		pressed = True

def odom_callBack(data):
    	global xr
    	global yr
    	xr = data.pose.pose.position.x
    	yr = data.pose.pose.position.y
d
def laserCallBack(data):
	global target_zone
	global target_dist
	global follow
	global zone1
	global zone2
	global zone3
	ranges = data.ranges
	zone1_range = ranges[437:629] # Left most zone
	zone2_range = ranges[221:427] # Center zone
	zone3_range = ranges[9:211] # Right most zone

	zone1_range = [x for x in zone1_range if x==x]
	zone2_range = [x for x in zone2_range if x==x]
	zone3_range = [x for x in zone3_range if x==x]
	zone1 = np.mean(zone1_range)
	zone2 = np.mean(zone2_range)
	zone3 = np.mean(zone3_range)


	
	'''
	if(zone1>=d_min and zone1<=d_max):
		target_zone = 1
		target_dist = zone1
		follow = True
	if(zone5>=d_min and zone5<=d_max):
		target_zone = 5
		target_dist = zone5
		follow = True
	if(zone2>=d_min and zone2<=d_max):
		target_zone = 2
		target_dist = zone2
		follow = True
	if(zone4>=d_min and zone4<=d_max):
		target_zone = 4
		target_dist = zone4
		follow = True
	if(zone3>=d_min and zone3<=d_max):
		target_zone = 3
		target_dist = zone3
		follow = True
	'''
	

	

def main():
	global pressed
	global target_zone
	global target_dist
	global follow
	global xr
	global yr
	global d
	global zone1
	global zone2
	global zone3
	sign = 1
	thresh = 0.3
	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	odom_sub = rospy.Subscriber('/odom',Odometry,odom_callBack)
    	reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty, queue_size=10)
	laser_sub = rospy.Subscriber('/scan',LaserScan,laserCallBack)
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
    	rospy.init_node('personFollower')
    	rate = rospy.Rate(10)

	timer = time()
    	while time() - timer < 0.25:
        	reset_odom.publish(Empty())

	while not rospy.is_shutdown():
		vel = Twist()
		if pressed:
			break
		v = 0
		z = 0

		if (zone1 < zone2 and zone1 < zone3 and zone1 < d_max):
			target_zone = 1
			target_dist = zone1
		if (zone2 < zone3 and zone2 < zone1 and zone2 < d_max):
			target_zone = 2
			target_dist = zone2
		if (zone3 < zone1 and zone3 < zone2 and zone3 < d_max):
			target_zone = 3
			target_dist = zone3
		if (zone1 < d_max and zone2 < d_max and zone3 < d_max):
			follow = False

		if (np.abs(target_dist - d)/d < tolerance) and target_zone==2:
			follow = False
		else:
			follow = True


		if follow:
			if target_zone == 1:
				z = 0.7
			if target_zone == 3:
				z = -0.7
			if target_zone == 2:
				z = 0
			e = target_dist - d
			v = kp*hysteresis(e,thresh,sign)
			sign = np.sign(e)

		vel.linear.x = v
		vel.angular.z = z
		vel_pub.publish(vel)

		print("[{:0.2f}\t{:0.2f}\t{:0.2f}\ttzone:{:d}]".format(zone1,zone2,zone3,target_zone))


if __name__=='__main__':
    main()
		
