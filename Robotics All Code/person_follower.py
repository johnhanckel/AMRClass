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
d_max = 2
d = 1
tolerance = 0.05
follow = False
kp = 0
kz = 1

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

def laserCallBack(data):
	global target_zone
	global target_dist
	global follow
	ranges = data.ranges
	zone1 = ranges[573:629] # Left most zone
	zone2 = ranges[387:563] # Inner left zone
	zone3 = ranges[261:377] # Center zone
	zone4 = ranges[135:251] # Inner right zone
	zone5 = ranges[9:125] # Right most zone

	zone1 = [x for x in zone1 if x==x]
	zone2 = [x for x in zone2 if x==x]
	zone3 = [x for x in zone3 if x==x]
	zone4 = [x for x in zone4 if x==x]
	zone5 = [x for x in zone5 if x==x]
	zone1 = np.mean(zone1)
	zone2 = np.mean(zone2)
	zone3 = np.mean(zone3)
	zone4 = np.mean(zone4)
	zone5 = np.mean(zone5)


	zones = [zone1,zone2,zone3,zone4,zone5]
	zones_sorted = [zone1,zone2,zone3,zone4,zone5]
	zones_sorted = [x for x in zones_sorted if x<=d_max]
	if len(zones_sorted) == 0:
		target_zone = 3
		target_dist = 1
		follow = False
	else:
		zones_sorted.sort()
		target_zone = zones.index(zones_sorted[0]) + 1
		target_dist = zones[target_zone - 1]
		follow = True
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
	print("[{:0.2f}\t,{:0.2f}\t,{:0.2f}\t,{:0.2f}\t,{:0.2f}\ttzone:{:d}]".format(zone1,zone2,zone3,zone4,zone5,target_zone))

	

def main():
	global pressed
	global target_zone
	global target_dist
	global follow
	global xr
	global yr
	global d
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
		if np.abs(target_dist - d)/d < tolerance:
			follow = False
		if follow:
			v = kp*(target_dist - d)
			z = -1*kz*(target_zone - 3)
		vel.linear.x = v
		vel.angular.z = z

		vel_pub.publish(vel)


if __name__=='__main__':
    main()
		
