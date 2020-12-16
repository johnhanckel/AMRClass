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

az_obsavoid = 0
obs_avoid = False
pressed = False

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
	global az_obsavoid
	global obs_avoid
	d_avoid = 1.2
	ranges = data.ranges
	ll = ranges[487:639] # Left left ranges
	cl = ranges[324:477] # Center left ranges
	cr = ranges[161:314] # Center right ranges
	rr = ranges[0:151] # Right right ranges
	left_ranges = ranges[430:639]
	center_ranges = ranges[214:423]
	right_ranges = ranges[0:209]
	left_ranges = [x for x in left_ranges if x == x]
	center_ranges = [x for x in center_ranges if x == x]
	right_ranges = [x for x in right_ranges if x == x]
	left_mean = np.mean(left_ranges)
	center_mean = np.mean(center_ranges)
	right_mean = np.mean(right_ranges)

	if (left_mean <= d_avoid or left_mean!=left_mean):
		if (center_mean<=d_avoid or center_mean!=center_mean):
			az_obsavoid=-0.65
		else:
			az_obsavoid=-0.45
		obs_avoid = True
	if (right_mean<=d_avoid or right_mean!=right_mean):
		if (center_mean<=d_avoid or center_mean!=center_mean):
			az_obsavoid = 0.65
		else:
			az_obsavoid = 0.45
		obs_avoid = True
	
	if (center_mean<=d_avoid):
		obs_avoid = True
		if (left_mean>right_mean):
			az_obsavoid=0.35
		else:
			az_obsavoid=-0.35

	if (left_mean>d_avoid and right_mean>d_avoid and center_mean>d_avoid):
		az_obsavoid = 0
		obs_avoid = False
	
	print([left_mean,center_mean,right_mean,az_obsavoid])
	'''
	if (np.mean(center_ranges)<=1.):
		if (np.mean(left_ranges)>np.mean(right_ranges)):
			az_obsavoid=0.35
		else:
			az_obsavoid=0
		if (np.mean(left_ranges)<np.mean(right_ranges)):
			az_obsavoid=-0.35
		else:
			az_obsavoid=0
	else:
		az_obsavoid = 0.
	'''

def main():
	global xr
    	global yr
	global xr_prev
	global yr_prev
	global az_obsavoid
	global pressed
	xg = 4
	yg = 0
	thetar = 0
	kp_z = 0.3
	ki_z = 0.0005
	xr = 0
	yr = 0
	xr_prev = -0.1
	yr_prev = 0
	vx = 0.1
	debug = False

    	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	odom_sub = rospy.Subscriber('/odom',Odometry,odom_callBack)
    	reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty, queue_size=10)
	laser_sub = rospy.Subscriber('/scan',LaserScan,laserCallBack)
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
    	rospy.init_node('goToGoal')
    	rate = rospy.Rate(10)
    	d = 100

    	ez_int = 0;

    	timer = time()
    	while time() - timer < 0.25:
        	reset_odom.publish(Empty())

    	while not rospy.is_shutdown():
		print("Start")
        	vel = Twist()
        	rospy.wait_for_message('/odom',Odometry)
        	thetag = atan2(yg,xg);
        	starting_xr = xr
        	starting_yr = yr

        	print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        	print('thetag: {}'.format(thetag))
        	print('theta r: {}'.format(thetar))
        	print('Goal point: ({},{})'.format(xg,yg))
	
        	if debug:
        	    f = open("debug.dat","w+")
	
        	while (d > 0.2) and (not rospy.is_shutdown()):
			if pressed:
				break
			d = np.sqrt((xg - xr)**2 + (yg - yr)**2)

			# Calculate go-to-goal angular input
			thetad = atan2(yg - yr,xg- xr)
			thetar = atan2(yr - yr_prev,xr - xr_prev)
        	    	ez = thetad - thetar
        	    	ez = atan2(np.sin(ez),np.cos(ez))
        	    	ez_int = ez_int + ez*0.1
        	    	az = kp_z*ez + ki_z*ez_int
			xr_prev = xr
			yr_prev = yr

			# Calculate obstacle-avoidance angular input
			if obs_avoid:
				az = az_obsavoid

			# Publish vel
        	    	vel.linear.x = vx
        	    	vel.angular.z = az
        	    	vel_pub.publish(vel)
			#print("ez = {:0.2f}\tthetar = {:0.2f}\tthetad = {:0.2f}\taz = {:0.2f}\taz_obsavoid = {:0.2f}\td = {:0.2f}".format(ez,thetar,thetad,az,az_obsavoid,d))
	
			if debug:
        	        	f.write("pose: ({:04.2f},{:04.2f},{:03.2f})\t".format(xr,yr,thetar))
        	        	f.write("thetad = {:03.2f}\t".format(thetad))
        	        	f.write("e = {:03.2f}, az = {:03.2f}\n\n".format(e,az))
	
        	    	rate.sleep()
        	print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        	print('Ending point: ({},{})'.format(xr,yr))
        	print('Reached Goal!')
        	break


if __name__=='__main__':
    main()
