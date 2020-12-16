#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from time import time

def odom_callBack(data):
    	global xr
    	global yr
    	global thetar
	global vx
    	xr = data.pose.pose.position.x
    	yr = data.pose.pose.position.y
    	thetar = data.pose.pose.orientation.z*np.pi
    	thetar = atan2(np.sin(thetar),np.cos(thetar))
	v = data.twist.twist.linear.x


v_d = 0.1
xg = 1.5
yg = 1.5
thetar = 0
kp_z = 1.5
ki_z = 1
kp_vx = 0.1
ki_vx = 0.3
kd_vx = 0.005
xr = 0
yr = 0
xr_prev = 0
yr_prev = 0
vx = 0
debug = False

def main():
	global xr
    	global yr
    	global thetar
	global vx
    	odom_sub = rospy.Subscriber('/odom',Odometry,odom_callBack)
    	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
    	reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty, queue_size=10)
    	rospy.init_node('goToGoal')
    	rate = rospy.Rate(10)
    	d = 100

    	ez_int = 0;
	evx_int = 0;
	evx_prev = 0;

    	timer = time()
    	while time() - timer < 0.25:
        	reset_odom.publish(Empty())

    	while not rospy.is_shutdown():
        	vel = Twist()
        	rospy.wait_for_message('/odom',Odometry)
        	thetag = -1*atan2(1.5,1.5) + thetar
        	thetag = atan2(np.sin(thetag),np.cos(thetag))
        	xg = xr + np.sqrt(2)*1.5*np.cos(thetag)
        	yg = yr + np.sqrt(2)*1.5*np.sin(thetag)
        	starting_xr = xr
        	starting_yr = yr

        	print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        	print('thetag: {}'.format(thetag))
        	print('theta r: {}'.format(thetar))
        	print('Goal point: ({},{})'.format(xg,yg))
	
        	if debug:
        	    f = open("debug.dat","w+")
	
        	while (d > 0.2) and (not rospy.is_shutdown()):
			# Calculate angular input
			thetad = atan2(yg - yr,xg - xr)
        	    	ez = thetad - thetar
        	    	ez = atan2(np.sin(ez),np.cos(ez))
        	    	ez_int = ez_int + ez*0.1
        	    	az = kp_z*ez + ki_z*ez_int
		    
			# Calculate vel input
			evx = v_d - vx;
			evx_int = evx_int + evx*0.1
			evx_der = (evx - evx_prev)/0.1
			vx = kp_vx*evx + ki_vx*evx_int + kd_vx*evx_der
			evx_prev = evx # Update previous error in velocity

        	    	vel.linear.x = vx
        	    	vel.angular.z = az
        	    	vel_pub.publish(vel)
			print("evx = {:0.2f}".format(evx))
	
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
