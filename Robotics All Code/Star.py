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
	global vx
    	xr = data.pose.pose.position.x
    	yr = data.pose.pose.position.y
	v = data.twist.twist.linear.x


v_d = 0.25
xgoal = [0.75,1.5,-0.5,2,0]
ygoal = [1.5,0,0.75,0.75,0]
thetar = 0
kp_z = 1.0
ki_z = 0.0005
kp_vx = 0.1
ki_vx = 0.3
kd_vx = 0.005
xr = 0
yr = 0
xr_prev = -0.1
yr_prev = 0
vx = 0
debug = False

def main():
	global xr
    	global yr
	global vx
	global xr_prev
	global yr_prev
	i=0;
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

	thetar = 0

    	while not rospy.is_shutdown():
        	vel = Twist()
		#thetar = 0
        	rospy.wait_for_message('/odom',Odometry)
        	thetag = -1*atan2(ygoal[i]-yr,xgoal[i]-xr) + thetar
        	thetag = atan2(np.sin(thetag),np.cos(thetag))
        	xg = xgoal[i]
        	yg = ygoal[i]
        	starting_xr = xr
        	starting_yr = yr

        	print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        	print('thetag: {}'.format(thetag))
        	print('theta r: {}'.format(thetar))
        	print('Goal point: ({},{})'.format(xg,yg))
	
        	if debug:
        	    f = open("debug.dat","w+")

		d = np.sqrt((xg - xr)**2 + (yg - yr)**2)
	
        	while (d > 0.2) and (not rospy.is_shutdown()):
			d = np.sqrt((xg - xr)**2 + (yg - yr)**2)
			# Calculate angular input
			thetad = atan2(yg - yr,xg- xr)
			thetar = atan2(yr - yr_prev,xr - xr_prev)

        	    	ez = thetad - thetar
        	    	ez = atan2(np.sin(ez),np.cos(ez))
        	    	ez_int = ez_int + ez*0.1
        	    	az = kp_z*ez + ki_z*ez_int
			xr_prev = xr
			yr_prev = yr
		    
			# Calculate vel input
			evx = v_d - vx;
			evx_int = evx_int + evx*0.1
			evx_der = (evx - evx_prev)/0.1
			vx = kp_vx*evx + ki_vx*evx_int + kd_vx*evx_der
			evx_prev = evx # Update previous error in velocity

        	    	vel.linear.x = vx
        	    	vel.angular.z = az
        	    	vel_pub.publish(vel)
			print("ez = {:0.2f}\tthetar = {:0.2f}\tthetag = {:0.2f}\taz = {:0.2f}\td = {:0.2f}".format(ez,thetar,thetag,az,d))
	
			if debug:
        	        	f.write("pose: ({:04.2f},{:04.2f},{:03.2f})\t".format(xr,yr,thetar))
        	        	f.write("thetad = {:03.2f}\t".format(thetad))
        	        	f.write("e = {:03.2f}, az = {:03.2f}\n\n".format(e,az))
	
        	    	rate.sleep()
        	print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        	print('Ending point: ({},{})'.format(xr,yr))
        	print('Reached Goal!')
		i=i+1
		if (i==5):
			break;


if __name__=='__main__':
    main()
