#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from math import atan2 
from math import pi
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
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
	#print([xr,yr])


def ball_callBack(data):
	global xb
	global zb
	xb = data.pose.pose.position.x
	zb = data.pose.pose.position.z

def bumperCallback(data):
	if(data.state==data.PRESSED):
		print("Bumper Pressed!")
		rospy.signal_shutdown("Bumper pressed")
	

v_d = 0.2
xg = 0
yg = 1.5
xb = 0.0
zb = 1.8
thetar = 0
kp_z = 0.5
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
goalset = False
ball_xs = np.array([])
ball_ys = np.array([])
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim([-1,1])
ax.set_ylim([-.5,4])
marks, = ax.plot(ball_xs,ball_ys,label='contrail')
ball_pos, = ax.plot(xb,zb,'o',label='ball')
robot_pos, = ax.plot(xr,yr,'x')
goal_pos, = ax.plot(xg,yg,'^',markersize=12)
ax.legend()
plt.ion()



def main():
	global xr
    	global yr
	global vx
	global xb
	global zb
	global xg
	global yg
	global ball_xs
	global ball_ys
	global xr_prev
	global yr_prev
	global goalset
    	odom_sub = rospy.Subscriber('/odom',Odometry,odom_callBack)
	ball_sub = rospy.Subscriber('/ball/position',Odometry,ball_callBack)
    	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
    	reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty, queue_size=10)
    	rospy.init_node('goToGoal')
    	rate = rospy.Rate(10)
    	d = 100
	   	
	ez_int = 0;
	evx_int = 0;
	evx_prev = 0;
	count = 1
	slope = 0
	cept = 0
	xba = 0
	yba = 0
	a = 1
	b = 1
	c = 1
    	timer = time()
	print(timer)
    	while time() - timer < 0.25:
        	reset_odom.publish(Empty())

    	while not rospy.is_shutdown():
        	vel = Twist()
		thetar = pi/2
        	rospy.wait_for_message('/odom',Odometry)
		if count == 1:
			rospy.wait_for_message('/ball/position',Odometry)
			xbi = xb
			ybi = zb 
        	starting_xr = xr
        	starting_yr = yr
		

        	#print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        	#print('thetag: {}'.format(thetag))
        	#print('theta r: {}'.format(thetar))
        	#print('Goal point: ({},{})'.format(xg,yg))
	
        	if debug:
        	    f = open("debug.dat","w+")
	
        	while (d > 0.1) and (not rospy.is_shutdown()):
			if goalset == False:
				if count < 10:
					ball_xs = np.append(ball_xs,xb)
					ball_ys = np.append(ball_ys,zb)						
				if count>=10:
					xba = np.average(ball_xs)
					yba = np.average(ball_ys)
					slope = (yba - ybi)/(xba - xbi)
					cept = yba - xba*slope
					a = -1*slope
					c = -1*cept
					xg = (b*(b*xr - a*yr) - a*c)/(a*a + b*b)
					yg = (a*(-1*b*xr + a*yr) - b*c)/(a*a + b*b)
				if xba > 0 or yba > 0:				
					d = np.sqrt((xg - xr)**2 + (yg - yr)**2)
					goalset = True		
			print(goalset)
			if goalset == True:

				# Calculate angular input								
				temp=yr				
				yr = xr
				xr = -temp
				thetad = atan2(yg - yr,xg- xr)#(xg-xr,yg-yr)#
				thetar = atan2(yr - yr_prev,xr - xr_prev)#(xr-xr_prev,yr-yr_prev)#

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
				d = np.sqrt((xg - xr)**2 + (yg - yr)**2)
				print('theta = {:03.2f}\td = {:03.2f}\txr = {:03.2f}\tyr = {:03.2f}\txg = {:03.2f}\tzg = {:03.2f}\t'.format(a,d,xr,yr,xg,yg))
			if count<10:	
				count+=1
			
				#print("ez = {:0.2f}\tthetar = {:0.2f}\tthetag = {:0.2f}\taz = {:0.2f}\td = {:0.2f}".format(ez,thetar,thetag,az,d))
			marks.set_xdata(ball_xs)
			marks.set_ydata(ball_ys)
			ball_pos.set_xdata(xb)
			ball_pos.set_ydata(zb)
			robot_pos.set_xdata(xr)
			robot_pos.set_ydata(yr)
			goal_pos.set_xdata(xg)
			goal_pos.set_ydata(yg)
			plt.draw()
			plt.show()
	
        	    	rate.sleep()
        	print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        	print('Ending point: ({},{})'.format(xr,yr))
		print("xg = {:0.2f}\tzg = {:0.2f}\t".format(xg,yg))
        	print('Reached Goal!')
        	plt.savefig("map.png")
		break
		

if __name__=='__main__':
    main()
