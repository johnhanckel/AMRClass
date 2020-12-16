#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import numpy as np
import time

rthresh = 1500 # og: 1400
cthresh = 1705 # og: 1600
lthresh = 1555 # og: 1500
prev_hit = 1 # 2 = left, 1 = center, 0 = right
leftsensor=0
rightsensor=0
centersensor=0

def cliffCallBack(data):
	global direction
	global c_count
	global speed
	global prev_hit
	global leftsensor
	global rightsensor
	global centersensor
	leftsensor=data.bottom[2]
	rightsensor=data.bottom[0]
	centersensor=data.bottom[1]

def odomCallback(data):
	global act_speed
	act_speed = data.twist.twist.linear.x

def bumperCallback(data):
	if(data.state==data.PRESSED):
		rospy.signal_shutdown("Bumper pressed")

def main():
	global direction
	global prev_hit
	global speeds
	global leftsensor
	global rightsensor
	global centersensor
	global act_speed
	act_speed = 0
	lhits = [0]
	chits = [1]
	rhits = [0]
	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('cliffTest',disable_signals=True)
	rospy.Subscriber("/mobile_base/sensors/core",SensorState,cliffCallBack)
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
	rospy.Subscriber("/odom",Odometry,odomCallback)
	rate = rospy.Rate(20)
	direction_max = 1
	speed_max = 0.4
	direction = 0
	speed = speed_max*(2.0/3)
	vel = Twist()
	vel.angular.z = direction;
	hardTurnSpeedRatio = 2.0/3
	hardTurnDirecRatio = 1
	centerTurnSpeedRatio = 2.0/3
	centerTurnDirecRatio = 0.5

	lhit = 0
	chit = 0
	rhit = 0

	lmean = 0
	cmean = 0
	rmean = 0


	prev_centertime = time.time()
	ccount = 0

	while not rospy.is_shutdown():
		a=[]
		
		if leftsensor<lthresh:
			lhit = 1
		else:
			lhit = 0
		lhits.append(lhit)
		if rightsensor<rthresh:
			rhit = 1
		else:
			rhit = 0
		rhits.append(rhit)
		if centersensor<cthresh:
			chit = 1
		else:
			chit = 0
		chits.append(chit)
		

		if(len(chits)>5):
			cmean=np.mean(chits[-1:])
		if(len(rhits)>5):
			rmean=np.mean(rhits[-1:])
		if(len(lhits)>5):
			lmean=np.mean(lhits[-1:])
			

		if lmean>0.5:
			if cmean<0.5:
				direction=direction_max
				speed=speed_max*hardTurnSpeedRatio
				prev_hit=2
				ccount = 0
		if rmean>0.5:
			if cmean<0.5:
				direction=-1*direction_max
				speed=speed_max*hardTurnSpeedRatio
				prev_hit=0
				ccount = 0
	 	if (cmean>0.5) and (lmean<0.5) and (rmean<0.5):
			delt = time.time() - prev_centertime
			if prev_hit==1 and delt>0.5:
				print("prev_centertime: {:0.2}".format(prev_centertime))
				ccount = ccount + 1
				direction = -1*direction/(ccount + 1)
			prev_centertime = time.time()
			if prev_hit==2:
				direction =-1*direction_max*centerTurnDirecRatio
				speed=speed_max*centerTurnSpeedRatio
			if prev_hit==0:
				direction =direction_max*centerTurnDirecRatio
				speed=speed_max*centerTurnSpeedRatio
			prev_hit = 1
			prev_centertime = time.time()
		'''
		delt = time.time() - prev_centertime
		if cmean>0.5 and delt > 0.25:
			if prev_hit==1:
				print("prev_centertime: {:0.2}".format(prev_centertime))
				ccount = ccount + 1
				direction = -1*direction*np.exp(-1*ccount)
		'''
	

		vel.linear.x = speed;
		vel.angular.z = direction;
		vel_pub.publish(vel)
		print("speed:{:0.2f}\tprev_centertime: {:0.2f}\tdirection:{:0.2f}\t\tprev_hit:{}\t(l,c,r):({},{},{})".format(speed,prev_centertime,direction,prev_hit,lhit,chit,rhit))
		rate.sleep()

if __name__ == "__main__":
	main()


