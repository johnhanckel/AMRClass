#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import numpy as np
import time

rthresh = 1555 # og: 1400
cthresh = 1700 # og: 1600
lthresh = 1450 # og: 1500
prev_hit = 1 # 2 = left, 1 = center, 0 = right
delrs_thresh = 70
delcs_thresh = 70
dells_thresh = 50
leftsensor=0
rightsensor=0
centersensor=0
delcs = 0
delrs = 0
dells = 0
rightsensor_prev = 0
centersensor_prev = 0
leftsensor_prev = 0
i = 1

def cliffCallBack(data):
	global direction
	global speed
	global prev_hit
	global delrs_thresh
	global delcs_thresh
	global dells_thresh
	global i
	global rightsensor_prev
	global leftsensor_prev
	global centersensor_prev
	global centersensor
	global rightsensor
	global leftsensor
	global delrs
	global delcs
	global dells
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
	global centersensor_prev
	global act_speed
	global delrs
	global delcs
	global dells
	global lhit
	global chit
	global rhit
	act_speed = 0
	lhits = [0]
	chits = [1]
	rhits = [0]
	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('cliffTest',disable_signals=True)
	rospy.Subscriber("/mobile_base/sensors/core",SensorState,cliffCallBack)
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
	rospy.Subscriber("/odom",Odometry,odomCallback)
	rate = rospy.Rate(10)
	direction_max = 0.75
	speed_max = 0.1
	direction = 0
	speed = speed_max
	vel = Twist()
	vel.angular.z = direction;
	hardTurnSpeedRatio = 2.0/3
	hardTurnDirecRatio = 1
	centerTurnSpeedRatio = 2.0/3
	centerTurnDirecRatio = 0.5

	lhit = 0
	chit = 1
	rhit = 0

	rospy.wait_for_message("mobile_base/sensors/core",SensorState)

	prev_centertime = time.time()
	ccount = 0
	rightsensor_prev = rightsensor
	centersensor_prev = centersensor
	leftsensor_prev = leftsensor

	while not rospy.is_shutdown():
		delrs = rightsensor - rightsensor_prev
		delcs = centersensor - centersensor_prev
		dells = leftsensor - leftsensor_prev
		if np.abs(delrs) > delrs_thresh:
			if delrs < 0:
				rhit = 1
			if delrs > 0:
				rhit = 0
		if np.abs(delcs) > delcs_thresh:
			if delcs < 0:
				chit = 1
			if delcs > 0:
				chit = 0
		if np.abs(dells) > dells_thresh:
			if dells < 0:
				lhit = 1			
			if dells > 0:
				lhit = 0
		rightsensor_prev = rightsensor
		leftsensor_prev = leftsensor
		centersensor_prev = centersensor

		if lhit==1:
			if chit==0:
				direction=direction_max
				speed=speed_max*hardTurnSpeedRatio
				prev_hit=2
				ccount = 0
		if rhit==1:
			if chit==0:
				direction=-1*direction_max
				speed=speed_max*hardTurnSpeedRatio
				prev_hit=0
				ccount = 0		
	 	if chit==1:
			delt = time.time() - prev_centertime
			if prev_hit==1 and delt>0.5:
				print("prev_centertime: {:0.2}".format(prev_centertime))
				ccount = ccount + 1
				direction = -1*direction/(ccount + 1)
				if ccount > 1:
					speed = speed_max
			prev_centertime = time.time()
			if prev_hit==2:
				direction =-1*direction_max*centerTurnDirecRatio
				speed=speed_max*centerTurnSpeedRatio
			if prev_hit==0:
				direction =direction_max*centerTurnDirecRatio
				speed=speed_max*centerTurnSpeedRatio
			prev_hit = 1
			prev_centertime = time.time()
	

		vel.linear.x = speed;
		vel.angular.z = direction;
		vel_pub.publish(vel)
		print("speed:{:0.2f}\tdells: {:0.2f}\tleftsensor:{:0.2f}\t\tprev_hit:{}\t(l,c,r):({},{},{})".format(speed,dells,leftsensor,prev_hit,lhit,chit,rhit))
		rate.sleep()

if __name__ == "__main__":
	main()


