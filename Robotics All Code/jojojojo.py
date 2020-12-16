#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry
import numpy as np

# right = bottom[0], thresh = 1400
# center = bottom[1], thresh = 1600
# left = bottom[2], thresh = 1500

rthresh = 1400 # og: 1400
cthresh = 1600 # og: 1600
lthresh = 1550 # og: 1500
c_count = 0
prev_hit = 1 # 2 = left, 1 = center, 0 = right
#curr_hit = 0 # -1 = left, 0 = center, 1 = right
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
	rospy.init_node('cliffTest')
	rospy.Subscriber("/mobile_base/sensors/core",SensorState,cliffCallBack)
	rospy.Subscriber("/odom",Odometry,odomCallback)
	rate = rospy.Rate(10)
	direction = 0
	speed = 0.2
	vel = Twist()
	vel.angular.z = direction;

	lhit = 0
	chit = 0
	rhit = 0

	lmean = 0
	cmean = 0
	rmean = 0

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
			

		if lmean>0.5:#leftsensor<lthresh:
			if cmean<0.5:#centersensor>cthresh:
				direction=0.4
				speed=0.1
				prev_hit=2		
	 	if cmean>0.5:#centersensor<cthresh:
			if prev_hit==2:
				direction =-0.2
				speed=0.1
				prev_hit=1
			if prev_hit==0:
				direction =0.2
				speed=0.1
				prev_hit=1
				
		if rmean>0.5:#rightsensor<rthresh:
			if cmean<0.5:#centersensor>cthresh:
				direction=-0.4
				speed=0.1
				prev_hit=0
		'''
		if centersensor<cthresh:
			if prev_hit==2:
				direction =-0.4
				speed=0.1
				prev_hit=1
			if prev_hit==0:
				direction=0.4
				speed=0.15
				prev_hit=1
		'''
		'''
			if prev_hit==1:
				speed=0.1
				prev_hit=1
				direction=-1*direction
			'''
	

		vel.linear.x = speed;
		vel.angular.z = direction;
		vel_pub.publish(vel)
		print("speed:{:0.2f}\tact_speed:{:0.2f}\tdirection:{:0.2f}\t\tprev_hit:{}\t(l,c,r):({},{},{})".format(speed,act_speed,direction,prev_hit,lhit,chit,rhit))
		rate.sleep()

if __name__ == "__main__":
	main()
		
