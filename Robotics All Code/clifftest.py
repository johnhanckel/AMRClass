#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState

# right = bottom[0], thresh = 1400
# center = bottom[1], thresh = 1600
# left = bottom[2], thresh = 1500

r_thresh = 1400
c_thresh = 1600
l_thresh = 1500
c_count = 0
prev_hit = 0 # -1 = left, 0 = center, 1 = right
curr_hit = 0 # -1 = left, 0 = center, 1 = right
def cliffCallBack(data):
	global direction
	global c_count
	global speed
	global prev_hit
	if(data.bottom[0] < r_thresh):
		print('right hit')
		direction = -0.6
		speed = 0.1
		#if(prev_hit)
	if(data.bottom[2] < l_thresh):
		print('left hit')
		direction = 0.6
		speed = 0.1
		#if (prev_hit!=curr_hit)
		#	prev_hit = curr_hit
		#curr_hit = -1
	if(data.bottom[1] < c_thresh):
		print('center hit')
		'''
		if(prev_hit == 0):
			direction = 0
			prev_hit = 0
		if(prev_hit == -1):
			direction = -0.2
			prev_hit = -1
		if(prev_hit == 1):
			direction = 0.2
			prev_hit = 1
		#c_count = c_count + 1
		'''
		direction = 0
		speed=0.2 

def main():
	global direction
	global prev_hit
	global speed
	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('cliffTest')
	rospy.Subscriber("/mobile_base/sensors/core",SensorState,cliffCallBack)
	rate = rospy.Rate(10)
	direction = 0
	speed = 0.1
	vel = Twist()
	vel.angular.z = direction;

	while not rospy.is_shutdown():
		vel.linear.x = speed;
		vel.angular.z = direction;
		vel_pub.publish(vel)
		print("speed: {:0.2f}\tdirection: {:0.2f}".format(speed,direction))
		rate.sleep()

if __name__ == "__main__":
	main()
		
