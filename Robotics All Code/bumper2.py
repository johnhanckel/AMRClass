#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

x=0.3
z=0


def bumperCallback(data):	
	global pressed
	data.bumper

	if data.state == data.PRESSED:	
		pressed = True
	
def main():
	global x 
	global z
	global pressed 
	pressed = False
	direc = 1
	vel_cb_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('bumper')
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
	rate = rospy.Rate(10)
	vel_cb = Twist()	
	while not rospy.is_shutdown():
		if pressed == True:	
			x = 0
			z = 0
			vel_cb.linear.x=x
			vel_cb.angular.z=z	
			vel_cb_pub.publish(vel_cb)
			rate_cb=rospy.Rate(10)
			rate_cb.sleep()
			i=0
			direc = direc * -1
			time=random.uniform(200000,600000)
			while i < time: 			
				z=direc
				i+=1
				vel_cb.linear.x=0
				vel_cb.angular.z=direc	
				vel_cb_pub.publish(vel_cb)
			pressed=False
		else:  
			x=0.2
			z=0
			vel_cb.linear.x=x
			vel_cb.angular.z=z	
			vel_cb_pub.publish(vel_cb)
		rate.sleep()

if __name__ == "__main__":
	main()
