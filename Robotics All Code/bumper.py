#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

x=0.3
z=0

def bumperCallback(data):
	global x	#forward velocity
	global z	#angular velocity
	global hit	#bumper signal 	
	data.bumper
	data.state	
	if data.state == data.PRESSED:	
		x = 0
		z = 0
		vel_cb_pub=rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
		vel_cb=Twist()
		vel_cb.linear.x=x
		vel_cb.angular.z=z	
		vel_cb_pub.publish(vel_cb)
		rate_cb=rospy.Rate(10)
		rate_cb.sleep()
		i=0
		while i < random.uniform(100,300): 
			z=1
			i+=1
			vel_cb.linear.x=x
			vel_cb.angular.z=z	
			vel_cb_pub.publish(vel_cb)
		print(i)	
	else:  
		x=0.3
		z=0
					
	#elif 
	#print(str(x)+"   "+str(z))
	#rate.sleep() 
	
def main():
	global x 
	global z 
	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('bumper')
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
	rate = rospy.Rate(10)
	vel = Twist()	
	while not rospy.is_shutdown():
		vel.linear.x=x
		vel.angular.z=z		
		vel_pub.publish(vel)
		rate.sleep()

if __name__ == "__main__":
	main()
