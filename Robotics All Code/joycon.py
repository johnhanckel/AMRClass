#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

x=0
z=0

def joycallBack(data):

	#z=args[1]
	global x
	global z	 	
	if data.buttons[4] == 1:	
		x = data.axes[1] * 0.3
		z = data.axes[0]
	else:
		x=0
		z=0 
	#print(str(x)+"   "+str(z))
	#rate.sleep() 
	
def main():
	global x 
	global z 
	vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('joycon')
	rospy.Subscriber("/joy",Joy,joycallBack)
	rate = rospy.Rate(10)
	vel = Twist()	
	while not rospy.is_shutdown():
		vel.linear.x=x
		vel.angular.z=z		
		vel_pub.publish(vel)
		rate.sleep()

if __name__ == "__main__":
	main()
