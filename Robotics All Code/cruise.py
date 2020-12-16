#!/usr/bin/env python
import rospy
import random
import time
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
#determine whether we are setting new setpoints after bumper or during cruise

velx=0
angz=0


def bumperCallback(data):	
	global pressed
	data.bumper

	if data.state == data.PRESSED:	
		pressed = True
def velCallback(data):
	global velx
	global angz	
	velx=data.twist.twist.linear.x
	#angz=data.twist.twist.angular.z
def main():
	global x 
	global z
	global pressed 
	global velx
	global angz
	velx = 0
	angz = 0
	pressed = False
	direc = 1
	vel_cb_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('bumper')
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
	rospy.Subscriber('/odom',Odometry,velCallback)	
	rate = rospy.Rate(20)
	vel_cb = Twist()
	preverror = 0
	integral = 0
	Kp = 0.1
	Ki = 0.3
	Kd = 0.005	
	setpoint = 0.5
	dt=0.1
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
			integral = 0
			output = 0
		else:  
			err=setpoint-velx
			integral = integral +err*dt
			derivative = (err-preverror)/dt
			output = (Kp*err)+(Ki*integral)+(Kd*derivative)
			preverror=err
			rospy.sleep(dt)
			vel_cb.linear.x=output
			vel_cb.angular.z=0	
			vel_cb_pub.publish(vel_cb)
		rate.sleep()

if __name__ == "__main__":
	main()
