#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry

#need to add joystick teleop mechanism
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
def joycallBack(data):
	global x
	global z
	global setpoint	 	
	if data.buttons[0]==1:
		setpoint = 0.5
	if data.buttons[1]==1:
		setpoint = 0.3
	if data.buttons[2]==1:
		setpoint = 0
	if data.buttons[3]==1:
		setpoint=0.6
	
def main():
	global x 
	global z
	global pressed 
	global velx
	global angz
	global setpoint 
	setpoint = 0.5
	velx = 0
	angz = 0
	pressed = False
	direc = 1
	vel_cb_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
	rospy.init_node('auto')
	rospy.Subscriber("/joy",Joy,joycallBack)
	rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,bumperCallback)
	rospy.Subscriber('/odom',Odometry,velCallback)	
	rate = rospy.Rate(20)
	vel_cb = Twist()
	preverror = 0
	integral = 0
	Kp = 0.1
	Ki = 0.3
	Kd = 0.005		
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
			time=random.uniform(200000,400000)
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
