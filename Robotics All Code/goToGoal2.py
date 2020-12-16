#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from time import time

def odom_callBack(data):
    global xr
    global yr
    global thetar
    xr = data.pose.pose.position.x
    yr = data.pose.pose.position.y
    thetar = data.pose.pose.orientation.z*np.pi
    thetar = atan2(np.sin(thetar),np.cos(thetar))


xg = 1.5
yg = 1.5
thetar = 0
kv = 1
kp = 1.5
ki = 1
xr = 0
yr = 0
debug = not False

def main():
    odom_sub = rospy.Subscriber('/odom',Odometry,odom_callBack)
    vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist,queue_size=1)
    reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty, queue_size=10)
    rospy.init_node('goToGoal')
    rate = rospy.Rate(10)
    d = 100

    e_int = 0;
    timer = time()
    while time() - timer < 0.25:
        reset_odom.publish(Empty())

    while not rospy.is_shutdown():
        vel = Twist()
        rospy.wait_for_message('/odom',Odometry)
        thetag = -1*atan2(1.5,1.5) + thetar
        thetag = atan2(np.sin(thetag),np.cos(thetag))
        xg = xr + np.sqrt(2)*1.5*np.cos(thetag)
        yg = yr + np.sqrt(2)*1.5*np.sin(thetag)
        starting_xr = xr
        starting_yr = yr

        print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        print('thetag: {}'.format(thetag))
        print('theta r: {}'.format(thetar))
        print('Goal point: ({},{})'.format(xg,yg))

        if debug:
            f = open("debug.dat","w+")

        while (d > 0.2) and (not rospy.is_shutdown()):
            d = np.sqrt((xg - xr)**2 + (yg - yr)**2)
            vx = kv*d
            if vx > 0.1:
                vx = 0.1

            thetad = atan2(yg - yr,xg - xr)
            e = thetad - thetar
            e = atan2(np.sin(e),np.cos(e))
            e_int = e_int + e*0.1
            az = kp*e + ki*e_int
            vel.linear.x = vx
            vel.angular.z = az
            vel_pub.publish(vel)

            if debug:
                f.write("pose: ({:04.2f},{:04.2f},{:03.2f})\t".format(xr,yr,thetar))
                f.write("thetad = {:03.2f}\t".format(thetad))
                f.write("e = {:03.2f}, az = {:03.2f}\n\n".format(e,az))

            rate.sleep()
        print('Starting point: ({},{})'.format(starting_xr,starting_yr))
        print('Ending point: ({},{})'.format(xr,yr))
        print('Reached Goal!')
        break


if __name__=='__main__':
    main()
