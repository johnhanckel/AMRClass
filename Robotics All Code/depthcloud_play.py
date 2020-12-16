#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import roslib
#roslib.load_manifest('my_package')
import cv2
import rospy
import sys
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image
from itertools import islice
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

depth_img = []
img_arr = np.array([[0],[0]])

def depthCallback(data):
	global depth_img
	global img_arr
	img_arr = np.fromstring(data.data, np.uint8)
	#depth_ball = data.data[int(img_ballx),int(img_bally)]
	img_arr = img_arr[2::4].copy()
	img_arr.resize(480,640)
	print(img_arr[240,320])

def main():
	print("hello")
	global depth_img
	global img_arr
	rospy.init_node("depthTest")
	depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,depthCallback)
	rate = rospy.Rate(10)

	fig = plt.figure()
	ax = fig.add_subplot(111)
	plt.ion()
	plt.plot()

	while not rospy.is_shutdown():
		rospy.wait_for_message("/camera/depth_registered/image_raw",Image)
		print("yo")
		#depth_img = np.array(depth_img)
		#depth_img.shape = (depth_img.size//640*2,640*2)
		#print(type(depth_img))
		pos = plt.imshow(img_arr[:,:])
		np.savetxt("foo.csv",img_arr[240-20:240+20,320-20:320+20],delimiter=',')
		#fig.colorbar(pos)
		plt.pause(0.2)
		plt.show()
		rate.sleep()

if __name__=='__main__':
	main()
