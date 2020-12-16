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
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
#http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
# [rows,cols] = [480,640]
# hsv bright: [0,245,106]
# hsv shadow: [8,255,30]
orange_lower = (90,0,110)
orange_upper = (110,255,255)
bridge = CvBridge()

img_ball = Image()
depth_ball = 0
img_ballx = 0
img_bally = 0
img_arr = np.array([[0],[0]])
fig = plt.figure()
ax = fig.add_subplot(111)
#plt.ion()
#plt.show()

def depthCallback(data):
	global depth_ball
	global img_arr
	img_arr = np.fromstring(data.data, np.uint8)
	#image_np = cv2.imdecode(np_arr,cv2.CV_LOAD_IMAGE_UNCHANGED)
	img_arr = img_arr[2::4].copy()
	img_arr.resize(480,640)
	#print(img_arr[0:9,0:9])
	#plt.imshow(img_arr[:,:])
	#plt.draw()
	#plt.pause(0.2)
	#plt.show()
	#depth_ball = np_arr[int(img_ballx),int(img_bally)]

def imageCallback(data):
	global img_ball
	try:
		cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
	except CvBridgeError as e:
		print(e)

	img_ball = cv_image
	

def main():
	global img_ball
	global img_arr
	rospy.init_node("colorOfBall")
	image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,imageCallback)
	depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,depthCallback)

	rospy.wait_for_message('/camera/rgb/image_raw',Image)

	rate = rospy.Rate(20)

	horiz_step = 58/2/640
	vert_step = 45/2/480
	while not rospy.is_shutdown():
		rospy.wait_for_message("/camera/rgb/image_raw",Image)
		rospy.wait_for_message("/camera/depth_registered/image_raw",Image)
		cv_image_hsv = cv2.cvtColor(img_ball,cv2.COLOR_BGR2HSV)
		#(rows,cols,channels) = cv_image.shape
		#cv2.circle(cv_image,(320,240),10,(255,0,0))
	
		#blurred = cv2.GaussianBlur(cv_image,(11,11),0)
		mask = cv2.inRange(cv_image_hsv,orange_lower,orange_upper)
		mask = cv2.erode(mask,None,iterations=1)
		mask = cv2.dilate(mask,None,iterations=1)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		center = None
		#print (cnts)
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			#center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			img_ballx = x
			img_bally = y
			depth_ball = img_arr[y,x]
			# only proceed if the radius meets a minimum size
			if radius > 5 and radius < 160 and y > 140:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(img_ball, (int(x), int(y)), int(radius),
					(0, 0, 255), 2)
				print([int(x),int(y),int(radius),depth_ball])
				#cv2.circle(mask, center, 5, (0, 0, 255), -1)
	
		cv2.imshow("Image window",img_ball)
		#print([0,0,0] + cv_image_hsv[320,240,:])
		print(img_arr[y-5:y+5,x-5:x+5])
		cv2.waitKey(1) & 0xFF

		#plt.imshow(img_arr[:,:])
		#plt.show()
		rate.sleep()
	cv2.destroyAllWindows()

if __name__=='__main__':
	main()
