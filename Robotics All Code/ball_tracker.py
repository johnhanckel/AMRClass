#!/usr/bin/env python
import numpy as np
import roslib
#roslib.load_manifest('my_package')
import cv2
import rospy
import sys
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# [rows,cols] = [480,640]
# hsv bright: [0,245,106]
# hsv shadow: [8,255,30]
orange_lower = (90,0,110)
orange_upper = (110,255,255)
bridge = CvBridge()

img_ballx = 0
img_bally = 0

def depthCallback(data):
	return

def imageCallback(data):
	try:
		cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
	except CvBridgeError as e:
		print(e)

	cv_image_hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
	#(rows,cols,channels) = cv_image.shape
	cv2.circle(cv_image,(320,240),10,(255,0,0))
	
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
		print([x,y])
		img_ballx = x
		img_bally = y
		# only proceed if the radius meets a minimum size
		if radius > 5:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(cv_image, (int(x), int(y)), int(radius),
				(0, 0, 255), 2)
			#cv2.circle(mask, center, 5, (0, 0, 255), -1)
	
	cv2.imshow("Image window",cv_image)
	#print([0,0,0] + cv_image_hsv[320,240,:])
	cv2.waitKey(1) & 0xFF

def main():
	rospy.init_node("colorOfBall")
	image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,imageCallback)
	depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw",Image,depthCallback)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		
	cv2.destroyAllWindows()

if __name__=='__main__':
	main()
