#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from math import pi
import roslib
#roslib.load_manifest('my_package')
import cv2
import rospy
import sys
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

# [rows,cols] = [480,640]
# hsv bright: [0,245,106]
# hsv shadow: [8,255,30]
orange_lower = (90,0,110)
orange_upper = (110,255,255)
bridge = CvBridge()
xs = np.array([])
zs = np.array([])
pred_zs = np.array([0,0,0,0])
pred_xs = np.linspace(-1,1,4)
show_ball = True
show_pred_path = True

img_ball = CompressedImage()
depth_img = CompressedImage()
if show_ball:
	img_arr = np.array([[0],[0]])
	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.set_xlim([-1,1])
	ax.set_ylim([0,2.8])
	marks, = ax.plot(xs,zs,label='contrail')
	current_mark, = ax.plot(0,0,'o',markersize=12,label='ball')
	if show_pred_path:
		pred_line, = ax.plot(pred_xs,pred_zs,label="predicted path")
	ax.legend()
	plt.xlabel("X Position (m)")
	plt.ylabel("Y Position (m)")
	plt.ion()
	#plt.plot()

def toDistance(x):
	if x < 0.000001:
		return 0
	c1 = 0.394
	c2 = -8.135
	c3 = 42.05
	y = np.log(x)
	return c1*y**2 + c2*y +c3	
	#c1 = -1.22
	#c2 = 12.26
	#return c1*np.log(x) + c2
	
def findLine(xs,zs):
	global pred_xs
	try:
		y = np.polyfit(xs,zs,1)
		p = np.poly1d(y)
		pred_zs = p(pred_xs)
	except:
		pred_zs = np.array([0,0,0,0])
	
	return pred_zs

def depthCallback(data):
	global img_arr
	x = data.data[12:]
	x = np.fromstring(x,np.uint8)
	x = cv2.imdecode(x,cv2.CV_LOAD_IMAGE_UNCHANGED)
	img_arr = x
	
	

def imageCallback(data):
	global img_ball
	try:
		cv_image = bridge.compressed_imgmsg_to_cv2(data,"bgr8")
	except CvBridgeError as e:
		print(e)
	img_ball = cv_image
	

def main():
	global img_ball
	global img_arr
	global xs
	global zs
	global pred_xs
	global pred_zs
	rospy.init_node("colorOfBall")
	image_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",CompressedImage,imageCallback)	
	depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw/compressedDepth",CompressedImage,depthCallback)
	ballPos_pub = rospy.Publisher("/ball/position",Odometry,queue_size=10)

	print("before wait")
	rospy.wait_for_message('/camera/rgb/image_raw/compressed',CompressedImage)
	rospy.wait_for_message('/camera/depth_registered/image_raw/compressedDepth',CompressedImage)
	print("after wait")

	rate = rospy.Rate(20)

	pixToDegree = float(58)/640
	
	while not rospy.is_shutdown():
		rospy.wait_for_message("/camera/rgb/image_raw/compressed",CompressedImage)
		rospy.wait_for_message("/camera/depth_registered/image_raw/compressedDepth",CompressedImage)
		cv_image_hsv = cv2.cvtColor(img_ball,cv2.COLOR_BGR2HSV)
		#(rows,cols,channels) = img_ball.shape
		#print([rows,cols,channels])
		#cv2.circle(cv_image,(320,240),10,(255,0,0))
		mask = cv2.inRange(cv_image_hsv,orange_lower,orange_upper)
		mask = cv2.erode(mask,None,iterations=1)
		mask = cv2.dilate(mask,None,iterations=1)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		center = None
		# only proceed if at least one contour was found
		x = 0
		z = 0
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((img_ballx, img_bally), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			z = toDistance(float(img_arr[img_bally,img_ballx]))
			#print(float(img_arr[img_bally,img_ballx]))
			
			# only proceed if the radius meets a minimum size
			if radius > 5 and radius < 200 and img_bally > 140 and (img_bally<380 and img_ballx<540):
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(img_ball, (int(img_ballx), int(img_bally)), int(radius),
					(0, 0, 255), 2)
				cv2.circle(mask,(int(img_ballx), int(img_bally)), int(radius),
					(0, 0, 255), 2)
				#print([int(img_ballx),int(img_bally),int(radius),z])
				deg = pixToDegree*float(img_ballx - 320)
				x = z*np.tan(deg*pi/180)
			
			pos = Odometry()
			if x!=0 and z!=0:
				print([x,z])
				xs = np.append(xs,x)
				zs = np.append(zs,z)
				pos.pose.pose.position.x = x
				pos.pose.pose.position.z = z
				ballPos_pub.publish(pos)
	
		cv2.imshow("Image window",img_ball)
		cv2.imshow("Mask",mask)
		#print([0,0,0] + cv_image_hsv[320,240,:])
		cv2.waitKey(1) & 0xFF
		if xs.size > 20:
			xs = xs[-19:]
		if zs.size > 20:
			zs = zs[-19:]
		if xs.size > 10:
			pred_zs = findLine(xs[-10:],zs[-10:])
		#plt.plot(xs,zs)
		#plt.plot(x,z,'x',markersize=12)
		if  show_ball:
			marks.set_xdata(xs)
			marks.set_ydata(zs)
			current_mark.set_xdata(x)
			current_mark.set_ydata(z)
			if show_pred_path:
				pred_line.set_ydata(pred_zs)
			plt.draw()
			plt.show()
		rate.sleep()
	cv2.destroyAllWindows()

if __name__=='__main__':
	main()
