#!/usr/bin/env python
import numpy as np
import roslib
#roslib.load_manifest('my_package')
import cv2
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# This simply print the butterfly.jpg image as b&w
'''
img = cv2.imread("butterfly.jpg",0)
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''

class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callBack)
	
	def callBack(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)
		'''
		(rows,cols,channels) = cv_image.shape
		if cols > 60 and rows > 60:
			cv2.circle(cv_image,(50,50),10,255)
		'''
		cv2.imshow("Image window",cv_image)
		cv2.waitKey(1)

	
def main(args):
	ic = image_converter()
	rospy.init_node('image_converter')
	rospy.spin()
	cv2.destroyAllWindows()

if  __name__=='__main__':
	main(sys.argv)
