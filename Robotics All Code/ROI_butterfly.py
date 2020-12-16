import cv2
import numpy as np

if __name__=='__main__':
	im=cv2.imread("butterfly.jpg")
	#r=cv2.selectROI(im,False)
	#imcrop=im[int(r[1]):int(r[1]+r[3]),int(r[0]):int(r[0]+r[2])]
	cv2.imshow("Image",imcrop)
	cv2.waitKey(1)
