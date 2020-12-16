#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from numpy import *

def laserCallBack(data):
	ranges = data.ranges
	left_ranges = ranges[430:639]
	center_ranges = ranges[214:423]
	right_ranges = ranges[0:209]
	left_ranges = [x for x in left_ranges if x == x]
	center_ranges = [x for x in center_ranges if x == x]
	right_ranges = [x for x in right_ranges if x == x]
	'''
	ranges_min = data.range_min
	where_are_Nans = isnan(ranges)
	#ranges[where_are_Nans] = 100
	print("min range index: {}".format(ranges.index(max(ranges))))

	'''
	print(size(center_ranges))


def main():
	laser_sub = rospy.Subscriber('/scan',LaserScan,laserCallBack)
	rospy.init_node('laserTest')
	rospy.spin()

if __name__=='__main__':
	main()
