#!/usr/bin/env python

import rospy
from vehicle_lane_odometry import *


if __name__ == '__main__':
	try:
		position = vehicle_odometry()
		rospy.init_node('vehicle_odometry', anonymous=True)
		position.get_path()
		# position.vehicle_position()
		rospy.spin()

	except rospy.ROSInterruptException:
		print("exception occured")
