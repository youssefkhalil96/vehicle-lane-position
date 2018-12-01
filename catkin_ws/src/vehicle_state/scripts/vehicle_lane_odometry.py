#!/usr/bin/env python
from math import *
import math
import utm
import matplotlib.pyplot as plt
import numpy as np
import rospy
from vehicle_state.msg import vehicle_pos
from vehicle_state.msg import vehicle_lane_position
import osmnx as ox
import utm
import sys
from sensor_msgs.msg import NavSatFix
from shapely.geometry import Point 
from shapely.geometry import LineString
from road_processing_planning.srv import *
from nav_msgs.msg import Path

class vehicle_odometry:
	def __init__(self):
		self._sub = rospy.Subscriber("fix", NavSatFix,self.call_gps)
		rospy.wait_for_service('path_getter')
		self._path_srv =  rospy.ServiceProxy('path_getter', getPath)
		self._pub =  rospy.Publisher("vehicle_lane_pos", vehicle_lane_position, queue_size=10, latch=True)
		self._pos = vehicle_lane_position()
		self._road_info = []
		self._road = []
		self._p = []
		self._array = []
  		self._gps_points = []
		self._path_msg = Path()
		self._path_msg = self._path_srv('Universidad Carlos III de Madrid, 30, Avenida de la Universidad')
		self._x = []
		self._y = []
		self._path_points = []
		self._s = 0
		self._i = 0

		

	def call_gps(self,gps):
		if gps.header.seq == 4169:
			for i in range (0,len(self._path_points)):
				plt.plot(self._path_points[i][0],self._path_points[i][1],'bo')

		UTMx, UTMy, _, _ = utm.from_latlon(gps.latitude, gps.longitude)
		self._array =[UTMx,UTMy] 	
		self._gps_points.append(self._array)

		
		plt.plot(UTMx,UTMy,'ro')

		self.vehicle_position()

		if gps.header.seq == 4534:
			print("done")
			plt.show()
			self._sub.unregister()
			print(len(self._gps_points))
			
	def get_path(self):	

		# for i in range(0, len(self._path_msg.path.poses)):
		for i in range (32,40):
			my_tuple = (self._path_msg.path.poses[i].pose.position.x, self._path_msg.path.poses[i].pose.position.y)
			self._path_points.append(my_tuple)
		print('path points = ')
		print(self._path_points)
	

	def vehicle_position(self):
		
		Ld = 0
		Rd = 0
		z = 0
		m = len(self._path_points)
		j = len(self._gps_points)-1
		# print("i = " + str(self._i)) 
		# print("j = " + str(j))
		if self._i < m:
		 	if j > 0:
				R = sqrt((self._path_points[self._i][1]-self._gps_points[j][1])**2+(self._path_points[self._i][0]-self._gps_points[j][0])**2)

				T = sqrt((self._path_points[self._i][1]-self._gps_points[j-1][1])**2+(self._path_points[self._i][0]-self._gps_points[j-1][0])**2)

				if R > T:
					plt.plot(self._gps_points[j][0],self._gps_points[j][1],'*')
					print("######################################")
					print("next")
					self._i = self._i+1
					print("######################################")
					


			if self._i == 0 and j == 0:   #this for to know if the vehicle in starting position
				yaw_angle_vehicle = math.degrees(math.atan2(self._gps_points[j][1] , self._gps_points[j][0]))
				yaw_angle_road = math.degrees(math.atan2(self._path_points[self._i][1] , self._path_points[self._i][0]))
				yaw_error = yaw_angle_road - yaw_angle_vehicle

				way_point_1 = Point(435008.393042,4465160.309248)
				way_point_2 = Point(self._path_points[self._i][0],self._path_points[self._i][1])
				gps = Point(self._gps_points[j][0],self._gps_points[j][1])
				points = []
				points.append(way_point_1)
				points.append(way_point_2)
				line = LineString(points)
				delta = line.distance(gps)

			elif self._i == 0 and j > 0:
				x_v = self._gps_points[j][0] - self._gps_points[j-1][0]
				y_v = self._gps_points[j][1] - self._gps_points[j-1][1]
				yaw_angle_vehicle = math.degrees(atan(y_v/x_v))

				x_r = self._path_points[self._i][0] - 435008.393042
				y_r = self._path_points[self._i][1] - 4465160.309248
				yaw_angle_road = math.degrees(math.atan(y_r / x_r))
				
				yaw_error = yaw_angle_road - yaw_angle_vehicle
				
				way_point_1 = Point(435008.393042,4465160.309248)
				way_point_2 = Point(self._path_points[self._i][0],self._path_points[self._i][1])
				gps = Point(self._gps_points[j][0],self._gps_points[j][1])
				points = []
				points.append(way_point_1)
				points.append(way_point_2)
				line = LineString(points)
				delta = line.distance(gps)


			else:
				x_v = self._gps_points[j][0] - self._gps_points[j-1][0]
				y_v = self._gps_points[j][1] - self._gps_points[j-1][1]
				yaw_angle_vehicle = math.degrees(atan(y_v/x_v))

				x_r = self._path_points[self._i][0] - self._path_points[self._i-1][0]
				y_r = self._path_points[self._i][1] - self._path_points[self._i-1][1]
				yaw_angle_road = math.degrees(atan(y_r/x_r))

				yaw_error = yaw_angle_road - yaw_angle_vehicle

				way_point_1 = Point(self._path_points[self._i-1][0],self._path_points[self._i-1][1])
				way_point_2 = Point(self._path_points[self._i][0],self._path_points[self._i][1])
				gps = Point(self._gps_points[j][0],self._gps_points[j][1])
				points = []
				points.append(way_point_1)
				points.append(way_point_2)
				line = LineString(points)
				delta = line.distance(gps)


			y = self._path_points[self._i][1] - self._gps_points[j][1]
			x = self._path_points[self._i][0] - self._gps_points[j][0]
			slope = float(y / x)
			
			if slope > 0:
				Ld = 1.5-delta
				Rd = 1.5+delta
			elif slope < 0:
				Ld = 1.5+delta
				Rd = 1.5-delta
			
			self._pos.yaw = yaw_error
			self._pos.rd = Rd
			self._pos.ld = Ld
			self._pub.publish(self._pos)

			print("yaw angle road = " + str(yaw_angle_road))
			print("yaw angle vehicle = " +str(yaw_angle_vehicle))
			print("yaw error = " + str(yaw_error))
			print("right distance = " + str(Rd))
			print("left distance = " + str(Ld))
			# print(z)
			print("****************************")
			
