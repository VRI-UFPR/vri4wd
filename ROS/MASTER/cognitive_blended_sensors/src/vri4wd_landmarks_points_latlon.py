#!/usr/bin/python2.7 

#http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html
#geometry_msgs/Point Message

import rospy
import time
import pandas as pd
#from tensorflow import keras

from geometry_msgs.msg import Point

rospy.init_node("vri4wd_landmarks_points")

pub_landmarks = rospy.Publisher('/landmark_vri4wd_points_latlon', Point, queue_size=10)

input_point = 0.0

r = rospy.Rate(2)

conta = 1

while not rospy.is_shutdown():

	print ("")		
	rospy.loginfo("***VRI4WD - LANDMARKS - Topography***")
	rospy.loginfo("Write(%d)", conta)
		
	teclado = raw_input('New landmark (w) Close landmarks (close): ')
	#teclado = 'w'
	
	if teclado == 'w':

		# Read the CSV file
		dataframe = pd.read_csv("/home/magrin/catkin_ws/src/cognitive_blended_sensors/dataset/topographicPointsUFPR_latlon.csv")

		input_point = input('UFPR-MAP | POINT NUMBER (1-123): ')

		input_point -= 1
		
		north = dataframe.iloc[input_point, 1]
		east = dataframe.iloc[input_point, 2]
		altitude = dataframe.iloc[input_point, 3]
		rospy.loginfo("Latitude: %.13f Longitude: %.13f Altitude: %.13f ", north, east, altitude)
		
		position_robot = Point()
	
		# X e Leste (Longitude)	
		position_robot.x = float(east)
		# Y Norte (Latitude)  
		position_robot.y = float(north)
		#Z a Altitude
		position_robot.z = float(altitude) 

		pub_landmarks.publish(position_robot)
	
		conta=conta+1
	
		r.sleep()		

	if teclado == "close":
		print ("close landmarks (Ctrl+C)")
		rospy.spin()