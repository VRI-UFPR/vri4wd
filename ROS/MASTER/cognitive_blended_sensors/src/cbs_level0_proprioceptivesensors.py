#!/usr/bin/python2.7 

import rospy
import time
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu

left_encoder = right_encoder = 0
accelerometer_x = accelerometer_y = accelerometer_z = 0
gyroscope_x = gyroscope_y = gyroscope_z = 0

def callback_imu(msg):
	global accelerometer_x, accelerometer_y, accelerometer_z
	global gyroscope_x, gyroscope_y, gyroscope_z
	accelerometer_x = msg.linear_acceleration.x
	accelerometer_y = msg.linear_acceleration.y
	accelerometer_z = msg.linear_acceleration.z
	gyroscope_x = msg.angular_velocity.x
	gyroscope_y = msg.angular_velocity.y
	gyroscope_z = msg.angular_velocity.z	

def callback_left_encoder(msg):
	global left_encoder
	left_encoder = msg.data
def callback_right_encoder(msg):
	global right_encoder
	right_encoder = msg.data

rospy.init_node("cbs_level0_proprioceptivesensors") 

rospy.Subscriber('/left_encoder', Int16, callback_left_encoder)
rospy.Subscriber('/right_encoder', Int16, callback_right_encoder)

rospy.Subscriber('/imu/data', Imu, callback_imu)

r = rospy.Rate(2)

while not rospy.is_shutdown():
	rospy.loginfo("***VRI4WD - PROPRIOCEPTIVE SENSORS***************************")
	rospy.loginfo("Encoder (tick count) | L %d R %d\n", left_encoder, right_encoder)	
	rospy.loginfo("Accelerometer	| X %f Y %f Z %f", accelerometer_x, accelerometer_y, accelerometer_z)
	rospy.loginfo("Gyroscope		| X %f Y %f Z %f", gyroscope_x, gyroscope_y, gyroscope_z)
	rospy.loginfo("***********************READING*******************************\n") 	

	r.sleep()

