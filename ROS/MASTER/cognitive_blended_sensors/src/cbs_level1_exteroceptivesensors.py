#!/usr/bin/python2.7 

import rospy
import time
from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

ir1_sharp = ir2_sharp = ir3_sharp = 0
us1_urm = us3_urm = us5_urm = 0
gps_lat = gps_lon = 0
temp = 0 
roll = pitch = yaw = yawDegree = 0.0
wlan1 = wlan2 = wlan3 = wlan4 = wlan5 = wlan6 = wlan7 = wlan8 = wlan9 = wlan10 = 0
wlan11 = wlan12 = wlan13 = wlan14 = wlan15 = wlan16 = wlan17 = wlan18 = wlan19 = 0

latitude_landmark = longitude_landmark = altitude_landmark = 0

#def callback_temp(msg):
#	global temp
#	temp = msg.temperature

def rf_sensor1 (msg):
	global wlan1
	wlan1 = msg.data
def rf_sensor2 (msg):
	global wlan2
	wlan2 = msg.data
def rf_sensor3 (msg):
	global wlan3
	wlan3 = msg.data
def rf_sensor4 (msg):
	global wlan4
	wlan4 = msg.data	
def rf_sensor5 (msg):
	global wlan5
	wlan5 = msg.data
def rf_sensor6 (msg):
	global wlan6
	wlan6 = msg.data
def rf_sensor7 (msg):
	global wlan7
	wlan7 = msg.data
def rf_sensor8 (msg):
	global wlan8
	wlan8 = msg.data
def rf_sensor9 (msg):
	global wlan9
	wlan9 = msg.data
def rf_sensor10 (msg):
	global wlan10
	wlan10 = msg.data
def rf_sensor11 (msg):
	global wlan11
	wlan11 = msg.data	
def rf_sensor12 (msg):
	global wlan12
	wlan12 = msg.data
def rf_sensor13 (msg):
	global wlan13
	wlan13 = msg.data
def rf_sensor14 (msg):
	global wlan14
	wlan14 = msg.data
def rf_sensor15 (msg):
	global wlan15
	wlan15 = msg.data
def rf_sensor16 (msg):
	global wlan16
	wlan16 = msg.data
def rf_sensor17 (msg):
	global wlan17
	wlan17 = msg.data
def rf_sensor18 (msg):
	global wlan18
	wlan18 = msg.data
def rf_sensor19 (msg):
	global wlan19
	wlan19 = msg.data

def get_rotation (msg):
	global roll, pitch, yaw, yawDegree
	quaternion = (
		msg.orientation.x,	
		msg.orientation.y,
		msg.orientation.z,
		msg.orientation.w)
	euler = euler_from_quaternion (quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	#yawDegree = yaw	
	yawDegree = (((yaw*57.3))+180)	

def callback_gps(msg):
	global gps_lat
	global gps_lon
	gps_lat = msg.latitude
	gps_lon = msg.longitude


def callback_landmark(msg):
	global latitude_landmark, longitude_landmark, altitude_landmark
	latitude_landmark = msg.y
	longitude_landmark = msg.x
	altitude_landmark = msg.z

def callback_us1(msg):
	global us1_urm
	us1_urm = msg.range
def callback_us3(msg):
	global us3_urm
	us3_urm = msg.range
def callback_us5(msg):
	global us5_urm
	us5_urm = msg.range

def callback_ir1(msg):
	global ir1_sharp
	ir1_sharp = msg.range
def callback_ir2(msg):
	global ir2_sharp
	ir2_sharp = msg.range
def callback_ir3(msg):
	global ir3_sharp
	ir3_sharp = msg.range

rospy.init_node("cbs_level0_exteroceptivesensors") 

rospy.Subscriber('/ir1_sharp', Range, callback_ir1)
rospy.Subscriber('/ir2_sharp', Range, callback_ir2)
rospy.Subscriber('/ir3_sharp', Range, callback_ir3)

rospy.Subscriber('/us1_urm', Range, callback_us1)
#rospy.Subscriber('/us2_urm', Range, callback_us2)
rospy.Subscriber('/us3_urm', Range, callback_us3)
#rospy.Subscriber('/us4_urm', Range, callback_us4)
rospy.Subscriber('/us5_urm', Range, callback_us5)

rospy.Subscriber('/gps_neo8m', NavSatFix, callback_gps)

rospy.Subscriber('/landmark_vri4wd_points_latlon', Point, callback_landmark)

rospy.Subscriber('/imu/data', Imu, get_rotation)

rospy.Subscriber('/rssi1_wlan', Int8, rf_sensor1)
rospy.Subscriber('/rssi2_wlan', Int8, rf_sensor2)
rospy.Subscriber('/rssi3_wlan', Int8, rf_sensor3)
rospy.Subscriber('/rssi4_wlan', Int8, rf_sensor4)
rospy.Subscriber('/rssi5_wlan', Int8, rf_sensor5)
rospy.Subscriber('/rssi6_wlan', Int8, rf_sensor6)
rospy.Subscriber('/rssi7_wlan', Int8, rf_sensor7)
rospy.Subscriber('/rssi8_wlan', Int8, rf_sensor8)
rospy.Subscriber('/rssi9_wlan', Int8, rf_sensor9)
rospy.Subscriber('/rssi10_wlan', Int8, rf_sensor10)
rospy.Subscriber('/rssi11_wlan', Int8, rf_sensor11)
rospy.Subscriber('/rssi12_wlan', Int8, rf_sensor12)
rospy.Subscriber('/rssi13_wlan', Int8, rf_sensor13)
rospy.Subscriber('/rssi14_wlan', Int8, rf_sensor14)
rospy.Subscriber('/rssi15_wlan', Int8, rf_sensor15)
rospy.Subscriber('/rssi16_wlan', Int8, rf_sensor16)
rospy.Subscriber('/rssi17_wlan', Int8, rf_sensor17)
rospy.Subscriber('/rssi18_wlan', Int8, rf_sensor18)
rospy.Subscriber('/rssi19_wlan', Int8, rf_sensor19)

#rospy.Subscriber('/temp_mpu6050', Temperature, callback_temp)

pub_robotOrientation = rospy.Publisher('/robotOrientation', Float32)
#rospy.Publisher('/robot_orientation', Float32)

r = rospy.Rate(2)

while not rospy.is_shutdown():
	
	#rospy.loginfo("Temperature (celsius) | %3.2f", temp)	
	rospy.loginfo("***VRI4WD - EXTEROCEPTIVE SENSORS****************************") 
	rospy.loginfo("IR (cm)	 	| L %5.2f	F %5.2f	R %5.2f	", ir1_sharp, ir2_sharp, ir3_sharp)	
	rospy.loginfo("Ultrasound (cm) 	| L %5.2f	F %5.2f	R %5.2f\n", us1_urm, us3_urm, us5_urm)
	rospy.loginfo("Landmark 	| Latitude %f Longitude %f", latitude_landmark, longitude_landmark)
	rospy.loginfo("GPS 	| Latitude %f Longitude %f\n", gps_lat, gps_lon)	
	rospy.loginfo("Orientation | Yaw Degree %f\n", yawDegree)
	rospy.loginfo("RF Sensor | W1 %d (VRI4WD_WLAN * Embedded * Signal (-dBm))",  wlan1)
	rospy.loginfo("RF Sensor | W2 %d	W3 %d	W4 %d	W5 %d	W6 %d	W7 %d",wlan2, wlan3, wlan4, wlan5, wlan6, wlan7)
	rospy.loginfo("RF Sensor | W8 %d 	W9 %d 	W10 %d 	W11 %d 	W12 %d 	W13 %d", wlan8, wlan9, wlan10, wlan11, wlan12, wlan13)
	rospy.loginfo("RF Sensor | W14 %d	W15 %d	W16 %d	W17 %d	W18 %d	W19 %d", wlan14, wlan15, wlan16, wlan17, wlan18, wlan19)
	rospy.loginfo("***********************READING*******************************\n") 

	pub_robotOrientation.publish(yawDegree)

	r.sleep()

