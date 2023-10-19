#!/usr/bin/python2.7 
import time
import os
import sys
import signal
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8

"""
MAC ADDRESS APS LAIG
1	VRI4WD_WLAN	1C:AF:F7:9B:CC:04
2	UFPR_SEM_FIO	20:BB:C0:F9:58:F0
3	UFPR_SEM_FIO	20:BB:C0:F9:58:FF
4	UFPR_SEM_FIO	50:06:04:2B:D7:50
5	eduroam		20:BB:C0:F9:58:F1
6	LAIG 1    	9C:53:22:63:5A:C3    	
7	eduroam		50:06:04:2B:D7:51
8	SALAMUNI	00:21:27:D0:85:A4
9	LAPE		10:FE:ED:C4:BA:BA
10	SR2015		14:CC:20:9A:14:3A
11	Lab_FOTO	18:A6:F7:38:8F:40
12	LABCARTO	58:10:8C:2F:D8:C6
13	LABAPWan2.4GHz	58:D5:6E:C8:1B:CF
14	Renatin02	64:DB:F7:30:98:69
15	Foyer CJ	84:D8:1B:3C:C5:C6
16	Macmini's Wi-Fi 90:72:40:25:87:6C
17	UFPR_SEM_FIO	50:06:04:2B:DE:90 	
18	eduroam   	50:06:04:2B:DE:91          	
19	UFPR_SEM_FIO  	50:06:04:2B:E0:70    

1C:AF:F7:9B:CC:04|20:BB:C0:F9:58:F0|20:BB:C0:F9:58:FF|50:06:04:2B:D7:50|20:BB:C0:F9:58:F1|9C:53:22:63:5A:C3|50:06:04:2B:D7:51|00:21:27:D0:85:A4|10:FE:ED:C4:BA:BA|14:CC:20:9A:14:3A|18:A6:F7:38:8F:40|58:10:8C:2F:D8:C6|58:D5:6E:C8:1B:CF|64:DB:F7:30:98:69|84:D8:1B:3C:C5:C6|90:72:40:25:87:6C|50:06:04:2B:DE:90|50:06:04:2B:DE:91|50:06:04:2B:E0:70
"""

rssi = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

def signal_term_handler(signal, frame):
	rospy.logger('User KeyboardInterrupt')
	sys.exit(0)

signal.signal(signal.SIGINT, signal_term_handler)

rospy.init_node("vri4wd_rpi3lidar_rssiread")

pub_rssi1 = rospy.Publisher('/rssi1_wlan', Int8, queue_size=10)
pub_rssi2 = rospy.Publisher('/rssi2_wlan', Int8, queue_size=10)
pub_rssi3 = rospy.Publisher('/rssi3_wlan', Int8, queue_size=10)
pub_rssi4 = rospy.Publisher('/rssi4_wlan', Int8, queue_size=10)
pub_rssi5 = rospy.Publisher('/rssi5_wlan', Int8, queue_size=10)
pub_rssi6 = rospy.Publisher('/rssi6_wlan', Int8, queue_size=10)
pub_rssi7 = rospy.Publisher('/rssi7_wlan', Int8, queue_size=10)
pub_rssi8 = rospy.Publisher('/rssi8_wlan', Int8, queue_size=10)
pub_rssi9 = rospy.Publisher('/rssi9_wlan', Int8, queue_size=10)
pub_rssi10 = rospy.Publisher('/rssi10_wlan', Int8, queue_size=10)
pub_rssi11 = rospy.Publisher('/rssi11_wlan', Int8, queue_size=10)
pub_rssi12 = rospy.Publisher('/rssi12_wlan', Int8, queue_size=10)
pub_rssi13 = rospy.Publisher('/rssi13_wlan', Int8, queue_size=10)
pub_rssi14 = rospy.Publisher('/rssi14_wlan', Int8, queue_size=10)
pub_rssi15 = rospy.Publisher('/rssi15_wlan', Int8, queue_size=10)
pub_rssi16 = rospy.Publisher('/rssi16_wlan', Int8, queue_size=10)
pub_rssi17 = rospy.Publisher('/rssi17_wlan', Int8, queue_size=10)
pub_rssi18 = rospy.Publisher('/rssi18_wlan', Int8, queue_size=10)
pub_rssi19 = rospy.Publisher('/rssi19_wlan', Int8, queue_size=10)

r = rospy.Rate(2)

while not rospy.is_shutdown():
	f=os.system('sudo python /home/rpi3/catkin_ws/src/vri4wd_rpi3lidar/src/iwlistparse.py > /home/rpi3/catkin_ws/src/vri4wd_rpi3lidar/src/iwlistparse.txt')

	f=os.system('cat /home/rpi3/catkin_ws/src/vri4wd_rpi3lidar/src/iwlistparse.txt | egrep "1C:AF:F7:9B:CC:04|20:BB:C0:F9:58:F0|20:BB:C0:F9:58:FF|50:06:04:2B:D7:50|20:BB:C0:F9:58:F1|9C:53:22:63:5A:C3|50:06:04:2B:D7:51|00:21:27:D0:85:A4|10:FE:ED:C4:BA:BA|14:CC:20:9A:14:3A|18:A6:F7:38:8F:40|58:10:8C:2F:D8:C6|58:D5:6E:C8:1B:CF|64:DB:F7:30:98:69|84:D8:1B:3C:C5:C6|90:72:40:25:87:6C|50:06:04:2B:DE:90|50:06:04:2B:DE:91|50:06:04:2B:E0:70" > /home/rpi3/catkin_ws/src/vri4wd_rpi3lidar/src/iwlistparseNote.txt')
	arqw = open('/home/rpi3/catkin_ws/src/vri4wd_rpi3lidar/src/iwlistparseNote.txt','r')
	texto = arqw.readlines()      
	
	i=0
	for linha in texto:	
		aux=linha.split(" dBm")
		aux=aux[0].split(" -")
		rssi[i] = aux[1]
		i+=1
	
	rssi1_i = int(rssi[0])
	rssi2_i = int(rssi[1])
	rssi3_i = int(rssi[2])
	rssi4_i = int(rssi[3])
	rssi5_i = int(rssi[4])
	rssi6_i = int(rssi[5])
	rssi7_i = int(rssi[6])
	rssi8_i = int(rssi[7])
	rssi9_i = int(rssi[8])
	rssi10_i = int(rssi[9])
	rssi11_i = int(rssi[10])
	rssi12_i = int(rssi[11])
	rssi13_i = int(rssi[12])
	rssi14_i = int(rssi[13])
	rssi15_i = int(rssi[14])
	rssi16_i = int(rssi[15])
	rssi17_i = int(rssi[16])
	rssi18_i = int(rssi[17])
	rssi19_i = int(rssi[18])

	print ""	
	rospy.loginfo("***VRI4WD ROBOT | RF Sensor | LAIG***")

	rospy.loginfo("rssi1: %d", rssi1_i)	
	rospy.loginfo("rssi2: %d", rssi2_i)
	rospy.loginfo("rssi3: %d", rssi3_i)
	rospy.loginfo("rssi4: %d", rssi4_i)
	rospy.loginfo("rssi5: %d", rssi5_i)
	rospy.loginfo("rssi6: %d", rssi6_i)
	rospy.loginfo("rssi7: %d", rssi7_i)
	rospy.loginfo("rssi8: %d", rssi8_i)
	rospy.loginfo("rssi9: %d", rssi9_i)
	rospy.loginfo("rssi10: %d", rssi10_i)
	rospy.loginfo("rssi11: %d", rssi11_i)
	rospy.loginfo("rssi12: %d", rssi12_i)
	rospy.loginfo("rssi13: %d", rssi13_i)
	rospy.loginfo("rssi14: %d", rssi14_i)
	rospy.loginfo("rssi15: %d", rssi15_i)
	rospy.loginfo("rssi16: %d", rssi16_i)
	rospy.loginfo("rssi17: %d", rssi17_i)
	rospy.loginfo("rssi18: %d", rssi18_i)
	rospy.loginfo("rssi19: %d", rssi19_i)
	
	pub_rssi1.publish(rssi1_i)
	pub_rssi2.publish(rssi2_i)
	pub_rssi3.publish(rssi3_i)
	pub_rssi4.publish(rssi4_i)
	pub_rssi5.publish(rssi5_i)
	pub_rssi6.publish(rssi6_i)
	pub_rssi7.publish(rssi7_i)
	pub_rssi8.publish(rssi8_i)
	pub_rssi9.publish(rssi9_i)
	pub_rssi10.publish(rssi10_i)
	pub_rssi11.publish(rssi11_i)
	pub_rssi12.publish(rssi12_i)
	pub_rssi13.publish(rssi13_i)
	pub_rssi14.publish(rssi14_i)
	pub_rssi15.publish(rssi15_i)
	pub_rssi16.publish(rssi16_i)
	pub_rssi17.publish(rssi17_i)
	pub_rssi18.publish(rssi18_i)
	pub_rssi19.publish(rssi19_i)

	r.sleep()
