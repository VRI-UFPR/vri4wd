rplidar.launch
$ sudo chmod 666 /dev/ttyUSB0
$ roslaunch rplidar_ros rplidar.launch

imu.launch
$ sudo chmod 666 /dev/ttyUSB1
$ roslaunch ros_imu_bno055 imu.launch

vri4wd_rpi3lidar_rssiread.py
$ rosrun vri4wd_rpi3lidar vri4wd_rpi3lidar_rssiread.py
mkdir -> arqw = open('/home/rpi3/catkin_ws/src/vri4wd_rpi3lidar/src/iwlistparseNote.txt','r')

serial_node.py 
$ sudo chmod 666 /dev/ttyS0
$ rosrun rosserial_arduino serial_node.py 

Firmware Atmega 2560
sketch_ros_2560_ago25a.ino
