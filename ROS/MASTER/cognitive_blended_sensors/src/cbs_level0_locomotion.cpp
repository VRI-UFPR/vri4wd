#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "kbhit.h" 

using namespace std;

int main(int argc, char **argv){

	ros::init(argc, argv, "cbs_level0_locomotion");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
 
	ros::Rate loop_rate(10);

	float vel_linear = 0.7;
	float vel_angular = 0.7;
	char tecla = '0';
	
	while (ros::ok()&&!(tecla == 'p')){

		geometry_msgs::Twist msg;

		if (kbhit())
			tecla = getchar();

		ros::spinOnce();//This command will read and update all ROS topics.

		switch(tecla){ 
			case 'w':	msg.linear.x = vel_linear;
								msg.angular.z = 0;
								ROS_INFO("VRI Robot | Front");
								break;
			case 's':	msg.linear.x = -(vel_linear);
								msg.angular.z = 0;
								ROS_INFO("VRI Robot | Rear");
								break;
			case 'a':	msg.linear.x = 0;
								msg.angular.z = vel_angular;
								ROS_INFO("VRI Robot | Left");
								break;
			case 'd':	msg.linear.x = 0;
								msg.angular.z = -(vel_angular);
								ROS_INFO("VRI Robot | Right");
								break;			
			default:	msg.linear.x = 0; //qualquer tecla
								msg.angular.z = 0;
								ROS_INFO("VRI Robot | Stop");
					
			}
			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
	}
 	ROS_WARN("Teleoperacao VRI Robot finalizada...");
	return 0;
}


