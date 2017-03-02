#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"

#include <sstream>


#define BACKLIGHT_POWER_TOPIC	"/crystalfontz/backlight_power"
#define LINE_1_TOPIC			"/crystalfontz/line_1"
#define LINE_2_TOPIC			"/crystalfontz/line_2"
#define LINE_3_TOPIC			"/crystalfontz/line_3"
#define LINE_4_TOPIC			"/crystalfontz/line_4"
#define KEY_ACTIVITY_TOPIC		"/crystalfontz/key_activity"

void keyActivityCallback(const std_msgs::Int8::ConstPtr& msg){
	ROS_INFO("keyActivityCallback: [%i]", msg->data);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "crystalfontz_logic");
	
	ros::NodeHandle n;
	
	ros::Publisher pubBacklightPower = n.advertise<std_msgs::Int32>(BACKLIGHT_POWER_TOPIC, 1000);
	ros::Publisher pubLine1 = n.advertise<std_msgs::String>(LINE_1_TOPIC, 1000);
	ros::Publisher pubLine2 = n.advertise<std_msgs::String>(LINE_2_TOPIC, 1000);
	ros::Publisher pubLine3 = n.advertise<std_msgs::String>(LINE_3_TOPIC, 1000);
	ros::Publisher pubLine4 = n.advertise<std_msgs::String>(LINE_4_TOPIC, 1000);
	
	ros::Subscriber subKeyActivity = n.subscribe(KEY_ACTIVITY_TOPIC, 1000, keyActivityCallback);
	
	ros::Rate loop_rate(1);
	
	int count1 = 0, count2 = 0, count3 = 0, count4 = 0, backLightTest = -10;
	
	while (ros::ok()){
		std_msgs::String line1, line2, line3, line4;
		std_msgs::Int32 backLightTestMsg;
		
		
		backLightTestMsg.data = backLightTest;
		ROS_INFO("%i", backLightTestMsg.data);
		pubBacklightPower.publish(backLightTestMsg);
		
		std::stringstream ss1;
		ss1 << "hello line 1 (" << count1 << ")";
		line1.data = ss1.str();
		ROS_INFO("%s", line1.data.c_str());
		pubLine1.publish(line1);
		
		std::stringstream ss2;
		ss2 << "hello line 2 (" << count2 << ")";
		line2.data = ss2.str();
		ROS_INFO("%s", line2.data.c_str());
		pubLine2.publish(line2);
		
		std::stringstream ss3;
		ss3 << "hello line 3 (" << count3 << ")";
		line3.data = ss3.str();
		ROS_INFO("%s", line3.data.c_str());
		pubLine3.publish(line3);
		
		std::stringstream ss4;
		ss4 << "hello line 4 (" << count4 << ")";
		line4.data = ss4.str();
		ROS_INFO("%s", line4.data.c_str());
		pubLine4.publish(line4);
				
		ros::spinOnce();
		loop_rate.sleep();
		backLightTest += 10;
		count1++;
		count2++;
		count3++;
		count4++;
	}
	
	
	return 0;
}
