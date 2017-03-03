#include <cstdlib>
#include <ctime>

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/ColorRGBA.h"

#include <sstream>


#define CONTRAST_TOPIC			"/crystalfontz/contrast"
#define BACKLIGHT_POWER_TOPIC	"/crystalfontz/backlight_power"
#define LINE_1_TOPIC			"/crystalfontz/line_1"
#define LINE_2_TOPIC			"/crystalfontz/line_2"
#define LINE_3_TOPIC			"/crystalfontz/line_3"
#define LINE_4_TOPIC			"/crystalfontz/line_4"
#define LED_1_TOPIC				"/crystalfontz/led_1"
#define LED_2_TOPIC				"/crystalfontz/led_2"
#define LED_3_TOPIC				"/crystalfontz/led_3"
#define LED_4_TOPIC				"/crystalfontz/led_4"
#define KEY_ACTIVITY_TOPIC		"/crystalfontz/key_activity"

void keyActivityCallback(const std_msgs::Int8::ConstPtr& msg){
	ROS_INFO("keyActivityCallback: [%i]", msg->data);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "crystalfontz_logic");
	
	ros::NodeHandle n;
	
	srand (static_cast <unsigned> (time(0)));
	
	ros::Publisher pubContrast = n.advertise<std_msgs::Int32>(CONTRAST_TOPIC, 1000);
	ros::Publisher pubBacklightPower = n.advertise<std_msgs::Int32>(BACKLIGHT_POWER_TOPIC, 1000);
	
	ros::Publisher pubLine1 = n.advertise<std_msgs::String>(LINE_1_TOPIC, 1000);
	ros::Publisher pubLine2 = n.advertise<std_msgs::String>(LINE_2_TOPIC, 1000);
	ros::Publisher pubLine3 = n.advertise<std_msgs::String>(LINE_3_TOPIC, 1000);
	ros::Publisher pubLine4 = n.advertise<std_msgs::String>(LINE_4_TOPIC, 1000);
	
	ros::Publisher pubLed1 = n.advertise<std_msgs::ColorRGBA>(LED_1_TOPIC, 1000);
	ros::Publisher pubLed2 = n.advertise<std_msgs::ColorRGBA>(LED_2_TOPIC, 1000);
	ros::Publisher pubLed3 = n.advertise<std_msgs::ColorRGBA>(LED_3_TOPIC, 1000);
	ros::Publisher pubLed4 = n.advertise<std_msgs::ColorRGBA>(LED_4_TOPIC, 1000);
	
	ros::Subscriber subKeyActivity = n.subscribe(KEY_ACTIVITY_TOPIC, 1000, keyActivityCallback);
	
	ros::Rate loop_rate(1);
	
	int count1 = 0, count2 = 0, count3 = 0, count4 = 0, contrastTest = 100, backLightTest = 50;
	
	while (ros::ok()){
		std_msgs::Int32 contrastTestMsg, backLightTestMsg;
		std_msgs::String line11, line12, line2, line3, line4;
		std_msgs::ColorRGBA led1, led2, led3, led4;
		
		
		contrastTestMsg.data = contrastTest%255;
		ROS_INFO("%i", contrastTestMsg.data);
		pubContrast.publish(contrastTestMsg);
		
		backLightTestMsg.data = backLightTest%101;
		ROS_INFO("%i", backLightTestMsg.data);
		pubBacklightPower.publish(backLightTestMsg);
		
		std::stringstream ss11;
		ss11 << "L11:" << count1 << "";
		line11.data = ss11.str();
		ROS_INFO("%s", line11.data.c_str());
		pubLine1.publish(line11);
		
		std::stringstream ss12;
		ss12 << "\t\t\t\t\t\t\t\t\t\tL12:" << count1 << "              ";
		line12.data = ss12.str();
		ROS_INFO("%s", line12.data.c_str());
		pubLine1.publish(line12);
		
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
		
		led1.r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		led1.g = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		led2.r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		led2.g = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		led3.r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		led3.g = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		led4.r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		led4.g = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.0));
		
		pubLed1.publish(led1);
		pubLed2.publish(led2);
		pubLed3.publish(led3);
		pubLed4.publish(led4);
				
		ros::spinOnce();
		loop_rate.sleep();
//		contrastTest += 10;
//		backLightTest += 10;
		count1++;
		count2++;
		count3++;
		count4++;
	}
	
	
	return 0;
}
