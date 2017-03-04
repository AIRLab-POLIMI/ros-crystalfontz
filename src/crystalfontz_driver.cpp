#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/ColorRGBA.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include "typedefs.h"
#include "serial.h"
#include "cf_packet.h"
#include "show_packet.h"


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

#define SERIAL_PORT "/dev/crystalfontz"
#define BAUD 115200

// Display state variables
//		All variables are updated by the topics and periodically the dysplay is updated with these values.
int contrast = 100, backlightPower = 50;
bool contrastUpToDate = false, backlightPowerUpToDate = false;

char line1[20], line2[20], line3[20], line4[20];
bool line1UpToDate = false, line2UpToDate = false, line3UpToDate = false, line4UpToDate = false;

std_msgs::ColorRGBA led1, led2, led3, led4;
bool led1UpToDate = false, led2UpToDate = false, led3UpToDate = false, led4UpToDate = false;


void contrastCallback(const std_msgs::Int32::ConstPtr& msg){
	ROS_INFO("contrastCallback: [%i]", msg->data);
	
	if(0 <= msg->data && msg->data <= 254){
		contrast = msg->data;
		contrastUpToDate = false;
	} else {
		ROS_ERROR("contrastCallback: received contrast value: %i\n Acceptable contrast values are between 0 and 254: 0 = light, 120 = about right, 150 = dark, 151-254 = very dark (may be useful at cold temperatures)", msg->data);
	}
}

void backlightPowerCallback(const std_msgs::Int32::ConstPtr& msg){
	ROS_INFO("backlightPowerCallback: [%i]", msg->data);
	
	if(0 <= msg->data && msg->data <= 100){
		backlightPower = msg->data;
		backlightPowerUpToDate = false;
	} else {
		ROS_ERROR("backlightPowerCallback: received backlight power value: %i\n Acceptable backlight power values are: 0 = off, 1-99 = variable brightness, 100 = on", msg->data);
	}
}

void line1Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line1Callback: [%s]", msg->data.c_str());
	std::string s = msg->data;
	s.resize(sizeof(line1), ' ');
	
	for(std::string::size_type i = 0; i < s.size() && i < sizeof(line1); i++)
    	if(s[i] != '\t')
    		line1[i] = s[i];
	
	line1UpToDate = false;
}

void line2Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line2Callback: [%s]", msg->data.c_str());
	std::string s = msg->data;
	s.resize(sizeof(line2), ' ');
	
	for(std::string::size_type i = 0; i < s.size() && i < sizeof(line2); i++)
    	if(s[i] != '\t')
    		line2[i] = s[i];
    
	line2UpToDate = false;
}

void line3Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line3Callback: [%s]", msg->data.c_str());
	std::string s = msg->data;
	s.resize(sizeof(line3), ' ');
	
	for(std::string::size_type i = 0; i < s.size() && i < sizeof(line3); i++)
    	if(s[i] != '\t')
    		line3[i] = s[i];
    
	line3UpToDate = false;
}

void line4Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line4Callback: [%s]", msg->data.c_str());
	std::string s = msg->data;
	s.resize(sizeof(line4), ' ');
	
	for(std::string::size_type i = 0; i < s.size() && i < sizeof(line4); i++)
    	if(s[i] != '\t')
    		line4[i] = s[i];
    
	line4UpToDate = false;
}

void led1Callback(const std_msgs::ColorRGBA::ConstPtr& msg){
//	ROS_INFO("led1Callback");
	
	if(0.0 <= led1.r && led1.r <= 1.0
	&& 0.0 <= led1.g && led1.g <= 1.0
	&& 0.0 <= led1.b && led1.b <= 1.0
	&& 0.0 <= led1.a && led1.a <= 1.0){
		led1.r = msg->r;
		led1.g = msg->g;
		led1.b = msg->b;
		led1.a = msg->a;
		led1UpToDate = false;
	} else {
		ROS_ERROR("led1Callback: received RGBA value: std_msgs::ColorRGBA(%f, %f, %f, %f)\n Acceptable RGBA values are between 0.0f and 1.0f. Only red and green values are used. Blue and Alpha values are ignored.", msg->r, msg->g, msg->b, msg->a);
	}
}

void led2Callback(const std_msgs::ColorRGBA::ConstPtr& msg){
//	ROS_INFO("led2Callback");
	
	if(0.0 <= led2.r && led2.r <= 1.0
	&& 0.0 <= led2.g && led2.g <= 1.0
	&& 0.0 <= led2.b && led2.b <= 1.0
	&& 0.0 <= led2.a && led2.a <= 1.0){
		led2.r = msg->r;
		led2.g = msg->g;
		led2.b = msg->b;
		led2.a = msg->a;
		led2UpToDate = false;
	} else {
		ROS_ERROR("led2Callback: received RGBA value: std_msgs::ColorRGBA(%f, %f, %f, %f)\n Acceptable RGBA values are between 0.0f and 1.0f. Only red and green values are used. Blue and Alpha values are ignored.", msg->r, msg->g, msg->b, msg->a);
	}
}

void led3Callback(const std_msgs::ColorRGBA::ConstPtr& msg){
//	ROS_INFO("led3Callback");
	
	if(0.0 <= led3.r && led3.r <= 1.0
	&& 0.0 <= led3.g && led3.g <= 1.0
	&& 0.0 <= led3.b && led3.b <= 1.0
	&& 0.0 <= led3.a && led3.a <= 1.0){
		led3.r = msg->r;
		led3.g = msg->g;
		led3.b = msg->b;
		led3.a = msg->a;
		led3UpToDate = false;
	} else {
		ROS_ERROR("led3Callback: received RGBA value: std_msgs::ColorRGBA(%f, %f, %f, %f)\n Acceptable RGBA values are between 0.0f and 1.0f. Only red and green values are used. Blue and Alpha values are ignored.", msg->r, msg->g, msg->b, msg->a);
	}
}

void led4Callback(const std_msgs::ColorRGBA::ConstPtr& msg){
//	ROS_INFO("led4Callback");
	
	if(0.0 <= led4.r && led4.r <= 1.0
	&& 0.0 <= led4.g && led4.g <= 1.0
	&& 0.0 <= led4.b && led4.b <= 1.0
	&& 0.0 <= led4.a && led4.a <= 1.0){
		led4.r = msg->r;
		led4.g = msg->g;
		led4.b = msg->b;
		led4.a = msg->a;
		led4UpToDate = false;
	} else {
		ROS_ERROR("led4Callback: received RGBA value: std_msgs::ColorRGBA(%f, %f, %f, %f)\n Acceptable RGBA values are between 0.0f and 1.0f. Only red and green values are used. Blue and Alpha values are ignored.", msg->r, msg->g, msg->b, msg->a);
	}
}




// CFA-635 communications protocol only allows
// one outstanding packet at a time. Wait for the response
// packet from the CFA-635 before sending another packet.
void sendPacket(int command){
	outgoing_response.command = command;
	outgoing_response.data_length = 0;
	send_packet();
}
void sendPacket(int command, int d1){
	outgoing_response.command = command;
	outgoing_response.data[0] = d1;
	outgoing_response.data_length = 1;
	send_packet();
}
void sendPacket(int command, int d1, int d2){
	outgoing_response.command = command;
	outgoing_response.data[0] = d1;
	outgoing_response.data[1] = d2;
	outgoing_response.data_length = 2;
	send_packet();
}
void sendPacket(int command, int d1, int d2, char* line){
	outgoing_response.command = command;
	outgoing_response.data[0] = d1;
	outgoing_response.data[1] = d2;
	memcpy(&outgoing_response.data[2], line, 20); // Hardcoded packet length
	outgoing_response.data_length = 22;
	send_packet();
}

// Waits a maximum of 300ms for a packet (an ack, after sending a command)
// TODO check the packet is the right ack?
bool waitForAck(){
	//wait a maximum of 300ms for the response ack
	bool timed_out = true;
	for (int k = 0; k <= 300; k++){
		if (check_for_packet()){
			timed_out = false;
			break;
		}
		usleep(1000);
	}
	return !timed_out;
}



int main(int argc, char **argv){
	ros::init(argc, argv, "crystalfontz_driver");
	
	ros::NodeHandle n;
	
	ros::Subscriber subContrast = n.subscribe(CONTRAST_TOPIC, 1000, contrastCallback);
	ros::Subscriber subBacklightPower = n.subscribe(BACKLIGHT_POWER_TOPIC, 1000, backlightPowerCallback);
	
	ros::Subscriber subLine1 = n.subscribe(LINE_1_TOPIC, 1000, line1Callback);
	ros::Subscriber subLine2 = n.subscribe(LINE_2_TOPIC, 1000, line2Callback);
	ros::Subscriber subLine3 = n.subscribe(LINE_3_TOPIC, 1000, line3Callback);
	ros::Subscriber subLine4 = n.subscribe(LINE_4_TOPIC, 1000, line4Callback);
	
	ros::Subscriber subLed1 = n.subscribe(LED_1_TOPIC, 1000, led1Callback);
	ros::Subscriber subLed2 = n.subscribe(LED_2_TOPIC, 1000, led2Callback);
	ros::Subscriber subLed3 = n.subscribe(LED_3_TOPIC, 1000, led3Callback);
	ros::Subscriber subLed4 = n.subscribe(LED_4_TOPIC, 1000, led4Callback);
	
	ros::Publisher pubKeyActivity = n.advertise<std_msgs::Int8>(KEY_ACTIVITY_TOPIC, 1000);

	ros::Rate loop_rate(10);

	if (Serial_Init(SERIAL_PORT, BAUD)){
		ROS_ERROR("Can not not open port \"%s\" at \"%d\" baud.", SERIAL_PORT, BAUD);
	} else
		ROS_INFO("\"%s\" opened at \"%d\" baud.\n\n", SERIAL_PORT, BAUD);

	
	// Initialise state variables from config	
	n.param<int>("/crystalfontz_driver/contrast", contrast, 100);
	if(contrast < 0 || contrast > 254) {
		ROS_ERROR("Invalid contrast configuration: %i\n Acceptable contrast values are between 0 and 254: 0 = light, 120 = about right, 150 = dark, 151-254 = very dark (may be useful at cold temperatures)", contrast);
		contrast = 100;
	}
	
	n.param<int>("/crystalfontz_driver/backlight_power", backlightPower, 50);
	if(backlightPower < 0 || backlightPower > 100) {
		ROS_ERROR("Invalid backlight power configuration: %i\n Acceptable backlight power values are: 0 = off, 1-99 = variable brightness, 100 = on", backlightPower);
		backlightPower = 50;
	}
	
	std::string line1Init, line2Init, line3Init, line4Init;
	n.param<std::string>("/crystalfontz_driver/init_line_1", line1Init, "");
	n.param<std::string>("/crystalfontz_driver/init_line_2", line2Init, "");
	n.param<std::string>("/crystalfontz_driver/init_line_3", line3Init, "");
	n.param<std::string>("/crystalfontz_driver/init_line_4", line4Init, "");
	line1Init.resize(sizeof(line1), ' ');
	line2Init.resize(sizeof(line2), ' ');
	line3Init.resize(sizeof(line3), ' ');
	line4Init.resize(sizeof(line4), ' ');
	for(auto i = 0; i < sizeof(line1); i++) line1[i] = line1Init[i];
	for(auto i = 0; i < sizeof(line2); i++) line2[i] = line2Init[i];
	for(auto i = 0; i < sizeof(line3); i++) line3[i] = line3Init[i];
	for(auto i = 0; i < sizeof(line4); i++) line4[i] = line4Init[i];
	
	n.param<float>("/crystalfontz_driver/red_led_1", led1.r, 0.0);
	n.param<float>("/crystalfontz_driver/green_led_1", led1.g, 0.0);
	n.param<float>("/crystalfontz_driver/red_led_2", led2.r, 0.0);
	n.param<float>("/crystalfontz_driver/green_led_2", led2.g, 0.0);
	n.param<float>("/crystalfontz_driver/red_led_3", led3.r, 0.0);
	n.param<float>("/crystalfontz_driver/green_led_3", led3.g, 0.0);
	n.param<float>("/crystalfontz_driver/red_led_4", led4.r, 0.0);
	n.param<float>("/crystalfontz_driver/green_led_4", led4.g, 0.0);
	if(0.0 > led1.r || led1.r > 1.0 && 0.0 > led1.g || led1.g > 1.0) {
		ROS_ERROR("Invalid led configuration. Acceptable RG values are between 0.0f and 1.0f.");
		led1.r = 0.0;
		led1.g = 0.0;
	}
	if(0.0 > led2.r || led2.r > 1.0 && 0.0 > led2.g || led2.g > 1.0) {
		ROS_ERROR("Invalid led configuration. Acceptable RG values are between 0.0f and 1.0f.");
		led2.r = 0.0;
		led2.g = 0.0;
	}
	if(0.0 > led3.r || led3.r > 1.0 && 0.0 > led3.g || led3.g > 1.0) {
		ROS_ERROR("Invalid led configuration. Acceptable RG values are between 0.0f and 1.0f.");
		led3.r = 0.0;
		led3.g = 0.0;
	}
	if(0.0 > led4.r || led4.r > 1.0 && 0.0 > led4.g || led4.g > 1.0) {
		ROS_ERROR("Invalid led configuration. Acceptable RG values are between 0.0f and 1.0f.");
		led4.r = 0.0;
		led4.g = 0.0;
	}
	
	
	// Main loop
	while (ros::ok()){
		
		sendPacket(0);
		if(!waitForAck()) {
			ROS_INFO("Timed out waiting for the ping response. Trying to reset serial connection...");
			
			Uninit_Serial();
			if (Serial_Init(SERIAL_PORT, BAUD)){
				ROS_INFO("Can not open port \"%s\" at \"%d\" baud.", SERIAL_PORT, BAUD);
			} else
				ROS_INFO("\"%s\" opened at \"%d\" baud.\n\n", SERIAL_PORT, BAUD);
			
			continue;
		}
		
		if( !contrastUpToDate
		 || !backlightPowerUpToDate
		 || !line1UpToDate
		 || !line2UpToDate
		 || !line3UpToDate
		 || !line4UpToDate
		 || !led1UpToDate
		 || !led2UpToDate
		 || !led3UpToDate
		 || !led4UpToDate){
		 	
		 	// CLEAR CONSOLE
			std::system("clear");
			
			std::stringstream ss1, ss2, ss3, ss4;
			for(auto c : line1) ss1 << c;
			for(auto c : line2) ss2 << c;
			for(auto c : line3) ss3 << c;
			for(auto c : line4) ss4 << c;

			ROS_INFO("\
			\n                                 |--------------------|\
			\nled1 (r, g): %0.2f, %0.2f    line1 |%s|\
			\nled2 (r, g): %0.2f, %0.2f    line2 |%s|\
			\nled3 (r, g): %0.2f, %0.2f    line3 |%s|\
			\nled4 (r, g): %0.2f, %0.2f    line4 |%s|\
			\n                                 |--------------------|\
			\ncontrast:\t%i\
			\nbacklight:\t%i%%", 
			led1.r,
			led1.g,
			ss1.str().c_str(),
			led2.r,
			led2.g,
			ss2.str().c_str(), 
			led3.r,
			led3.g,
			ss3.str().c_str(), 
			led4.r,
			led4.g,
			ss4.str().c_str(),
			contrast,
			backlightPower);
		}

		
		
		if(!contrastUpToDate){
			sendPacket(13, contrast); // contrast setting
			if(waitForAck()) contrastUpToDate = true;
			else ROS_ERROR("Timed out waiting for a response for LCD contrast.\n");
		}
		
		if(!backlightPowerUpToDate){
			sendPacket(14, backlightPower); // backlight power setting
			if(waitForAck()) backlightPowerUpToDate = true;
			else ROS_ERROR("Timed out waiting for a response for LCD And Keypad Backlight.\n");
		}
		
		
		if(!line1UpToDate){
			sendPacket(31, 0, 0, line1);
			if(waitForAck()) line1UpToDate = true;
			else ROS_ERROR("Timed out waiting for a response on line 1.\n");
		}

		if(!line2UpToDate){
			sendPacket(31, 0, 1, line2);
			if(waitForAck()) line2UpToDate = true;
			else ROS_ERROR("Timed out waiting for a response on line 2.\n");
		}

		if(!line3UpToDate){
			sendPacket(31, 0, 2, line3);
			if(waitForAck()) line3UpToDate = true;
			else ROS_ERROR("Timed out waiting for a response on line 3.\n");
		}

		if(!line4UpToDate){
			sendPacket(31, 0, 3, line4);
			if(waitForAck()) line4UpToDate = true;
			else ROS_ERROR("Timed out waiting for a response on line 4.\n");
		}
		
		
		if(!led1UpToDate){
			sendPacket(34, 12, (int)(led1.r*100)); // red led 1
			bool timed_out1 = !waitForAck();
			
			sendPacket(34, 11, (int)(led1.g*100)); // green led 1
			bool timed_out2 = !waitForAck();			
			
			if (timed_out1 || timed_out2) ROS_ERROR("Timed out waiting for a response for led 1.\n");
			else led1UpToDate = true;
		}
		
		if(!led2UpToDate){
			sendPacket(34, 10, (int)(led2.r*100)); // red led 2
			bool timed_out1 = !waitForAck();
			
			sendPacket(34, 9, (int)(led2.g*100)); // green led 2
			bool timed_out2 = !waitForAck();
			
			if (timed_out1 || timed_out2) ROS_ERROR("Timed out waiting for a response for led 2.\n");
			else led2UpToDate = true;
		}
		
		if(!led3UpToDate){
			sendPacket(34, 8, (int)(led3.r*100)); // red led 3
			bool timed_out1 = !waitForAck();
			
			sendPacket(34, 7, (int)(led3.g*100)); // green led 3
			bool timed_out2 = !waitForAck();
			
			if (timed_out1 || timed_out2) ROS_ERROR("Timed out waiting for a response for led 3.\n");
			else led3UpToDate = true;
		}
		
		if(!led4UpToDate){
			sendPacket(34, 6, (int)(led4.r*100)); // red led 4
			bool timed_out1 = !waitForAck();
			
			sendPacket(34, 5, (int)(led4.g*100)); // green led 4
			bool timed_out2 = !waitForAck();
			
			if (timed_out1 || timed_out2) ROS_ERROR("Timed out waiting for a response for led 4.\n");
			else led4UpToDate = true;
		}
		
		
		if (check_for_packet()) {
			if(receivedKeyActivity()) {
				std_msgs::Int8 keyActivity;
				keyActivity.data = getReceivedKeyActivity();
				pubKeyActivity.publish(keyActivity);
			}
			
			showReceivedPacket();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	Uninit_Serial();
	
	return 0;
}
