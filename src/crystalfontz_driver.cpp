#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "typedefs.h"
#include "serial.h"
#include "cf_packet.h"
#include "show_packet.h"


#define BACKLIGHT_POWER_TOPIC	"/crystalfontz/backlight_power"
#define LINE_1_TOPIC			"/crystalfontz/line_1"
#define LINE_2_TOPIC			"/crystalfontz/line_2"
#define LINE_3_TOPIC			"/crystalfontz/line_3"
#define LINE_4_TOPIC			"/crystalfontz/line_4"
#define KEY_ACTIVITY_TOPIC		"/crystalfontz/key_activity"

#define SERIAL_PORT "/dev/crystalfontz"
#define BAUD 115200

// Display state variables
//		All variables are updated by the topics and periodically the dysplay is updated with these values.
char line1[20], line2[20], line3[20], line4[20];
bool line1UpToDate = false, line2UpToDate = false, line3UpToDate = false, line4UpToDate = false;

int backlightPower = 50;
bool backlightPowerUpToDate = false;

void backlightPowerCallback(const std_msgs::Int32::ConstPtr& msg){
	ROS_INFO("backlightPowerCallback: [%i]", msg->data);
//	int& blp = msg->data;
	
	if(0 <= msg->data && msg->data <= 100){
		backlightPower = msg->data;
		backlightPowerUpToDate = false;
	} else {
		ROS_ERROR("backlightPowerCallback: received backlight power value: %i\n Acceptable backlight power values are: 0 = off, 1-99 = variable brightness, 100 = on", msg->data);
	}

}
void line1Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line1Callback: [%s]", msg->data.c_str());
	strncpy(line1, msg->data.c_str(), sizeof(line1));
	line1UpToDate = false;
}
void line2Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line2Callback: [%s]", msg->data.c_str());
	strncpy(line2, msg->data.c_str(), sizeof(line2));
	line2UpToDate = false;
}
void line3Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line3Callback: [%s]", msg->data.c_str());
	strncpy(line3, msg->data.c_str(), sizeof(line3));
	line3UpToDate = false;
}
void line4Callback(const std_msgs::String::ConstPtr& msg){
//	ROS_INFO("line4Callback: [%s]", msg->data.c_str());
	strncpy(line4, msg->data.c_str(), sizeof(line4));
	line4UpToDate = false;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "crystalfontz_driver");
	
	ros::NodeHandle n;
	
	ros::Subscriber subBacklightPower = n.subscribe(BACKLIGHT_POWER_TOPIC, 1000, backlightPowerCallback);
	ros::Subscriber subLine1 = n.subscribe(LINE_1_TOPIC, 1000, line1Callback);
	ros::Subscriber subLine2 = n.subscribe(LINE_2_TOPIC, 1000, line2Callback);
	ros::Subscriber subLine3 = n.subscribe(LINE_3_TOPIC, 1000, line3Callback);
	ros::Subscriber subLine4 = n.subscribe(LINE_4_TOPIC, 1000, line4Callback);
	
	ros::Publisher pubKeyActivity = n.advertise<std_msgs::Int8>(KEY_ACTIVITY_TOPIC, 1000);

	ros::Rate loop_rate(100);

	if (Serial_Init(SERIAL_PORT, BAUD)){
		ROS_INFO("Could not open port \"%s\" at \"%d\" baud.\n", SERIAL_PORT, BAUD);
		return (1);
	} else
		ROS_INFO("\"%s\" opened at \"%d\" baud.\n\n", SERIAL_PORT, BAUD);


	// Initialise the lines with white spaces
	// TODO from config
	std::stringstream ssi1, ssi2, ssi3, ssi4;
	for(auto c : line1) ssi1 << ' ';
	for(auto c : line2) ssi2 << ' ';
	for(auto c : line3) ssi3 << ' ';
	for(auto c : line4) ssi4 << ' ';


	// Main loop
	while (ros::ok()){
				
		if(!line1UpToDate || !line2UpToDate || !line3UpToDate || !line4UpToDate || !backlightPowerUpToDate){
			std::stringstream ss1, ss2, ss3, ss4;
			for(auto c : line1) ss1 << c;
			for(auto c : line2) ss2 << c;
			for(auto c : line3) ss3 << c;
			for(auto c : line4) ss4 << c;

			ROS_INFO("\
			\nline1:\t\t%s\
			\nline2:\t\t%s\
			\nline3:\t\t%s\
			\nline4:\t\t%s\
			\nbacklight:\t%i%%",
			ss1.str().c_str(), 
			ss2.str().c_str(), 
			ss3.str().c_str(), 
			ss4.str().c_str(),
			backlightPower);
		}

		//CFA-635 communications protocol only allows
		//one outstanding packet at a time. Wait for the response
		//packet from the CFA-635 before sending another
		//packet.
		
		
		if(!backlightPowerUpToDate){
			// set LCD And Keypad Backlight
			outgoing_response.command = 14;
			outgoing_response.data[0] = backlightPower; //backlight power setting
			outgoing_response.data_length = 1;
			send_packet();
		
			//wait a maximum of 300ms for the response acknowledging the display update
			bool timed_out = true;
			for (int k = 0; k <= 300; k++){
				if (check_for_packet()){
					showReceivedPacket();
					timed_out = false;
					break;
				}
				usleep(1000);
			}
			if (timed_out){
				ROS_ERROR("Timed out waiting for a response for LCD And Keypad Backlight.\n");
			} else {
				backlightPowerUpToDate = true;
			}
		}
			
		
		
		if(!line1UpToDate){
			//Send line 1 to the 635 using command 31
			outgoing_response.command = 31;
			outgoing_response.data[0] = 0; //col
			outgoing_response.data[1] = 0; //row
			memcpy(&outgoing_response.data[2], line1, 20);
			outgoing_response.data_length = 22; //the col & row position + the 20 char data length
			send_packet();

			//wait a maximum of 300ms for the response acknowledging the display update
			bool timed_out = true;
			for (int k = 0; k <= 300; k++){
				if (check_for_packet()){
					showReceivedPacket();
					timed_out = false;
					break;
				}
				usleep(1000);
			}
			if (timed_out){
				ROS_ERROR("Timed out waiting for a response on line 1.\n");
			} else {
				line1UpToDate = true;
			}
		}


		if(!line2UpToDate){
			//Send line 2 to the 635 using command 31
			outgoing_response.command = 31;
			outgoing_response.data[0] = 0; //col
			outgoing_response.data[1] = 1; //row
			memcpy(&outgoing_response.data[2], line2, 20);
			outgoing_response.data_length = 22;
			send_packet();

			//wait a maximum of 300ms for the response acknowledging the display update
			bool timed_out = true;
			for (int k = 0; k <= 300; k++) {
				if (check_for_packet()) {
					showReceivedPacket();
					timed_out = false;
					break;
				}
				usleep(1000);
			}
			if (timed_out) {
				ROS_ERROR("Timed out waiting for a response on line 2.\n");
			} else {
				line2UpToDate = true;
			}
		}
		

		if(!line3UpToDate){
			//Send line 3 to the 635 using command 31
			outgoing_response.command = 31;
			outgoing_response.data[0] = 0; //col
			outgoing_response.data[1] = 2; //row
			memcpy(&outgoing_response.data[2], line3, 20);
			outgoing_response.data_length = 22;
			send_packet();

			//wait a maximum of 300ms for the response acknowledging the display update
			bool timed_out = true;
			for (int k = 0; k <= 300; k++) {
				if (check_for_packet()) {
					showReceivedPacket();
					timed_out = false;
					break;
				}
				usleep(1000);
			}
			if (timed_out) {
				ROS_ERROR("Timed out waiting for a response on line 3.\n");
			} else {
				line3UpToDate = true;
			}
		}
		

		if(!line4UpToDate){
			//Send line 4 to the 635 using command 31
			outgoing_response.command = 31;
			outgoing_response.data[0] = 0; //col
			outgoing_response.data[1] = 3; //row
			memcpy(&outgoing_response.data[2], line4, 20);
			outgoing_response.data_length = 22;
			send_packet();

			//wait a maximum of 300ms for the response acknowledging the display update
			bool timed_out = true;
			for (int k = 0; k <= 300; k++) {
				if (check_for_packet()) {
					showReceivedPacket();
					timed_out = false;
					break;
				}
				usleep(1000);
			}
			if (timed_out) {
				ROS_ERROR("Timed out waiting for a response on line 4.\n");
			} else {
				line4UpToDate = true;
			}
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
