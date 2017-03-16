//============================================================================
// This file is part of ros-crystalfontz
//
// Copyright (C) 2017 Artificial Intelligence & Robotics Laboratory
// of Politecnico di Milano (AIRLab) <admin.airlab-deib@polimi.it>
// Distributed under the GNU General Public License version 3.
//
// Special permission to use ros-crystalfontz under the conditions of a 
// different license can be requested from the author.
//============================================================================

#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/ColorRGBA.h"

#include <unistd.h>
#include <signal.h>
#include <sstream>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <boost/filesystem.hpp>

#include "tree.hh"
#include "tree_util.hh"

using namespace std;
using namespace boost::filesystem;

#define LINE_4_TOPIC			"/crystalfontz/line_4"
#define LED_1_TOPIC				"/crystalfontz/led_1"
#define LED_2_TOPIC				"/crystalfontz/led_2"
#define LED_3_TOPIC				"/crystalfontz/led_3"
#define LED_4_TOPIC				"/crystalfontz/led_4"
#define KEY_ACTIVITY_TOPIC		"/crystalfontz/key_activity"


enum KeyActivity {
	KEY_UP_PRESS		= 1,
	KEY_DOWN_PRESS		= 2,
	KEY_LEFT_PRESS		= 3,
	KEY_RIGHT_PRESS		= 4,
	KEY_ENTER_PRESS		= 5,
	KEY_EXIT_PRESS		= 6,
	KEY_UP_RELEASE		= 7,
	KEY_DOWN_RELEASE	= 8,
	KEY_LEFT_RELEASE	= 9,
	KEY_RIGHT_RELEASE	= 10,
	KEY_ENTER_RELEASE	= 11,
	KEY_EXIT_RELEASE	= 12
};

enum MenuState {
	DISABLED,
	ENABLED,
	RUNNING_SCRIPT
};

enum DISPLAY_CHARS {
	KEYS_CHAR 			= ' ',
	RIGHT_ARROW_CHAR 	= 16,
	LEFT_ARROW_CHAR 	= 17,
	UP_ARROW_CHAR 		= 26,
	DOWN_ARROW_CHAR 	= 27,
	ENTER_CHAR 			= 141,
	EXIT_CHAR			= 120
};

MenuState menuState = DISABLED;
pid_t currentScriptPid = -1;

tree<path> folderTree;
tree<path>::iterator folderRoot, selectedEntry;

ros::Publisher pubLine4;
std_msgs::String line4;
ros::Publisher pubLed1;
ros::Publisher pubLed2;
ros::Publisher pubLed3;
ros::Publisher pubLed4;




void expandFolder(tree<path> folderTree, tree<path>::iterator folderNode){
	
	try {
		path p(*folderNode);
		
		if (exists(p)) {
			if (is_directory(p)) {
				for(auto& entry : boost::make_iterator_range(directory_iterator(p), {})){
					path entryPath = entry.path();//.string();
					
					if (is_regular_file(entry)){
						if(entry.path().extension() == ".sh") {
							tree<path>::iterator folderChild = folderTree.append_child(folderNode, entryPath);
						}
					} else {
						tree<path>::iterator folderChild = folderTree.append_child(folderNode, entryPath);
						expandFolder(folderTree, folderChild);
					}
				}
			}
		}
		
	} catch (const filesystem_error& ex) {
		ROS_ERROR("%s", ex.what());
	}
	
}


void print_tree(const tree<path>& tr, tree<path>::pre_order_iterator it, tree<path>::pre_order_iterator end){
	if(!tr.is_valid(it)) return;
	int rootdepth=tr.depth(it);
	std::cout << "-----" << std::endl;
	while(it!=end) {
		for(int i=0; i<tr.depth(it)-rootdepth; ++i) std::cout << "  ";
		std::cout << (*it) << std::endl << std::flush;
		++it;
	}
	std::cout << "-----" << std::endl;
}

bool has_parent(tree<path>::iterator folderNode){
	tree<path>::iterator parent = folderTree.parent(folderNode);
	return parent != folderTree.begin();
}

bool has_prev_sibling(tree<path>::iterator folderNode){
	tree<path>::sibling_iterator siblings = folderNode;
	return --siblings != siblings.end();
}

bool has_next_sibling(tree<path>::iterator folderNode){
	tree<path>::sibling_iterator siblings = folderNode;
	return ++siblings != siblings.end();
}

bool has_children(tree<path>::iterator folderNode){
	return folderNode != folderTree.end() && folderTree.begin(folderNode) != folderTree.end(folderNode);
}


string formatLineEntry(tree<path>::iterator folderNode){
	string s = folderNode->stem().string();
	s.resize(20, ' ');
	
	s[15] = KEYS_CHAR;
	s[16] = has_parent(selectedEntry)?LEFT_ARROW_CHAR:' ';
	s[17] = has_prev_sibling(selectedEntry)?UP_ARROW_CHAR:' ';
	s[18] = has_next_sibling(selectedEntry)?DOWN_ARROW_CHAR:' ';
	s[19] = has_children(selectedEntry)?RIGHT_ARROW_CHAR:(is_regular_file(*selectedEntry)?ENTER_CHAR:' ');
	
	ROS_INFO("formatLineEntry: \t[%s]", s.c_str());
	
	return s;
}


void keyActivityCallback(const std_msgs::Int8::ConstPtr& msg){
	
	switch(menuState){
	
	case RUNNING_SCRIPT:
	
		if(msg->data == KEY_EXIT_RELEASE){
			menuState = ENABLED;
			ROS_INFO("EXIT");
			int failedKill = kill(currentScriptPid, 15);  // Sends the SIGTERM Signal to the process, telling it to stop.
		
			if(failedKill) ROS_ERROR("Can not kill script with pid: %i; It may be already dead.", currentScriptPid);
			else ROS_INFO("Sent SIGTERM to the script (pid: %i)", currentScriptPid);
			
			ROS_INFO("Menu enabled");
			ROS_INFO("current entry:\t\t%s", (*selectedEntry).c_str());
			currentScriptPid = -1;
			line4.data = formatLineEntry(selectedEntry).c_str();
			pubLine4.publish(line4);
		}
		break;
	
	case DISABLED:
	
		if(msg->data == KEY_ENTER_RELEASE){
			menuState = ENABLED;
			ROS_INFO("Menu enabled");
			ROS_INFO("current entry:\t\t%s", (*selectedEntry).c_str());
			line4.data = formatLineEntry(selectedEntry).c_str();
			pubLine4.publish(line4);
		}
		break;
	
	case ENABLED:
	
		tree<path>::iterator parent = folderTree.parent(selectedEntry);
		tree<path>::sibling_iterator siblings = selectedEntry;
	
		switch(msg->data){
		case KEY_UP_RELEASE:
			if(--siblings != siblings.end()){
				ROS_INFO("UP");
				selectedEntry = siblings;
				ROS_INFO("current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
				line4.data = formatLineEntry(selectedEntry).c_str();
				pubLine4.publish(line4);
			}
			break;
	
		case KEY_DOWN_RELEASE:
			if(++siblings != siblings.end()){
				ROS_INFO("DOWN");
				selectedEntry = siblings;
				ROS_INFO("current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
				line4.data = formatLineEntry(selectedEntry).c_str();
				pubLine4.publish(line4);
			}
			break;
	
		case KEY_LEFT_RELEASE:
			if(parent != folderTree.begin()){
				ROS_INFO("LEFT");
				selectedEntry = parent;
				ROS_INFO("current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
				line4.data = formatLineEntry(selectedEntry).c_str();
				pubLine4.publish(line4);
			}
			break;
	
		case KEY_RIGHT_RELEASE:
			if(selectedEntry!=folderTree.end() && folderTree.begin(selectedEntry) != folderTree.end(selectedEntry)) {
				ROS_INFO("RIGHT");
				selectedEntry = folderTree.begin(selectedEntry);
				ROS_INFO("current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
				line4.data = formatLineEntry(selectedEntry).c_str();
				pubLine4.publish(line4);
			}
			break;
	
		case KEY_ENTER_RELEASE:
			if(is_regular_file(path(*selectedEntry))){
				ROS_INFO("ENTER");
			
				currentScriptPid = fork();
				if(currentScriptPid == 0) {
					ROS_INFO("Script %s started", (*selectedEntry).c_str());
					
					int r = execl("/bin/bash", "/bin/bash", (*selectedEntry).c_str(), (char*) NULL);
					
					ROS_INFO("execl terminated with return value %i", r);
					exit(1);
				}
				
				stringstream ssr;
				ssr << "Running " << (*selectedEntry).stem().c_str() << "...";
				line4.data = ssr.str();
				line4.data.resize(20, ' ');
				line4.data[18] = KEYS_CHAR;
				line4.data[19] = EXIT_CHAR;
				pubLine4.publish(line4);
				
				menuState = RUNNING_SCRIPT;
				ROS_INFO("Executing script %s; pid: %i", (*selectedEntry).c_str(), currentScriptPid);
				
			}
			break;
	
		case KEY_EXIT_RELEASE:
			ROS_INFO("EXIT");
			ROS_INFO("Menu disabled");
			line4.data = "press ENTER for menu";
			pubLine4.publish(line4);
			menuState = DISABLED;
		
			break;
	
		}
		
		break;
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "crystalfontz_logic");
	
	ros::NodeHandle n;
	
	pubLine4 = n.advertise<std_msgs::String>(LINE_4_TOPIC, 1000);
	
	pubLed1 = n.advertise<std_msgs::ColorRGBA>(LED_1_TOPIC, 1000);
	pubLed2 = n.advertise<std_msgs::ColorRGBA>(LED_2_TOPIC, 1000);
	pubLed3 = n.advertise<std_msgs::ColorRGBA>(LED_3_TOPIC, 1000);
	pubLed4 = n.advertise<std_msgs::ColorRGBA>(LED_4_TOPIC, 1000);
	
	ros::Subscriber subKeyActivity = n.subscribe(KEY_ACTIVITY_TOPIC, 1000, keyActivityCallback);
	
	string rootFolderPathName;
	if(!n.getParam("/crystalfontz_logic/menu_folder", rootFolderPathName)) {
		ROS_ERROR("Missing script folder in configuration");
		return -1;
	}
	
	path rootFolderPath(rootFolderPathName);
	
	if(!is_directory(rootFolderPath)){
		ROS_ERROR("Script folder %s does not exists or is not a directory", rootFolderPathName.c_str());
		return -2;
	}
	
	folderRoot = folderTree.insert(folderTree.begin(), rootFolderPath);
	selectedEntry = folderRoot;
	
	expandFolder(folderTree, folderRoot);
	
	std::less<path> pathComparator;
	folderTree.sort(folderTree.begin(), folderTree.end(), pathComparator, true);
	
	cout << "Available scripts:" << endl;
	print_tree(folderTree, folderTree.begin(), folderTree.end());
	
	if(selectedEntry != folderTree.end() && folderTree.begin(selectedEntry) != folderTree.end(selectedEntry)) {
		selectedEntry = folderTree.begin(selectedEntry);
	}
	
	
	// Sleep for a second to let pubLine4 find the subscribers 
	ros::Duration init_sleep(1.0);
	init_sleep.sleep();
	
	line4.data = "press ENTER for menu";
	pubLine4.publish(line4);
	ROS_INFO("Menu disabled");
	
	ros::spin();
	
	return 0;
}

