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

#include "node/LogicNode.hpp"

using namespace std;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LogicNode>();

    if(!node->initialization()) {
        return -1;
        RCLCPP_ERROR(node->get_logger(), "Failed to initialise node");
    }

    rclcpp::spin(node);




    // Sleep for a second to let pubLine4 find the subscribers
//	ros::Duration init_sleep(1.0);
//	init_sleep.sleep();

//	ros::spin();

	return 0;
}

