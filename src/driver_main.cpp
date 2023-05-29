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

#include "node/DriverNode.hpp"

#define CLEAR_CONSOLE true

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriverNode>();

	if (Serial_Init(SERIAL_PORT, BAUD)){
		RCLCPP_ERROR(node->get_logger(), "Can not not open port \"%s\" at \"%d\" baud.", SERIAL_PORT, BAUD);
	} else
		RCLCPP_INFO(node->get_logger(), "\"%s\" opened at \"%d\" baud.\n\n", SERIAL_PORT, BAUD);

    rclcpp::spin(node);

    Uninit_Serial();
	
	return 0;
}
