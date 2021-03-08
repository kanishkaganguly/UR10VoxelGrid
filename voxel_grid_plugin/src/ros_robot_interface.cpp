//
// Created by Kanishka Ganguly on 7/10/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#include "../include/ros_robot_interface.hpp"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "ros_gazebo_interface");

	RobotControlInterface robot_controller;
	robot_controller.Init();
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Starting robot control interface");
	while (ros::ok()) {
		ros::spinOnce();
	}
	return 0;
}