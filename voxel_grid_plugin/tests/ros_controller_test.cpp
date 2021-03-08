//
// Created by Kanishka Ganguly on 7/9/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#include "../include/ros_robot_controller.hpp"

/** @example tests/ros_controller_test.cpp
 * This is an example of how to use the ROSRobotController class.
 * Creates a new ROS node, instantiates a new ROSRobotController class and sends a sample command to the robot.
 * Resulting output can be seen in Gazebo.
 */

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "controller_test");
	ros::NodeHandle nh("ros_test_controller_node");
	std::unique_ptr <ROSRobotController> controller(new ROSRobotController(nh));
	controller->SendControlCommand("shoulder_lift_joint", math_helper::DEG2RAD(-50));
	ros::spinOnce();
	return 0;
}