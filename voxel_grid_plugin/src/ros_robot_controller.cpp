
//
// Created by Kanishka Ganguly on 7/6/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#include "../include/ros_robot_controller.hpp"

void ROSRobotController::CreateTrajectoryPoint(const std::string &joint_name, const double &joint_value) {
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Creating ROS trajectory point");

	// This sets every joint position to current position
	std::vector<std::pair<std::string, double>> curr_joint_states = ROSRobotController::GetCurrentJointStates();
	for (int i = 0; i < curr_joint_states.size(); ++i) {
		ROSRobotController::target_positions[i] = curr_joint_states[i].second;
	}
	// This replaces the target position of the specified joint only
	int joint_idx = ROSRobotController::JointIdxFromName(joint_name);
	if (joint_idx != -1) {
		ROSRobotController::target_positions[joint_idx] = joint_value;
	}
	ROSRobotController::trajectory_point.positions.clear();
	ROSRobotController::trajectory_point.positions = target_positions;
	// This essentially sets the speed of execution
	ROSRobotController::trajectory_point.time_from_start = ROSRobotController::execution_time_limit;
	ROSRobotController::joint_trajectories.clear();
	ROSRobotController::joint_trajectories.emplace_back(ROSRobotController::trajectory_point);
}

void ROSRobotController::CreateTrajectoryPoint(const std::vector<std::string> &joint_names,
											   const std::vector<double> &joint_values) {
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Creating ROS trajectory point");
	std::fill(ROSRobotController::target_positions.begin(), ROSRobotController::target_positions.end(), 0);
	for (int i = 0; i < joint_names.size(); i++) {
		int joint_idx = ROSRobotController::JointIdxFromName(joint_names[i]);
		if (joint_idx != -1) {
			target_positions[joint_idx] = joint_values[i];
		}
	}
	ROSRobotController::trajectory_point.positions.clear();
	ROSRobotController::trajectory_point.positions = target_positions;
	/** This essentially sets the speed of execution */
	ROSRobotController::trajectory_point.time_from_start = ROSRobotController::execution_time_limit;
	ROSRobotController::joint_trajectories.clear();
	ROSRobotController::joint_trajectories.emplace_back(ROSRobotController::trajectory_point);
}

void ROSRobotController::ConstructControlCommand() {
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Constructing ROS control command");
	ROSRobotController::joint_trajectory_msg.joint_names = ROSRobotController::arm_joints;
	ROSRobotController::joint_trajectory_msg.points.clear();
	ROSRobotController::joint_trajectory_msg.points = joint_trajectories;
}

bool ROSRobotController::ExecuteControlCommand(const std::string &joint_name, const double &joint_value) {
	ROSRobotController::CreateTrajectoryPoint(joint_name, joint_value);
	ROSRobotController::ConstructControlCommand();
	ROS_INFO_STREAM(joint_name << "  " << joint_value);

	bool result = ROSRobotController::ExecutionThread(ROSRobotController::joint_trajectory_msg);
	if (result) {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Executing trajectory successful");
	} else {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Executing trajectory failed");
	}
	return result;
}

bool ROSRobotController::ExecuteControlCommand(const std::vector<std::string> &joint_names, const std::vector<double> &joint_values) {
	ROSRobotController::CreateTrajectoryPoint(joint_names, joint_values);
	ROSRobotController::ConstructControlCommand();

	bool result = ROSRobotController::ExecutionThread(ROSRobotController::joint_trajectory_msg);
	if (result) {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Executing trajectory successful");
	} else {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Executing trajectory failed");
	}
	return result;
}
