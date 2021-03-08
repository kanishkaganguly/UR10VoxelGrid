
//
// Created by Kanishka Ganguly on 7/6/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#pragma once

#include "../include/imports.hpp"

/**
 * @class ROSRobotController ros_robot_controller.hpp "include/ros_robot_controller.hpp"
 * @brief This class interfaces with ROS to receive control commands from the gazebo::RobotControllerPlugin
 * and relay that to the robot's JointTrajectory controller via a ROS publisher.
 * This publishing is done via a separate thread, to prevent Gazebo locking up.
 */
class ROSRobotController {
public:
	/** @brief ROSRobotController Constructor */
	ROSRobotController(ros::NodeHandle &_n) :
			ros_node(_n),
			control_topic("/arm_controller/command"),
			loop_rate(ros::Rate(100)),
			arm_joints({"elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint",
						"wrist_3_joint"}) {
		/** ROS related setup */
		target_positions.resize(arm_joints.size());

		GetCurrentJointStates();
	};

	~ROSRobotController() {
		std::terminate();
	}

	/**
	 * @brief Gets current joint states, so that we can send joint commands
	 * only for the specified joint, keeping everything else the same.
	 * @return A vector of pairs of joint states and their current positions
	 */
	std::vector<std::pair<std::string, double>> GetCurrentJointStates() {
		sensor_msgs::JointState curr_joint_states_msg = *ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros_node);
		std::vector<std::pair<std::string, double>> curr_joint_states;
		for (int i = 0; i < curr_joint_states_msg.name.size(); ++i) {
			curr_joint_states.emplace_back(std::make_pair(curr_joint_states_msg.name[i], curr_joint_states_msg.position[i]));
		}
		assert(curr_joint_states.size() == arm_joints.size());

		return curr_joint_states;
	}

	/**
	 * @fn void SendControlCommand(const std::string &joint_name, const double &joint_value)
	 * @brief Sends control command to the robot for execution
	 * @param joint_name Joint name to control
	 * @param joint_value Target value for joint
	 */
	bool ExecuteControlCommand(const std::string &joint_name, const double &joint_value);

	/**
	 * @overload
	 */
	bool ExecuteControlCommand(const std::vector<std::string> &joint_names, const std::vector<double> &joint_values);

	/**
	 * @brief Check if arm_controller service is running
	 * Used to verify controller is started and running before sending commands
	 */
	inline bool IsControllerRunning() {
		ros::ServiceClient client = ROSRobotController::ros_node.serviceClient<controller_manager_msgs::ListControllers>(
				"/controller_manager/list_controllers");
		controller_manager_msgs::ListControllers srv;
		if (client.call(srv)) {
			spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Fetched controller info");
			std::vector<controller_manager_msgs::ControllerState> controller_states = srv.response.controller;
			for (const auto &state:controller_states) {
				std::locale loc;
				if (std::string("arm_controller").compare(state.name) == 0) {
					spdlog::get("file_logger")->info("[{}]: Controller {} is {}.", __FUNCTION__, state.name, state.state);
					return std::string("running").compare(state.state) == 0;
				}
			}
			return false;
		} else {
			spdlog::get("file_logger")->critical("[{}]: {}.", __FUNCTION__, "Fetching controller info FAILED");
			return false;
		}
	}

	/**
	 * @brief Check if /clock is being published from Gazebo
	 * This is because ROS relies on simulated clock from Gazebo for everything
	 */
	inline bool WaitForClockReady() {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "/clock is not ready yet");
		while (ros::Time::now().toSec() == 0)
			continue;
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "/clock is ready");
		return true;
	}

	/**
	 * @brief Thread function to publish ROS command.
	 */
	inline bool
	ExecutionThread(trajectory_msgs::JointTrajectory &trajectory) {
		actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> ac("/arm_controller/follow_joint_trajectory", true);
		ac.waitForServer();
		control_msgs::FollowJointTrajectoryGoal trajectory_goal;
		trajectory.header.stamp = ros::Time::now();
		trajectory_goal.trajectory = trajectory;
		ac.sendGoal(trajectory_goal);
		bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
		if (finished_before_timeout) {
			actionlib::SimpleClientGoalState state = ac.getState();
			spdlog::get("file_logger")->info("[{}]: {} {}.", __FUNCTION__, "Trajectory execution finished with ", state.toString().c_str());
			return true;
		} else {
			spdlog::get("file_logger")->info("[{}]: {}", __FUNCTION__, "Trajectory execution timed out");
			return false;
		}
	}

private:
	/** ROS node handle */
	ros::NodeHandle ros_node;
	/** ROS loop rate */
	ros::Rate loop_rate;
	/** ROS topic on which to publish */
	const std::string control_topic;
	/** Setup joint trajectory message */
	trajectory_msgs::JointTrajectory joint_trajectory_msg;
	/** List of joints in robot arm */
	const std::vector<std::string> arm_joints;
	/** Vector of points in trajectory */
	std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectories;
	/** JointTrajectoryPoint message to populate with targets */
	trajectory_msgs::JointTrajectoryPoint trajectory_point;
	/** Holder for joint targets */
	std::vector<double> target_positions;
	/** Time within which to execute robot motion */
	ros::Duration execution_time_limit = ros::Duration(2.0);

	/**
	 * @brief Get index of joint in array given joint name
	 * This is necessary for updating respective joint targets when sending control commands
	 * @param joint_name Name of joint whose index is to be returned
	 */
	inline int JointIdxFromName(const std::string &joint_name) {
		auto it = std::find(arm_joints.begin(), arm_joints.end(), joint_name);
		if (it != arm_joints.end()) {
			return std::distance(arm_joints.begin(), it);
		} else {
			return -1;
		}
	}

	/**
	 * @fn CreateTrajectoryPoint(const std::string &joint_name, const double &joint_value)
	 * @brief Create and populate a TrajectoryPoint with joint target
	 * @param joint_name Joint name to control
	 * @param joint_value Target value for joint
	 */
	void CreateTrajectoryPoint(const std::string &joint_name, const double &joint_value);

	/**
	 * @overload CreateTrajectoryPoint(const std::vector <std::string> &joint_names, const std::vector<double> &joint_values)
	 */
	void CreateTrajectoryPoint(const std::vector<std::string> &joint_names, const std::vector<double> &joint_values);


	/**
	 * @brief Create JointTrajectory message from provided data
	 */
	void ConstructControlCommand();

};

