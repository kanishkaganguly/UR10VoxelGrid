//
// Created by Kanishka Ganguly on 8/6/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#pragma once

#include "../include/imports.hpp"
#include <pybind11/pybind11.h>

/**
 *  @class GazeboInterface pygazebo.hpp "include/ros_gazebo_interface.hpp"
 *  @brief Controller class for controlling Gazebo simulation events, like stepping, pausing and resuming.
 *  Will be wrapped in pybind11 to allow Python calls from neural network.
 */
class GazeboInterface {
private:
	gazebo::transport::NodePtr gazebo_node;                             /**< @var Gazebo node for message transport */
	gazebo::transport::PublisherPtr world_control_pub;                  /**< @var world_control_pub Publish robot world control messages */
	gazebo::transport::PublisherPtr physics_pub;                        /**< @var physics_pub Publish physics control messages */
	gazebo::msgs::WorldControl world_control_request;                   /**< @var world_control_request Protobuf message for Gazebo world control */
	gazebo::msgs::Physics physics_control_request;                      /**< @var physics_control_request Protobuf message for Gazebo physics control */
	const std::string world_control_topic = "~/world_control";          /**< @var robot_control_topic Message topic for Gazebo  world control */
	const std::string physics_control_topic = "~/physics";              /**< @var robot_control_topic Message topic for Gazebo  world control */
	Logger logger;                                                      /**< @var Logger */

public:
	/** @brief GazeboInterface Constructor */
	GazeboInterface()
			: logger("gazebo_interface"),
			  gazebo_node(new gazebo::transport::Node()) {
		gazebo::client::setup();
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Initialized Gazebo Controller");
	}

	/** @brief GazeboInterface Destructor */
	~GazeboInterface() {
		gazebo::transport::fini();
		gazebo::client::shutdown();
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Finishing Gazebo Controller");
	};

	/** @brief Initialize Gazebo transport and advertise relevant nodes */
	void Init() {
		gazebo_node->Init();
		world_control_pub = gazebo_node->Advertise<gazebo::msgs::WorldControl>(world_control_topic);
		physics_pub = gazebo_node->Advertise<gazebo::msgs::Physics>(physics_control_topic);
		spdlog::get("file_logger")->info("Waiting for connection...");
		world_control_pub->WaitForConnection();
		physics_pub->WaitForConnection();
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Started Gazebo Controller");
	}

	/**
	  * @brief Initialize Gazebo world_control message for simulation.
	  *        Sets initial state to paused and manual control over time steps.
	  *        real_time_update_rate * time_step = real_time_factor
	  *        default time_step = 0.001ms
	  * @param real_time_factor - Time between two physics updates
	  */
	inline void InitSim(const double &real_time_factor) {
		world_control_request.set_pause(false);
		world_control_request.set_step(true);
		physics_control_request.set_real_time_update_rate(real_time_factor / 0.001);
		physics_pub->Publish(physics_control_request);
		world_control_pub->Publish(world_control_request);
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Initializing simulator.");
	}

	/**
	 * @brief Step through one iteration of the simulation with step_time wait between iterations.
	 */
	inline void StepSim(const double &real_time_factor) {
		physics_control_request.set_real_time_update_rate(real_time_factor / 0.001);
		world_control_request.set_pause(true);
		world_control_request.set_step(true);
		physics_pub->Publish(physics_control_request);
		world_control_pub->Publish(world_control_request);
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Stepping simulator.");
	}

	/** @brief Pause simulation. */
	inline void PauseSim() {
		world_control_request.set_pause(true);
		world_control_request.set_step(false);
		world_control_pub->Publish(world_control_request);
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Pausing simulator.");
	}

	/** @brief Start simulation. */
	inline void StartSim() {
		world_control_request.set_pause(false);
		world_control_request.set_step(false);
		world_control_pub->Publish(world_control_request);
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Starting simulator.");
	}

	/** @brief Reset simulation. */
	inline void ResetSim() {
		world_control_request.set_pause(true);
		world_control_request.mutable_reset()->set_all(true);
		world_control_pub->Publish(world_control_request);
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Resetting simulator.");
	}


};