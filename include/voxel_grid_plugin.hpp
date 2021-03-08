//
// Created by Kanishka Ganguly on 6/4/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#pragma once

// C++
#include "../include/robot_utils.hpp"
#include "../include/voxel_grid.hpp"
#include "../include/ros_robot_interface.hpp"

namespace gazebo {
	/**
	 *  @class VoxelGridPlugin voxel_grid_plugin.hpp "include/voxel_grid_plugin.hpp"
	 *  @brief WorldPlugin that interfaces with robot and updates the voxel grid based on bounding box data.
	 *  The WorldPlugin is the only way to access robot state from simulation.
	 */

	class VoxelGridPlugin : public WorldPlugin {
	public:
		/** @brief Constructor for VoxelGridPlugin */
		VoxelGridPlugin() :
				logger("voxel_grid_plugin"),
				world_ready(false),
				robot_loaded(false),
				iteration_skip(500),
				robot_model_name("robot"),
				obstacle_model_prefix("obstacle"),
				grid_ptr(new VoxelGrid(127, 3, Eigen::Vector3f(0, 0, 0))),
				voxel_grid_interface(),
				model_control_interface(),
				utils(),
				mgr(sensors::SensorManager::Instance()) {}

		/** @brief Destructor for VoxelGridPlugin */
		~VoxelGridPlugin() {}

		/**
		 *  @brief - internal Gazebo callback when the plugin has been loaded
		 *  @param _world - reference to the world
		 *  @param _sdf - reference to the sdf model
		 */
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

		/** @brief Fetch robot model from Gazebo. Initialize robot states array. */
		void InitializeRobotState();

		/** @brief Fetch all obstacles */
		void InitializeObstacleState();

		/**
		 * @brief Update map with robot state
		 * @param force - Force update map
		 */
		void UpdateRobotMap(const bool &force = false);

		/**
		 * @brief Update map with obstacle state
		 * @param force - Force update map
		 */
		void UpdateObstacleMap(const bool &force = false);

		/** @brief Callback function for each time the simulator world updates */
		void OnUpdate();

		void OnSensorUpdate();

		/** @brief Check if robot model is loaded */
		void IsRobotLoaded();

		/** @brief Called when all necessary models have been loaded */
		void OnWorldReady();

	private:
		Logger logger;                                                       /**< @var Initialize Logger */
		Box box;                                                             /**< @var Object of Box class */
		physics::WorldPtr world;                                             /**< @var Pointer to the world */
		sdf::ElementPtr sdf;                                                 /**< @var Pointer to the SDF */
		sensors::SensorManager *mgr;                                         /**< @var Managers all the sensors in World */
		physics::ModelPtr robot_model;                                       /**< @var Pointer to the model */
		std::string robot_model_name;                                        /**< @var Robot model name */
		std::string obstacle_model_prefix;                                   /**< @var All obstacle models have this prefix, use for filtering models */
		bool world_ready;                                                    /**< @var Updating world the first time, use for initialization */
		bool robot_loaded;                                                   /**< @var Check if robot has been loaded */
		int iteration_skip;                                                  /**< @var Iteration skip for updating map */
		event::ConnectionPtr update_connection;                              /**< @var Pointer to the update event connection */
		event::ConnectionPtr update_sensor;                                  /**< @var Pointer to the sensor update event */
		std::shared_ptr<VoxelGrid> grid_ptr;                                 /**< @var Voxel grid */
		VoxelGridVisualizer grid_visualizer;                                 /**< @var Voxel grid visualizer */
		VoxelGridInterface voxel_grid_interface;                             /**< @var Voxel grid interface to Python */
		ModelControlInterface model_control_interface;                       /**< @var Model control interface to Python */
		std::vector<std::shared_ptr<Box>> robot_state;                       /**< @var Vector of pointers to collision boxes for each link */
		std::vector<std::shared_ptr<Box>> obstacle_state;                    /**< @var Vector of pointers to collision boxes for each link */
		std::set<std::string> local_obstacle_set;                            /**< @var Set of obstacles currently being tracked locally */
		std::set<std::string> world_obstacle_set;                            /**< @var Set of obstacles available in Gazebo world */
		std::once_flag world_ready_flag;                                     /**< @var Flags for std::call_once */
		RobotUtils utils;                                                    /**< @var Utility library for reading config files */
	};
};