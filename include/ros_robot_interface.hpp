//
// Created by Kanishka Ganguly on 7/10/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#pragma once

#include "../include/robot_utils.hpp"
#include "../include/ros_robot_controller.hpp"
#include "../include/voxel_grid.hpp"

/**
 *  @class RobotControlInterface ros_gazebo_interface.hpp "include/ros_gazebo_interface.hpp"
 *  @brief Controller class for sending control commands to robot.
 *  Will be wrapped in ROS services to allow Python calls from neural network.
 */
class RobotControlInterface {
protected:
	const std::string action_name;                                                     /**< @var Name of action server */
	ros::NodeHandle nh;                                                                /**< @var ROS node handle */
	actionlib::SimpleActionServer <voxel_grid_plugin::RobotControlAction> as;          /**< @var ROS action server */
	voxel_grid_plugin::RobotControlFeedback feedback;                                  /**< @var Action feedback */
	voxel_grid_plugin::RobotControlResult result;                                      /**< @var Action results */
	Logger logger;                                                                     /**< @var Logger */
private:
	std::unique_ptr<ROSRobotController> ros_controller;                                /**< @var ROS controller for robot */

public:
	/** @brief RobotControlInterface Constructor */
	RobotControlInterface() :
			logger("robot_control_interface"),
			action_name("RobotControlActionServer"),
			as(nh, action_name, boost::bind(&RobotControlInterface::ExecuteAction, this, _1), false) {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Starting Robot Controller Action Server");
	};

	/** @brief RobotControlInterface Destructor */
	~RobotControlInterface() {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Finishing Robot Controller Action Server");
	};

	/**
	 * @brief Initialize ROS node handle and robot controller object
	 */
	void Init() {
		ros_controller.reset(new ROSRobotController(nh));
		ros_controller->WaitForClockReady();
		as.start();
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Started Robot Controller Action Server");
	}

	/**
	 * @fn inline void AddRobotCommand(const std::string &joint_name, const double &joint_target)
	 * @brief Send joint control command to trajectory queue, but do not execute
	 * @param joint_name - Name of joint to control
	 * @param joint_target - Joint target to execute
	 */
	void ExecuteRobotCommand(const std::string &joint_name, const double &joint_target) {
		if (ros_controller->IsControllerRunning()) {
			spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Sending joint target to robot");
			bool exec_result = ros_controller->ExecuteControlCommand(joint_name, joint_target);
			feedback.status = std::string("Trajectory Execution: ") + (exec_result ? "success" : "failed");
			as.publishFeedback(feedback);
			result.finished = exec_result;
		} else {
			spdlog::get("file_logger")->warn("[{}]: {}.", __FUNCTION__, "Controller is not running, cannot send command to robot");
			feedback.status = "Controller is not running, cannot send command to robot";
			as.publishFeedback(feedback);
			result.finished = false;
		}
		return;
	}

	/**
	 * @brief Action server callback
	 */
	void ExecuteAction(const voxel_grid_plugin::RobotControlGoalConstPtr &command) {
		spdlog::get("file_logger")->info("[{}-{}] {}.", __FUNCTION__, action_name, "Received action goal");
		ExecuteRobotCommand(command->joint_name, command->joint_target);
		as.setSucceeded(result);
	}
};

/**
 *  @class ModelControlInterface ros_gazebo_interface.hpp "include/ros_gazebo_interface.hpp"
 *  @brief Controller class for spawning, deleting and modifying obstacle models.
 *  Will be wrapped in ROS services to allow Python calls from neural network.
 */
class ModelControlInterface {

public:
	ModelControlInterface() :
			action_name("ModelControlActionServer"),
			as(nh, action_name, boost::bind(&ModelControlInterface::ExecuteAction, this, _1), false),
			utils() {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Starting ModelControl Action Server");
	}

	~ModelControlInterface() {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Finishing ModelControl Action Server");
	}

	void Init(gazebo::physics::WorldPtr _world) {
		this->world = _world;
		as.start();
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Started ModelControl Action Server");
	}

	void ExecuteAction(const voxel_grid_plugin::ModelControlGoalConstPtr &command) {
		std::string pkg_path = ros::package::getPath("voxel_grid_plugin");
		std::string models_path = pkg_path + "/models/obstacle_box_";
		switch (command->model_command) {
			case voxel_grid_plugin::ModelControlGoal::MODEL_ADD: {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Adding model");
				if (!this->CheckModelExists(static_cast<std::string>(command->model_name))) {
					spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Model does not exist in world");
					switch (command->model_size) {
						case voxel_grid_plugin::ModelControlGoal::MODEL_SMALL:
							spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Spawning Small model");
							models_path += "small.sdf";
							break;
						case voxel_grid_plugin::ModelControlGoal::MODEL_MED:
							spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Spawning Medium model");
							models_path += "med.sdf";
							break;
						case voxel_grid_plugin::ModelControlGoal::MODEL_LARGE:
							spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Spawning Large model");
							models_path += "large.sdf";
							break;
					}

					geometry_msgs::Pose pose = command->initial_pose;
					Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
					Eigen::Vector3f euler = math_helper::eigen_conversions::EigenQuaternionToEulerXYZ(q);
					std::stringstream ss;
					ss << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
					   << euler(0) << " " << euler(1) << " " << euler(2);
					std::string prefixed_name = voxel_grid_plugin::ModelControlGoal::MODEL_PREFIX + command->model_name;
					std::string updated_name_sdf = utils.SetModelNameFromFile(models_path, prefixed_name);
					std::string updated_pose_sdf = utils.SetInitialPoseFromString(updated_name_sdf, ss.str());
					std::string updated_collision_sdf = utils.SetSensorDataFromString(updated_pose_sdf, prefixed_name);
					this->world->InsertModelString(updated_collision_sdf);
					result.finished = true;
				} else {
					result.finished = false;
				}
				break;
			}
			case voxel_grid_plugin::ModelControlGoal::MODEL_DELETE: {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Deleting model");
				if (this->CheckModelExists(command->model_name)) {
					this->world->RemoveModel(command->model_name);
					result.finished = true;
				} else {
					result.finished = false;
				}
				break;
			}
			case voxel_grid_plugin::ModelControlGoal::MODEL_MOVE: {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Moving model");
				if (this->CheckModelExists(command->model_name)) {
					spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Model exists");
					std::string prefixed_name = voxel_grid_plugin::ModelControlGoal::MODEL_PREFIX + command->model_name;
					gazebo::physics::ModelPtr model = this->world->GetModel(prefixed_name);
					geometry_msgs::Pose pose = command->target_pose;
					gazebo::math::Vector3 pos(pose.position.x, pose.position.y, pose.position.z);
					gazebo::math::Quaternion rot(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
					gazebo::math::Pose gz_pose(pos, rot);
					model->SetWorldPose(gz_pose, true, true);
				} else {
					spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Model does not exist");
				}
				break;
			}
			default:
				result.finished = false;
				break;
		}
		as.setSucceeded(result);
	}

private:
	/** @var Pointer to Gazebo world */
	gazebo::physics::WorldPtr world;
	/** @var Name of action server */
	const std::string action_name;
	/** @var ROS node handle */
	ros::NodeHandle nh;
	/** @var ROS action server */
	actionlib::SimpleActionServer <voxel_grid_plugin::ModelControlAction> as;
	/** @var ROS action feedback */
	voxel_grid_plugin::ModelControlFeedback feedback;
	/** @var ROS action result */
	voxel_grid_plugin::ModelControlResult result;
	/** @var Utility library for reading config files */
	RobotUtils utils;


	/**
	 * @brief Checks if model with given name exists in Gazebo world
	 * @param model_name - Name of model to check for
	 * @return True if model exists, False if it doesn't
	 */
	bool CheckModelExists(const std::string &model_name) {
		std::string prefixed_name = voxel_grid_plugin::ModelControlGoal::MODEL_PREFIX + model_name;
		gazebo::physics::ModelPtr model_ptr = this->world->GetModel(prefixed_name);
		return model_ptr == nullptr ? false : true;
	}
};

/**
 *  @class VoxelGridInterface ros_gazebo_interface.hpp "include/ros_gazebo_interface.hpp"
 *  @brief Class to interface with VoxelGridPlugin, to fetch updated voxel grids as needed.
 *  Will be wrapped in ROS services to allow Python calls from neural network.
 */
class VoxelGridInterface {
public:
	/** @brief VoxelGridInterface Constructor */
	VoxelGridInterface() :
			action_name("VoxelGridActionServer"),
			as(nh, action_name, boost::bind(&VoxelGridInterface::ExecuteAction, this, _1), false) {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Starting VoxelGrid Action Server");
	}

	/** @brief RobotControlInterface Destructor */
	~VoxelGridInterface() {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Finishing VoxelGrid Action Server");
	}

	/** @brief Initialize VoxelGridInterface */
	void Init(std::shared_ptr<VoxelGrid> &voxel_grid_ptr) {
		this->voxel_grid_ptr = voxel_grid_ptr;
		as.start();
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Started VoxelGrid Action Server");
	}

	/** @brief Action server callback */
	void ExecuteAction(const voxel_grid_plugin::VoxelGridGoalConstPtr &command) {
		spdlog::get("file_logger")->info("[{}-{}] {}.", __FUNCTION__, action_name, "Received action goal");
		// Set total number of voxels in grid
		result.num_voxels_per_dim = voxel_grid_ptr->NUM_VOXELS_PER_DIM;
		// Fetch latest voxel grid
		const Tensor &grid = this->voxel_grid_ptr->GetGrid();
		// Total voxels
		const float total_voxels = (float) std::pow(voxel_grid_ptr->NUM_VOXELS_PER_DIM + 1, 3);
		// Start timer
		auto start = std::chrono::high_resolution_clock::now();
		// Iterate through all the voxels
		for (int x = 0; x < voxel_grid_ptr->NUM_VOXELS_PER_DIM; ++x) {
			for (int y = 0; y < voxel_grid_ptr->NUM_VOXELS_PER_DIM; ++y) {
				for (int z = 0; z < voxel_grid_ptr->NUM_VOXELS_PER_DIM; ++z) {
					// Set the voxel coordinates
					voxel_grid_plugin::VoxelData voxel_data;
					voxel_data.voxel_coordinate.push_back(x);
					voxel_data.voxel_coordinate.push_back(y);
					voxel_data.voxel_coordinate.push_back(z);
					// Check and set voxel state
					if (grid(x, y, z) == voxel_grid_ptr->CELL_STATE::ROBOT) {
						voxel_data.voxel_value = voxel_grid_ptr->CELL_STATE::ROBOT;
						// Add voxel data to corresponding array
						result.robot_nodes.push_back(voxel_data);
					} else if (grid(x, y, z) == voxel_grid_ptr->CELL_STATE::OCCUPIED) {
						voxel_data.voxel_value = voxel_grid_ptr->CELL_STATE::OCCUPIED;
						// Add voxel data to corresponding array
						result.occupied_nodes.push_back(voxel_data);
					}
					// Compute progress
//					float progress = (((x * voxel_grid_ptr->NUM_VOXELS_PER_DIM * voxel_grid_ptr->NUM_VOXELS_PER_DIM) +
//									   (y * voxel_grid_ptr->NUM_VOXELS_PER_DIM) + z) / total_voxels) * 100;
//					feedback.completion_percent = progress;
//					as.publishFeedback(feedback);
				}
			}
		}
		// End timer
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
		spdlog::get("file_logger")->info("[{}-{}] {} {} {}.", __FUNCTION__, action_name, "Sending VoxelGrid data took", std::to_string((float) duration.count()), "ms");
		as.setSucceeded(result);
	}

private:
	/** @var Pointer to VoxelGrid from VoxelGridPlugin */
	std::shared_ptr<VoxelGrid> voxel_grid_ptr;
	/** @var Name of action server */
	const std::string action_name;
	/** @var ROS node handle */
	ros::NodeHandle nh;
	/** @var ROS action server */
	actionlib::SimpleActionServer <voxel_grid_plugin::VoxelGridAction> as;
	/** @var Action feedback */
	voxel_grid_plugin::VoxelGridFeedback feedback;
	/** @var Action results */
	voxel_grid_plugin::VoxelGridResult result;
};
