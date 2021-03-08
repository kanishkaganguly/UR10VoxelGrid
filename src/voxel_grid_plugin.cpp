//
// Created by Kanishka Ganguly on 6/4/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#include "../include/voxel_grid_plugin.hpp"

namespace gazebo {
	void VoxelGridPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Starting Voxel Grid Plugin");

		// Store the pointer to the world, model and SDF
		VoxelGridPlugin::world = _world;
		VoxelGridPlugin::sdf = _sdf;

		// Setup visualization
		VoxelGridPlugin::grid_visualizer = VoxelGridVisualizer();

		// Start VoxelGridInterface
		voxel_grid_interface.Init(grid_ptr);

		// Start ModelControlInterface
		model_control_interface.Init(VoxelGridPlugin::world);

		// WorldUpdate event
		VoxelGridPlugin::update_connection = event::Events::ConnectWorldUpdateEnd(
				std::bind(&VoxelGridPlugin::OnUpdate, this));

	}

	void VoxelGridPlugin::IsRobotLoaded() {
		// Wait for robot model to be spawned in Gazebo
		for (auto model:VoxelGridPlugin::world->GetModels()) {
			if (VoxelGridPlugin::robot_model_name.compare(model->GetName()) == 0) {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Robot has been loaded");
				VoxelGridPlugin::robot_loaded = true;
			}
		}
	}

	void VoxelGridPlugin::OnWorldReady() {
		// Access robot model and get all links in model
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Setting up robot");
		VoxelGridPlugin::robot_model = VoxelGridPlugin::world->GetModel(robot_model_name);
		// Setup robot
		VoxelGridPlugin::InitializeRobotState();
		// Update robot map
		VoxelGridPlugin::UpdateRobotMap(true);
		// Done
		VoxelGridPlugin::world_ready = true;
	}

	void VoxelGridPlugin::InitializeRobotState() {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Initializing robot state");

		for (auto &link:VoxelGridPlugin::robot_model->GetLinks()) {
			spdlog::get("file_logger")->info("[{}]: Initializing link {}.", __FUNCTION__, link->GetName());

			// Empty box with some default data
			std::shared_ptr<Box> box_ptr = std::make_shared<Box>();
			box_ptr->SetName(link->GetName());
			// Right face placeholder
			std::array<Eigen::Vector4f, 4> right_face;
			// Left face placeholder
			std::array<Eigen::Vector4f, 4> left_face;
			// Get bounding box for link
			math::Box cbox = link->GetCollisionBoundingBox();
			// Get canonical positions
			utils.GetCanonicalPositionsWithAxisMapping(link->GetName(), cbox, left_face, right_face);
			// Set all the data
			box_ptr->SetCanonical("left", left_face);
			box_ptr->SetCanonical("right", right_face);
			// Set box data
			VoxelGridPlugin::robot_state.push_back(box_ptr);
		}
	}

	void VoxelGridPlugin::InitializeObstacleState() {
		// Update every iteration_skip times
		if (VoxelGridPlugin::world->GetIterations() % VoxelGridPlugin::iteration_skip == 0) {
			spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Updating obstacle list");

			// Make a list of all models that are "obstacles"
			VoxelGridPlugin::world_obstacle_set.clear();
			for (auto &model:VoxelGridPlugin::world->GetModels()) {
				std::string model_name = model->GetName();
				// Check if model is an obstacle
				std::size_t is_obstacle = model_name.find(VoxelGridPlugin::obstacle_model_prefix);
				if (is_obstacle != std::string::npos) {
					// Find attached contact sensor
					sensors::Sensor_V sensors = mgr->GetSensors();
					for (const auto &sensor:sensors) {
						sensors::ContactSensorPtr contact_sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
						VoxelGridPlugin::update_sensor = contact_sensor->ConnectUpdated(std::bind(&VoxelGridPlugin::OnSensorUpdate, this));
						spdlog::get("file_logger")->info("[{}]: {} {}.", __FUNCTION__, "Registering callback for", contact_sensor->Name());
					}
					// Save filtered obstacles to list
					VoxelGridPlugin::world_obstacle_set.insert(model_name);
				}
			}
			// Compare local set and world set, both ways
			std::set<std::string> diff_world_local, diff_local_world;
			// Items in world, not in local
			utils.CompareSets(VoxelGridPlugin::world_obstacle_set, VoxelGridPlugin::local_obstacle_set, diff_world_local);
			// Items in local, not in world
			utils.CompareSets(VoxelGridPlugin::local_obstacle_set, VoxelGridPlugin::world_obstacle_set, diff_local_world);
			// Check whether to add from local list or remove
			// Synchronizes both world and local obstacle lists
			if (diff_world_local.size() > 0) {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Adding to local obstacle set");
				utils.UpdateList(VoxelGridPlugin::local_obstacle_set, diff_world_local, true);
			} else if (diff_local_world.size() > 0) {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Removing from local obstacle set");
				utils.UpdateList(VoxelGridPlugin::local_obstacle_set, diff_local_world, false);
			}
			assert(VoxelGridPlugin::local_obstacle_set.size() == VoxelGridPlugin::world_obstacle_set.size());
			spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Synchronized local and world obstacles");

			// Compare local list of obstacles and list of obstacles in voxel grid
			std::vector<std::string> diff_list_state, diff_state_list;
			// Items in state, not in list
			utils.CompareSets(VoxelGridPlugin::obstacle_state, VoxelGridPlugin::local_obstacle_set, diff_state_list);
			// Items in list, not in state
			utils.CompareSets(VoxelGridPlugin::local_obstacle_set, VoxelGridPlugin::obstacle_state, diff_list_state);
			// Check whether to add to grid or remove
			// Synchronizes both local list of obstacles and voxel grid
			if (diff_state_list.size() > 0) {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Removing from state vector");
				// Make sure to clear voxel grid before deleting object
				for (const std::string &to_delete:diff_state_list) {
					std::shared_ptr<Box> box_ptr = utils.GetBoxStateByName(to_delete, VoxelGridPlugin::obstacle_state);
					std::vector<Eigen::Vector3i> nodes_hit = box_ptr->GetNodesHit();
					for (auto &hit:nodes_hit) {
						VoxelGridPlugin::grid_ptr->SetVoxelState(hit, VoxelGridPlugin::grid_ptr->CELL_STATE::FREE);
					}
				}
				// Now delete the obstacle
				utils.UpdateList(VoxelGridPlugin::obstacle_state, diff_state_list, false);
			} else if (diff_list_state.size() > 0) {
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Adding to state vector");
				utils.UpdateList(VoxelGridPlugin::obstacle_state, diff_list_state, true);

				// Set canonical positions for left and right faces
				for (const auto &box_ptr:VoxelGridPlugin::obstacle_state) {
					// Get bounding box for obstacle
					math::Box cbox = VoxelGridPlugin::world->GetModel(box_ptr->GetName())->GetCollisionBoundingBox();
					// Right and left face placeholder
					std::array<Eigen::Vector4f, 4> left_face, right_face;
					// Get canonical positions
					utils.GetCanonicalPositionsNoAxisMapping(box_ptr->GetName(), cbox, left_face, right_face);
					// Set all the data
					box_ptr->SetCanonical("left", left_face);
					box_ptr->SetCanonical("right", right_face);
				}
				spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Updated canonical poses");
			}
			spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Synchronized local obstacles and voxel grid objects");
		}
	}

	void VoxelGridPlugin::UpdateObstacleMap(const bool &force) {
		if (force || VoxelGridPlugin::world->GetIterations() % VoxelGridPlugin::iteration_skip == 0) {
			spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Updating obstacle map");
			for (const auto &box_ptr:VoxelGridPlugin::obstacle_state) {
				spdlog::get("file_logger")->info("[{}]: Processing link {}.", __FUNCTION__, box_ptr->GetName());

				// Get model and pose from Gazebo
				physics::ModelPtr model = VoxelGridPlugin::world->GetModel(box_ptr->GetName());
				physics::Link_V links = model->GetLinks();
				physics::LinkPtr link = links[0];
				math::Pose obstacle_pose = link->GetWorldPose();
				math::Box obstacle_box = link->GetCollisionBoundingBox();

				// ================ Transform canonical positions to world positions =======================//
				std::array<Eigen::Vector4f, 4> transformed_left_corners, transformed_right_corners;
				utils.TransformCanonicalToWorld(obstacle_pose.rot, obstacle_box.GetCenter(),
												box_ptr->GetCanonical("left"), box_ptr->GetCanonical("right"),
												transformed_left_corners, transformed_right_corners);

				// ================ Raycast from corners of left face to right face ========================//
				//  Clear voxel grid
				for (auto &hit:box_ptr->GetNodesHit()) {
					VoxelGridPlugin::grid_ptr->SetVoxelState(hit, VoxelGridPlugin::grid_ptr->CELL_STATE::FREE);
				}
				// Raycast from corners of left face to corresponding corner in right face
				std::vector<Eigen::Vector3i> hit_nodes;
				Eigen::Vector3f left_point, right_point;
				assert(transformed_left_corners.size() == transformed_right_corners.size());
				for (int i = 0; i < transformed_left_corners.size(); ++i) {
					// Get translation component from transformation matrix
					left_point = transformed_left_corners[i].head(3);
					right_point = transformed_right_corners[i].head(3);
					try {
						// Convert points from world coordinates to grid coordinates
						Eigen::Vector3i start_point = VoxelGridPlugin::grid_ptr->WorldToGrid(left_point);
						Eigen::Vector3i end_point = VoxelGridPlugin::grid_ptr->WorldToGrid(right_point);
						// Does raycasting
						VoxelGridPlugin::grid_ptr->GetHitNodes(start_point, end_point, hit_nodes);
					} catch (std::string &e) {
						spdlog::get("file_logger")->error("[{}]: {}.", __FUNCTION__, e);
					}
					// Set new raycast hits to occupied
					for (auto &hit:hit_nodes) {
						VoxelGridPlugin::grid_ptr->SetVoxelState(hit, VoxelGridPlugin::grid_ptr->CELL_STATE::OCCUPIED);
					}
					// Update nodes hit
					box_ptr->SetNodesHit(hit_nodes);
				}
			}
		}
	}

	void VoxelGridPlugin::UpdateRobotMap(const bool &force) {
		if (force || VoxelGridPlugin::world->GetIterations() % VoxelGridPlugin::iteration_skip == 0) {
			spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Updating robot map");
			Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

			// Loop through all link states
			for (const auto &box_ptr:VoxelGridPlugin::robot_state) {
				spdlog::get("file_logger")->info("[{}]: Processing link {}.", __FUNCTION__, box_ptr->GetName());

				// Get link pose from Gazebo
				physics::LinkPtr link = VoxelGridPlugin::robot_model->GetLink(box_ptr->GetName());
				math::Pose link_pose = link->GetWorldPose();
				math::Box link_box = link->GetCollisionBoundingBox();

				// ================ Transform canonical positions to world positions =======================//
				std::array<Eigen::Vector4f, 4> transformed_left_corners, transformed_right_corners;
				utils.TransformCanonicalToWorld(link_pose.rot, link_box.GetCenter(),
												box_ptr->GetCanonical("left"), box_ptr->GetCanonical("right"),
												transformed_left_corners, transformed_right_corners);

				// ================ Raycast from corners of left face to right face ========================//
				//  Clear voxel grid
				for (auto &hit:box_ptr->GetNodesHit()) {
					VoxelGridPlugin::grid_ptr->SetVoxelState(hit, VoxelGridPlugin::grid_ptr->CELL_STATE::FREE);
				}
				// Raycast from corners of left face to corresponding corner in right face
				std::vector<Eigen::Vector3i> hit_nodes;
				Eigen::Vector3f left_point, right_point;
				assert(transformed_left_corners.size() == transformed_right_corners.size());
				for (int i = 0; i < transformed_left_corners.size(); ++i) {
					// Get translation component from transformation matrix
					left_point = transformed_left_corners[i].head(3);
					right_point = transformed_right_corners[i].head(3);
					try {
						// Convert points from world coordinates to grid coordinates
						Eigen::Vector3i start_point = VoxelGridPlugin::grid_ptr->WorldToGrid(left_point);
						Eigen::Vector3i end_point = VoxelGridPlugin::grid_ptr->WorldToGrid(right_point);
						// Does raycasting
						VoxelGridPlugin::grid_ptr->GetHitNodes(start_point, end_point, hit_nodes);
					} catch (std::string &e) {
						spdlog::get("file_logger")->error("[{}]: {}.", __FUNCTION__, e);
					}
					// Set new raycast hits to occupied
					for (auto &hit:hit_nodes) {
						VoxelGridPlugin::grid_ptr->SetVoxelState(hit, VoxelGridPlugin::grid_ptr->CELL_STATE::ROBOT);
					}
					// Update nodes hit
					box_ptr->SetNodesHit(hit_nodes);
				}
			}

			spdlog::get("file_logger")->info("[{}]: \n", __FUNCTION__);
		}
	}

	void VoxelGridPlugin::OnSensorUpdate() {
		// Update collision data
		sensors::Sensor_V sensors = VoxelGridPlugin::mgr->GetSensors();
		for (const auto &sensor:sensors) {
			sensors::ContactSensorPtr contact_sensor = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
			for (int i = 0; i < contact_sensor->GetCollisionCount(); i++) {
				spdlog::get("file_logger")->info("[{}]: Fetching contact data for {}.", __FUNCTION__, contact_sensor->Name());
				std::string collision_name = contact_sensor->GetCollisionName(i);
				std::map<std::string, physics::Contact> contacts = contact_sensor->Contacts(collision_name);
				if (contacts.size() == 0) {
					spdlog::get("file_logger")->info("[{}]: No contacts detected", __FUNCTION__);
				} else {
					for (const auto &contact:contacts) {
						physics::Collision *first_body = contact.second.collision1;
						physics::Collision *second_body = contact.second.collision2;
						spdlog::get("file_logger")->error("[{}]: Detected contact between {} and {}.", __FUNCTION__, first_body->GetName(), second_body->GetName());
					}
				}
			}
		}
	}

	void VoxelGridPlugin::OnUpdate() {
		int iters = VoxelGridPlugin::world->GetIterations();
		if (iters % 5000 == 0) {
			spdlog::get("file_logger")->info("[{}]: {} = {}.", __FUNCTION__, "Simulator iterations", std::to_string(iters));
		}

		// Check if robot has been loaded
		if (!VoxelGridPlugin::robot_loaded) {
			VoxelGridPlugin::IsRobotLoaded();
		}

		// If robot is loaded, initialize everything
		if (iters > VoxelGridPlugin::iteration_skip && VoxelGridPlugin::robot_loaded) {
			std::call_once(VoxelGridPlugin::world_ready_flag, std::bind(&VoxelGridPlugin::OnWorldReady, this));
		}

		// Main update logic goes here
		if (VoxelGridPlugin::robot_loaded && VoxelGridPlugin::world_ready) {
			// Check for and update obstacles
			VoxelGridPlugin::InitializeObstacleState();
			// Update map with new robot state
			VoxelGridPlugin::UpdateRobotMap();
			// Update map with new obstacle states
			VoxelGridPlugin::UpdateObstacleMap();
			// Convert voxel grid to cloud
			VoxelGridPlugin::grid_visualizer.VoxelGridToPointCloud(*VoxelGridPlugin::grid_ptr);
			VoxelGridPlugin::grid_visualizer.ShowGrid();
		}
	}

	GZ_REGISTER_WORLD_PLUGIN(VoxelGridPlugin);
};
