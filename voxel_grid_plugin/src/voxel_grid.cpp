
//
// Created by Kanishka Ganguly on 6/19/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#include "../include/voxel_grid.hpp"

#ifdef SHOW_VIZ
void VoxelGridVisualizer::VoxelGridToPointCloud(VoxelGrid &voxgrid) {
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Converting VoxelGrid to MarkerArray");

	const Tensor &grid = voxgrid.GetGrid();
	const auto &dims = voxgrid.NUM_VOXELS_PER_DIM;
	float voxel_size = voxgrid.GetVoxelSize();

	std::vector<Eigen::Vector3f> free_points;
	std::vector<Eigen::Vector3f> occupied_points;
	std::vector<Eigen::Vector3f> robot_points;
	auto start = std::chrono::high_resolution_clock::now();
	for (int x = 0; x < dims; ++x) {
		for (int y = 0; y < dims; ++y) {
			for (int z = 0; z < dims; ++z) {
				if (grid(x, y, z) == voxgrid.CELL_STATE::FREE) {
					free_points.push_back(voxgrid.GridToWorld(x, y, z));
				} else if (grid(x, y, z) == voxgrid.CELL_STATE::ROBOT) {
					robot_points.push_back(voxgrid.GridToWorld(x, y, z));
				} else if (grid(x, y, z) == voxgrid.CELL_STATE::OCCUPIED) {
					occupied_points.push_back(voxgrid.GridToWorld(x, y, z));
				}
			}
		}
	}

	viz.addObject<cilantro::CoordinateFrameRenderable>("axis", Eigen::Matrix4f::Identity(), 0.4f, cilantro::RenderingProperties().setLineWidth(5.0f));
	viz.addObject<cilantro::PointCloudRenderable>("occupied", occupied_points, cilantro::RenderingProperties().setPointColor(occupied_color));
	viz.addObject<cilantro::PointCloudRenderable>("robot", robot_points, cilantro::RenderingProperties().setPointColor(robot_color));
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
//	spdlog::get("file_logger")->info("[{}]: {} {} {}.", __FUNCTION__, "Populating MarkerArray took", std::to_string((float) duration.count()), "ms");
}

void VoxelGridVisualizer::VoxelGridToPointCloud(VoxelGrid &voxgrid, const CustomCellState &name_state_color_mapping) {
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Converting VoxelGrid to MarkerArray");
	const Tensor &grid = voxgrid.GetGrid();
	const auto &dims = voxgrid.NUM_VOXELS_PER_DIM;
	float voxel_size = voxgrid.GetVoxelSize();
	std::vector<std::vector<Eigen::Vector3f>> points;
	std::vector<int> point_counter(name_state_color_mapping.size(), 0);
	points.resize(name_state_color_mapping.size());

	auto start = std::chrono::high_resolution_clock::now();
	for (int x = 0; x < dims; ++x) {
		for (int y = 0; y < dims; ++y) {
			for (int z = 0; z < dims; ++z) {
				try {
					points[grid(x, y, z)].push_back(voxgrid.GridToWorld(x, y, z));
					point_counter[grid(x, y, z)] += 1;
				} catch (std::string &e) {
					spdlog::get("file_logger")->error("[{}]: {}.", __FUNCTION__, e);
				}
			}
		}
	}

	viz.addObject<cilantro::CoordinateFrameRenderable>("axis", Eigen::Matrix4f::Identity(), 0.4f, cilantro::RenderingProperties().setLineWidth(5.0f));
	for (auto &mapping:name_state_color_mapping) {
		if(std::get<0>(mapping)=="free"){
		viz.addObject<cilantro::PointCloudRenderable>(std::get<0>(mapping), points[std::get<1>(mapping)],
													  cilantro::RenderingProperties().setPointColor(std::get<2>(mapping)).setOpacity(0.2));
		}else{
			viz.addObject<cilantro::PointCloudRenderable>(std::get<0>(mapping), points[std::get<1>(mapping)],
													  cilantro::RenderingProperties().setPointColor(std::get<2>(mapping)));
		}
	}
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
	spdlog::get("file_logger")->info("[{}]: {} {} {}.", __FUNCTION__, "Populating MarkerArray took", std::to_string((float) duration.count()), "ms");
}

void VoxelGridVisualizer::ShowGrid() {
	if (!viz.wasStopped()) {
		viz.spinOnce();
	}
}

#endif