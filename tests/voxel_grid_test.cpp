//
// Created by Kanishka Ganguly on 7/9/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#include "../include/voxel_grid.hpp"

/** @example "tests/voxel_grid_test.cpp"
 * This is an example of how to use the VoxelGrid and VoxelGridVisualizer classes.
 * Creates a new VoxelGrid, demonstrates ray-tracing functionality to populate it.
 * Visualizes populated VoxelGrid through VoxelGridVisualizer.
 */

int main(int argc, char *argv[]) {
	/**
	 * Test voxel grid
	 */
	VoxelGrid grid = VoxelGrid(127, 3, Eigen::Vector3f(0, 0, 0));
	VoxelGridVisualizer grid_visualizer = VoxelGridVisualizer();
	Eigen::Vector3i start, end;
	std::vector<Eigen::Vector3i> nodes_hit;
/* --------------------------------------------------------------------*/
	// Get start and end points in grid
	nodes_hit.clear();
	try {
		start = Eigen::Vector3i(grid.WorldToGrid(0.837756, 0.517617, 1.34872));
		end = Eigen::Vector3i(grid.WorldToGrid(1.06764, 0.517579, 0.855588));
	} catch (std::string e) {
		std::cout << e << std::endl;
	}
	// Get hit nodes
	grid.GetHitNodes(start, end, nodes_hit);
	for (auto &node:nodes_hit) {
		grid.SetVoxelState(node, grid.CELL_STATE::ROBOT);
	}
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Robot nodes: " + std::to_string(nodes_hit.size()));
/* --------------------------------------------------------------------*/
	// Get start and end points in grid
	nodes_hit.clear();
	try {
		start = Eigen::Vector3i(grid.WorldToGrid(2.4, 1.2, 2.0));
		end = Eigen::Vector3i(grid.WorldToGrid(2.3, 3.0, 1.0));
	} catch (std::string e) {
		std::cout << e << std::endl;
	}
	// Get hit nodes
	grid.GetHitNodes(start, end, nodes_hit);
	for (auto &node:nodes_hit) {
		grid.SetVoxelState(node, grid.CELL_STATE::OCCUPIED);
	}
	spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Occupied nodes: " + std::to_string(nodes_hit.size()));
/* --------------------------------------------------------------------*/
	while (true) {
		grid_visualizer.VoxelGridToPointCloud(grid);
		grid_visualizer.ShowGrid();
	}
	return 0;
}

