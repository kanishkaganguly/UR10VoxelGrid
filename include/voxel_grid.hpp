
//
// Created by Kanishka Ganguly on 6/19/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#pragma once

/** C++ */
#include "../include/imports.hpp"

/**
 *  @class VoxelGrid voxel_grid.hpp "include/voxel_grid.hpp"
 *  @brief Utility class for creating and managing voxel grids.
 *  Sets states of grid, and can raycast between voxels.
 */
class VoxelGrid {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int NUM_VOXELS_PER_DIM;         /**< @var Number of voxels along each dimension */
	float dim_length_;              /**< @var Length of each dimension in meters */
	float voxel_size_;              /**< @var Size of each voxel in meters */
	Eigen::Vector3f grid_start_;    /**< @var Starting coordinate of the grid in meters */
	Tensor grid_;                   /**< @var Eigen FixedTensor for grid */

	/**
	 * @enum CELL_STATE
	 * @brief Enum to hold state of each voxel
	 */
	enum CELL_STATE {
		FREE = 0,                   /**< @var Free voxel */
		OCCUPIED = 1,               /**< @var Voxel occupied by obstacle */
		ROBOT = 2                   /**< @var Voxel occupied by robot */
	};

	/**
	 * @brief Constructor for VoxelGrid. Sets all voxels in grid to "free" initially.
	 * @param voxels_per_side Set number of voxels along each dimension
	 * @param dim_length Set length of each dimension in meters
	 * @param grid_start Set starting coordinate of grid in meters
	 */
	VoxelGrid(int voxels_per_side, float dim_length, Eigen::Vector3f grid_start) :
			NUM_VOXELS_PER_DIM(voxels_per_side),
			dim_length_(dim_length),
			voxel_size_(dim_length / NUM_VOXELS_PER_DIM),
			grid_start_(grid_start) {
		grid_.setConstant(CELL_STATE::FREE);
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Created voxel grid");
	};

	/**
	 * @brief Get current voxel grid
	 * @return Voxel grid
	 */
	const Tensor &GetGrid() const { return grid_; }

	/**
	 * @brief Get size of each voxel
	 * @return Voxel size
	 */
	float GetVoxelSize() { return voxel_size_; }

	/**
	 * @brief Get dimension of voxel grid
	 * @return Dimension of grid
	 */
	float GetGridDimension() { return dim_length_; }

	/**
	 * @fn Eigen::Vector3i WorldToGrid(const Eigen::Vector3f &world_coords)
	 * @brief Convert world coordinates to 3D grid index
	 * @param world_coords - 3D coordinates in meters
	 * @return grid_idx - 3D index on grid
	 */
	inline Eigen::Vector3i WorldToGrid(const Eigen::Vector3f &world_coords) {
		Eigen::Vector3i grid_idx;
		grid_idx(0) = floor((world_coords(0) - grid_start_(0)) / voxel_size_);
		grid_idx(1) = floor((world_coords(1) - grid_start_(1)) / voxel_size_);
		grid_idx(2) = floor((world_coords(2) - grid_start_(2)) / voxel_size_);
		// Throw exception if world coordinate out of grid bounds
		if ((grid_idx.array() > NUM_VOXELS_PER_DIM).any()) {
			throw std::string("Grid index out of bounds.");
		} else {
			return grid_idx;
		}
	}

	/**
	 * @overload Eigen::Vector3i WorldToGrid(const gazebo::math::Vector3 &world_coords)
	 */
	inline Eigen::Vector3i WorldToGrid(const gazebo::math::Vector3 &world_coords) {
		Eigen::Vector3i grid_idx;
		grid_idx(0) = floor((world_coords.x - grid_start_(0)) / voxel_size_);
		grid_idx(1) = floor((world_coords.y - grid_start_(1)) / voxel_size_);
		grid_idx(2) = floor((world_coords.z - grid_start_(2)) / voxel_size_);
		// Throw exception if world coordinate out of grid bounds
		if ((grid_idx.array() > NUM_VOXELS_PER_DIM).any()) {
			throw std::string("Grid index out of bounds.");
		} else {
			return grid_idx;
		}
	}

	/**
	 * @overload Eigen::Vector3i WorldToGrid(const float &world_coord_x, const float &world_coord_y, const float &world_coord_z)
	 */
	inline Eigen::Vector3i
	WorldToGrid(const float &world_coord_x, const float &world_coord_y, const float &world_coord_z) {
		Eigen::Vector3i grid_idx;
		grid_idx(0) = floor((world_coord_x - grid_start_(0)) / voxel_size_);
		grid_idx(1) = floor((world_coord_y - grid_start_(1)) / voxel_size_);
		grid_idx(2) = floor((world_coord_z - grid_start_(2)) / voxel_size_);
		// Throw exception if world coordinate out of grid bounds
		if ((grid_idx.array() > NUM_VOXELS_PER_DIM).any()) {
			throw std::string("Grid index out of bounds.");
		} else {
			return grid_idx;
		}
	}

	/**
	 * @fn Eigen::Vector3f GridToWorld(const Eigen::Vector3i &grid_coords)
	 * @brief Convert grid index to 3D world coordinates
	 * @param grid_coords - 3D coordinates as grid indices
	 * @return world_coords - 3D coordinates of center of voxel
	 */
	inline Eigen::Vector3f GridToWorld(const Eigen::Vector3i &grid_coords) {
		Eigen::Vector3f world_coords;
		world_coords(0) = (grid_coords(0) * voxel_size_) + grid_start_(0) + (voxel_size_ / 2.0f);
		world_coords(1) = (grid_coords(1) * voxel_size_) + grid_start_(1) + (voxel_size_ / 2.0f);
		world_coords(2) = (grid_coords(2) * voxel_size_) + grid_start_(2) + (voxel_size_ / 2.0f);
		// Throw exception if world coordinate out of grid bounds
		if ((world_coords.array() > dim_length_).any()) {
			throw std::string("World coordinates out of bounds.");
		} else {
			return world_coords;
		}
	}

	/**
	 * @overload VoxelGrid::GridToWorld(const float &grid_idx_x, const float &grid_idx_y, const float &grid_idx_z)
	 */
	inline Eigen::Vector3f GridToWorld(const float &grid_idx_x, const float &grid_idx_y, const float &grid_idx_z) {
		Eigen::Vector3f world_coords;
		world_coords(0) = (grid_idx_x * voxel_size_) + grid_start_(0) + (voxel_size_ / 2.0f);
		world_coords(1) = (grid_idx_y * voxel_size_) + grid_start_(1) + (voxel_size_ / 2.0f);
		world_coords(2) = (grid_idx_z * voxel_size_) + grid_start_(2) + (voxel_size_ / 2.0f);
		// Throw exception if world coordinate out of grid bounds
		if ((world_coords.array() > dim_length_).any()) {
			throw std::string("World coordinates out of bounds.");
		} else {
			return world_coords;
		}
	}

	/**
	 * @fn inline void SetVoxelState(const Eigen::Vector3i &voxel_coords, CELL_STATE cell_state)
	 * @brief - Set a voxel in grid to CELL_STATE
	 */
	inline void SetVoxelState(const Eigen::Vector3i &voxel_coords, CELL_STATE cell_state) {
		grid_(voxel_coords(0), voxel_coords(1), voxel_coords(2)) = cell_state;
	}

	/**
	 * @overload inline void SetVoxelState(const int &voxel_coord_x, const int &voxel_coord_y, const int &voxel_coord_z, CELL_STATE cell_state)
	 */
	inline void
	SetVoxelState(const int &voxel_coord_x, const int &voxel_coord_y, const int &voxel_coord_z, CELL_STATE cell_state) {
		grid_(voxel_coord_x, voxel_coord_y, voxel_coord_z) = cell_state;
	}

	/**
	 * @fn inline void SetVoxelState(const Eigen::Vector3i &voxel_coords, CELL_STATE cell_state)
	 * @brief - Set a voxel in grid to user-defined value
	 */
	inline void SetVoxelState(const Eigen::Vector3i &voxel_coords, int cell_state) {
		grid_(voxel_coords(0), voxel_coords(1), voxel_coords(2)) = cell_state;
	}

	/**
	 * @overload inline void SetVoxelState(const int &voxel_coord_x, const int &voxel_coord_y, const int &voxel_coord_z, CELL_STATE cell_state)
	 */
	inline void
	SetVoxelState(const int &voxel_coord_x, const int &voxel_coord_y, const int &voxel_coord_z, int cell_state) {
		grid_(voxel_coord_x, voxel_coord_y, voxel_coord_z) = cell_state;
	}

	/**
	 * @brief Modified Bresenham's 3D line drawing algorithm for integer grids
	 * @param start_point - Starting vector for line (in grid indices)
	 * @param end_point - Ending vector for line (in grid indices)
	 * @param hit_nodes - Returns grid indices of all voxels hit
	 * @see - https://stackoverflow.com/questions/55263298/draw-all-voxels-that-pass-through-a-3d-line-in-3d-voxel-space
	 * @see - http://www.cs.yorku.ca/~amana/research/grid.pdf
	 */
	inline void GetHitNodes(const Eigen::Vector3i &start_point, const Eigen::Vector3i &end_point,
							std::vector<Eigen::Vector3i> &hit_nodes) {
		spdlog::get("file_logger")->info("[{}]: {}.", __FUNCTION__, "Finding hit voxels");
		// Include start point
		hit_nodes.push_back(start_point);
		// Set start and end points
		int x1 = start_point(0);
		int y1 = start_point(1);
		int z1 = start_point(2);
		int x2 = end_point(0);
		int y2 = end_point(1);
		int z2 = end_point(2);
		// Absolute differences
		int abs_dx = std::abs(x2 - x1);
		int abs_dy = std::abs(y2 - y1);
		int abs_dz = std::abs(z2 - z1);
		// Step directions
		int xs, ys, zs;
		x2 > x1 ? xs = 1 : xs = -1;
		y2 > y1 ? ys = 1 : ys = -1;
		z2 > z1 ? zs = 1 : zs = -1;

		double hypotenuse = sqrt(abs_dx * abs_dx + abs_dy * abs_dy + abs_dz * abs_dz);
		// We determine the value of t at which the ray crosses the first vertical voxel boundary and
		// store it in variable maxX. We perform a similar computation in y and store the result in maxY. The
		// minimum of these two values will indicate how much we can travel along the ray and still remain in the
		// current voxel.
		double maxX = hypotenuse * 0.5 / abs_dx;
		double maxY = hypotenuse * 0.5 / abs_dy;
		double maxZ = hypotenuse * 0.5 / abs_dz;
		// We compute deltaX and deltaY. deltaX indicates how far along the ray we must move (in units of t)
		// for the horizontal component of such a movement to equal the width of a voxel.
		// Similarly, we store in deltaY the amount of movement along the ray
		// which has a vertical component equal to the height of a voxel.
		double deltaX = hypotenuse / abs_dx;
		double deltaY = hypotenuse / abs_dy;
		double deltaZ = hypotenuse / abs_dz;

		while (x1 != x2 || y1 != y2 || z1 != z2) {
			if (maxX < maxY) {
				if (maxX < maxZ) {
					x1 = x1 + xs;
					maxX = maxX + deltaX;
				} else if (maxX > maxZ) {
					z1 = z1 + zs;
					maxZ = maxZ + deltaZ;
				} else {
					x1 = x1 + xs;
					maxX = maxX + deltaX;
					z1 = z1 + zs;
					maxZ = maxZ + deltaZ;
				}
			} else if (maxX > maxY) {
				if (maxY < maxZ) {
					y1 = y1 + ys;
					maxY = maxY + deltaY;
				} else if (maxY > maxZ) {
					z1 = z1 + zs;
					maxZ = maxZ + deltaZ;
				} else {
					y1 = y1 + ys;
					maxY = maxY + deltaY;
					z1 = z1 + zs;
					maxZ = maxZ + deltaZ;
				}
			} else {
				if (maxY < maxZ) {
					y1 = y1 + ys;
					maxY = maxY + deltaY;
					x1 = x1 + xs;
					maxX = maxX + deltaX;
				} else if (maxY > maxZ) {
					z1 = z1 + zs;
					maxZ = maxZ + deltaZ;
				} else {
					x1 = x1 + xs;
					maxX = maxX + deltaX;
					y1 = y1 + ys;
					maxY = maxY + deltaY;
					z1 = z1 + zs;
					maxZ = maxZ + deltaZ;
				}
			}
			hit_nodes.push_back(Eigen::Vector3i(x1, y1, z1));
		}
	}
};

/**
 *  @class VoxelGridVisualizer voxel_grid.hpp "include/voxel_grid.hpp"
 *  @brief Visualizer class for voxel grids.
 *  Converts voxel centers to point clouds. Uses cilantro::Visualizer to display output.
*/
class VoxelGridVisualizer {
#ifdef SHOW_VIZ
	/** Set occupied marker color */
	Color occupied_color;
	/** Set free marker color */
	Color free_color;
	/** Set robot marker color */
	Color robot_color;
private:
	/** Visualizer */
	cilantro::Visualizer viz;
public:
	/** Voxel grid visualizer constructor */
	VoxelGridVisualizer() :
			viz("Voxel Grid Visualizer", "grid_viz"),
			occupied_color(1.0, 0.0, 0.0),
			free_color(0.0, 1.0, 0.0),
			robot_color(0.0, 0.0, 1.0) {
	};

	/**
	 * @brief Used to convert voxel grid into cilantro::PointCloud for free, robot and occupied spaces.
	 * Should generate cilantro::PointCloudRenderable, directly into cilantro::Visualizer.
	 */
	void VoxelGridToPointCloud(VoxelGrid &voxgrid);

	/**
	 * @brief Used to convert voxel grid into cilantro::PointCloud.
	 * This function also contains a tuple of name, state and color, for user-defined cell states.
	 * Should generate cilantro::PointCloudRenderable, directly into cilantro::Visualizer.
	 */
	void VoxelGridToPointCloud(VoxelGrid &voxgrid, const CustomCellState &name_state_color_mapping);

	/** @brief Spins cilantro::Visualizer for one loop, render function */
	void ShowGrid();
#else
public:
	VoxelGridVisualizer() {
		spdlog::get("file_logger")->warn("[{}]: {}.", __FUNCTION__, "Visualization disabled. Compile with -DUSE_VIZ=ON");
	};

	inline void VoxelGridToPointCloud(VoxelGrid &voxgrid) { ; };

	inline void VoxelGridToPointCloud(VoxelGrid &voxgrid, CustomCellState name_state_color_mapping) { ; };

	inline void ShowGrid() { ; };
#endif
};
