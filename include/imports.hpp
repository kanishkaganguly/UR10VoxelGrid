//
// Created by Kanishka Ganguly on 6/25/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//

#pragma once

/** C++ Imports */
#include <algorithm>
#include <chrono>
#include <ctime>
#include <cmath>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <locale>
#include <math.h>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <tinyxml2.h>
#include <type_traits>
#include <vector>
#include "../include/logger.hpp"
#include "../include/prettyprint.hpp"

/** Boost imports */
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

/** Gazebo Imports */
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sdf/sdf.hh>

/** Eigen Imports */
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>

/** ROS imports */
#ifdef USE_ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <controller_manager_msgs/ControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <voxel_grid_plugin/ModelControlAction.h>
#include <voxel_grid_plugin/RobotControlAction.h>
#include <voxel_grid_plugin/SimControlAction.h>
#include <voxel_grid_plugin/VoxelGridAction.h>
#include <voxel_grid_plugin/VoxelData.h>
#endif

/** Visualization */
#ifdef SHOW_VIZ
#include <cilantro/common_renderables.hpp>
#include <cilantro/point_cloud.hpp>
#include <cilantro/visualizer.hpp>
#endif


typedef Eigen::TensorFixedSize<int, Eigen::Sizes<127, 127, 127>> Tensor;    /**< @typedef Eigen::TensorFixedSizefor grid management */
typedef Eigen::Vector3f Color;                                              /**< @typedef Eigen::Vector3f for managing cilantro::PointCloud colors */
typedef Eigen::Vector3f v3f;                                                /**< @typedef Eigen::Vector3f shorthand */
typedef Eigen::Vector3i v3i;                                                /**< @typedef Eigen::Vector3i shorthand */
typedef std::vector<std::tuple<std::string, int, Color>> CustomCellState;   /**< @typedef Vector of data for voxel grid with multiple items */

/**
 * @namespace math_helper
 * @brief Small helper functions for math
 */
namespace math_helper {

/**
 * @fn inline static double DEG2RAD(const T &DEG)
 * @brief Convert degrees to radians
 * @tparam T
 * @param DEG - Degrees
 * @return Radians
 */
	template<typename T>
	inline static double DEG2RAD(const T &DEG) {
		return DEG * 0.017453293;
	}

/**
 * @fn inline static double RAD2DEG(const T &RAD)
 * @brief Convert radians to degrees
 * @tparam T
 * @param RAD - Radians
 * @return Degrees
 */
	template<typename T>
	inline static double RAD2DEG(const T &RAD) {
		return RAD * 57.29577951;
	}

	namespace gazebo_eigen_conversions {
		/**
		* @brief Converts gazebo::math::Matrix3 to Eigen::Matrix3f
		* @param input_mat - Gazebo Matrix3 to be converted
		* @return output_mat - Eigen Matrix3f after conversion
		*/
		inline static Eigen::Matrix3f GazeboMat3ToEigenMat3(const gazebo::math::Matrix3 &input_mat) {
			Eigen::Matrix3f output_mat;
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					output_mat(i, j) = input_mat[i][j];
				}
			}
			return output_mat;
		}

		/**
		 * @brief Converts Eigen::Matrix3f to gazebo::math::Matrix3
		 * @param input_mat - Eigen Matrix3f to be converted
		 * @return output_mat - Gazebo Matrix3 after conversion
		 */
		inline static gazebo::math::Matrix3 EigenMat3ToGazeboMat3(const Eigen::Matrix3f &input_mat) {
			gazebo::math::Matrix3 output_mat;
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					output_mat[i][j] = input_mat(i, j);
				}
			}
			return output_mat;
		}

		/**
		 * @brief Converts gazebo::math::Vector3 to Eigen::Vector3f
		 * @param input_vec - Gazebo Vector3 to be converted
		 * @return output_vec - Eigen Vector3f after conversion
		 */
		inline static Eigen::Vector3f GazeboVec3ToEigenVec3(const gazebo::math::Vector3 &input_vec) {
			Eigen::Vector3f output_vec;
			for (int i = 0; i < 3; ++i) {
				output_vec(i) = input_vec[i];
			}
			return output_vec;
		}

		/**
		 * @brief Converts Eigen::Vector3f to gazebo::math::Vector3
		 * @param input_vec - Eigen Vector3f to be converted
		 * @return output_vec - Gazebo Vector3 after conversion
		 */
		inline static gazebo::math::Vector3 EigenVec3ToGazeboVec3(const Eigen::Vector3f &input_vec) {
			gazebo::math::Vector3 output_vec;
			output_vec.x = input_vec(0);
			output_vec.y = input_vec(1);
			output_vec.z = input_vec(2);
			return output_vec;
		}

		/**
		 * @brief Converts Eigen::Quaternionf to gazebo::math::Quaternion
		 * @param input_quat - Eigen Quaternion to be converted
		 * @return output_quat - Gazebo Quaternion after conversion
		 */
		inline static gazebo::math::Quaternion EigenQuaternionToGazeboQuaternion(const Eigen::Quaternionf &input_quat) {
			gazebo::math::Quaternion output_quat;
			output_quat.Set(input_quat.w(), input_quat.x(), input_quat.y(), input_quat.z());
			return output_quat;
		}

		/**
		 * @brief Converts gazebo::math::Quaternion to Eigen::Quaternionf
		 * @param input_quat - Gazebo Quaternion to be converted
		 * @return output_quat - Eigen Quaternion after conversion
		 */
		inline static Eigen::Quaternionf GazeboQuaternionToEigenQuaternion(const gazebo::math::Quaternion &input_quat) {
			Eigen::Quaternionf output_quat;
			output_quat.w() = input_quat.w;
			output_quat.x() = input_quat.x;
			output_quat.y() = input_quat.y;
			output_quat.z() = input_quat.z;
			return output_quat;
		}
	}

	namespace eigen_conversions {
		/**
		 * @brief Convert roll, pitch, yaw to Quaternion
		 * @param r - Roll
		 * @param p - Pitch
		 * @param y - Yaw
		 * @return q - Quaternion
		 */
		inline static Eigen::Quaternionf EulerXYZToEigenQuaternion(const float &r, const float &p, const float &y) {
			Eigen::Quaternionf q;
			q = Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ());
			return q;
		}

		/**
		 * @brief Convert Quaternion to roll, pitch, yaw
		 * @param q - Quaternion
		 * @return - Vector3 of roll, pitch, yaw
		 */
		inline static Eigen::Vector3f EigenQuaternionToEulerXYZ(const Eigen::Quaternionf &q) {
			return q.toRotationMatrix().eulerAngles(0, 1, 2);
		};

		/**
		 * @brief Get 3x3 rotation and 3x1 translation components from 4x4 transformation matrix
		 * @param matrix4 Transformation matrix to decompose
		 * @param rot 3x3 rotation matrix
		 * @param pos 3x1 position vector
		 */
		inline void GetRotationTranslationEigen(const Eigen::Matrix4f &matrix4, Eigen::Matrix3f &rot, Eigen::Vector3f &pos) {
			// Rightmost 3x1 column of 4x4 matrix is translation
			pos = matrix4.block<3, 1>(0, 3);
			// Top left 3x3 matrix of 4x4 matrix is rotation
			rot = matrix4.block<3, 3>(0, 0);
		}

		/**
		 * @brief Get 3x1 translation components from 4x4 transformation matrix
		 * @param matrix4 Transformation matrix to decompose
		 * @param pos 3x1 position vector
		 */
		inline void GetTranslationEigen(const Eigen::Matrix4f &matrix4, Eigen::Vector3f &pos) {
			// Rightmost 3x1 column of 4x4 matrix is translation
			pos = matrix4.block<3, 1>(0, 3);
		}

		/**
		 * @brief Get 3x3 rotation components from 4x4 transformation matrix
		 * @param matrix4 Transformation matrix to decompose
		 * @param rot 3x3 rotation matrix
		 */
		inline void GetRotationEigen(const Eigen::Matrix4f &matrix4, Eigen::Matrix3f &rot) {
			// Top left 3x3 matrix of 4x4 matrix is rotation
			rot = matrix4.block<3, 3>(0, 0);
		}

		/**
		 * @brief Converts Eigen::Vector3f and Eigen::Matrix3f to Eigen::Matrix4f
		 * @param pos - Translation Vector3f to convert
		 * @param rot - Matrix3f to convert
		 * @param matrix4 - Converted Eigen::Matrix4f output
		 */
		inline void PoseToMatrix4(const Eigen::Vector3f &pos, const Eigen::Matrix3f &rot, Eigen::Matrix4f &matrix4) {
			matrix4.topLeftCorner(3, 3) = rot;
			matrix4.topRightCorner(3, 1) = pos;
			matrix4(3, 3) = 1;
		}
	}

	namespace gazebo_conversions {
		/**
		 * @brief Convert roll, pitch, yaw to Quaternion
		 * @param r - Roll
		 * @param p - Pitch
		 * @param y - Yaw
		 * @return q - Quaternion
		 */
		inline static gazebo::math::Quaternion EulerXYZToGazeboQuaternion(const double &r, const double &p, const double &y) {
			gazebo::math::Quaternion q;
			q.SetFromEuler(r, p, y);
			return q;
		}

		/**
		 * @brief Convert Quaternion to roll, pitch, yaw
		 * @return rpy - roll, pitch, yaw
		 */
		inline static gazebo::math::Vector3 GazeboQuaternionToEulerXYZ(const gazebo::math::Quaternion &q) {
			gazebo::math::Vector3 EulerZYX = q.GetAsEuler();
			gazebo::math::Vector3 EulerXYZ;
			EulerXYZ.Set(EulerZYX.z, EulerZYX.y, EulerZYX.x);
			return EulerXYZ;
		}

		/**
		 * @brief Converts math::Pose to math::Matrix4
		 * @param pose - Pose to convert
		 * @param matrix4 - Converted math::Matrix4 output
		 */
		inline void PoseToMatrix4(const gazebo::math::Pose &pose, gazebo::math::Matrix4 &matrix4) {
			/** Initialize to ZERO */
			matrix4 = gazebo::math::Matrix4::ZERO;
			/** Set last element to 1 for valid transformation matrix */
			matrix4[3][3] = 1;
			/** Set rotation data in transformation matrix */
			gazebo::math::Matrix3 rot_as_matrix3 = pose.rot.GetAsMatrix3();
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					matrix4[i][j] = rot_as_matrix3[i][j];
				}
			}
			/** Set translation data in transformation matrix */
			for (int j = 0; j < 3; ++j) {
				matrix4[j][3] = pose.pos[j];
			}
		}

		/**
		* @brief Converts math::Vector3 and math::Quaternion to math::Matrix4
		* @param pos - Translation vector to convert
		* @param rot - Quaternion to convert
		* @param matrix4 - Converted math::Matrix4 output
		*/
		inline void PoseToMatrix4(const gazebo::math::Vector3 &pos, const gazebo::math::Quaternion &rot, gazebo::math::Matrix4 &matrix4) {
			/** Initialize to ZERO */
			matrix4 = gazebo::math::Matrix4::ZERO;
			/** Set last element to 1 for valid transformation matrix */
			matrix4[3][3] = 1;
			/** Set rotation data in transformation matrix */
			gazebo::math::Matrix3 rot_as_matrix3 = rot.GetAsMatrix3();
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					matrix4[i][j] = rot_as_matrix3[i][j];
				}
			}
			/** Set translation data in transformation matrix */
			for (int j = 0; j < 3; ++j) {
				matrix4[j][3] = pos[j];
			}
		}

		/**
		 * @brief Converts math::Vector3 and math::Matrix3 to math::Matrix4
		 * @param pos - Translation vector to convert
		 * @param rot - Matrix3 to convert
		 * @param matrix4 - Converted math::Matrix4 output
		 */
		inline void PoseToMatrix4(const gazebo::math::Vector3 &pos, const gazebo::math::Matrix3 &rot, gazebo::math::Matrix4 &matrix4) {
			/** Initialize to ZERO */
			matrix4 = gazebo::math::Matrix4::ZERO;
			/** Set last element to 1 for valid transformation matrix */
			matrix4[3][3] = 1;
			/** Set rotation data in transformation matrix */
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					matrix4[i][j] = rot[i][j];
				}
			}
			/** Set translation data in transformation matrix */
			for (int j = 0; j < 3; ++j) {
				matrix4[j][3] = pos[j];
			}
		}

	}

/**
* @fn inline void UpdateElement(const std::string &elem, const double &value, math::Vector3 &vec)
* @brief Updates value of math::Vector3 at index given by axis specifier "x", "y" or "z"
* @param elem - Axis to modify
* @param value - Value to set
* @param vec - Vector that will be updated
*/
	inline void UpdateElement(const std::string &elem, const double &value, gazebo::math::Vector3 &vec) {
		if (elem == "x") {
			vec.x = value;
		} else if (elem == "y") {
			vec.y = value;
		} else if (elem == "z") {
			vec.z = value;
		}
	}

/**
 * @overload
 */
	inline void UpdateElement(const int &idx, const double &value, gazebo::math::Vector3 &vec) {
		assert(idx >= 0 && idx <= 3);
		switch (idx) {
			case 0:
				vec.x = value;
				break;
			case 1:
				vec.y = value;
				break;
			case 2:
				vec.z = value;
				break;
		}
	}

}
