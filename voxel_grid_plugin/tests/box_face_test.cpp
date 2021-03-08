//
// Created by Kanishka Ganguly on 7/24/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include "../include/catch.hpp"
#include "../include/voxel_grid.hpp"
#include "../include/robot_utils.hpp"

/** @example tests/box_face_test.cpp
 * This is an example executable testing computation of centers of opposing faces of a bounding box,
 * given the center of a box, its dimensions along three axes and the rotation.
 * Once the centers of opposing faces are found, a voxelized line is drawn to join them.
 */

/** @struct TestBox
 * @brief Simple struct to hold boxes for testing
 */
struct TestBox {
	TestBox(std::string name, int cell_state) :
			name(name), cell_state(cell_state) {}

	/** Name of the box, for debugging */
	std::string name;
	/** The position of the center of the box */
	Eigen::Vector3f pos;
	/** The rotation around the center of the box */
	Eigen::Quaternionf rot;
	/** The dimensions of the box in all three axes */
	std::vector<double> dims;
	/** The cell state to use for this box */
	int cell_state;

	/** Set the dimensions of the box using initializer_list */
	template<typename T>
	void DimsSet(std::initializer_list<T> dim_vals) {
		for (auto elem:dim_vals) {
			this->dims.push_back(elem);
		}
	}

	/** Get value of largest dimension */
	double LargestDim() {
		return *std::max_element(this->dims.begin(), this->dims.end());
	}

	/** Get index of largest dimension */
	int LargestDimIdx() {
		return std::distance(this->dims.begin(), std::max_element(this->dims.begin(), this->dims.end()));
	}
};

bool EqualityTestEigen(const Eigen::Matrix3f &m1, const Eigen::Matrix3f &m2) {
	return m1.isApprox(m2, 1e-3);
}

bool EqualityTestEigen(const Eigen::Matrix4f &m1, const Eigen::Matrix4f &m2) {
	return m1.isApprox(m2, 1e-2);
}

TEST_CASE("Draw line through centers of opposite faces of a box, given center of box and length of longest axis" "[BoxFaceTest]") {
	// Eigen formatting
	Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	std::string sep = "\n----------------------------------------\n\n";
	// Utility class
	RobotUtils utils;

	SECTION("[rotation: (45,0,0)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 1);
		box.rot = math_helper::eigen_conversions::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), 0, 0);
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 1.0000000, 0.0000000, 0.0000000,
				0.0000000, 0.7071069, -0.7071066,
				0.0000000, 0.7071066, 0.7071069;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 1, 0, 0, 0,
				0, 0.7071, -0.7071, 1,
				0, 0.7071, 0.7071, 1,
				0, 0, 0, 1;
		sol_right << 1, 0, 0, 2,
				0, 0.7071, -0.7071, 1,
				0, 0.7071, 0.7071, 1,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (0,45,0)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		box.rot = math_helper::eigen_conversions::EulerXYZToEigenQuaternion(0, math_helper::DEG2RAD(45), 0);
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, 0.0000000, 0.7071066,
				0.0000000, 1.0000000, 0.0000000,
				-0.7071066, 0.0000000, 0.7071069;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, 0, 0.7071, 0.2929,
				0, 1, 0, 1,
				-0.7071, 0, 0.7071, 1.7071,
				0, 0, 0, 1;
		sol_right << 0.7071, 0, 0.7071, 1.7071,
				0, 1, 0, 1,
				-0.7071, 0, 0.7071, 0.2929,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (0,0,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		box.rot = math_helper::eigen_conversions::EulerXYZToEigenQuaternion(0, 0, math_helper::DEG2RAD(45));
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, -0.7071066, 0.0000000,
				0.7071066, 0.7071069, 0.0000000,
				0.0000000, 0.0000000, 1.0000000;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, -0.7071, 0, 0.2929,
				0.7071, 0.7071, 0, 0.2929,
				0, 0, 1, 1,
				0, 0, 0, 1;
		sol_right << 0.7071, -0.7071, 0, 1.7071,
				0.7071, 0.7071, 0, 1.7071,
				0, 0, 1, 1,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (45,0,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		box.rot = math_helper::eigen_conversions::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), 0, math_helper::DEG2RAD(45));
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, -0.7071066, -0.0000000,
				0.5000000, 0.5000002, -0.7071066,
				0.4999999, 0.5000000, 0.7071069;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, -0.7071, 0, 0.2929,
				0.5, 0.5, -0.7071, 0.5,
				0.5, 0.5, 0.7071, 0.5,
				0, 0, 0, 1;
		sol_right << 0.7071, -0.7071, 0, 1.7071,
				0.5, 0.5, -0.7071, 1.5,
				0.5, 0.5, 0.7071, 1.5,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (0,45,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		box.rot = math_helper::eigen_conversions::EulerXYZToEigenQuaternion(0, math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.5000002, -0.5000000, 0.7071066,
				0.7071066, 0.7071069, 0.0000000,
				-0.5000000, 0.4999999, 0.7071069;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.5, -0.5, 0.7071, 0.5,
				0.7071, 0.7071, 0, 0.2929,
				-0.5, 0.5, 0.7071, 1.5,
				0, 0, 0, 1;
		sol_right << 0.5, -0.5, 0.7071, 1.5,
				0.7071, 0.7071, 0, 1.7071,
				-0.5, 0.5, 0.7071, 0.5,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (45,45,0)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		box.rot = math_helper::eigen_conversions::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), 0);
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, -0.0000000, 0.7071066,
				0.4999999, 0.7071069, -0.5000000,
				-0.5000000, 0.7071066, 0.5000002;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, 0, 0.7, 0.2929,
				0.5, 0.7071, -0.5, 0.5,
				-0.5, 0.7071, 0.5, 1.5,
				0, 0, 0, 1;
		sol_right << 0.7071, 0, 0.7, 1.7071,
				0.5, 0.7071, -0.5, 1.5,
				-0.5, 0.7071, 0.5, 0.5,
				0, 0, 0, 1;

		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (45,45,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		box.rot = math_helper::eigen_conversions::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.5000002, -0.5000000, 0.7071066,
				0.8535534, 0.1464469, -0.5000000,
				0.1464464, 0.8535534, 0.5000002;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.5, -0.5, 0.7071, 0.5,
				0.85, 0.1464, -0.5, 0.15,
				0.14, 0.85, 0.5, 0.86,
				0, 0, 0, 1;
		sol_right << 0.5, -0.5, 0.7071, 1.5,
				0.85, 0.1464, -0.5, 1.85,
				0.14, 0.85, 0.5, 1.14,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}
}

TEST_CASE("Draw line through centers of opposite faces of a box, given center of box and length of longest axis, after explicit conversion from Gazebo to Eigen" "[BoxFaceTestWithConversion]") {
	// Eigen formatting
	Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	std::string sep = "\n----------------------------------------\n\n";
	// Utility class
	RobotUtils utils;

	SECTION("[rotation: (45,0,0)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 1);
		// Convert Gazebo to Eigen
		gazebo::math::Quaternion gq = gazebo::math::Quaternion::EulerToQuaternion(math_helper::DEG2RAD(45), 0, 0);
		Eigen::Quaternionf eq = math_helper::gazebo_eigen_conversions::GazeboQuaternionToEigenQuaternion(gq);

		// Set box parameters
		box.rot = eq;
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 1.0000000, 0.0000000, 0.0000000,
				0.0000000, 0.7071069, -0.7071066,
				0.0000000, 0.7071066, 0.7071069;

		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 1, 0, 0, 0,
				0, 0.7071, -0.7071, 1,
				0, 0.7071, 0.7071, 1,
				0, 0, 0, 1;
		sol_right << 1, 0, 0, 2,
				0, 0.7071, -0.7071, 1,
				0, 0.7071, 0.7071, 1,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (0,45,0)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		// Convert Gazebo to Eigen
		gazebo::math::Quaternion gq = gazebo::math::Quaternion::EulerToQuaternion(0, math_helper::DEG2RAD(45), 0);
		Eigen::Quaternionf eq = math_helper::gazebo_eigen_conversions::GazeboQuaternionToEigenQuaternion(gq);

		// Set box parameters
		box.rot = eq;
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, 0.0000000, 0.7071066,
				0.0000000, 1.0000000, 0.0000000,
				-0.7071066, 0.0000000, 0.7071069;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, 0, 0.7071, 0.2929,
				0, 1, 0, 1,
				-0.7071, 0, 0.7071, 1.7071,
				0, 0, 0, 1;
		sol_right << 0.7071, 0, 0.7071, 1.7071,
				0, 1, 0, 1,
				-0.7071, 0, 0.7071, 0.2929,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (0,0,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		// Convert Gazebo to Eigen
		gazebo::math::Quaternion gq = gazebo::math::Quaternion::EulerToQuaternion(0, 0, math_helper::DEG2RAD(45));
		Eigen::Quaternionf eq = math_helper::gazebo_eigen_conversions::GazeboQuaternionToEigenQuaternion(gq);

		// Set box parameters
		box.rot = eq;
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, -0.7071066, 0.0000000,
				0.7071066, 0.7071069, 0.0000000,
				0.0000000, 0.0000000, 1.0000000;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, -0.7071, 0, 0.2929,
				0.7071, 0.7071, 0, 0.2929,
				0, 0, 1, 1,
				0, 0, 0, 1;
		sol_right << 0.7071, -0.7071, 0, 1.7071,
				0.7071, 0.7071, 0, 1.7071,
				0, 0, 1, 1,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (45,0,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		// Convert Gazebo to Eigen
		gazebo::math::Quaternion gq = gazebo::math::Quaternion::EulerToQuaternion(math_helper::DEG2RAD(45), 0, math_helper::DEG2RAD(45));
		Eigen::Quaternionf eq = math_helper::gazebo_eigen_conversions::GazeboQuaternionToEigenQuaternion(gq);

		// Set box parameters
		box.rot = eq;
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, -0.5000000, 0.4999999,
				0.7071066, 0.5000002, -0.5000000,
				-0.0000000, 0.7071066, 0.7071069;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, -0.7071, 0, 0.2929,
				0.5, 0.5, -0.7071, 0.5,
				0.5, 0.5, 0.7071, 0.5,
				0, 0, 0, 1;
		sol_right << 0.7071, -0.7071, 0, 1.7071,
				0.5, 0.5, -0.7071, 1.5,
				0.5, 0.5, 0.7071, 1.5,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (0,45,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		// Convert Gazebo to Eigen
		gazebo::math::Quaternion gq = gazebo::math::Quaternion::EulerToQuaternion(0, math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
		Eigen::Quaternionf eq = math_helper::gazebo_eigen_conversions::GazeboQuaternionToEigenQuaternion(gq);

		// Set box parameters
		box.rot = eq;
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.5000002, -0.7071066, 0.5000000,
				0.5000000, 0.7071069, 0.4999999,
				-0.7071066, -0.0000000, 0.7071069;

		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.5, -0.5, 0.7071, 0.5,
				0.7071, 0.7071, 0, 0.2929,
				-0.5, 0.5, 0.7071, 1.5,
				0, 0, 0, 1;
		sol_right << 0.5, -0.5, 0.7071, 1.5,
				0.7071, 0.7071, 0, 1.7071,
				-0.5, 0.5, 0.7071, 0.5,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (45,45,0)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);
		// Convert Gazebo to Eigen
		gazebo::math::Quaternion gq = gazebo::math::Quaternion::EulerToQuaternion(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), 0);
		Eigen::Quaternionf eq = math_helper::gazebo_eigen_conversions::GazeboQuaternionToEigenQuaternion(gq);

		// Set box parameters
		box.rot = eq;
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.7071069, 0.4999999, 0.5000000,
				-0.0000000, 0.7071069, -0.7071066,
				-0.7071066, 0.5000000, 0.5000002;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.7071, 0, 0.7, 0.2929,
				0.5, 0.7071, -0.5, 0.5,
				-0.5, 0.7071, 0.5, 1.5,
				0, 0, 0, 1;
		sol_right << 0.7071, 0, 0.7, 1.7071,
				0.5, 0.7071, -0.5, 1.5,
				-0.5, 0.7071, 0.5, 0.5,
				0, 0, 0, 1;

		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}

	SECTION("[rotation: (45,45,45)deg] [center: (1,1,1)] [dimensions: (2,1,1)]") {
		TestBox box("box", 2);

		// Convert Gazebo to Eigen
		gazebo::math::Quaternion gq = gazebo::math::Quaternion::EulerToQuaternion(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
		Eigen::Quaternionf eq = math_helper::gazebo_eigen_conversions::GazeboQuaternionToEigenQuaternion(gq);

		// Set box parameters
		box.rot = eq;
		box.pos << 1, 1, 1;
		box.DimsSet({2, 1, 1});

		// Test quaternion to rotation matrix
		Eigen::Matrix3f sol1;
		sol1 << 0.5000002, -0.1464467, 0.8535533,
				0.5000000, 0.8535534, -0.1464467,
				-0.7071066, 0.5000000, 0.5000002;
		REQUIRE(EqualityTestEigen(box.rot.toRotationMatrix(), sol1));

		// Convert center of box to 4x4 transformation matrix
		Eigen::Matrix4f center = Eigen::Matrix4f::Zero();
		math_helper::eigen_conversions::PoseToMatrix4(box.pos, box.rot.toRotationMatrix(), center);

		// Create empty transformation matrices for opposing faces
		Eigen::Matrix4f left_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix4f right_face_transform = Eigen::Matrix4f::Zero();
		Eigen::Matrix3f identity_33 = Eigen::Matrix3f::Identity();

		// Get translation along longest axis
		Eigen::Vector3f longest_axis_translation_left = Eigen::Vector3f::Zero();
		Eigen::Vector3f longest_axis_translation_right = Eigen::Vector3f::Zero();
		longest_axis_translation_left(box.LargestDimIdx()) = -(box.LargestDim() / 2);
		longest_axis_translation_right(box.LargestDimIdx()) = box.LargestDim() / 2;

		// Create transformation matrices for each face
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_left, identity_33, left_face_transform);
		math_helper::eigen_conversions::PoseToMatrix4(longest_axis_translation_right, identity_33, right_face_transform);

		// Perform transformation to get center of each face in world coordinates
		Eigen::Matrix4f center_left = center * left_face_transform;
		Eigen::Matrix4f center_right = center * right_face_transform;

		Eigen::Matrix4f sol_left, sol_right;
		sol_left << 0.5, -0.5, 0.7071, 0.5,
				0.85, 0.1464, -0.5, 0.15,
				0.14, 0.85, 0.5, 0.86,
				0, 0, 0, 1;
		sol_right << 0.5, -0.5, 0.7071, 1.5,
				0.85, 0.1464, -0.5, 1.85,
				0.14, 0.85, 0.5, 1.14,
				0, 0, 0, 1;
		REQUIRE(EqualityTestEigen(center_left, sol_left));
		REQUIRE(EqualityTestEigen(center_right, sol_right));

		Eigen::Vector3f left_point, right_point;
		math_helper::eigen_conversions::GetTranslationEigen(center_left, left_point);
		math_helper::eigen_conversions::GetTranslationEigen(center_right, right_point);
	}
}
