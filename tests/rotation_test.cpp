//
// Created by Kanishka Ganguly on 7/23/19.
// Copyright (c) 2019 Amazon Robotics. All rights reserved.
//
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "../include/catch.hpp"
#include "../include/imports.hpp"

/** @example tests/rotation_test.cpp
 * This tests the weird Gazebo issues when creating and converting rotation data using gazebo::math library.
 * Compared to Eigen, Gazebo gives incorrect output for same points of comparison.
 *
 * @note Gazebo treats Euler angles in ZYX format. Eigen uses XYZ. Conversion utilities are provided in include/imports.hpp
 * @note Gazebo also treats quaternions in WZYX format. Conversion utilities are provided in include/imports.hpp
 */
bool QuaternionTestGazebo(const gazebo::math::Quaternion &q1, const gazebo::math::Quaternion &q2) {
  return q1 == q2;
}

bool EqualityTestGazebo(const gazebo::math::Matrix3 &m1, const gazebo::math::Matrix3 &m2) {
  return m1 == m2;
}

bool EqualityTestEigen(const Eigen::Matrix3f &m1, const Eigen::Matrix3f &m2) {
  return m1.isApprox(m2, 1e-3);
}

TEST_CASE("Gazebo conversions from Quaternion to 3x3 Matrix" "[GazeboEqualityTest]") {
  SECTION("Quaternion from x axis") {
    gazebo::math::Quaternion rot1(0, 0, math_helper::DEG2RAD(45));
    gazebo::math::Matrix3 mat1 = rot1.GetAsMatrix3();
    gazebo::math::Matrix3 sol1(1.0000000, 0.0000000, 0.0000000,
                               0.0000000, 0.7071069, -0.7071066,
                               0.0000000, 0.7071066, 0.7071069);
    REQUIRE(EqualityTestGazebo(mat1, sol1));
  }
  SECTION("Quaternion from y axis") {
    gazebo::math::Quaternion rot2(0, math_helper::DEG2RAD(45), 0);
    gazebo::math::Matrix3 mat2 = rot2.GetAsMatrix3();
    gazebo::math::Matrix3 sol2(0.7071069, 0.0000000, 0.7071066,
                               0.0000000, 1.0000000, 0.0000000,
                               -0.7071066, 0.0000000, 0.7071069);
    REQUIRE(EqualityTestGazebo(mat2, sol2));
  }
  SECTION("Quaternion from z axis") {
    gazebo::math::Quaternion rot3(math_helper::DEG2RAD(45), 0, 0);
    gazebo::math::Matrix3 mat3 = rot3.GetAsMatrix3();
    gazebo::math::Matrix3 sol3(0.7071069, -0.7071066, 0.0000000,
                               0.7071066, 0.7071069, 0.0000000,
                               0.0000000, 0.0000000, 1.0000000);
    REQUIRE(EqualityTestGazebo(mat3, sol3));
  }
  SECTION("Quaternion from x,y axis") {
    gazebo::math::Quaternion rot4(0, math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
    gazebo::math::Matrix3 mat4 = rot4.GetAsMatrix3();
    gazebo::math::Matrix3 sol4(0.7071069, 0.4999999, 0.5000000,
                               -0.0000000, 0.7071069, -0.7071066,
                               -0.7071066, 0.5000000, 0.5000002);
    REQUIRE(EqualityTestGazebo(mat4, sol4));
  }
  SECTION("Quaternion from x,z axis") {
    gazebo::math::Quaternion rot5(math_helper::DEG2RAD(45), 0, math_helper::DEG2RAD(45));
    gazebo::math::Matrix3 mat5 = rot5.GetAsMatrix3();
    gazebo::math::Matrix3 sol5(0.7071069, -0.5000000, 0.4999999,
                               0.7071066, 0.5000002, -0.5000000,
                               -0.0000000, 0.7071066, 0.7071069);
    REQUIRE(EqualityTestGazebo(mat5, sol5));
  }
  SECTION("Quaternion from y,z axis") {
    gazebo::math::Quaternion rot6(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), 0);
    gazebo::math::Matrix3 mat6 = rot6.GetAsMatrix3();
    gazebo::math::Matrix3 sol6(0.5000002, -0.7071066, 0.5000000,
                               0.5000000, 0.7071069, 0.4999999,
                               -0.7071066, -0.0000000, 0.7071069);
    REQUIRE(EqualityTestGazebo(mat6, sol6));
  }
  SECTION("Quaternion from x,y,z axis") {
    gazebo::math::Quaternion rot7(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
    gazebo::math::Matrix3 mat7 = rot7.GetAsMatrix3();
    gazebo::math::Matrix3 sol7(0.5000002, -0.1464467, 0.8535533,
                               0.5000000, 0.8535534, -0.1464467,
                               -0.7071066, 0.5000000, 0.5000002);
    REQUIRE(EqualityTestGazebo(mat7, sol7));
  }
}

TEST_CASE("Gazebo Quaternion from EulerAngles" "[GazeboQuaternionTest]") {
  SECTION("Quaternion from x angle") {
    gazebo::math::Quaternion rot1(0, 0, math_helper::DEG2RAD(45));
    gazebo::math::Quaternion quat1(0.9238796, 0, 0, 0.3826834);
    REQUIRE(QuaternionTestGazebo(rot1, quat1));
  }
  SECTION("Quaternion from y angle") {
    gazebo::math::Quaternion rot2(0, math_helper::DEG2RAD(45), 0);
    gazebo::math::Quaternion quat2(0.9238796, 0, 0.3826834, 0);
    REQUIRE(QuaternionTestGazebo(rot2, quat2));
  }
  SECTION("Quaternion from z angle") {
    gazebo::math::Quaternion rot3(math_helper::DEG2RAD(45), 0, 0);
    gazebo::math::Quaternion quat3(0.9238796, 0.3826834, 0, 0);
    REQUIRE(QuaternionTestGazebo(rot3, quat3));
  }
  SECTION("Quaternion from x,y angles") {
    gazebo::math::Quaternion rot4(0, math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
    gazebo::math::Quaternion quat4(0.8535534, -0.1464466, 0.3535533, 0.3535533);
    REQUIRE(QuaternionTestGazebo(rot4, quat4));
  }
  SECTION("Quaternion from x,z angles") {
    gazebo::math::Quaternion rot5(math_helper::DEG2RAD(45), 0, math_helper::DEG2RAD(45));
    gazebo::math::Quaternion quat5(0.8535534, 0.3535533, 0.1464466, 0.3535533);
    REQUIRE(QuaternionTestGazebo(rot5, quat5));
  }
  SECTION("Quaternion from y,z angles") {
    gazebo::math::Quaternion rot6(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), 0);
    gazebo::math::Quaternion quat6(0.8535534, 0.3535533, 0.3535533, -0.1464466);
    REQUIRE(QuaternionTestGazebo(rot6, quat6));
  }
  SECTION("Quaternion from x,y,z angles") {
    gazebo::math::Quaternion rot7(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
    gazebo::math::Quaternion quat7(0.8446232, 0.1913417, 0.4619397, 0.1913417);
    REQUIRE(QuaternionTestGazebo(rot7, quat7));
  }
}

TEST_CASE("Eigen conversions from Quaternion to 3x3 Matrix" "[EigenEqualityTest]") {
  SECTION("Quaternion from x axis") {
    Eigen::Quaternionf rot1 = math_helper::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), 0, 0);
    Eigen::Matrix3f mat1 = rot1.toRotationMatrix();
    Eigen::Matrix3f sol1;
    sol1 << 1.0000000, 0.0000000, 0.0000000,
        0.0000000, 0.7071069, -0.7071066,
        0.0000000, 0.7071066, 0.7071069;
    REQUIRE(EqualityTestEigen(mat1, sol1));
  }
  SECTION("Quaternion from y axis") {
    Eigen::Quaternionf rot2 = math_helper::EulerXYZToEigenQuaternion(0, math_helper::DEG2RAD(45), 0);
    Eigen::Matrix3f mat2 = rot2.toRotationMatrix();
    Eigen::Matrix3f sol2;
    sol2 << 0.7071069, 0.0000000, 0.7071066,
        0.0000000, 1.0000000, 0.0000000,
        -0.7071066, 0.0000000, 0.7071069;
    REQUIRE(EqualityTestEigen(mat2, sol2));
  }
  SECTION("Quaternion from z axis") {
    Eigen::Quaternionf rot3 = math_helper::EulerXYZToEigenQuaternion(0, 0, math_helper::DEG2RAD(45));
    Eigen::Matrix3f mat3 = rot3.toRotationMatrix();
    Eigen::Matrix3f sol3;
    sol3 << 0.7071069, -0.7071066, 0.0000000,
        0.7071066, 0.7071069, 0.0000000,
        0.0000000, 0.0000000, 1.0000000;
    REQUIRE(EqualityTestEigen(mat3, sol3));
  }
  SECTION("Quaternion from x,y axis") {
    Eigen::Quaternionf rot4 = math_helper::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), 0);
    Eigen::Matrix3f mat4 = rot4.toRotationMatrix();
    Eigen::Matrix3f sol4;
    sol4 << 0.7071069, -0.0000000, 0.7071066,
        0.4999999, 0.7071069, -0.5000000,
        -0.5000000, 0.7071066, 0.5000002;
    REQUIRE(EqualityTestEigen(mat4, sol4));
  }
  SECTION("Quaternion from xz axis") {
    Eigen::Quaternionf rot5 = math_helper::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), 0, math_helper::DEG2RAD(45));
    Eigen::Matrix3f mat5 = rot5.toRotationMatrix();
    Eigen::Matrix3f sol5;
    sol5 << 0.7071069, -0.7071066, -0.0000000,
        0.5000000, 0.5000002, -0.7071066,
        0.4999999, 0.5000000, 0.7071069;
    REQUIRE(EqualityTestEigen(mat5, sol5));
  }
  SECTION("Quaternion from yz axis") {
    Eigen::Quaternionf rot6 = math_helper::EulerXYZToEigenQuaternion(0, math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
    Eigen::Matrix3f mat6 = rot6.toRotationMatrix();
    Eigen::Matrix3f sol6;
    sol6 << 0.5000002, -0.5000000, 0.7071066,
        0.7071066, 0.7071069, -0.0000000,
        -0.5000000, 0.4999999, 0.7071069;
    REQUIRE(EqualityTestEigen(mat6, sol6));
  }
  SECTION("Quaternion from x,y,z axis") {
    Eigen::Quaternionf rot7 = math_helper::EulerXYZToEigenQuaternion(math_helper::DEG2RAD(45), math_helper::DEG2RAD(45), math_helper::DEG2RAD(45));
    Eigen::Matrix3f mat7 = rot7.toRotationMatrix();
    Eigen::Matrix3f sol7;
    sol7 << 0.5000002, -0.5000000, 0.7071066,
        0.8535534, 0.1464469, -0.5000000,
        0.1464464, 0.8535534, 0.5000002;
    REQUIRE(EqualityTestEigen(mat7, sol7));
  }
}