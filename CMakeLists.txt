cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(voxel_grid_plugin)
add_compile_options(-std=c++11)
set(PYBIND11_PYTHON_VERSION 2.7)

# Find all required packages
find_package(gazebo REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Pangolin REQUIRED)
find_package(cilantro REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Doxygen REQUIRED)
find_package(spdlog REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(catkin REQUIRED
        COMPONENTS
        genmsg roscpp gazebo_ros gazebo_plugins actionlib
        control_msgs sensor_msgs controller_manager_msgs actionlib_msgs
        std_msgs geometry_msgs
        message_generation
        )
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Project names
set(VOXEL_GRID_PLUGIN "voxel_grid_plugin")
set(ROS_ROBOT_INTERFACE "ros_robot_interface")
set(PYGAZEBO_INTERFACE "pygazebo_interface")
set(ROBOT_CONTROL_MSGS "robot_controller_msgs")
set(VOXEL_GRID_TEST "voxel_grid_test")
set(ROS_CONTROLLER_TEST "ros_controller_test")
set(ROTATION_TEST "rotation_test")
set(BOX_FACE_TEST "box_face_test")

# Project compilation rules
option(BUILD_GAZEBO_LIB "Build shared library for Gazebo plugin" OFF)
option(BUILD_INTERFACE "Build executable for Gazebo-ROS interface" OFF)
option(USE_VIZ "Use cilantro visualizer" OFF)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_DOCS "Build documentation" OFF)

# ROS setup
add_message_files(
        DIRECTORY msg
        FILES VoxelData.msg
)
add_action_files(
        DIRECTORY action
        FILES RobotControl.action SimControl.action VoxelGrid.action ModelControl.action
)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp gazebo_ros gazebo_plugins actionlib_msgs message_runtime
        LIBRARIES ${VOXEL_GRID_PLUGIN} ${ROBOT_CONTROL_MSGS} ${PYGAZEBO_INTERFACE}
)

link_directories(${Eigen_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OpenMP_LIBRARY_DIRS} ${cilantro_LIBRARY_DIRS} ${tinyxml2_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenMP_INCLUDE_DIRS} ${cilantro_INCLUDE_DIRS} ${pybind11_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS})
# Targets to link against
list(APPEND LINKED_LIBS ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${ROBOT_CONTROL_MSGS} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${OpenMP_LIBRARIES} ${cilantro_LIBRARIES} ${spdlog_lib} ${tinyxml2_LIBRARIES})
#-------------------- Build documentation --------------------------------#
if (BUILD_DOCS)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${PROJECT_SOURCE_DIR}/docs/Doxyfile)
        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message(STATUS "Doxygen build started")
        add_custom_target(doc_doxygen ALL
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/docs
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM)
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif ()
#-------------------- Gazebo Protobuf ------------------------------------#
set(PROTOBUF_CUSTOM_DIRS "msg/proto")
set(PROTOBUF_GAZEBO_DIRS "/usr/include/gazebo-7/gazebo/msgs/proto")
set(PROTOBUF_PROTOC_EXECUTABLE "/usr/bin/protoc")
set(msgs
        ${PROTOBUF_CUSTOM_DIRS}/robot_control_request.proto
        ${PROTOBUF_GAZEBO_DIRS}/pose.proto
        ${PROTOBUF_GAZEBO_DIRS}/vector3d.proto
        ${PROTOBUF_GAZEBO_DIRS}/quaternion.proto
        )
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(${ROBOT_CONTROL_MSGS} ${PROTO_SRCS})
target_link_libraries(${ROBOT_CONTROL_MSGS} ${PROTOBUF_LIBRARY})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
message(STATUS "Building shared library for robot controller messages.")
#-------------------------------------------------------------------------#

#--------------------- Build targets -------------------------------------#
# Visualizer settings
if (USE_VIZ)
    message(STATUS "Enabling visualizer.")
    add_definitions(-DSHOW_VIZ=ON)
endif ()
# Build VoxelGridPlugin
if (BUILD_GAZEBO_LIB)
    message(STATUS "Building shared library for WorldPlugin.")
    add_library(${VOXEL_GRID_PLUGIN} src/voxel_grid_plugin.cpp src/voxel_grid.cpp)
    set(VOXEL_GRID_PLUGIN_STRING "<plugin name=\"voxel_grid_plugin\" filename=\"lib${VOXEL_GRID_PLUGIN}.so\"/>")
    target_link_libraries(${VOXEL_GRID_PLUGIN} ${LINKED_LIBS})
    set(BUILD_TESTS OFF)
endif ()
# Build GazeboInterface
if (BUILD_INTERFACE)
    add_definitions(-DUSE_ROS=ON)
    message(STATUS "Building executable for ROS-Robot interface.")
    add_executable(${ROS_ROBOT_INTERFACE} src/ros_robot_interface.cpp src/ros_robot_controller.cpp)
    target_link_libraries(${ROS_ROBOT_INTERFACE} ${LINKED_LIBS})
    # Can't use typical pybind11_add_module since catkin doesn't support that
    # This is the  modified version that is catkin compliant
    message(STATUS "Building executable for Gazebo interface.")
    add_library(${PYGAZEBO_INTERFACE} MODULE src/pygazebo.cpp)
    target_link_libraries(${PYGAZEBO_INTERFACE} PRIVATE ${LINKED_LIBS} pybind11::module)
    set_target_properties(${PYGAZEBO_INTERFACE} PROPERTIES
            PREFIX ""
            LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
endif ()
# Build tests
if (BUILD_TESTS)
    # Voxel grid test
    message(STATUS "Building unit tests.")
    add_definitions(-DUSE_ROS=ON)
    # Build test for voxel grid
    add_executable(${VOXEL_GRID_TEST} tests/voxel_grid_test.cpp src/voxel_grid.cpp)
    target_link_libraries(${VOXEL_GRID_TEST} ${LINKED_LIBS})
    # Build test for ROS controller
    add_executable(${ROS_CONTROLLER_TEST} tests/ros_controller_test.cpp src/ros_robot_controller.cpp)
    target_link_libraries(${ROS_CONTROLLER_TEST} ${LINKED_LIBS})
    # Build test for debugging Gazebo rotation errors
    add_executable(${ROTATION_TEST} tests/rotation_test.cpp)
    target_link_libraries(${ROTATION_TEST} ${LINKED_LIBS})
    # Build test for bounding box raytracing
    add_executable(${BOX_FACE_TEST} tests/box_face_test.cpp src/voxel_grid.cpp)
    target_link_libraries(${BOX_FACE_TEST} ${LINKED_LIBS})
endif ()
#-------------------------------------------------------------------------#
# Load corresponding Gazebo plugins at build time
configure_file(${PROJECT_SOURCE_DIR}/worlds/custom_empty.world.in ${PROJECT_SOURCE_DIR}/worlds/custom_empty.world @ONLY)