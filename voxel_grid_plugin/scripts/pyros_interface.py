#! /usr/bin/env python3
"""@package ROS Interface
This package will interface Python with C++ using ROS Action Server-Client calls
"""
from __future__ import print_function

# Math utils
import math
import random
# Timing functions
from timeit import default_timer as timer

import actionlib
# For plotting
import matplotlib
import numpy as np
import rospkg
# ROS imports
import rospy
from geometry_msgs.msg import Pose
# Custom messages from voxel_grid_plugin
import voxel_grid_plugin.msg


class PyRobotController:
	"""
	Class for controlling the robot via Action calls to RobotControlActionServer

	Attributes:
		robot_control_client(actionlib.SimpleActionClient): The ROS ActionClient for calling the robot controller
	"""

	def __init__(self):
		"""
		The constructor for RobotController class

		Creates and initializes the ActionClient for sending commands to the robot
		"""
		self.robot_control_client = actionlib.SimpleActionClient('RobotControlActionServer',
																 voxel_grid_plugin.msg.RobotControlAction)
		rospy.loginfo("Waiting for robot controller server")
		self.robot_control_client.wait_for_server()

	@staticmethod
	def execution_done(_terminal_state, _result):
		print("GoalStatus: {}".format(_terminal_state))
		print("GoalResult: {}".format(_result))

	@staticmethod
	def execution_feedback(feedback):
		print(feedback)

	def robot_control_command(self, _joint_name="", _joint_target=0.0):
		"""
		Function to send control command to the robot control server
		This constructs a RobotControlGoal goal for the ActionServer, sends it, and waits for response
		:param joint_name: The name of the joint to control
		:param joint_target: The target angle for the joint
		:param add_to_queue: Add command to queue or execute the queue
		:return: None
		"""
		robot_control_goal = voxel_grid_plugin.msg.RobotControlGoal(joint_name=_joint_name, joint_target=_joint_target)
		self.robot_control_client.send_goal(robot_control_goal, feedback_cb=self.execution_feedback,
											done_cb=self.execution_done)
		res = self.robot_control_client.wait_for_result()
		return res


class PySimController:
	"""
	Class for controlling the Gazebo simulator via calls to pybind module
	Also contains action client for calling model controller, spawning and deleting models
	"""

	def __init__(self):
		"""
		The constructor for SimController class

		Creates and initializes the GazeboInterface for sending commands to the simulator
		"""
		self.model_control_client = actionlib.SimpleActionClient('ModelControlActionServer',
																 voxel_grid_plugin.msg.ModelControlAction)
		rospy.loginfo("Wait for model control server")
		self.model_control_client.wait_for_server()
		"""Dictionary mapping simulator states to usable message types"""

	def model_control_command(self, goal):
		self.model_control_client.send_goal(goal)
		wait = self.model_control_client.wait_for_result()
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
			return False
		else:
			return self.model_control_client.get_result()

	def model_spawn(self, model_name="box", model_size=None, initial_pose=None):
		model_control_goal = voxel_grid_plugin.msg.ModelControlGoal()
		model_control_goal.model_command = voxel_grid_plugin.msg.ModelControlGoal.MODEL_ADD
		model_control_goal.model_name = model_name
		if "small" in model_size.lower():
			model_control_goal.model_size = voxel_grid_plugin.msg.ModelControlGoal.MODEL_SMALL
		if "med" in model_size.lower():
			model_control_goal.model_size = voxel_grid_plugin.msg.ModelControlGoal.MODEL_MED
		if "large" in model_size.lower():
			model_control_goal.model_size = voxel_grid_plugin.msg.ModelControlGoal.MODEL_LARGE
		if initial_pose is not None:
			model_control_goal.initial_pose = initial_pose
		else:
			default_pose = Pose()
			default_pose.position.x, default_pose.position.y, default_pose.position.z = 0, 0, 0
			default_pose.orientation.x, default_pose.orientation.y, default_pose.orientation.z, default_pose.orientation.w = 0, 0, 0, 1
			model_control_goal.initial_pose = default_pose

		return self.model_control_command(goal=model_control_goal)

	def model_delete(self, model_name="obstacle"):
		model_control_goal = voxel_grid_plugin.msg.ModelControlGoal()
		model_control_goal.model_command = voxel_grid_plugin.msg.ModelControlGoal.MODEL_DELETE
		model_control_goal.model_name = model_name

		return self.model_control_command(goal=model_control_goal)

	def model_move(self, model_name="box", target_pose=None):
		model_control_goal = voxel_grid_plugin.msg.ModelControlGoal()
		model_control_goal.model_command = voxel_grid_plugin.msg.ModelControlGoal.MODEL_MOVE
		model_control_goal.model_name = model_name
		if target_pose is not None:
			model_control_goal.target_pose = target_pose
		else:
			default_pose = Pose()
			default_pose.position.x, default_pose.position.y, default_pose.position.z = 0, 0, 0
			default_pose.orientation.x, default_pose.orientation.y, default_pose.orientation.z, default_pose.orientation.w = 0, 0, 0, 1
			model_control_goal.initial_pose = default_pose

		return self.model_control_command(goal=model_control_goal)


class PyVoxelGrid:
	"""
	Class for fetching the the latest voxel grid via Action calls to VoxelGridActionServer

	Attributes:
		voxel_grid_client(actionlib.SimpleActionClient): The ROS ActionClient for fetching the voxel grid data
	"""

	def __init__(self):
		"""
		The constructor for PyVoxelGrid class

		Creates and initializes the ActionClient for fetching the voxel grid
		"""
		self.voxel_grid_client = actionlib.SimpleActionClient('VoxelGridActionServer',
															  voxel_grid_plugin.msg.VoxelGridAction)
		rospy.loginfo("Wait for voxel grid server")
		self.voxel_grid_client.wait_for_server()

		self.pygrid = np.empty(shape=(0, 0))
		self.pygrid_colors = np.empty(shape=(0, 0))

	def voxel_grid_feedback(self, msg):
		"""
		Callback function that processes feedback while voxel grid data is being processed by server

		:param msg: The feedback message to be parsed
		"""
		print("Voxel Grid {}% processed\t\t\t".format(msg.completion_percent), end="\r")

	def voxel_grid_fetch_command(self):
		"""
		Function to send control command to the Gazebo simulator control server
		This constructs a SimControlGoal goal for the ActionServer, sends it, and waits for response

		:param command: One possible command from SimulatorActions dictionary
		:param time_delay: The delay between simulator updates
		:return: None
		"""
		voxel_grid_control_goal = voxel_grid_plugin.msg.VoxelGridGoal()
		self.voxel_grid_client.send_goal(voxel_grid_control_goal, feedback_cb=self.voxel_grid_feedback)
		wait = self.voxel_grid_client.wait_for_result()
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
		else:
			return self.voxel_grid_client.get_result()

	def create_voxel_grid(self, voxels_per_dim, robot_nodes, occupied_nodes):
		"""
		Function to construct a voxel grid from occupied and robot nodes, obtained from Action Response

		:param command: One possible command from SimulatorActions dictionary
		:param time_delay: The delay between simulator updates
		:return: None
		"""
		# Create voxel grids and colors, zero out everything
		self.pygrid = np.zeros((voxels_per_dim, voxels_per_dim, voxels_per_dim))
		self.pygrid_colors = np.zeros(self.pygrid.shape, dtype=object)
		# Populate robot nodes
		for i in range(len(robot_nodes)):
			self.pygrid[
				robot_nodes[i].voxel_coordinate[0], robot_nodes[i].voxel_coordinate[1], robot_nodes[i].voxel_coordinate[
					2]] = True
			self.pygrid_colors[
				robot_nodes[i].voxel_coordinate[0], robot_nodes[i].voxel_coordinate[1], robot_nodes[i].voxel_coordinate[
					2]] = 'red'
		# Populate occupied nodes
		for i in range(len(occupied_nodes)):
			self.pygrid[occupied_nodes[i].voxel_coordinate[0], occupied_nodes[i].voxel_coordinate[1],
						occupied_nodes[i].voxel_coordinate[2]] = True
			self.pygrid_colors[occupied_nodes[i].voxel_coordinate[0], occupied_nodes[i].voxel_coordinate[1],
							   occupied_nodes[i].voxel_coordinate[2]] = 'blue'

		print("RobotNodes: {} ObstacleNodes: {}".format(len(robot_nodes), len(occupied_nodes)))
		return self.pygrid, self.pygrid_colors


class PyVoxelGridViz():
	"""
	Class for fetching the the latest voxel grid via Action calls to VoxelGridActionServer

	Attributes:
	  voxel_grid_client(actionlib.SimpleActionClient): The ROS ActionClient for fetching the voxel grid data
	"""

	def show_voxel_grid(self, pygrid, pygrid_colors, fig_name, save_fig=False):
		"""
		Function to construct a voxel grid from occupied and robot nodes, obtained from Action Response

		:param fig_name: Name of figure that will saved, if saving is enabled
		:param save_fig: Toggle to either save or show voxel grid output
		:return: None
		"""
		# Show grid
		if save_fig:
			matplotlib.use('Agg', warn=False, force=True)
			import matplotlib.pyplot as plt
			from mpl_toolkits.mplot3d import Axes3D

			fig = plt.figure()
			ax = fig.gca(projection='3d')
			ax.voxels(pygrid, facecolors=pygrid_colors, edgecolor='k')
			rospack = rospkg.RosPack()
			pkg_path = rospack.get_path('voxel_grid_plugin')
			fig.savefig(pkg_path + "/outputs/" + fig_name + ".png")
		else:
			matplotlib.use('TKAgg', warn=False, force=True)
			import matplotlib.pyplot as plt
			from mpl_toolkits.mplot3d import Axes3D

			fig = plt.figure()
			ax = fig.gca(projection='3d')
			ax.voxels(pygrid, facecolors=pygrid_colors, edgecolor='k')
			plt.show()


def create_box_pose(box_type):
	# Generate object on table
	position_lower_bound = (0, 0, 0.46 * 2)
	position_upper_bound = (0.76 * 2, 0.38 * 2, 0.46 * 2)  # center of table in gazebo world
	small_obstacle_offset = (0.05 / 2, 0.05 / 2, 0.05 / 2)  # size of box from model sdf
	medium_obstacle_offset = (0.15 / 2, 0.15 / 2, 0.15 / 2)  # size of box from model sdf
	large_obstacle_offset = (0.35 / 2, 0.35 / 2, 0.35 / 2)  # size of box from model sdf

	random_position = lambda xl, yl, xu, yu: (random.uniform(xl, xu), random.uniform(yl, yu))
	on_table = Pose()

	pos = random_position(position_lower_bound[0], position_lower_bound[1],
						  position_upper_bound[0], position_upper_bound[1])

	if box_type is "small":
		on_table.position.x = pos[0] + small_obstacle_offset[0]
		on_table.position.y = pos[1] + small_obstacle_offset[1]
		on_table.position.z = position_lower_bound[2] + small_obstacle_offset[2]
	elif box_type is "medium":
		on_table.position.x = pos[0] + medium_obstacle_offset[0]
		on_table.position.y = pos[1] + medium_obstacle_offset[1]
		on_table.position.z = position_lower_bound[2] + medium_obstacle_offset[2]
	elif box_type is "large":
		on_table.position.x = pos[0] + large_obstacle_offset[0]
		on_table.position.y = pos[1] + large_obstacle_offset[1]
		on_table.position.z = position_lower_bound[2] + large_obstacle_offset[2]

	return on_table


if __name__ == '__main__':
	try:
		rospy.init_node('PyInterface')
		sim_controller = PySimController()
		robot_controller = PyRobotController()
		voxel_grid_fetcher = PyVoxelGrid()
		viz = PyVoxelGridViz()

		# # Create initial poses
		# start_pose = create_box_pose("small")
		# move_pose = Pose()
		#
		# # Spawn one model
		# sim_controller.model_spawn(model_name="testy", model_size="small", initial_pose=start_pose)
		# rospy.sleep(3)
		#
		# # Set target position
		# move_pose.position.x = start_pose.position.x - 0.05
		# move_pose.position.y = start_pose.position.y + 0.05
		# move_pose.position.z = start_pose.position.z + 0.03
		# sim_controller.model_move(model_name="testy", target_pose=move_pose)
		# rospy.sleep(3)
		#
		# # Spawn second model
		# sim_controller.model_spawn(model_name="moretesty", model_size="small", initial_pose=start_pose)
		# rospy.sleep(3)
		#
		# # Set target position
		# move_pose.position.x = start_pose.position.x + 0.05
		# move_pose.position.y = start_pose.position.y - 0.05
		# move_pose.position.z = start_pose.position.z + 0.03
		# sim_controller.model_move(model_name="more_testy", target_pose=move_pose)
		# rospy.sleep(3)
		#
		# # Delete model
		# sim_controller.model_delete(model_name="testy")

		# Move robot
		rospy.loginfo("Queueing robot commands")
		commands = [{
			"shoulder_lift_joint": math.radians(-25),
			"shoulder_pan_joint": math.radians(-10),
			"shoulder_lift_joint": math.radians(-40),
			"shoulder_pan_joint": math.radians(20),
			"elbow_joint": math.radians(-10)}]

		for idx, command in enumerate(commands):
			joint_name, joint_target = list(command.items())[0]
			rospy.loginfo("Executing command")
			result = robot_controller.robot_control_command(_joint_name=joint_name, _joint_target=joint_target)
			if result:
				rospy.loginfo("Command succeeded")
			else:
				rospy.loginfo("Command failed")
				continue
		# rospy.sleep(0)

		# 	# Fetch voxel grid
		# 	rospy.loginfo("Fetching voxel grid")
		# 	start = timer()
		# 	result = voxel_grid_fetcher.voxel_grid_fetch_command()
		# 	grid_fetch_time = timer()
		# 	print("Fetching voxel grid took: {:.3f}s".format(grid_fetch_time - start))
		#
		# 	# Reconstruct voxel grid
		# 	rospy.loginfo("Reconstructing voxel grid")
		# 	start = timer()
		# 	grid, colors = voxel_grid_fetcher.create_voxel_grid(result.num_voxels_per_dim, result.robot_nodes,
		# 														result.occupied_nodes)
		# 	grid_create_time = timer()
		# 	print("Reconstructing voxel grid took: {:.3f}s".format(grid_create_time - start))
		#
		# 	# Display or save voxel grid
		# 	rospy.loginfo("Saving voxel grid")
		# 	start = timer()
		# 	viz.show_voxel_grid(pygrid=grid, pygrid_colors=colors, fig_name="save_" + str(idx), save_fig=True)
		# 	grid_show_time = timer()
		# 	print("Showing voxel grid took: {:.3f}s".format(grid_show_time - start))
		#
		# 	print("--------------------------------------------------------------\n")

		# Finish commands
		rospy.loginfo("Done")

	except rospy.ROSInterruptException:
		rospy.logfatal("ERROR")
