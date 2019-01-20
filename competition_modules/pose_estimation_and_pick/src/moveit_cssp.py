#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from math import pi
from std_msgs.msg import String, Float64
from moveit_commander.conversions import pose_to_list
import ik_4dof

class moveit_cssp(object):
	def __init__(self):
		# initial publisher for gripper command topic which is used for gripper control
		self.pub_gripper = rospy.Publisher("/gripper_joint/command",Float64,queue_size=1)

		check = True
		# you need to check if moveit server is open or not
		while(check):
			check = False
			try:
				# Instantiate a MoveGroupCommander object. This object is an interface to a planning group 
				# In our case, we have "arm" and "gripper" group

				self.move_group = moveit_commander.MoveGroupCommander("arm")
			except:
				print "moveit server isn't open yet"
				check = True


		# First initialize moveit_commander
		moveit_commander.roscpp_initialize(sys.argv)

		# Instantiate a RobotCommander object. 
		# Provides information such as the robot's kinematic model and the robot's current joint states
		self.robot = moveit_commander.RobotCommander()


		# We can get the name of the reference frame for this robot:
		planning_frame = self.move_group.get_planning_frame()
		print "============ Planning frame_cssp: %s" % planning_frame

		# We can also print the name of the end-effector link for this group:
		eef_link = self.move_group.get_end_effector_link()
		print "============ End effector link_cssp: %s" % eef_link

		# We can get a list of all the groups in the robot:
		group_names = self.robot.get_group_names()
		print "============ Available Planning Groups_cssp:", group_names

		# Sometimes for debugging it is useful to print the entire state of the robot:
		print "============ Printing robot state_cssp", self.robot.get_current_state()
		print ""
		
		### Go home
		self.home() 

		############################ Method : Using IK to calculate joint value ############################

		# After determining a specific point where arm should move, we input x,y,z,degree to calculate joint value for each wrist.

		pose_goal = Pose()
		pose_goal.position.x = 0.141
		pose_goal.position.y = 0.020
		pose_goal.position.z = 0.042
		# ik_4dof.ik_solver(x, y, z, degree)
		joint_value = ik_4dof.ik_solver(pose_goal.position.x, pose_goal.position.y, pose_goal.position.z, -90)

		for joint in joint_value:
			joint = list(joint)
			# determine gripper state
			joint.append(0)
			try:
				self.move_group.go(joint, wait=True)
				print("move_cssp")
			except:
				rospy.loginfo(str(joint) + " isn't a valid configuration.")

		# Reference: https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html

		### close gripper
		
		print("close gripper_cssp")
		rospy.sleep(2)
		grip_data = Float64()
		grip_data.data = 2.0 
		self.pub_gripper.publish(grip_data)

		### open gripper

		# rospy.sleep(2)
		# grip_data = Float64()
		# grip_data.data = 0.5
		# self.pub_gripper.publish(grip_data)
		# rospy.sleep(2)

		### Go home
		# self.home() 


	def home(self):

		############################ Method : Joint values (Go home!)############################


		# We can get the joint values from the group and adjust some of the values:

		# Go home!!!
		joint_goal = self.move_group.get_current_joint_values()
		joint_goal[0] = 0   		# arm_shoulder_plane_joint
		joint_goal[1] = -pi*5/13   	# arm_shoulder_lift_joint
		joint_goal[2] = pi*3/4   	# arm_elbow_flex_joint
		joint_goal[3] = pi/3   		# arm_wrist_flex_joint
		joint_goal[4] = 0   		# gripper_joint

		# The go command can be called with joint values, poses, or without any
		# parameters if you have already set the pose or joint target for the group
		self.move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		self.move_group.stop()
		print("Go home_cssp")
		
	def onShutdown(self):
		rospy.loginfo("Shutdown.")
		print("Shutdown_cssp")

if __name__ == '__main__': 
	rospy.init_node('moveit_cssp',anonymous=False)
	print("start!_cssp")
	rospy.sleep(5)
	print("start moveit_cssp")
	moveit_cssp = moveit_cssp()
	rospy.on_shutdown(moveit_cssp.onShutdown)
	rospy.spin()