#!/usr/bin/env python

import copy
import sys
from math import pi

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
import tf

#THIS SCRIPT PRINTS OUT THE POSITION OF THE END EFFECTOR

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_pose', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

rospy.sleep(1)
pose_goal = move_group.get_current_pose("tool_link_ee") #gets current pose
print("current pose: ", pose_goal) #prints out current pose
print("joints: ", move_group.get_current_joint_values()) #prints joint position
