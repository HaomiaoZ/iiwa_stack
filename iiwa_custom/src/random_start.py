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
import random

#MOVES THE END EFFECTOR TO A RANDOM PLACE IN THE START ZONE

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mover', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

rospy.sleep(1)
move_group.set_planner_id("RRTConnectkConfigDefault")
move_group.set_planning_time(10)
pose_goal = move_group.get_current_pose("tool_link_ee")
pose_goal.pose.position.x = -0.239 #random x position between -0.35 and -0.205
pose_goal.pose.position.y = 0.115 #random y position between 0.042 and 0.492
pose_goal.pose.position.z = 0.928


print("new pose goal:" , pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z)

move_group.set_start_state_to_current_state()
move_group.set_pose_target(pose_goal, "tool_link_ee")

plan = move_group.go(wait = True)
move_group.stop()
move_group.clear_pose_targets()
