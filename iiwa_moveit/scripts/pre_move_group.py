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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mover', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


"""
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -pi/4
joint_goal[2] = 0
joint_goal[3] = -pi/2
joint_goal[4] = 0
joint_goal[5] = pi/3
joint_goal[6] = 0

move_group.go(joint_goal, wait=True)

move_group.stop()
"""

rospy.sleep(1)
move_group.set_planner_id("RRTConnectkConfigDefault")
move_group.set_planning_time(5)
print("EE: ", move_group.get_end_effector_link())
pose_goal = move_group.get_current_pose("tool_link_ee")
print("current pose: ", pose_goal)

pose_goal.pose.position.z = 0.6
pose_goal.pose.position.x = 0
pose_goal.pose.position.y = 0.5
pose_goal.pose.orientation.x = -0.92
pose_goal.pose.orientation.y = 0.08
pose_goal.pose.orientation.z = -0.01
pose_goal.pose.orientation.w = 0.36



print("new pose goal:" , pose_goal)

move_group.set_start_state_to_current_state()
move_group.set_pose_target(pose_goal, "tool_link_ee")

plan = move_group.go(wait = True)
move_group.stop()
move_group.clear_pose_targets()

"""
move_group.go(wait=True)
# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()
"""

