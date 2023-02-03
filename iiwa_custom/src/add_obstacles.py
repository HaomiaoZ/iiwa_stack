#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#THIS SCRIPT ADDS THE OBSTACLES TO THE SCENE, DONE AUTOMATICALLY IN DEMO.LAUNCH

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('random', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

rospy.sleep(1)

height = 0.08 #height of the obstacle

box_pose = geometry_msgs.msg.PoseStamped() #see box shape API for information
box_pose.header.frame_id = "world" #global frame
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = -0.07 #x y z positions, origin is at center of box
box_pose.pose.position.y = 0.19
box_pose.pose.position.z = 0.918+0.04
scene.add_box("box1", box_pose, size=(0.12, 0.30, height)) #add first obstacle, see box API for size info

box_pose.pose.position.x = 0.17
box_pose.pose.position.y = 0.34
scene.add_box("box2", box_pose, size=(0.12, 0.30, height)) #add second obstacle with updated position, see box API for size info


rospy.sleep(1)
