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

#THIS SCRIPT CLEARS OBSTACLES

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('random', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

rospy.sleep(1)
scene.remove_world_object() #remove object