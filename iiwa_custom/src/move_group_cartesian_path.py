#!/usr/bin/env python

import copy
import sys
from math import pi
import rosbag
import time
import shape_msgs.msg
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionFKRequest, GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import subprocess, shlex, psutil
rosbag_proc = None
command = None
def rosbag_record(demo_num):
    global command, rosbag_proc
    command = "rosbag record -O raw_demo" + str(demo_num) +" /ee_pose"
    command = shlex.split(command)
    rosbag_proc = subprocess.Popen(command)

def rogbag_kill(demo_num):
    global command, rosbag_proc
    command = "rosbag record -O raw_demo" + str(demo_num) +" /ee_pose"
    command = shlex.split(command)
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
             proc.send_signal(subprocess.signal.SIGINT)
    rosbag_proc.send_signal(subprocess.signal.SIGINT)
#THIS SCRIPT MOVES THE END EFFECTOR IN A PLANAR MOTION FROM THE STARTING POINT TO THE END POINT,
#THROUGH THE OBSTACLES.
demo_id = 250
rosbag_record(demo_id)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mover', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

rospy.sleep(1)
move_group.set_planner_id("RRTConnectkConfigDefaultNoInterpolate") #custom planner ID that does not interpolate in between

#tesing points 2: 1:[(-0.014, 0.392), (0.0, 0.389), (0.063, 0.258), (0.186, 0.152), (0.3, 0.4)], 2:[(-0.146, 0.385), (0.05, 0.38), (0.072, 0.27), (0.17, 0.16), (0.3, 0.4)], 3:[(-0.15, 0.4), (0.02, 0.388), (0.067, 0.268), (0.167, 0.156), (0.3, 0.4)], 4:[(-0.142, 0.38), (0.01, 0.393), (0.05, 0.262), (0.15, 0.148), (0.3, 0.4)]
move_group.set_planning_time(120) #allow 10 seconds to plan
pose_goal = move_group.get_current_pose("tool_link_ee")
pose_goal.pose.position.x = 0.3 #end position is 0.3, 0.4, change this to alter the goal
pose_goal.pose.position.y = 0.4

main = moveit_msgs.msg.Constraints() #create constraint message

pos_con = moveit_msgs.msg.PositionConstraint() #create position_constraint message
pos_con.header = pose_goal.header #copy current header
pos_con.link_name = "tool_link_ee" #specify end effector link
pos_con.target_point_offset.x = 0
pos_con.target_point_offset.y = 0
pos_con.target_point_offset.z = 0 #point is right on origin

#position constraint, a flat "plane" so the end effector cannot travel over obstacles
shape = shape_msgs.msg.SolidPrimitive() #create primitives message
shape.type = 1 #cube
shape.dimensions = [0.725, 0.47, 0.01] #xyz of cube
shapePose = geometry_msgs.msg.Pose() #create Pose msg
shapePose.position.x = 0
shapePose.position.y = 0.265
shapePose.position.z = pose_goal.pose.position.z #have it at current z
BoundVol = moveit_msgs.msg.BoundingVolume() #create BoundingVolume message
BoundVol.primitives.append(shape) #add shape info
BoundVol.primitive_poses.append(shapePose) #add shape pose info
pos_con.constraint_region = BoundVol #add BoundVol to constraint_region
pos_con.weight = 1 #weight of 1

#orientation constraint, maintains end effector to be vertical
ori_con = moveit_msgs.msg.OrientationConstraint() #create orientation_constraint
ori_con.header = pose_goal.header #same header 
ori_con.link_name = "tool_link_ee" #for EE link
ori_con.orientation = move_group.get_current_pose("tool_link_ee").pose.orientation #set as current orientation
ori_con.absolute_x_axis_tolerance = 3.14 #ignore this axis for constraints#
ori_con.absolute_y_axis_tolerance = 0.1
ori_con.absolute_z_axis_tolerance = 0.1 
ori_con.weight = 0.9 #set weight slightly less

main.position_constraints.append(pos_con) #add to position constraints
main.orientation_constraints.append(ori_con) #add to orientation constraints
move_group.set_path_constraints(main) #set path constraints

move_group.set_start_state_to_current_state() #move to goal
move_group.set_pose_target(pose_goal, "tool_link_ee") #move to goal with constraints

plan1 = move_group.plan()
plan1_points = plan1.joint_trajectory.points #obtain the joint poses generated along path

state = robot.get_current_state() #get current state of robot
waypoints = [] #generate empty waypoints list
wpose = move_group.get_current_pose().pose #get current position
i = 1

while i < len(plan1_points):
    state.joint_state.position = plan1_points[i].positions #obtain joint state
    test = GetPositionFKRequest()  #service to find position based on forward kinematics
    test.header.frame_id = "world" 
    test.fk_link_names.append("tool_link_ee")
    test.robot_state = state #set test.robot_state to be the current state

    rospy.wait_for_service("compute_fk") #wait for service to be online
    fk_service = rospy.ServiceProxy("compute_fk", GetPositionFK)
    resp = fk_service(test) #sends the test file to get a response (resp)

    wpose.position.x = resp.pose_stamped[0].pose.position.x #append pose message to waypoints list
    wpose.position.y = resp.pose_stamped[0].pose.position.y
    wpose.position.z = resp.pose_stamped[0].pose.position.z

    waypoints.append(copy.deepcopy(wpose))

    i+=1

move_group.set_start_state_to_current_state() #sets start state to current state, just in case anything wonky happened
plan2, fraction = move_group.compute_cartesian_path(waypoints, 0.001, 0.0, avoid_collisions = False) #use move_cartesian_path to interpolate trajectory between points, from moveit tutorials
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan2) #show the new plan
#display_trajectory_publisher.publish(display_trajectory)


move_group.execute(plan2, wait=True) #execute the new plan
'''
#additional point for flat
time.sleep(1.5)
pose_goal.pose.position.x = 0.01 #end position is 0.3, 0.4, change this to alter the goal
pose_goal.pose.position.y = 0.393
move_group.set_start_state_to_current_state() #move to goal
move_group.set_pose_target(pose_goal, "tool_link_ee") #move to goal with constraints

plan1 = move_group.plan()
plan1_points = plan1.joint_trajectory.points #obtain the joint poses generated along path

state = robot.get_current_state() #get current state of robot
waypoints = [] #generate empty waypoints list
wpose = move_group.get_current_pose().pose #get current position
i = 1

while i < len(plan1_points):
    state.joint_state.position = plan1_points[i].positions #obtain joint state
    test = GetPositionFKRequest()  #service to find position based on forward kinematics
    test.header.frame_id = "world" 
    test.fk_link_names.append("tool_link_ee")
    test.robot_state = state #set test.robot_state to be the current state

    rospy.wait_for_service("compute_fk") #wait for service to be online
    fk_service = rospy.ServiceProxy("compute_fk", GetPositionFK)
    resp = fk_service(test) #sends the test file to get a response (resp)

    wpose.position.x = resp.pose_stamped[0].pose.position.x #append pose message to waypoints list
    wpose.position.y = resp.pose_stamped[0].pose.position.y
    wpose.position.z = resp.pose_stamped[0].pose.position.z

    waypoints.append(copy.deepcopy(wpose))

    i+=1

move_group.set_start_state_to_current_state() #sets start state to current state, just in case anything wonky happened
plan2, fraction = move_group.compute_cartesian_path(waypoints, 0.001, 0.0, avoid_collisions = False) #use move_cartesian_path to interpolate trajectory between points, from moveit tutorials
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan2) #show the new plan
#display_trajectory_publisher.publish(display_trajectory)
move_group.execute(plan2, wait=True) #execute the new plan
#additional point for flat
time.sleep(1.5)
pose_goal.pose.position.x = 0.05 #end position is 0.3, 0.4, change this to alter the goal
pose_goal.pose.position.y = 0.262
move_group.set_start_state_to_current_state() #move to goal
move_group.set_pose_target(pose_goal, "tool_link_ee") #move to goal with constraints

plan1 = move_group.plan()
plan1_points = plan1.joint_trajectory.points #obtain the joint poses generated along path

state = robot.get_current_state() #get current state of robot
waypoints = [] #generate empty waypoints list
wpose = move_group.get_current_pose().pose #get current position
i = 1

while i < len(plan1_points):
    state.joint_state.position = plan1_points[i].positions #obtain joint state
    test = GetPositionFKRequest()  #service to find position based on forward kinematics
    test.header.frame_id = "world" 
    test.fk_link_names.append("tool_link_ee")
    test.robot_state = state #set test.robot_state to be the current state

    rospy.wait_for_service("compute_fk") #wait for service to be online
    fk_service = rospy.ServiceProxy("compute_fk", GetPositionFK)
    resp = fk_service(test) #sends the test file to get a response (resp)

    wpose.position.x = resp.pose_stamped[0].pose.position.x #append pose message to waypoints list
    wpose.position.y = resp.pose_stamped[0].pose.position.y
    wpose.position.z = resp.pose_stamped[0].pose.position.z

    waypoints.append(copy.deepcopy(wpose))

    i+=1

move_group.set_start_state_to_current_state() #sets start state to current state, just in case anything wonky happened
plan2, fraction = move_group.compute_cartesian_path(waypoints, 0.001, 0.0, avoid_collisions = False) #use move_cartesian_path to interpolate trajectory between points, from moveit tutorials
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan2) #show the new plan
#display_trajectory_publisher.publish(display_trajectory)
move_group.execute(plan2, wait=True) #execute the new plan

time.sleep(1.5)
pose_goal.pose.position.x = 0.15 #end position is 0.3, 0.4, change this to alter the goal
pose_goal.pose.position.y = 0.148
move_group.set_start_state_to_current_state() #move to goal
move_group.set_pose_target(pose_goal, "tool_link_ee") #move to goal with constraints

plan1 = move_group.plan()
plan1_points = plan1.joint_trajectory.points #obtain the joint poses generated along path

state = robot.get_current_state() #get current state of robot
waypoints = [] #generate empty waypoints list
wpose = move_group.get_current_pose().pose #get current position
i = 1

while i < len(plan1_points):
    state.joint_state.position = plan1_points[i].positions #obtain joint state
    test = GetPositionFKRequest()  #service to find position based on forward kinematics
    test.header.frame_id = "world" 
    test.fk_link_names.append("tool_link_ee")
    test.robot_state = state #set test.robot_state to be the current state

    rospy.wait_for_service("compute_fk") #wait for service to be online
    fk_service = rospy.ServiceProxy("compute_fk", GetPositionFK)
    resp = fk_service(test) #sends the test file to get a response (resp)

    wpose.position.x = resp.pose_stamped[0].pose.position.x #append pose message to waypoints list
    wpose.position.y = resp.pose_stamped[0].pose.position.y
    wpose.position.z = resp.pose_stamped[0].pose.position.z

    waypoints.append(copy.deepcopy(wpose))

    i+=1

move_group.set_start_state_to_current_state() #sets start state to current state, just in case anything wonky happened
plan2, fraction = move_group.compute_cartesian_path(waypoints, 0.001, 0.0, avoid_collisions = False) #use move_cartesian_path to interpolate trajectory between points, from moveit tutorials
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan2) #show the new plan
move_group.execute(plan2, wait=True) #execute the new plan

time.sleep(1.5)
pose_goal.pose.position.x = 0.3 #end position is 0.3, 0.4, change this to alter the goal
pose_goal.pose.position.y = 0.4
move_group.set_start_state_to_current_state() #move to goal
move_group.set_pose_target(pose_goal, "tool_link_ee") #move to goal with constraints

plan1 = move_group.plan()
plan1_points = plan1.joint_trajectory.points #obtain the joint poses generated along path

state = robot.get_current_state() #get current state of robot
waypoints = [] #generate empty waypoints list
wpose = move_group.get_current_pose().pose #get current position
i = 1

while i < len(plan1_points):
    state.joint_state.position = plan1_points[i].positions #obtain joint state
    test = GetPositionFKRequest()  #service to find position based on forward kinematics
    test.header.frame_id = "world" 
    test.fk_link_names.append("tool_link_ee")
    test.robot_state = state #set test.robot_state to be the current state

    rospy.wait_for_service("compute_fk") #wait for service to be online
    fk_service = rospy.ServiceProxy("compute_fk", GetPositionFK)
    resp = fk_service(test) #sends the test file to get a response (resp)

    wpose.position.x = resp.pose_stamped[0].pose.position.x #append pose message to waypoints list
    wpose.position.y = resp.pose_stamped[0].pose.position.y
    wpose.position.z = resp.pose_stamped[0].pose.position.z

    waypoints.append(copy.deepcopy(wpose))

    i+=1

move_group.set_start_state_to_current_state() #sets start state to current state, just in case anything wonky happened
plan2, fraction = move_group.compute_cartesian_path(waypoints, 0.001, 0.0, avoid_collisions = False) #use move_cartesian_path to interpolate trajectory between points, from moveit tutorials
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan2) #show the new plan
move_group.execute(plan2, wait=True) #execute the new plan
'''
rogbag_kill(demo_id)

