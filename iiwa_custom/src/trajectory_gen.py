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
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy import interpolate
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
import time
import shape_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import rosbag
from moveit_msgs.msg import RobotState
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy import interpolate
from scipy.signal import butter, filtfilt,lfilter
from scipy import interpolate as itp
from matplotlib import patches as patches
import os
import shutil
from shutil import copyfile
from matplotlib.patches import Rectangle
from scipy import interpolate
from scipy.signal import butter, filtfilt,lfilter
from scipy import interpolate as itp
from matplotlib import patches as patches
import math

## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class trajectory(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self, groupname):
    super(trajectory, self).__init__()
    counter = 0
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('move_group_python_interface_tutorial',
                    #anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    #add marker
    marker_pub = rospy.Publisher('visualization_marker',Marker,queue_size = 10)
    '''
    rospy.sleep(2)
    box = PoseStamped()
    box.header.frame_id = robot.get_planning_frame()
    box.pose.position.x = -0.06
    box.pose.position.y = 0.17
    box.pose.position.z = -1.306+0.04
    scene.add_box("obstacle1", box, (0.12,0.3,0.08))
    box.pose.position.x = 0.17
    box.pose.position.y = 0.36
    scene.add_box("obstacle2", box, (0.12,0.3,0.08))
    '''
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = groupname
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planning_time(10)
    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.counter = counter
    self.marker_pub = marker_pub
    #self.eff_link = "iiwa_link_ee"
    self.eff_link = "tool_link_ee"
    self.goal_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)

  def go_to_joint_state(self, joints):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = joints[0]#0
    joint_goal[1] = joints[1]#-pi/4
    joint_goal[2] = joints[2]#0
    joint_goal[3] = joints[3]#-pi/2
    joint_goal[4] = joints[4]#0
    joint_goal[5] = joints[5]#pi/3
    joint_goal[6] = joints[6]#0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.set_start_state_to_current_state()
    group.set_joint_value_target(joint_goal)
    group.go(wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()
    group.clear_pose_targets()
    #initwpose = group.get_current_joint_values()
    initwpose = group.get_current_pose().pose
    print("initial pose is: !!!!!++++++++++ " + str(initwpose))

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  def remove_white_board(self):
    self.scene.remove_world_object("white_board")
  def add_white_board(self):
    base_plane = PoseStamped()
    base_plane.header.frame_id = 'world'
    #plane for whiteboard
    base_plane.pose.position.x = 0.0
    base_plane.pose.position.y = 0.0
    base_plane.pose.position.z = 0.405-0.05#-1.306+0.4+1.306 - 0.16
    self.scene.add_box("white_board", base_plane, (1,1,0.08))
  def go_to_pose_goal(self, data):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    group.clear_path_constraints()
    #self.add_white_board()
    #starting_joint = [2.688, -1.1889, 2.518, 1.649776, -0.8901, 0.7806, -0.601]
    #starting_joint = [2.7042, -1.2866, 2.47179, 1.67686, -1.0, 0.79564, -0.552423]
    #self.go_to_joint_state(starting_joint)
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    #group.set_goal_orientation_tolerance(0.01)
    #group.set_goal_position_tolerance(0.01)
    #pose_goal = geometry_msgs.msg.Pose()
    group.clear_path_constraints()
    q = quaternion_from_euler(np.deg2rad(data[3]), np.deg2rad(data[4]), np.deg2rad(data[5]))
    pose_goal = group.get_current_pose(self.eff_link)
    pose_goal.pose.orientation = Quaternion(*q)
    #pose_goal.orientation.w = 1.0
    pose_goal.pose.position.x = data[0]
    pose_goal.pose.position.y = data[1]
    pose_goal.pose.position.z = data[2]
    group.set_start_state_to_current_state()
    group.set_pose_target(pose_goal,self.eff_link)
    #self.goal_pub.publish(pose_goal)
    group.set_planning_time(40)
    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    #self.goal_pub.publish(pose_goal)

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal.pose, current_pose, 0.01)
  def show_marker(self,pos_, frame_id):# lifetime_):
    #global marker_pub 
    print("showing marker in rviz!")
    marker_ = Marker()
    marker_.header.frame_id = "world"#self.group.get_current_pose("iiwa_link_ee").header.frame_id
    #marker_.id = 100
    marker_.header.stamp = rospy.Time.now()
    marker_.type = marker_.CUBE
    marker_.action = marker_.ADD

    marker_.pose.position.x = pos_[0]
    marker_.pose.position.y = pos_[1]
    marker_.pose.position.z = pos_[2]
    q = quaternion_from_euler(np.deg2rad(0.0), np.deg2rad(0.0), np.deg2rad(0.0))
    #marker_.pose.orientation = Quaternion(*q)
    marker_.pose.orientation.x = q[1]
    marker_.pose.orientation.y = q[2]
    marker_.pose.orientation.z = q[3]
    marker_.pose.orientation.w = q[0]

    marker_.lifetime = rospy.Duration.from_sec(30)
    marker_.scale.x = 0.725
    marker_.scale.y = 0.455
    marker_.scale.z = 0.1
    marker_.color.a = 0.5
    #red_, green_, blue_ = color_
    marker_.color.r = 0.3
    marker_.color.g = 0.9
    marker_.color.b = 0.2
    self.marker_pub.publish(marker_)
  def path_planner(self, data, box, box1, box_dim, cons_en, first_point, joint_state):
    group = self.group
    group.clear_path_constraints()
    #group.set_planner_id("RRTConnectkConfigDefault")
    #group.clearPathConstraints()
    group.set_planning_time(150)
    group.set_planner_id("RRTConnectkConfigDefaultNoInterpolate")
    #group.set_planner_id("RRTkConfigDefault")
    
    
    #shape = shape_msgs.msg.SolidPrimitive() #create primitives message
    
    #shape.type = 1 #cube
    #bound_len = 0.725
    #bound_wd = abs(box.pose.position.y - box_dim[1]/2.0 - (box1.pose.position.y+box_dim[1]/2.0)) + 0.1
    #shape.dimensions = [bound_len + 0.1, bound_wd, 0.5] #xyz of cube
    waypoints = [] #empty waypoints list
    planner_points = []
    for i in range(0, len(data), 6):
      #data[i+2] = data[i+2] + 1.306
      #pose goal set
      pose_goal = group.get_current_pose(self.eff_link)
      pose_goal.pose.position.x = data[i] #set goal position
      pose_goal.pose.position.y = data[i+1] 
      pose_goal.pose.position.z = data[i+2] 
      q = quaternion_from_euler(np.deg2rad(data[i+3]), np.deg2rad(data[i+4]), np.deg2rad(data[i+5]))
      #pose_goal.pose.orientation = Quaternion(*q)
      print("pose goal is now: " + str(pose_goal.pose.orientation))
      #pos_con.header = pose_goal.header #same as goal header
      '''
      shapePose = geometry_msgs.msg.Pose() #create Pose msg
      shapePose.position.x = 0
      shapePose.position.y = (bound_wd-0.1)/2.0 + (box.pose.position.y-box_dim[1]/2.0)
      shapePose.position.z = data[i+2]
      BoundVol = moveit_msgs.msg.BoundingVolume() #create BoundingVolume message
      BoundVol.primitives.append(shape) #add shape info
      BoundVol.primitive_poses.append(shapePose) #add shape pose info
      #adding path constraints
      pos_con = moveit_msgs.msg.PositionConstraint() #create position_constraint message
      pos_con.link_name = "iiwa_link_ee" #specify end effector link
      #pos_con.target_point_offset.x = 0
      #pos_con.target_point_offset.y = 0
      #pos_con.target_point_offset.z = data[i+2] #box1.pose.position.z #- box_dim[2]/2.0
      pos_con.constraint_region = BoundVol #add BoundVol to constraint_region
      pos_con.weight = 1 #weight of 1
      '''
      #copying stuff
      pos_con = moveit_msgs.msg.PositionConstraint() #create position_constraint message
      pos_con.header = pose_goal.header #copy current header
      pos_con.link_name = self.eff_link #specify end effector link
      pos_con.target_point_offset.x = 0
      pos_con.target_point_offset.y = 0
      pos_con.target_point_offset.z = 0 #point is at the tip of the endeffector
      shape = shape_msgs.msg.SolidPrimitive() #create primitives message
      shape.type = 1 #cube
      if cons_en == False:
      	shape.dimensions = [0.825, 0.7, 0.3] #xyz of cube
      else:
        shape.dimensions = [0.725, 0.45, 0.01] #xyz of cube
      shapePose = geometry_msgs.msg.Pose() #create Pose msg
      shapePose.position.x = 0.0 - 0.225
      shapePose.position.y = 0.265-0.05
      shapePose.position.z = pose_goal.pose.position.z #have it at current z
      BoundVol = moveit_msgs.msg.BoundingVolume() #create BoundingVolume message
      BoundVol.primitives.append(shape) #add shape info
      BoundVol.primitive_poses.append(shapePose) #add shape pose info
      pos_con.constraint_region = BoundVol #add BoundVol to constraint_region
      pos_con.weight = 1 #weight of 1
      #copying stuff
      

      #orientation constraint, maintains end effector to be vertical
      ori_con = moveit_msgs.msg.OrientationConstraint() #create orientation_constraint
      ori_con.link_name = self.eff_link #for EE link
      if cons_en == True:
        ori_con.absolute_x_axis_tolerance = 3.14 #ignore this axis for constraints#
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
      else:
        ori_con.absolute_x_axis_tolerance = 3.14 #ignore this axis for constraints#
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
      ori_con.weight = 0.9 #set weight slightly less
      ori_con.header = pose_goal.header
      #ori_con.header.frame_id = 'iiwa_link_0'
      #ori_con.orientation = pose_goal.pose.orientation
      ori_con.orientation = group.get_current_pose(self.eff_link).pose.orientation
      
      
      #constraint message
      constraint = moveit_msgs.msg.Constraints()
      constraint.name = "straight_down"
      constraint.position_constraints.append(pos_con)
      if 1:
      	constraint.orientation_constraints.append(ori_con) #adding orientation constarint to overal constraint
      group.set_path_constraints(constraint) #set path overall constraints
      group.set_goal_orientation_tolerance(0.0001)
      group.set_goal_position_tolerance(0.0001)
      #change it to end of the last planned state
      if first_point == True:
          print("THIS IS first point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
          group.set_start_state_to_current_state() #move to goal
      else:
          print("not first point at all!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
          joints = JointState()
          joints.header = Header()
          joints.header.stamp = rospy.Time.now()
          joints.position = joint_state.joint_trajectory.points[-1].positions
          joints.name = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6","iiwa_joint_7" ]
          moveit_robot_state = RobotState()
          moveit_robot_state.joint_state = joints
          group.set_start_state(moveit_robot_state)
      group.set_pose_target(pose_goal, self.eff_link) #move to goal with constraints
      attempts = 1
      planner = group.plan()
      self.display_trajectory(planner)
      
      while len(planner.joint_trajectory.points) == 0 and attempts < 5:
      	planner = group.plan()
	print("planning for " + str(attempts) + "'th time")
        attempts += 1
      #planner_points = []
      planner_points = planner.joint_trajectory.points
      #planner_points.append(planner.joint_trajectory.points) #obtain the joint poses generated along path
      
      state = self.robot.get_current_state() #get current state of robot
      
      wpose = group.get_current_pose("tool_link_ee").pose #get current position
      #print("true orientation is: " + str(ori_con.orientation))
      #time_stamp = 0
      waypoints = []
      '''
      for j in range(len(planner_points[i/6])):
        if j > 0:
		state.joint_state.position = planner_points[i][j].positions #obtain joint state
		FK = GetPositionFKRequest()  #service to find position based on forward kinematics
		FK.header.frame_id = "world" 
		FK.fk_link_names.append(self.eff_link)
		FK.robot_state = state #set FK.robot_state to be the current state

		rospy.wait_for_service("compute_fk")
		fk_service = rospy.ServiceProxy("compute_fk", GetPositionFK)
		resp = fk_service(FK) #sends the FK file to get a response (resp)

		#new_position = self.points_post_processing(resp.pose_stamped[-1].pose.position, box, box1, box_dim)
		wpose.position.x = resp.pose_stamped[-1].pose.position.x
		wpose.position.y = resp.pose_stamped[-1].pose.position.y
		wpose.position.z = resp.pose_stamped[-1].pose.position.z
		#wpose.orientation = Quaternion(*q)
		waypoints.append(copy.deepcopy(wpose))
    '''
    i = 1

    while i < len(planner_points):
	    state.joint_state.position = planner_points[i].positions #obtain joint state
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
    print("waypoints are: " + str(waypoints))
    #print("bound_wd is: " + str(bound_wd))
    return waypoints, planner
    

  def plan_cartesian_path(self,data, box, box1, box_dim, cons_en,scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group
    self.counter += 1
    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    spline_plan = False
    log_data = False
    waypoints = []
    waypoints_all = []

    wpose = group.get_current_pose(self.eff_link).pose
    
    #x_sp = []
    #y_sp = []
    #if len(y_sp) == 1 or spline_plan == False:
    joint_state = JointState()
    first_point = True
    for i in range(0,len(data), 6):
        time.sleep(1)
        print("i is: " + str(i) + " ###########################################")
        if (i == 0):
            first_point = True
            waypoints, planner = self.path_planner(data[i:i+6], box, box1, box_dim,cons_en, first_point, joint_state)
        else:
            first_point = False
            waypoints, planner = self.path_planner(data[i:i+6], box, box1, box_dim,cons_en, first_point, planner)
        for x in waypoints:
            waypoints_all.append(x)
    # We want the Cartesian path to be interpolated at a resolution of 0.1 cm
    # which is why we will specify 0.001 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    #plan = None
    attempt = 0
    fraction = 0.0
    group.set_start_state_to_current_state() #move to goal
    while attempt <= 10 and fraction < 0.95:
    	(plan, fraction) = group.compute_cartesian_path(
                                       	waypoints_all,   # waypoints to follow
                                       	0.001,        # eef_step
                                       	0.0,avoid_collisions = False)         # jump_threshold
        print("fraction is: " + str(fraction))
        attempt += 1

    #get cartesian trajectory with FK, unless it is the first point
    if log_data == False:
        self.execute_plan(plan)
        return plan, fraction
    data_length = 29 #[time_secs, time_nsecs, cartisianx, cartisiany, cartisianvx,  cartisianvy,  cartisianaccx,  cartisianaccy, joint1pos-joint7pos, joint1vel-joint7vel, joint1acc-joint7acc]
    matrix_to_write = np.zeros((len(plan.joint_trajectory.points), data_length))
    '''
    for i in range(len(plan.joint_trajectory.points)):
        request = GetPositionFKRequest()
        request.header.frame_id = 'world'
        request.fk_link_names = [self.eff_link]
        request.header.stamp = rospy.Time.now()
        request.robot_state.joint_state.name = plan.joint_trajectory.joint_names
	#get the fk for position
	fk_resultpos = self.get_fk(request, plan.joint_trajectory.points[i].positions)
	#get the fk for velocity
        fk_resultvel = self.get_fk(request, plan.joint_trajectory.points[i].velocities)
	#get the fk for accelerations
	fk_resultacc = self.get_fk(request, plan.joint_trajectory.points[i].accelerations)
	#write to the matrix
	#time
	matrix_to_write[i,0] = plan.joint_trajectory.points[i].time_from_start.secs
	matrix_to_write[i,1] = plan.joint_trajectory.points[i].time_from_start.secs
	#cartisian trajectory
	matrix_to_write[i,2] = fk_resultpos.pose_stamped[-1].pose.position.x
	matrix_to_write[i,3] = fk_resultpos.pose_stamped[-1].pose.position.y
	matrix_to_write[i,4] = fk_resultvel.pose_stamped[-1].pose.position.x
	matrix_to_write[i,5] = fk_resultvel.pose_stamped[-1].pose.position.y
	matrix_to_write[i,6] = fk_resultacc.pose_stamped[-1].pose.position.x
	matrix_to_write[i,7] = fk_resultacc.pose_stamped[-1].pose.position.y
	#joint trajectory
	for j in range(0,7):
	    #print("i is: " + str(i) + " and j is: " + str(j))
            #print("matrix is: " + str(matrix_to_write))
	    #print("points are: " + str(plan.joint_trajectory.points))
	    matrix_to_write[i,j+8] = plan.joint_trajectory.points[i].positions[j]
	    matrix_to_write[i,j+15] = plan.joint_trajectory.points[i].velocities[j]
	    matrix_to_write[i,j+22] = plan.joint_trajectory.points[i].accelerations[j]
    #save the planned joint trajectory and cartsian path to .npy file
    timestr = time.strftime("date_%Y-%m-%d_time-%H-%M-%S")
    with open('trajectory_trial-'+timestr+'.npy','a+') as f:
        np.save(f,matrix_to_write)
    '''
    self.execute_plan(plan)
    #self.show_marker([0.0,0.265,box.pose.position.z], "world")
    return plan, fraction


  def get_fk(self, request, plan_points):
    #get the fk for position/velocity/acceleration
    request.robot_state.joint_state.position = plan_points #plan.joint_trajectory.points[i].positions
    rospy.wait_for_service("compute_fk")
    compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
    try:
      fk_result = compute_fk(request)
    except:
      print("ERROR: FK failed")
    return fk_result


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
  
  def select_points(self, num_points, start, end):
      append_list = []
      for i in range(num_points):
          append_list.append(int(math.floor((end-start)/num_points))*i+start)
      return append_list

  def convert_npy(self, demo_num, folder_name, reverse = False):
	#print("Enter the file number: ") 
	#demo_num = input() #input the file number

	filename = '/home/camilo/custom_ws/raw_demo' + str(demo_num) + '.bag'
        

	bag = rosbag.Bag(filename) #import the bag file

	initial_pose = Pose() #initiate empty pose messages
	final_pose = Pose()

	i = 1
	first = 1
	last = bag.get_message_count()
	initial_time = 0

	for topic, msg, t in bag.read_messages(topics='/ee_pose'): #find the initial and final pose
	    if i==1:
		initial_pose = msg
	    elif i==bag.get_message_count():
		final_pose = msg
	    i += 1
	i = 1

	for topic, msg, t in bag.read_messages(topics='/ee_pose'): #find the beginning and end msg numbers
	    if msg == initial_pose:
		first = i
		initial_time = t
	    if msg != final_pose:
		last = i
	    i += 1
	i=1
	last += 1

	bag.close() #close the bag file

	num_msgs = last-first #determine number of unique messages in bag file

	with rosbag.Bag('/home/camilo/custom_ws/traj_demos/demo' + str(demo_num) + '.bag', 'w') as outbag: #export trimmed bag file
	    for topic, msg, t in rosbag.Bag(filename).read_messages():
		if i >= first and i <= last:
		    t -= initial_time
		    outbag.write(topic, msg, t)
		i+=1

	bag = rosbag.Bag('/home/camilo/custom_ws/traj_demos/demo' + str(demo_num) + '.bag') #import trimmed bag fiile

	x = list()
	y = list()
	z = list() #should be t

	i=0
	for topic, msg, t in bag.read_messages(topics='/ee_pose'): #take the xyz (z is time here) from new bag file and add to list
	    x.append(msg.position.x)
	    y.append(msg.position.y)
	    z.append(i)
	    print(t.to_sec())
	    print(msg)
	    i += 1

	print(z)
        print("duration is: ", len(z))
	duration = z[len(z)-1] #linear interpolation to create 100 points, with time being 10 seconds as required for the TPGMM
        
        duration_num = 100
	new_t = np.linspace(0,duration,num = duration_num)
	new_x = np.interp(new_t, z, x)
	new_y = np.interp(new_t, z, y)
	z = np.linspace(0,10,num=duration_num)

	data = np.row_stack((z, new_x, new_y)) #add the lists together to create a numpy array
	data = np.transpose(data) #tranpose the data so it conforms to TPGMM format
       
        ######################pchip interpolation###########################
        plt.figure("original data")
	plt.plot(data[:,1], data[:,2], label = "original")

        demo_x = data[:,1]
        demo_y = data[:,2]
        new_demox = np.zeros(len(demo_x))
	new_demoy = np.zeros(len(demo_y))
	for i in range(0, len(demo_x)):
	    new_demox[i] = -100
	    new_demoy[i] = -100
	new_demox[0] = demo_x[0]
	new_demoy[0] = demo_y[0]
	count_new = 1
	#get rid of duplicate points
	for i in range(1, len(demo_x)):
	    if demo_x[i] != new_demox[count_new-1]:

		new_demox[count_new] = demo_x[i]
		new_demoy[count_new] = demo_y[i]
		count_new += 1

	new_demox = new_demox[new_demox!=-100]
	new_demoy = new_demoy[new_demoy!=-100]

	'''
	pchip interpolation
	'''
	t_observed = np.linspace(0, 10, len(new_demox))
	t = np.linspace(0, 10, 100)


	#getting rid of clustering near sharp angle
	#1: find maxima or minima?
	#2: get rid of points next to the maxima and minima
	index_del = []
	too_close = 0.001
        interp_count = 2
	temp_storage = [[demo_x[0], demo_y[0]]]
	temp_index = []
	count_den = 0
	set_pos = []
	selected_points = []
	flag = [0,0,0,0]
	num_points = 8
        d_i = 1
        ''' Just temperory for using linear interpolation
	for j in range(1, len(new_demox)-1):
	    if (new_demox[j] <= 0.25 and abs(new_demox[j] - new_demox[j-d_i]) < too_close) or (new_demoy[j] > 0.25 and new_demox[j] > 0.02) or (reverse == False and new_demox[j] < new_demox[j-1] and new_demoy[j] > 0.3 and new_demox[j] > -0.57 and new_demox[j] < -0.25):
		index_del.append(j)
                interp_count +=1
                d_i += 1
            else:
                d_i = 1
            if (reverse == True and ((new_demox[j] <= data[-1,1] + 0.025 and new_demoy[j]<= data[-1,2] + 0.025) or (new_demox[j] <= data[-1,1] and new_demox[j] >= data[-1,1] - 0.025 and new_demoy[j] <= data[-1,2] and new_demoy[j] >= data[-1,2] - 0.025))):
                if (not(j in index_del)):
                    index_del.append(j)
                    d_i += 1
        #modified for keypoints
        print("interp_count is: ", interp_count)
	new_demox = np.delete(new_demox, index_del)
	new_demoy = np.delete(new_demoy, index_del)

	xyt = np.linspace(0,1,num = interp_count)
	dt = np.linspace(0,1,num = 2)
	new_x_add = np.interp(xyt, dt, np.array([new_demox[-2], new_demox[-1]]))
	new_y_add = np.interp(xyt, dt, np.array([new_demoy[-2], new_demoy[-1]]))
	print ("new_y_add is: ", new_y_add)
	new_demox = new_demox[:len(new_demox)-2]
	new_demoy = new_demoy[:len(new_demoy)-2]
	for i in range(len(new_x_add)):
	    new_demox = np.append(new_demox, new_x_add[i])
	    new_demoy = np.append(new_demoy, new_y_add[i])

	print("length of new_demoy is now: ", len(new_demoy)-1)
	print("length of new_demox is now: ", len(new_demox)-1)
	for j in range(1, len(new_demox)):
	    if new_demox[j]>=(-0.13-0.0075-0.225) and flag[0] == 0:
		flag[0] = 1
		temp_list = self.select_points(num_points, 0, j)
		for x in temp_list:
		    set_pos.append(x)
		temp_list = []
		set_pos.append(j)
	    if new_demox[j]>=(-0.01+0.0075-0.225) and flag[1] == 0:
		flag[1] = 1
                print("num_points is: ", int(num_points/2))
		temp_list = self.select_points(int(num_points/2), set_pos[-1], j)
		for x in temp_list:
		    set_pos.append(x)
		temp_list = []
		set_pos.append(j)
	    if new_demox[j]>=(0.11-0.0075-0.225) and flag[2] == 0:
		flag[2] = 1
		temp_list = self.select_points(num_points, set_pos[-1], j)
		for x in temp_list:
		    set_pos.append(x)
		temp_list = []
		set_pos.append(j)
	    if new_demox[j]>=(0.23+0.0075-0.225) and flag[3] == 0:
		flag[3] = 1
		temp_list = self.select_points(int(num_points / 2), set_pos[-1], j)
		for x in temp_list:
		    set_pos.append(x)
		temp_list = []

	temp_list = self.select_points(int(num_points), set_pos[-1], len(new_demox)-1)
	print("previous set point is: ", set_pos, "length of new demox is: ", len(new_demox)-1, "length of new demoy is: ", len(new_demoy)-1)
	for x in temp_list:
	    set_pos.append(x)
	set_pos.append(len(new_demox)-1)
	print("final set point is: ", set_pos, "length of new demox is: ", len(new_demox)-1)
	set_posx = [new_demox[0]]
	set_posy = [new_demoy[0]]
	for i in range(len(set_pos)):
	    set_posx.append(new_demox[set_pos[i]])
	    set_posy.append(new_demoy[set_pos[i]])
	new_demox = set_posx
	new_demoy = set_posy
	t_observed = np.linspace(0, 10, len(new_demox))
	t = np.linspace(0, 10, 100)
	pchipx = itp.pchip_interpolate(t_observed, new_demox, t)
	pchipy = itp.pchip_interpolate(t_observed, new_demoy, t)
	print("length of pchipx is: ", len(pchipx))
	'''
	#temp for linear interp----------------------------------


	#temp for linear interp-------------------------------------
        '''
	array_save = np.zeros((100,3))
        array_save[:, 0] = t
        array_save[:,1] = pchipx
        array_save[:,2] = pchipy
        '''
        save_folder = '/home/camilo/custom_ws/kuka-iiwa-lfd/processed_path_demos/'
	np.save(save_folder + folder_name + 'demo' + str(demo_num) + '.npy', data)#array_save) #exports data as a .npy file

	plt.figure("pchip")
	plt.plot(demo_x, demo_y, label = "pchip")
        #plt.plot(pchipx, pchipy, 'x', label = "new demos")
        #################pchip interpolation#######################        

	

	print(first,last) #prints out the first and last numbers as a sanity check
	bag.close()
        plt.show()
