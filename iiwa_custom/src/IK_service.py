#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
import numpy as np
from numpy import linalg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from trajectory_gen import trajectory
from math import pi
import scipy
from scipy import interpolate
import time
from visualization_msgs.msg import Marker
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import subprocess, shlex, psutil
import os
import re
import glob
import shutil
from shutil import copy


input_folder_name = raw_input("Enter the name of the folder: ")
print("you entered: ", input_folder_name)
save_folder = '/home/camilo/custom_ws/kuka-iiwa-lfd/processed_path_demos/'

rospy.init_node('listener', anonymous=True)
#IKPub = rospy.Publisher('joint_IK_pub', JointState, queue_size=10)
JointPub = rospy.Publisher('joint_trajectory_pub', JointTrajectory, queue_size=10)
TPGMM_send = rospy.Publisher('TPGMM_directory', String, queue_size=10)
helper_respond = rospy.Publisher('helper_response', String, queue_size=10)

group_name = "manipulator"#"iiwa_arm"#"manipulator"
trajectory_planning = trajectory(group_name)
time.sleep(2)

print("moving to second part")
#starting_joint = [0.9317, 1.4424, -0.01034, 0.94336, 0.02147, -0.49905, -0.94844]
starting_joint = [0.9017, 1.6495, -0.0127, 1.155, 0.0267, -0.4947, -0.9254]
transfer_starting_joint = [2.197, 1.744, -0.0069, 1.6436, 0.0678, -0.1, -2.2658]
end_joint = [2.2707, 1.7057, -0.0178, 1.655, 0.339, -0.053, -2.612]
#trajectory_planning.go_to_joint_state(starting_joint)
group = MoveGroupCommander(group_name)

#rospy.init_node('joint_state_publisher')
learning_status = rospy.Publisher('learning_status', Float32MultiArray, queue_size = 10)
rate = rospy.Rate(10) # 10hz
#scene and robot
scene = PlanningSceneInterface()
robot = RobotCommander()
rospy.sleep(2)
box = PoseStamped()
box1 = PoseStamped()
base_plane = PoseStamped()
box_dim = [0.12,0.31,0.08]
#box_dim = [0.12,0.30,0.08]
cmdcount = 0
global_demo = 0
#global_record = False
rosbag_proc = None
command = None
command1 = None
shift = [-0.225, -0.05]
def add_obstacle():
    global box, box1,box_dim, scene, robot, base_plane
    box.header.frame_id = 'world'#robot.get_planning_frame()
    #box.pose.orientation.w = 1.0
    box.pose.position.x = -0.07 + shift[0]
    box.pose.position.y = 0.19 + shift[1]
    box.pose.position.z = 0.405+0.04+0.529-0.016
    scene.add_box("obstacle1", box, (box_dim[0],box_dim[1],box_dim[2]))
    box1.header.frame_id = 'world'
    #box1.pose.orientation.w = 1.0
    box1.pose.position.x = 0.17+ shift[0]
    box1.pose.position.y = 0.34 + shift[1]
    box1.pose.position.z = 0.405+0.04+0.529-0.016
    scene.add_box("obstacle2", box1, (box_dim[0],box_dim[1],box_dim[2]))
def add_white_board():
    global box, box1,box_dim, scene, robot, base_plane
    #plane for whiteboard
    base_plane.header.frame_id = 'world'
    base_plane.pose.position.x = 0.0
    base_plane.pose.position.y = 0.0
    base_plane.pose.position.z = 0.405+0.529-0.03
    scene.add_box("white_board", base_plane, (2,2,0.01))


def Path_compute(data, cons_en, plan_en):
    global JointPub, trajectory_planning, box, box1, box_dim, cmdcount
    print("path planning service being called, data is: " + str(data))
    if cons_en == False and plan_en == False:
        trajectory_planning.go_to_pose_goal(data)
        return
    plan, fraction = trajectory_planning.plan_cartesian_path(data, box, box1, box_dim, cons_en)
    print(plan.joint_trajectory)
    #trajectory_planning.go_to_joint_state(plan.joint_trajectory.points[-1].positions)
    #publish the result
    #print("plan length is: " + str(len(plan.joint_trajectory.points)))
    #JointPub.publish(plan.joint_trajectory)
def IK_compute(data):
    global JointPub, group_name
    #Wait for theIK service to become available
    rospy.wait_for_service('compute_ik')
    #rospy.init_node('service_query')
    #Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = group_name
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = data[0] #0.5
    request.ik_request.pose_stamped.pose.position.y = data[1] #0.5
    request.ik_request.pose_stamped.pose.position.z = data[2] #0.5

    q = quaternion_from_euler(np.deg2rad(data[3]), np.deg2rad(data[4]), np.deg2rad(data[5]))
    request.ik_request.pose_stamped.pose.orientation = Quaternion(*q) #0.0

    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    try:
        #Send the request to the service
        response = compute_ik(request)
        #response.solution.joint_state.velocity = [0, 0, 0, 0, 0, 0, 0]
        #response.solution.joint_state.effort = [0, 0, 0, 0, 0, 0, 0]
        rospy.loginfo(response)
        IKPub.publish(response.solution.joint_state)
        '''
        if response.error_code == 1:
            print("success")
        elif response.error_code == -31:
            print("NO IK solution")
        '''
    except  rospy.ServiceException, e:
        print "Service callfailed: %s"%e

def rosbag_record(demo_num):
    global command, command1, rosbag_proc
    for filename in glob.glob("/home/camilo/custom_ws/raw_demo" + str(demo_num) +"*"):
        os.remove(filename) 
    command = "rosbag record -O raw_demo" + str(demo_num) +" /ee_pose" 
    command = shlex.split(command)
    rosbag_proc = subprocess.Popen(command)
    print("rosbag recording into raw_demo" + str(demo_num))
    #record the joint states to a seperate file
    command1 = "rosbag record -O raw_demo_joint" + str(demo_num) +" /iiwa/state/JointPosition" 
    command1 = shlex.split(command1)
    rosbag_proc = subprocess.Popen(command1)
    print("rosbag recording into raw_demo_joint" + str(demo_num))
def rogbag_kill(demo_num):
    global command, rosbag_proc, command1
    command = "rosbag record -O raw_demo" + str(demo_num) +" /ee_pose"
    command = shlex.split(command)
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
             proc.send_signal(subprocess.signal.SIGINT)
    rosbag_proc.send_signal(subprocess.signal.SIGINT)

    #stop recording for joint space
    command1 = "rosbag record -O raw_demo_joint" + str(demo_num) +" /iiwa/state/JointPosition" 
    command1 = shlex.split(command1)
    for proc in psutil.process_iter():
        if "record" in proc.name() and set(command1[2:]).issubset(proc.cmdline()):
             proc.send_signal(subprocess.signal.SIGINT)
    rosbag_proc.send_signal(subprocess.signal.SIGINT)
    print("rosbag process killed")
def callback(data):
    global cmdcount, starting_point
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    lst = str(data.data)#.split('" ')
    print("lst is: ", lst)
    #lst1 = lst[1].split('"')
    lst0 = lst[1:].split(" ")
    print("lst0 is: ", lst0)
    lst_new = map(float, lst0)
    lst_round = [round(num, 3) for num in lst_new]
    lst_round[2] = box.pose.position.z-0.03
    Path_compute([-0.531, 0.231, 0.933, 0.0, 270, 0.0], False, False)
    time.sleep(1)
    Path_compute([lst_round[0],lst_round[1], 0.933, 0.0, 270, 0.0], True, True)
    time.sleep(1)
    
def callback_joystick_homing(home):
    global starting_joint
    print("received homing")
    trajectory_planning.go_to_joint_state(starting_joint)
def callback_joystick_help(data):
    global global_demo, cmdcount, input_folder_name, box
    #print("data received for helper is: ", data.data.split())
    stop_list = data.data.split()
    folder_sub = input_folder_name.split()
    #end_joint
    if data.data == "delete":
        #easier_joint = [2.85, 1.604, -0.0123, 1.528, 0.1623,-0.0764, -3.0121]
        #trajectory_planning.go_to_joint_state(easier_joint)
        #helper_respond.publish("helper done")
        print("delete operation called")
        if global_demo == cmdcount:
            rogbag_kill(global_demo)
        else:
            remove_file(input_folder_name)
        set_first_index (input_folder_name, "/home/camilo/custom_ws/kuka-iiwa-lfd/processed_path_demos/")
    elif data.data == "save":
        global_demo = cmdcount
        rosbag_record(global_demo)
    elif data.data == "initiate":
        if (folder_sub[1] == "transfer"):
            Path_compute([0.075, 0.35, 1.1, 0.0, 270, 0.0], False, True)
            time.sleep(1)
            trajectory_planning.go_to_joint_state(transfer_starting_joint)
            time.sleep(1)
            Path_compute([0.075, 0.35, box.pose.position.z-0.025, 0.0, 270, 0.0], False, True)
            time.sleep(1)
        else:
            Path_compute([-0.239-0.225, 0.415-0.05, box.pose.position.z-0.025, 0.0, 270, 0.0], False, True)
            time.sleep(1)
    elif stop_list[0] == "stop":
        print("c_record mode, getting data to send to tpgmm")
        #Path_compute([0.075, 0.4-0.05, float(stop_list[1]), 0.0, 270, 0.0], True, True)
        #time.sleep(1)
        rogbag_kill(global_demo)
        folder_sub = input_folder_name.split()
        if folder_sub[1] == "entropy_point":
            check_folder(input_folder_name)
        elif folder_sub[1] == "heatmap":
            check_folder(input_folder_name)
        elif folder_sub[1] == "transfer":
            check_folder(input_folder_name)
        elif folder_sub[1] == "baseline":
            check_folder(input_folder_name)
        elif folder_sub[1] == "retention":
            check_folder(input_folder_name)
        time.sleep(1.5)
        if (folder_sub[1] == "transfer"):
            trajectory_planning.convert_npy(global_demo, folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/", True)
        elif (len(folder_sub)>3):
            trajectory_planning.convert_npy(global_demo, folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/" + folder_sub[3] + "/")
        else:
            trajectory_planning.convert_npy(global_demo, folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/")
        #copy the raw_demos to the designated location
        filename = '/home/camilo/custom_ws/raw_demo' + str(global_demo) + '.bag'
        copy(filename, save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos")
        print("File copied successfully.")
        filename = '/home/camilo/custom_ws/raw_demo_joint' + str(global_demo) + '.bag'
        copy(filename, save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos")
        print("File copied successfully.")
        helper_respond.publish("helper done")
        time.sleep(1)
        if (folder_sub[1] == "transfer"):
            Path_compute([-0.239-0.225, 0.415-0.05, box.pose.position.z-0.025, 0.0, 270, 0.0], True, True)
            time.sleep(1)
            Path_compute([-0.239-0.225, 0.415-0.05, 0.405+0.529+0.12, 0.0, 270, 0.0], False, True)
            time.sleep(1)
            trajectory_planning.go_to_joint_state(starting_joint)
            time.sleep(1)
            #trajectory_planning.go_to_joint_state(end_joint)
            #time.sleep(1)
        else:
            Path_compute([0.075, 0.4, 0.405+0.529+0.12, 0.0, 270, 0.0], False, False)
            time.sleep(1)
            trajectory_planning.go_to_joint_state(starting_joint)
            time.sleep(1)
        cmdcount+=1
    #Path_compute([0.28, 0.25, data.data[-1], 0.0, 270, 0.0], True, True)
def callback_joystick(data):
    #new_location1 information_entropy 2d
    global cmdcount, starting_point, shift
    demo_num = cmdcount
    cmdcount+=1
    print ("demo num is now: " + str(demo_num))
    lower = False
    lst_new = data.data
    lst_round = [round(num, 3) for num in lst_new]
    
    endpoint = [0.3+ shift[0], 0.4 + shift[1], box.pose.position.z-0.03, 0.0, 270, 0.0]
    for x in endpoint:
        lst_round.append(x)
    print("joystic data is: " + str(lst_round))
    #add to elevate for the current point
    folder_sub = input_folder_name.split()
    #if (len(folder_sub)<=3):
    Path_compute([0.3+ shift[0], 0.4 + shift[1], 0.405+0.529+0.12, 0.0, 270, 0.0], False, False)
    time.sleep(1.5)
    trajectory_planning.go_to_joint_state(starting_joint)
    time.sleep(1)
    Path_compute([-0.25+ shift[0], 0.2 + shift[1], 0.405+0.529+0.03, 0.0, 270, 0.0], False, False)
    time.sleep(1.5)
    
    if (round(lst_new[0],3) >= -0.23-0.015+ shift[0] and round(lst_new[0],3) <= -0.23+0.015+ shift[0] and round(lst_new[1],3) >= 0.055-0.015+ shift [0]and round(lst_new[1],3) <= 0.055+0.015+ shift[0]):
	lst_round[2] = box.pose.position.z-0.0335
        Path_compute(lst_round[0:6], False, False)
        lower = True
    else:
        lst_round[2] = box.pose.position.z-0.03
        Path_compute([-0.239+ shift[0], 0.115 + shift[1], box.pose.position.z-0.03, 0.0, 270, 0.0], False, False)
        time.sleep(1.5)
        Path_compute(lst_round[0:6], True, True)

    rospy.loginfo(rospy.get_caller_id() + "data to be sent for planning %s", lst_new)
    rosbag_record(demo_num) #record the rodbag file to be converted to a npy file for the learning rpocess
    
    for i in range(6, len(lst_round), 6):
        #time.sleep(1)
        if (lower == True):
	    lst_round[i+2] = box.pose.position.z-0.0335
        else:
	    lst_round[i+2] = box.pose.position.z-0.03
        
	#cmdcount += 1
    Path_compute(lst_round[6:], True, True)

    time.sleep(1)
    #if (lower == True):
    #    Path_compute([0.3, 0.4, box.pose.position.z-0.0335, 0.0, 270, 0.0], True, True)
    #else:
    #    Path_compute([0.3, 0.4, box.pose.position.z-0.03, 0.0, 270, 0.0], True, True)
    rogbag_kill(demo_num) # stop recording the rosbag file
    time.sleep(1)
    #Path_compute([0.3, 0.4, 0.405+0.529+0.03, 0.0, 270, 0.0], False, False)
    
    time.sleep(1)
    folder_sub = input_folder_name.split()
    if folder_sub[1] == "entropy_point":
            check_folder(input_folder_name + " entropy_point")
    elif folder_sub[1] == "heatmap":
            check_folder(input_folder_name + " heatmap")
    if (len(folder_sub)>3):
        trajectory_planning.convert_npy(demo_num, folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/" + folder_sub[3] + "/")
    else:
        trajectory_planning.convert_npy(demo_num, folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/")
    time.sleep(1.5)
    Path_compute([0.3+ shift[0], 0.4 + shift[1], 0.405+0.529+0.12, 0.0, 270, 0.0], False, False)
    time.sleep(1.5)
    trajectory_planning.go_to_joint_state(starting_joint)
    time.sleep(1)
def check_folder(folder_name):
       folder_sub = folder_name.split()
       print("folder_sub ", folder_sub)
       if os.path.isdir(save_folder + folder_sub[0]):
           if (os.path.isdir(save_folder + folder_sub[0] + "/" + folder_sub[1])):
               if len(folder_sub) <= 3 and not (os.path.isdir(save_folder + folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2])):
                   path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/", folder_sub[2])
                   os.mkdir(path)
               elif len(folder_sub) > 3:
                   if os.path.isdir(save_folder + folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2]):
                       if not (os.path.isdir(save_folder + folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/" + folder_sub[3])):
                           path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/"+ folder_sub[2]+"/"+folder_sub[3])
                           os.mkdir(path)
                   else:
                       path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/", folder_sub[2])
                       os.mkdir(path)
                       path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/"+ folder_sub[2]+"/"+folder_sub[3])
                       os.mkdir(path)
           else:
               path = os.path.join(save_folder + folder_sub[0] + "/", folder_sub[1])
               os.mkdir(path)
               path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/", folder_sub[2])
               os.mkdir(path)
               if len(folder_sub) > 3:
                   path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/"+ folder_sub[2]+"/"+folder_sub[3])
                   os.mkdir(path)
       else:
           path = os.path.join(save_folder, folder_sub[0])
           os.mkdir(path)
           path = os.path.join(save_folder + folder_sub[0] + "/", folder_sub[1])
           os.mkdir(path)
           path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/", folder_sub[2])
           os.mkdir(path)
           if len(folder_sub) > 3:
               path = os.path.join(save_folder + folder_sub[0] + "/"+ folder_sub[1] + "/"+ folder_sub[2]+"/"+folder_sub[3])
               os.mkdir(path)
       if len(folder_sub) <= 3:
           if not os.path.isdir(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos"):
               path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/", "raw_demos")
               os.mkdir(path)
               #path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/",folder_sub[2])
               #os.mkdir(path)
           #elif not os.path.isdir(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/" + folder_sub[2]):
           #    path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/",folder_sub[2])
           #    os.mkdir(path)
       else:
           if not os.path.isdir(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos"):
               path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/", "raw_demos")
               os.mkdir(path)
               #path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/",folder_sub[2])
               #os.mkdir(path)
               path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/",folder_sub[3])
               os.mkdir(path)
           #elif not os.path.isdir(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/" + folder_sub[2]):
           #    path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/",folder_sub[2])
           #    os.mkdir(path)
           #    path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/",folder_sub[2] + "/",folder_sub[3])
           #    os.mkdir(path)
           elif not os.path.isdir(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/" + folder_sub[3]):
               path = os.path.join(save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/",folder_sub[3])
               os.mkdir(path)
def remove_file (path):
    global global_demo
    folder_sub = path.split()
    if (len(folder_sub)>3):
        filePath = save_folder + folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/" + folder_sub[3] + "/" + "demo" + str(global_demo) + ".npy"
        filePath_raw = save_folder + folder_sub[0] + "/" + folder_sub[1]   + "/" + "raw_demos"+ "/" + folder_sub[3] + "/" +"raw_demo" + str(global_demo) + ".bag"
    else:
        filePath = save_folder + folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/" + "demo" + str(global_demo) + ".npy"
        filePath_raw = save_folder + folder_sub[0] + "/" + folder_sub[1]   + "/" + "raw_demos"+ "/" +"raw_demo" + str(global_demo) + ".bag"
    if os.path.exists(filePath):
        os.remove(filePath)
    else:
        print("Can not delete the demo file as it doesn't exists")          
    if os.path.exists(filePath_raw):
        os.remove(filePath_raw)
    else:
        print("Can not delete the raw demo file as it doesn't exists")  
def listener():
    rospy.Subscriber("IK_goal_bridge", String, callback)
    rospy.Subscriber("Joystick_data_pub", Float32MultiArray, callback_joystick)
    rospy.Subscriber("Joystick_homing", String, callback_joystick_homing)
    rospy.Subscriber("Joystick_help", String, callback_joystick_help)
    rospy.spin()

def test_points_reachability():
    row = 14
    col = 4
    increment = 0.03
    x_points = np.linspace(-0.36+increment/2, -0.215-increment/2, round(((0.36-increment/2)-(0.215+increment/2))/increment)) #x-grid generated
    y_points = np.linspace(0.04+increment/2, 0.49-increment/2, round(((0.49-increment/2)-(0.04+increment/2))/increment)) #y-grid generated
    print(len(x_points), "by", len(y_points))
    print("X is: " + str(x_points) + " Y is " + str(y_points) )
    #Path_compute([x_points[0], y_points[0], 0.405+0.529+0.03, 0.0, 270, 0.0], False, True)
    #time.sleep(1.5)
    #Path_compute([-0.25, 0.2, box.pose.position.z-0.03, 0.0, 270, 0.0], False, False)
    for r_i in range(row):
        for c_i in range(col):
            if (round(x_points[c_i],3) >= -0.23-0.015 and round(x_points[c_i],3) <= -0.23+0.015 and round(y_points[r_i], 3) >= 0.055-0.015 and round(y_points[r_i], 3) <= 0.055+0.015):
		z_pos = box.pose.position.z-0.0335
                Path_compute([round(x_points[c_i],3), round(y_points[r_i], 3), z_pos, 0.0, 270, 0.0], False, False)
            else:
        	z_pos = box.pose.position.z-0.03
                Path_compute([round(x_points[c_i],3), round(y_points[r_i], 3), z_pos, 0.0, 270, 0.0], True, True)
            
            time.sleep(1.5)

def get_directory(string):
    msg = string.split()
    #print("vis_mode is: ", settings.vis_mode, "msg is: ", msg)
    return msg

def get_all_filenames(path):
    files = os.listdir(path)
    mylist = [f for f in files]
    print("all files in " + path + " are: " + str(mylist))
    return mylist
def get_first_last_index(filelist):
    index_list = []
    for i in range(len(filelist)):
        index_list.append(int (re.findall(r"\d+",filelist[i])[-1]))
    print("index list is: ",index_list)
    if len(index_list) == 0:
        return -1, -1
    else:
        return min(index_list), max(index_list)
def set_first_index (input_folder_name, path_prefix):
    global global_demo, cmdcount
    msg_list = get_directory(input_folder_name)
    if (len(msg_list)>3):
        path = path_prefix + msg_list[0] + "/" + msg_list[1]+  "/" + msg_list[2]+  "/" + msg_list[3]
    else:
        path = path_prefix + msg_list[0] + "/" + msg_list[1]+  "/" + msg_list[2]
    filelist = get_all_filenames(path)
    minindex, maxindex = get_first_last_index(filelist)
    cmdcount = maxindex + 1
    global_demo = cmdcount

if __name__ == '__main__':
    #add obstacles
    # for simulation


    #location_test heatmap hololens 1
    #Name_test baseline learning_data
    #Name_test heatmap learning_data 1/2/3
    #newtest entropy_point learning_data 1/2/3
    #Name_test retention learning_data
    #newtest transfer learning_data
    check_folder(input_folder_name)
    #trajectory_planning.go_to_joint_state(starting_joint)
    #Path_compute([-0.239, 0.415, 0.405+0.04+0.529-0.016-0.03, 0.0, 270, 0.0], False, False)
    set_first_index (input_folder_name, "/home/camilo/custom_ws/kuka-iiwa-lfd/processed_path_demos/")
    add_obstacle() 
    add_white_board()
    time.sleep(1.5)
    demo_id = 406
    #Path_compute([-0.239-0.225, 0.415-0.05, box.pose.position.z-0.025, 0.0, 270, 0.0], False, True)
    '''IF NOT REACH ENDGOAL
    Path_compute([0.075, 0.35, box.pose.position.z-0.03, 0.0, 270, 0.0], True, True)
    time.sleep(1)
    Path_compute([0.075, 0.4, 0.405+0.529+0.12, 0.0, 270, 0.0], False, False)
    time.sleep(1)
    trajectory_planning.go_to_joint_state(starting_joint)
    '''
    #Path_compute([0.075, 0.4, 0.405+0.529+0.12, 0.0, 270, 0.0], False, False)
    #time.sleep(1)
    #trajectory_planning.go_to_joint_state(starting_joint)
    #time.sleep(1)
    
    
    #need to change everytime!!!!!!!!!!!!!!!!!!
    autodemo = 55
    start_pos = [-0.57, 0.425]
    #Path_compute([0.075, 0.35, box.pose.position.z-0.03, 0.0, 270, 0.0], True, True)
    #Path_compute([-0.239-0.225, 0.415-0.05, 1.0249, 0.0, 270, 0.0], True, True)
    #time.sleep(1)
    #Path_compute([-0.239-0.225, 0.415-0.05, 0.932, 0.0, 270, 0.0], False, False)
    #time.sleep(1)
    
    #Path_compute([-0.239-0.225, 0.415-0.05, 0.932, 0.0, 270, 0.0], False, True)
    #time.sleep(1)
    #Path_compute([start_pos[0], start_pos[1], 0.932, 0.0, 270, 0.0], True, True)
    #time.sleep(1)
    
    '''
    rosbag_record(autodemo)
    time.sleep(1)
    Path_compute([0.075, 0.35, box.pose.position.z-0.03, 0.0, 270, 0.0], True, True)
    time.sleep(1)
    
    #after execution
    rogbag_kill(autodemo)
    folder_sub = input_folder_name.split()
    if folder_sub[1] == "entropy_point":
        check_folder(input_folder_name + " entropy_point")
    elif folder_sub[1] == "heatmap":
        check_folder(input_folder_name + " heatmap")

    time.sleep(1.5)
    trajectory_planning.convert_npy(autodemo, folder_sub[0] + "/" + folder_sub[1] + "/" + folder_sub[2] + "/")
    #copy the raw_demos to the designated location
    filename = '/home/camilo/custom_ws/raw_demo' + str(autodemo) + '.bag'
    copy(filename, save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/"+folder_sub[2])
    print("File copied successfully.")
    #copy joint values
    filename = '/home/camilo/custom_ws/raw_demo_joint' + str(autodemo) + '.bag'
    copy(filename, save_folder + folder_sub[0]  + "/"+ folder_sub[1] + "/" + "raw_demos" + "/"+folder_sub[2])
    print("File copied successfully.")

    time.sleep(1)
    Path_compute([0.075, 0.4, 0.405+0.529+0.12, 0.0, 270, 0.0], False, False)
    time.sleep(1)
    trajectory_planning.go_to_joint_state(starting_joint)
    '''
    
    listener()
