#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Move from Point A to Point B

from tokenize import group
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped
from copy import deepcopy

# 初始化group的API
moveit_commander.roscpp_initialize(sys.argv)

# 初始化ROS节点
rospy.init_node('moveit_cartesian_test', anonymous=True)

# 是否需要使用笛卡尔空间的运动规划
cartesian = rospy.get_param('~cartesian', True)

# 初始化需要使用move group控制的机械臂中的arm group
group = moveit_commander.MoveGroupCommander('manipulator')

# 当运动规划失败后，允许重新规划
group.allow_replanning(True)

# 设置目标位置所使用的参考坐标系
group.set_pose_reference_frame('world')

# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
group.set_goal_position_tolerance(0.001)
group.set_goal_orientation_tolerance(0.001)

# 设置允许的最大速度和加速度
group.set_max_acceleration_scaling_factor(0.05) # was 0.5
group.set_max_velocity_scaling_factor(0.05) # was 0.5

# 获取终端link的名称
end_effector_link = group.get_end_effector_link()

reference_frame = "world"

# orientation constraint, maintains end effector to be vertical
ori_con = moveit_msgs.msg.OrientationConstraint()  # create orientation_constraint
# ori_con.link_name = self.end_effector_link  # for EE link
ori_con.link_name = "tool_link_ee"
ori_con.header.frame_id = reference_frame
# ori_con.orientation = group.get_current_pose(self.end_effector_link).pose.orientation
ori_con.orientation.x = 1.8236e-5
ori_con.orientation.y = -0.30307
ori_con.orientation.z = -6.2059e-5
ori_con.orientation.w = 0.952967
ori_con.absolute_x_axis_tolerance = 3.14  # ignore this axis for constraints#
ori_con.absolute_y_axis_tolerance = 0.1
ori_con.absolute_z_axis_tolerance = 0.1
ori_con.weight = 1.0
# constraint message
constraint = moveit_msgs.msg.Constraints()
constraint.name = "straight_down"
constraint.orientation_constraints.append(ori_con)
group.set_path_constraints(constraint)

# 控制机械臂先回到初始化位置
# group.set_named_target('home_pose')
# group.go()
# rospy.sleep(1)

# 获取当前位姿数据最为机械臂运动的起始位姿
start_pose = group.get_current_pose("tool_link_ee").pose

# 初始化路点列表
waypoints = []

# 将初始位姿加入路点列表 there is a bug in MoveIt, we can't add starting point to waypoints
# waypoints.append(start_pose)

# 设置路点数据，并加入路点列表
wpose = deepcopy(start_pose)

'''
# Home point #
wpose.position.x = -0.4
wpose.position.y = 0.4
wpose.position.z = 1.2
waypoints.append(deepcopy(wpose))
'''

# First point #
wpose.position.x = -0.4
wpose.position.y = 0.4
wpose.position.z = 1.0
waypoints.append(deepcopy(wpose))


# Second point #
wpose.position.x += 0.5
# wpose.position.y -= 0.1
#wpose.position.z += 0.1
waypoints.append(deepcopy(wpose))

fraction = 0.0  # 路径规划覆盖率
maxtries = 100  # 最大尝试规划次数
attempts = 0  # 已经尝试规划次数

# 设置机器臂当前的状态作为运动初始状态
group.set_start_state_to_current_state()

# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
while fraction < 0.95 and attempts < maxtries:
    (plan, fraction) = group.compute_cartesian_path(waypoints,0.01,0.00, True)
           # waypoint poses，路点列表, eef_step，终端步进值, jump_threshold，跳跃阈值, collision
               
    # 尝试次数累加
    attempts += 1

    # 打印运动规划进程
    if attempts % 10 == 0:
        rospy.loginfo("Still trying after " +
                      str(attempts) + " attempts...")

    # 如果路径规划成功（覆盖率95%）,则开始控制机械臂运动
if fraction > 0.95:
    rospy.loginfo("Path computed successfully. Moving the group.")
    group.execute(plan)
    rospy.loginfo("Path execution complete.")
    # 如果路径规划失败，则打印失败信息
else:
    rospy.loginfo("Path planning failed with only " + str(fraction) +
                  " success after " + str(maxtries) + " attempts.")

rospy.sleep(1)
    # 控制机械臂先回到初始化位置
    # group.set_named_target('home_pose')
    # group.go()
    # rospy.sleep(1)
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)
