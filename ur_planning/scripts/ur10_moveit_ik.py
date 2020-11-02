#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from copy import deepcopy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('ur10_moveit_ik')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.005)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
	NewPose = deepcopy(rospy.Subscriber("chatter", PoseStamped, queue_size = 1))
        
	target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        
        # plan 1
	
        target_pose.pose.position.x = Newpose.pose.pose.position.x
        target_pose.pose.position.y = Newpose.pose.pose.position.y
        target_pose.pose.position.z = Newpose.pose.pose.position.z
        target_pose.pose.orientation.w = Newpose.pose.pose.orientation.x


        # plan 2
        # target_pose.pose.position.x = 0.858
        # target_pose.pose.position.y = -0.0609
        # target_pose.pose.position.z = 0.467
        # target_pose.pose.orientation.x = 0.706
        # target_pose.pose.orientation.y = 0.706
        # target_pose.pose.orientation.z = -0.014
        # target_pose.pose.orientation.w = -0.014

        # plan 3
        # target_pose.pose.position.x = 0.92681
        # target_pose.pose.position.y = 0.25311
        # target_pose.pose.position.z = 0.28889
        # target_pose.pose.orientation.x = 0.68725
        # target_pose.pose.orientation.y = 0.68723
        # target_pose.pose.orientation.z = 0.16642
        # target_pose.pose.orientation.w = -0.16646
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(3)
         
        # 控制机械臂终端向右移动5cm
        # 使用shift_pose_target函数，argv[0]=0,1,2,3,4,5;对应x,y,z,r,p,y
        
        # arm.shift_pose_target(1, -0.05, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
  
        # # 控制机械臂终端反向旋转90度
        # arm.shift_pose_target(3, -1.57, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
           
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    	# spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()


if __name__ == "__main__":
    MoveItIkDemo()

    
    
