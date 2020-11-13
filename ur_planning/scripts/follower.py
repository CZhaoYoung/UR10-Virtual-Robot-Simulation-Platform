#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import rospy
import sys
import moveit_commander
import math
import tf2_ros
import geometry_msgs.msg 
import turtlesim.srv

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def test_callback(msg):
    if not isinstance(msg, Pose):
        return

    time.sleep(2)
    new_msg  = copy.deepcopy(msg)

    print(new_msg.x, new_msg.y, new_msg.theta)


def listener():

    rospy.init_node('listener', anonymous=True)

    NewPose = rospy.Subscriber("/turtle1/pose", Pose, test_callback)

    rospy.spin()


class MoveItIkDemo:
    def __init__(self, x,y,z):
        self.x = x
        self.y = y
        self.z = z

        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        # rospy.init_node('ur10_moveit_ik')
                
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
        target_pose = geometry_msgs.msg.Pose()
        
        # plan 1
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
  
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose)
        arm.go(wait=True)
        arm.stop()
        arm.clear_pose_targets()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)




if __name__ == '__main__':
    