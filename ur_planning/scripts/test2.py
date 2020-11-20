#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import copy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose


from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def all_close(goal, actual, tolerance):
    """
    Convenient method for testing if a list of values are within a tolerance of their counterparts in another list
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


class MoveIt_Python_Interface():
    def __init__(self):
        # --SETUP--
        # Initialize moveit_commander and rospy_nore
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur10_moveit_ik', anonymous = True)
        
        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot
        robot = moveit_commander.RobotCommander()
        
        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object. 
        # group_name = "panda_arm"
        group_name = 'manipulator'
        group = moveit_commander.MoveGroupCommander(group_name)
     
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        # --BASIC_INFO--
        # the reference name of the robot
        planning_frame = group.get_planning_frame()
        print("========= reference_frame: %s" % planning_frame)

        # the end effector name
        end_effector_link = group.get_end_effector_link()
        print("========= end_effector: %s" % end_effector_link)

        # the groups name of the robot
        group_names = robot.get_group_names()
        print("========= Robot Groups: %s" % group_names)

        # print the entire state of the robot
        print("========= Printing the robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = end_effector_link
        self.group_names = group_names

    def go_home(self):
        group = self.group

        # --GO HOME PLANNING--
        group.set_named_target('home')
        group.go()
        group.stop()

    def go_to_joint_state(self):

        group = self.group

        # --FK PLANNING--
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        # joint_goal[3] = -pi/2
        # joint_goal[4] = 0
        # joint_goal[5] = pi/3

        # enable when using panda arm
        # joint_goal[6] = 0

        group.go(joint_goal, wait = True)
        group.stop()

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, value_x, value_y, value_z):

        group = self.group
        eef_link = self.eef_link

        # --IK PLANNING--
        pose_goal = geometry_msgs.msg.Pose()
        
        # enable when using panda arm
        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.4

        pose_goal.position.x = value_x/2    # 2d simulation only x & y
        pose_goal.position.y = value_y/2
        pose_goal.position.z = value_z/2
        pose_goal.orientation.w = -0.014
        pose_goal.orientation.x = 0.706
        pose_goal.orientation.y = 0.706
        pose_goal.orientation.z = -0.014
        group.set_pose_target(pose_goal, eef_link)

        plan = group.go(wait = True)
        group.stop()
        # It is always good to clear your targets after planning with poses.
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose

        print("current_pose: ", current_pose)
        print("")
        print("get_goal_position_tolerance: ", self.group.get_goal_position_tolerance())

        return all_close(pose_goal, current_pose, 0.01)

	def plan_cartesian_path(self, scale = 1):

		group = self.group
		# --CARTESIAN PLANNING-
		waypoints = []

		wpose = group.get_current_pose().pose
		wpose.position.z -= scale * 0.1  # First move up (z)
		wpose.position.y += scale * 0.2  # and sideways (y)

		waypoints.append(copy.deepcopy(wpose))

		wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		waypoints.append(copy.deepcopy(wpose))

		wpose.position.y -= scale * 0.1  # Third move sideways (y)
		waypoints.append(copy.deepcopy(wpose))

		(plan, fraction) = group.compute_cartesian_path(
		                                   waypoints,   # waypoints to follow
		                                   0.01,        # eef_step 1cm
		                                   0.0)         # jump_threshold

		# Note: We are just planning, not asking move_group to actually move the robot yet:
		return plan, fraction

	def display_trajectory(self, plan):

		robot = self.robot
		display_trajectory_publisher = self.display_trajectory_publisher

		# --DISPLAY A TRAJECTORY
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = robot.get_current_state()
		display_trajectory.trajectory.append(plan)

		display_trajectory_publisher.publish(display_trajectory)

	def execute_plan(self, plan):

		group = self.group
		group.execute(plan, wait = True)


global MOVE
MOVE = MoveIt_Python_Interface()

def call_back(msg):
    if not isinstance(msg, PoseStamped):
        return

    time.sleep(1)
    new_msg  = copy.deepcopy(msg)
    print("[x,y] = ", new_msg.pose.position.x, new_msg.pose.position.y, new_msg.pose.position.z)
    MOVE.go_to_pose_goal(new_msg.pose.position.x, new_msg.pose.position.y, new_msg.pose.position.z)

    # new_msg  = copy.deepcopy(msg)
    # # test 1
    # print(new_msg)
    # print("")
    # MOVE = MoveIt_Python_Interface()
    # MOVE.go_to_pose_goal(new_msg.x, new_msg.y)


def listener():
	new_pose = rospy.Subscriber("/vrpn_client_node/cf4/pose", PoseStamped, call_back, queue_size=1)
	rospy.spin()


def main():

    try:
		print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
		raw_input()
		MOVE = MoveIt_Python_Interface()

		print ("============ Press `Enter` to execute a movement using a joint state goal ...")
		raw_input()
		MOVE.go_to_joint_state()

		print ("============ Press `Enter` to execute a movement using a pose goal ...")
		raw_input()
		MOVE.go_to_pose_goal(1.716, 0.1218, 0.76)
		MOVE.go_home()

		# print ("============ Press `Enter` to plan and display a Cartesian path ...")
		# raw_input()
		# cartesian_plan, fraction = MOVE.plan_cartesian_path()

		# print ("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
		# raw_input()
		# MOVE.display_trajectory(cartesian_plan)

		# print ("============ Press `Enter` to execute a saved path ...")
		# raw_input()
		# MOVE.execute_plan(cartesian_plan)

		print ("============ Press `Enter` to execute a movement using a pose goal ...")
		raw_input()
		listener()
		MOVE.go_home()

    except rospy.ROSInterruptException:
        return

    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()