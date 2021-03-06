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


class MoveIt_Python_Interface(object):
    def __init__(self):
    	super(MoveIt_Python_Interface, self).__init__()
        # --SETUP--
        # Initialize moveit_commander and rospy_nore
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur10_moveit', anonymous=True)
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('ur10_moveit_ik', anonymous = True)
        
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

        scene_publisher = rospy.Publisher('planning_scene', 
                                            moveit_msgs.msg.PlanningScene, 
                                            queue_size = 5)

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
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.scene_publisher = scene_publisher
        self.planning_frame = planning_frame
        self.eef_link = end_effector_link
        self.group_names = group_names

        self.colors = dict()

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

        pose_goal.position.x = value_x/10    
        pose_goal.position.y = value_y/10
        pose_goal.position.z = value_z/2

        # for cf4
        # pose_goal.position.x = value_x/2    
        # pose_goal.position.y = value_y/2
        # pose_goal.position.z = value_z/2
        # pose_goal.position.z = 0 			# 2d simulation only x & y

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
        joint_values = self.group.get_current_joint_values()

        print("current_pose: ", current_pose)
        print("")
        print("get_current_joint_values: ", joint_values)

        return all_close(pose_goal, current_pose, 0.01)


    def set_color(self, name, r, g, b, a = 0.9):
        # moveit color object
        color = moveit_msgs.msg.ObjectColor()

        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # update color
        self.colors[name] = color

    def send_color(self):
        # moveit scene object
        p = moveit_msgs.msg.PlanningScene()

        p.is_diff = True

        #publish color
        for color in self.colors.values():
            p.object_colors.append(color)

        self.scene_publisher.publish(p)

    def add_object(self):
        planning_frame = self.planning_frame
        scene = self.scene

        # table_id
        table_id = 'table'
        
        #table_size
        table_size = [2, 2, 0.02]   

        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = planning_frame
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = -0.02
        table_pose.pose.orientation.w = 1
        scene.add_box(table_id, table_pose, table_size)

        self.set_color(table_id, 0.9, 0.9, 0.9, 1.0)
        self.send_color()

	
    def plan_cartesian(self, scale = 1):
	group = self.group

	waypoints = []
	wpose = group.get_current_pose().pose
	wpose.position.z -= scale * 0.1
	wpose.position.y += scale * 0.2

	waypoints.append(copy.deepcopy(wpose))

	wpose.position.x += scale * 0.1
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.y -= scale * 0.1
	waypoints.append(copy.deepcopy(wpose))

	(plan, fraction) = group.compute_cartesian_path(
							waypoints, 		# waypoints to follow
							0.01,			# eef_step 1cm
							0.0)			# jump_threshold
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
# END--MoveIt_Python_Interface--



global MOVE
MOVE = MoveIt_Python_Interface()

def call_back(msg):
    if not isinstance(msg, Pose):
        return

    # time.sleep(1)
    # new_msg  = copy.deepcopy(msg)
    # print("[x,y] = ", new_msg.pose.position.x, new_msg.pose.position.y, new_msg.pose.position.z)
    # MOVE.go_to_pose_goal(new_msg.pose.position.x, new_msg.pose.position.y, new_msg.pose.position.z)
    time.sleep(0.25)

    print("[x,y] = ", msg.x, msg.y)
    MOVE.go_to_pose_goal(msg.x, msg.y, 0.4)

    # new_msg  = copy.deepcopy(msg)
    # # test 1
    # print(new_msg)
    # print("")
    # MOVE = MoveIt_Python_Interface()
    # MOVE.go_to_pose_goal(new_msg.x, new_msg.y)


def listener():
	# new_pose = rospy.Subscriber("/vrpn_client_node/cf4/pose", PoseStamped, call_back, queue_size=1)
	new_pose = rospy.Subscriber("/turtle1/pose", Pose, call_back, queue_size = 1, tcp_nodelay = True)
	rospy.spin()


def main():
    try:
		print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
		raw_input()
       	 	MOVE.add_object()

		print ("============ Press `Enter` to execute a movement using a joint state goal ...")
		raw_input()
		MOVE.go_to_joint_state()

		print ("============ Press `Enter` to execute a movement using a pose goal ...")
		raw_input()
		MOVE.go_to_pose_goal(1.716, 0.1218, 0.76)
		MOVE.go_home()

		# print ("============ Press `Enter` to plan and display a Cartesian path ...")
		# raw_input()
		# (cartesian_plan, fraction) = MOVE.plan_cartesian()

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

