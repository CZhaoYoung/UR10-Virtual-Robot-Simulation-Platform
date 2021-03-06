#!/usr/bin/env python
import rospy
import time
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from turtlesim.msg import Pose

def topic_callback(msg):
    if not isinstance(msg, Pose):
    	return
    print("x =", msg.x)
    print("y =", msg.y)
    print("theta =", msg.theta)
    # time.sleep()
 
def callback(data):
    rospy.loginfo("I heard %s",data.x)
    time.sleep(10)

def test_callback(msg):
    if not isinstance(msg, PoseStamped):
        return

    time.sleep(0.25)
    new_msg  = copy.deepcopy(msg)
    print("[x]=", new_msg.pose.position.x)
    # print("[x, y, z]=", new_msg.pose.position.x, new_msg.pose.position.y, new_msg.pose.position.z)
    # print("[w, x, y, z]=",  new_msg.pose.orientation.w, new_msg.pose.orientation.x, 
                            # new_msg.pose.orientation.y, new_msg.pose.orientation.z)
    print("---------------------------------------------------")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # NewPose = rospy.Subscriber("/turtle1/pose", Pose, test_callback, queue_size=1)

    NewPose = rospy.Subscriber("/vrpn_client_node/cf1120/pose", PoseStamped, test_callback, queue_size=1)

    # print(NewPose.x, NewPose.y, NewPose.z)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
