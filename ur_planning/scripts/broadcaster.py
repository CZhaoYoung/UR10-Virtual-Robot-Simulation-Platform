#!/usr/bin/env python
import rospy
import time
import copy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def topic_callback(msg):
    if not isinstance(msg, Pose):
    	return
    print("x =", msg.x)
    print("y =", msg.y)
    print("theta =", msg.theta)
    time.sleep(5)
 
def callback(data):
    rospy.loginfo("I heard %s",data.x)
    time.sleep(10)

def test_callback(msg):
    if not isinstance(msg, Pose):
        return

    time.sleep(2)
    new_msg  = copy.deepcopy(msg)
    print(new_msg.x, new_msg.y, new_msg.theta)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    NewPose = rospy.Subscriber("/turtle1/pose", Pose, test_callback)

    # NewPose = rospy.Subscriber("/vrpn_client_node/cf4/pose", PoseStamped, topic_callback)

    # print(NewPose.x, NewPose.y, NewPose.z)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
