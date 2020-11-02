#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def topic_callback(msg):
    if not isinstance(msg, Pose):
    	return
    print("x =", msg.x)
    print("y =", msg.y)
    print("theta =", msg.theta)
    time.sleep(10)
 
def callback(data):
    rospy.loginfo("I heard %s",data.x)
    time.sleep(10)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    NewPose = rospy.Subscriber("/turtle1/pose", Pose, callback)
    # NewPose = rospy.Subscriber("/vrpn_client_node/cf4/pose", PoseStamped, topic_callback)
    # print(NewPose.callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
