#!/usr/bin/env python  
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv





def listen():
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()

    #build tf listener object
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.wait_for_service('spawn')

    # rospy.ServiceProxy method details:
    # __init__(Constructor)
    # 1.name(str)
    # 2.service_class
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    turtle_name = rospy.get_param('turtle', 'turtle2')
    print (turtle_name)

    # __call__(Call operator)
    # turtlesim/Spawn Service
    # float32 x,y,theta
    # string name
    spawner(4, 2, 0, turtle_name)

    # 1HZ
    rate = rospy.Rate(0.25)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
            return trans
            continue

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
            

if __name__ == '__main__':

    print(listen())
    # [x,y]= listen()
    # print("x, y= ", [x,y])