#!/usr/bin/env python  
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
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

    # __call__(Call operator)
    # turtlesim/Spawn Service
    # float32 x,y,theta
    # string name
    spawner(4, 2, 0, turtle_name)

    # rospy.Publisher method details:
    # __init__(Constructor)
    # 1.name(str)
    # 2.data_class
    # 3.tcp_nodelay. If True, sets TCP_NODELAY on publisher's socket (disables Nagle algorithm). 
    #   This results in lower latency publishing at the cost of efficiency.
    # 4.latch. If True, the last message published is 'latched', 
    #   meaning that any future subscribers will be sent that message immediately upon connection.
    # 5.queue_size. The queue size used for asynchronously publishing messages from different threads. 
    #   A size of zero means an infinite queue, which can be dangerous.
    turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, 
                                tcp_nodelay = True, latch = True, queue_size = 1)
    # 1HZ
    rate = rospy.Rate(4.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()
        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        turtle_vel.publish(msg)

        rate.sleep()