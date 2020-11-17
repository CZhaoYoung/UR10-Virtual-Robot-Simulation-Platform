Questions:
1. Do I need to add python scripts to CMakeLists.txt everytime when adding new python scripts?

2. Difference between catkin_make and catkin_make_isolated?
https://answers.ros.org/question/320613/catkin_make-vs-catkin_make_isolated-which-is-preferred/


Bugs:
1. Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info

2. Unable to identify any set of controllers that can actuate the specified joints


roslaunch ur_planning start_demo.launch
roslaunch ur10_moveit_config demo.launch 
rosrun ur_planning test2.py

















API
rospy 
tf2_ros 

##static_pose _tf2 _broadcaster.py
	Send TransformStamped message. You can also use commmand line tool in the launchfiles. static_transform_publisher x y z yaw pitch roll frame_id child_frame_id 

	usage:	rosrun ur_planning static_pose_tf_broadcaster.py mystaticturtle 0 0 1 0 0 0

	rostopic: /tf_static


##pose_tf2 _broadcaster.py
    usage:	1.roslaunch ur_planning start_demo.launch
    		2.rosrun tf tf_echo /world /turtle1

##pose_tf2 _listener.py
	usage: 	1.

