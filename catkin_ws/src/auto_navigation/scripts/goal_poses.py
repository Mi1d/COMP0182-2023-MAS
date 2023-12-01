#!/usr/bin/env python3

import rospy
import math
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

rospy.init_node('goal_pose')
  
def check_goal_reached(init_pose, goal_pose, bias):
    if(init_pose.pose.position.x > goal_pose.pose.position.x - bias and init_pose.pose.position.x < goal_pose.pose.position.x + bias\
        and init_pose.pose.position.y > goal_pose.pose.position.y - bias and init_pose.pose.position.y < goal_pose.pose.position.y + bias):
        return True
    else:
        return False

cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
init_pose = rospy.wait_for_message('/id321/aruco_single/pose', PoseStamped)
goal_pose1 = '/id325/aruco_single/pose'
goal_pose2 = '/id326/aruco_single/pose'
goal_pose3 = '/id327/aruco_single/pose'
goal_poses = [goal_pose1, goal_pose2, goal_pose3]
twist = Twist()

for current_pose in goal_poses:
	goal_pose = rospy.wait_for_message(current_pose, PoseStamped)
	while not check_goal_reached(init_pose, goal_pose, 0.02):
		init_pose = rospy.wait_for_message('/id321/aruco_single/pose', PoseStamped)
		orientation_q = init_pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		Orientation = yaw
		dx = goal_pose.pose.position.x - init_pose.pose.position.x
		dy = goal_pose.pose.position.y - init_pose.pose.position.y
		distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y], [goal_pose.pose.position.x, goal_pose.pose.position.y])
		goal_direct = math.atan2(dy, dx)

		print("init_pose", [init_pose.pose.position.x, init_pose.pose.position.y])
		print("goal_pose", [goal_pose.pose.position.x, goal_pose.pose.position.y])
		print("Orientation", Orientation)

		print("goal_direct", goal_direct)
		if(Orientation < 0):
			Orientation = Orientation + 2 * math.pi
		if(goal_direct < 0):
			goal_direct = goal_direct + 2 * math.pi

		theta = goal_direct - Orientation

		if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
			theta = theta + 2 * math.pi
		elif theta > 0 and abs(theta - 2 * math.pi) < theta:
			theta = theta - 2 * math.pi

		print("theta:", theta)

		k2 = 3
		linear = 1.5
		angular = k2 * theta
		twist.linear.x = linear * distance * math.cos(theta)
		twist.angular.z = -angular
		cmd_pub.publish(twist)

