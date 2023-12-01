#!/usr/bin/env python3

import rospy
import math
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import yaml

rospy.init_node('goal_pose')
  
def check_goal_reached(init_pose, goal_coor, bias):
    if(init_pose.pose.position.x > goal_coor[0] - bias and init_pose.pose.position.x < goal_coor[0] + bias\
        and init_pose.pose.position.y > goal_coor[1] - bias and init_pose.pose.position.y < goal_coor[1] + bias):
        return True
    else:
        return False

cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
init_pose = rospy.wait_for_message('/id321/aruco_single/pose', PoseStamped)

init_x = init_pose.pose.position.x
init_y = init_pose.pose.position.y
scale = 0.075

"""
coordinates = [
	[-1, 0],
	[-1, -1],
	[-1, -2],
	[0, -2],
	[0, -3],
	[0, -4],
	[-1, -4],
	[-1, -5],
	[-1, -6],
	[-2, -6],
	[-3, -6],
	[-4, -6],
	[-5, -6],
	[-5, -7],
	[-5, -8],
	[-4, -8],
	[-3, -8],
	[-2, -8],
	[-1, -8],
	[0, -8],
	[0, -9]
]
"""
# Read the YAML data from the file
with open("/home/zeyu/catkin_ws/src/auto_navigation/scripts/cbs_output.yaml", "r") as yaml_file:
    yaml_data = yaml.safe_load(yaml_file)

# Extract coordinates for agent 1 (schedule 1)
agent_1_schedule = yaml_data["schedule"][1]

# Initialize a list to store the coordinates
coordinates = []

# Iterate through the schedule and extract x, y values
for item in agent_1_schedule:
    x = item["x"]
    y = item["y"]
    coordinates.append([x - 8, y - 9])  # Subtract 8 from x and 9 from y

twist = Twist()

for coordinate in coordinates:
	goal_x = coordinate[0] * scale + init_x
	goal_y = coordinate[1] * -scale + init_y
	# goal_pose = rospy.wait_for_message(current_pose, PoseStamped)
	coordinate
	while not check_goal_reached(init_pose, [goal_x, goal_y], 0.02):
		init_pose = rospy.wait_for_message('/id321/aruco_single/pose', PoseStamped)
		orientation_q = init_pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		Orientation = yaw
		dx = goal_x - init_pose.pose.position.x
		dy = goal_y - init_pose.pose.position.y
		distance = math.dist([init_pose.pose.position.x, init_pose.pose.position.y], [goal_x, goal_y])
		goal_direct = math.atan2(dy, dx)

		print("current_coor", [init_pose.pose.position.x, init_pose.pose.position.y])
		print("goal_coor", [goal_x, goal_y])
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

		k2 = 10
		linear = 3
		angular = k2 * theta
		twist.linear.x = linear * distance * math.cos(theta)
		twist.angular.z = -angular
		cmd_pub.publish(twist)

